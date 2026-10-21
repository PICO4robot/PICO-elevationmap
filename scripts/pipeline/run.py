"""`pico-elevationmap run`：合成场景实时建图管线（Phase 1）。

内置合成场景发生器（平地 + 轴对齐方台/台阶，零外部资产，家族测试约定），
沿圆周轨迹模拟带偏航的传感器位姿，按 YAML 数据源噪声模型注入径向测距噪声，
逐帧送入 ElevationMapper；逐步打印耗时，结束输出地图统计，可 --save 导出 npz。

模块级不 import cupy/mapper —— `pico-elevationmap run --help` 在无 GPU 机器上
也必须可用。
"""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from pico_elevationmap.sensors.config import SensorSourceConfig

_PROGRESS_EVERY = 10


@dataclass(frozen=True)
class BoxRegion:
    """轴对齐的方台/台阶区域：范围内地面抬升到 height。"""

    x_min: float
    x_max: float
    y_min: float
    y_max: float
    height: float

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> BoxRegion:
        try:
            return cls(*(float(data[k]) for k in ("x_min", "x_max", "y_min", "y_max", "height")))
        except KeyError as exc:
            raise ValueError(f"scene box missing key {exc}") from exc


class SyntheticScene:
    """平地 + 若干方台/台阶的高度场；sample_world 生成世界系点云。"""

    def __init__(self, ground: float = 0.0, boxes: list[BoxRegion] | None = None) -> None:
        self.ground = float(ground)
        self.boxes = boxes or []

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> SyntheticScene:
        data = data or {}
        boxes = [BoxRegion.from_dict(b) for b in data.get("boxes", [])]
        return cls(ground=float(data.get("ground", 0.0)), boxes=boxes)

    def height_at(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        z = np.full(x.shape, self.ground, dtype=np.float64)
        for b in self.boxes:
            inside = (x >= b.x_min) & (x <= b.x_max) & (y >= b.y_min) & (y <= b.y_max)
            z = np.where(inside, b.height, z)
        return z

    def sample_world(self, x_center: float, y_center: float, extent: float, spacing: float) -> np.ndarray:
        """传感器足迹附近的世界系点云（Nx3），按场景高度场采样。"""
        half = extent / 2.0
        xs = np.arange(x_center - half, x_center + half, spacing)
        ys = np.arange(y_center - half, y_center + half, spacing)
        X, Y = np.meshgrid(xs, ys)
        Z = self.height_at(X, Y)
        return np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)


@dataclass
class SimConfig:
    """YAML `sim` 段：仿真步数/节拍/轨迹/场景/采样。"""

    steps: int = 60
    dt: float = 0.1
    seed: int = 42
    sensor_height: float = 1.0
    spacing: float = 0.04  # 世界点云采样间距（米）
    trajectory: dict[str, Any] = field(default_factory=lambda: {"type": "circle", "radius": 1.5, "angular_speed": 0.15})
    scene: dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_dict(cls, data: dict[str, Any] | None) -> SimConfig:
        if data is None:
            return cls()
        if not isinstance(data, dict):
            raise ValueError("'sim' section must be a mapping")
        allowed = {"steps", "dt", "seed", "sensor_height", "spacing", "trajectory", "scene"}
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(f"'sim' section: unknown keys {sorted(unknown)}")
        cfg = cls(**data)
        if cfg.steps <= 0:
            raise ValueError("sim.steps must be > 0")
        if cfg.dt <= 0.0:
            raise ValueError("sim.dt must be > 0")
        if cfg.spacing <= 0.0:
            raise ValueError("sim.spacing must be > 0")
        return cfg

    def pose_at(self, stamp: float) -> np.ndarray:
        """轨迹上的 T_map_sensor（圆周 + 切向偏航，传感器水平）。"""
        kind = self.trajectory.get("type", "circle")
        if kind != "circle":
            raise ValueError(f"unknown trajectory type '{kind}' (supported: circle)")
        radius = float(self.trajectory.get("radius", 1.5))
        w = float(self.trajectory.get("angular_speed", 0.15))
        angle = w * stamp
        yaw = angle + np.pi / 2.0  # 切向
        c, s = np.cos(yaw), np.sin(yaw)
        T = np.eye(4)
        T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c
        T[0, 3] = radius * np.cos(angle)
        T[1, 3] = radius * np.sin(angle)
        T[2, 3] = self.sensor_height
        return T


def inject_noise(
    rng: np.random.Generator, points_sensor: np.ndarray, source: SensorSourceConfig
) -> np.ndarray:
    """按源的噪声模型做径向测距噪声注入（传感器系）。"""
    if len(points_sensor) == 0:
        return points_sensor
    d = np.linalg.norm(points_sensor, axis=1)
    std = source.measurement_std(d)
    direction = points_sensor / d[:, None]
    noisy = points_sensor + direction * (std * rng.standard_normal(d.shape))[:, None]
    return noisy.astype(np.float32)


def _sync_gpu() -> None:
    import cupy as cp

    cp.cuda.Stream.null.synchronize()


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="pico-elevationmap run",
        description="Synthetic-scene live mapping demo (Phase 1).",
    )
    parser.add_argument("--config", required=True, type=Path, help="YAML: map / sources / plugins / sim")
    parser.add_argument("--save", type=Path, default=None, help="optional npz dump of all layers")
    parser.add_argument("--steps", type=int, default=None, help="override sim.steps")
    args = parser.parse_args()

    # 懒加载 GPU 栈（--help 路径不触达）
    from pico_elevationmap.core.mapper import ElevationMapper, MapperConfig

    cfg = MapperConfig.from_yaml(args.config)
    sim = SimConfig.from_dict(cfg.sim)
    steps = args.steps if args.steps is not None else sim.steps
    source = cfg.primary_source
    scene = SyntheticScene.from_dict(sim.scene)
    extent = cfg.map_length * 0.8  # 传感器足迹覆盖地图大部分区域
    rng = np.random.default_rng(sim.seed)

    mapper = ElevationMapper(cfg)
    print(
        f"[run] map {cfg.map_length}m @ {cfg.resolution}m/cell, source '{source.name}' "
        f"({source.sensor_type}, c={source.noise_coefficient}), steps={steps}"
    )

    times: list[float] = []
    for i in range(steps):
        stamp = i * sim.dt
        pose = sim.pose_at(stamp)
        mapper.add_pose(stamp, pose)

        world = scene.sample_world(pose[0, 3], pose[1, 3], extent, sim.spacing)
        sensor = (pose[:3, :3].T @ (world - pose[:3, 3]).T).T  # 世界系 → 传感器系
        sensor = sensor[sensor[:, 2] < 0.0]  # 只保留下方点（下视传感器）
        noisy = inject_noise(rng, sensor, source)

        t0 = time.perf_counter()
        accepted = mapper.input(noisy, stamp, source=source.name)
        _sync_gpu()
        times.append((time.perf_counter() - t0) * 1e3)

        mapper.update_variance()
        mapper.update_time()
        mapper.move_to([pose[0, 3], pose[1, 3], 0.0], pose[:3, :3])

        if (i + 1) % _PROGRESS_EVERY == 0 or i + 1 == steps:
            print(f"[run] step {i + 1}/{steps}: {len(noisy)} pts ({accepted} fused), {times[-1]:.2f} ms")

    stats = mapper.stats()
    arr = np.asarray(times)
    print(
        f"[run] timing ms/step: mean {arr.mean():.2f}, min {arr.min():.2f}, max {arr.max():.2f}, "
        f"total {arr.sum():.0f}"
    )
    print(
        f"[run] map stats: coverage {stats.coverage:.1%} ({stats.valid_cells}/{stats.total_cells}), "
        f"height mean {stats.mean_height:.3f} m, range [{stats.min_height:.3f}, {stats.max_height:.3f}] m, "
        f"center {stats.center}"
    )

    if args.save is not None:
        layers = mapper.get_map()
        np.savez(
            args.save,
            **layers,
            resolution=np.float32(cfg.resolution),
            center=np.asarray(stats.center, dtype=np.float32),
        )
        print(f"[run] saved {len(layers)} layers -> {args.save}")
