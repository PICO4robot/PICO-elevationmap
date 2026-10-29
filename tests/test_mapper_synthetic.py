"""ElevationMapper 合成点云验证（GPU）：平地均值、方台顶面、位姿漂移下的相干性。

模块级不 import cupy/mapper —— 通过 mapper_mod fixture 懒加载；无 GPU 环境由
conftest 的 gpu fixture（cupy/CUDA 不可用时 skip）保证整组跳过。
"""

from __future__ import annotations

import numpy as np
import pytest

from pico_elevationmap.sensors.config import SensorSourceConfig


def _cupy_cuda_available() -> bool:
    try:
        import cupy as cp

        return bool(cp.cuda.is_available())
    except Exception:
        return False


# 复用 conftest 同款判断，作为整组 skip 标记
pytestmark = pytest.mark.skipif(not _cupy_cuda_available(), reason="cupy/CUDA not available")


@pytest.fixture(scope="module")
def mapper_mod():
    pytest.importorskip("cupy", reason="cupy not installed")
    from pico_elevationmap.core import mapper as mapper_module

    return mapper_module


def _sync() -> None:
    import cupy as cp

    cp.cuda.Stream.null.synchronize()


def _rz(deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _straight_down_pose(height: float = 1.0) -> np.ndarray:
    T = np.eye(4)
    T[2, 3] = height
    return T


def _world_grid(half: float, spacing: float, z: float) -> np.ndarray:
    xs = np.arange(-half, half, spacing)
    X, Y = np.meshgrid(xs, xs)
    return np.stack([X.ravel(), Y.ravel(), np.full(X.size, z, dtype=np.float64)], axis=1)


def _perfect_cam() -> dict[str, SensorSourceConfig]:
    return {"cam": SensorSourceConfig(name="cam", sensor_type="perfect")}


def test_flat_ground_mean_elevation(gpu, mapper_mod, tmp_path) -> None:
    cfg = mapper_mod.MapperConfig(
        resolution=0.05,
        map_length=6.0,
        sources=_perfect_cam(),
        plugins={
            "min_filter": {
                "enable": True,
                "layer_name": "min_filter",
                "fill_nan": False,
                "is_height_layer": True,
                "extra_params": {"dilation_size": 2, "iteration_n": 5},
            },
            "smooth_filter": {
                "enable": True,
                "layer_name": "smooth",
                "fill_nan": False,
                "is_height_layer": True,
                "extra_params": {"input_layer_name": "elevation"},
            },
        },
    )
    mapper = mapper_mod.ElevationMapper(cfg, plugin_dir=tmp_path)
    mapper.add_pose(0.0, _straight_down_pose())

    world = _world_grid(2.5, 0.04, z=0.0)
    sensor = world - np.array([0.0, 0.0, 1.0])  # R = I，传感器系 = 世界系平移
    fused = mapper.input(sensor, stamp=0.0, source="cam")
    _sync()

    assert fused > 10000
    stats = mapper.stats()
    assert stats.coverage > 0.4
    assert abs(stats.mean_height) < 0.02  # 平地真值 0，无噪声注入

    layers = mapper.get_map()
    assert {"elevation", "min_filter", "smooth"} <= set(layers)
    for name in ("min_filter", "smooth"):
        assert layers[name].shape == layers["elevation"].shape


def test_box_top_detected(gpu, mapper_mod, tmp_path) -> None:
    box_h = 0.4
    cfg = mapper_mod.MapperConfig(resolution=0.05, map_length=6.0, sources=_perfect_cam())
    mapper = mapper_mod.ElevationMapper(cfg, plugin_dir=tmp_path)
    mapper.add_pose(0.0, _straight_down_pose())

    ground = _world_grid(2.5, 0.04, z=0.0)
    # 方台 footprint 内只采样顶面（真实传感器无法穿透方台看到其下方地面；
    # 单帧同 cell 双表面会被内核先平均，那是 2.5D 假设外的场景）
    under_box = (ground[:, 0] >= 0.5) & (ground[:, 0] <= 1.5) & (ground[:, 1] >= -0.5) & (ground[:, 1] <= 0.5)
    ground = ground[~under_box]
    xs = np.arange(0.5, 1.5, 0.04)
    ys = np.arange(-0.5, 0.5, 0.04)
    X, Y = np.meshgrid(xs, ys)
    top = np.stack([X.ravel(), Y.ravel(), np.full(X.size, box_h)], axis=1)
    world = np.vstack([ground, top]) - np.array([0.0, 0.0, 1.0])
    mapper.input(world, stamp=0.0, source="cam")
    _sync()

    elev = mapper.get_layer("elevation")
    valid = np.isfinite(elev)
    assert valid.sum() > 0
    # 方台顶面被识别在真值附近
    assert float(np.nanmax(elev)) == pytest.approx(box_h, abs=0.05)
    top_cells = valid & (elev > 0.5 * box_h)
    expected = (1.0 * 1.0) / (0.05**2)  # 台顶面积 / 分辨率²
    assert top_cells.sum() > 0.5 * expected
    # 地面区域均值仍贴近 0
    ground_cells = valid & (elev < 0.1)
    assert ground_cells.sum() > 0
    assert abs(float(elev[ground_cells].mean())) < 0.02


def test_pose_drift_keeps_map_coherent(gpu, mapper_mod, tmp_path) -> None:
    """随机游走位姿漂移 + laser 测距噪声下，地图保持相干（漂移补偿生效）。"""
    rng = np.random.default_rng(7)
    src = SensorSourceConfig(name="lidar", sensor_type="laser", noise_coefficient=0.005, min_range=0.3, max_range=10.0)
    cfg = mapper_mod.MapperConfig(resolution=0.05, map_length=6.0, sources={"lidar": src})
    mapper = mapper_mod.ElevationMapper(cfg, plugin_dir=tmp_path)

    radius, w, dt, steps = 0.8, 0.3, 0.1, 25
    drift_xy = np.zeros(2)
    for i in range(steps):
        stamp = i * dt
        angle = w * stamp
        T = np.eye(4)
        T[:3, :3] = _rz(np.rad2deg(angle + np.pi / 2.0))
        T[0, 3], T[1, 3], T[2, 3] = radius * np.cos(angle), radius * np.sin(angle), 1.0

        drift_xy += rng.normal(0.0, 0.008, size=2)  # 里程计随机游走漂移
        T_reported = T.copy()
        T_reported[0, 3] += drift_xy[0]
        T_reported[1, 3] += drift_xy[1]
        T_reported[:3, :3] = T[:3, :3] @ _rz(np.rad2deg(rng.normal(0.0, 0.01)))

        mapper.add_pose(stamp, T_reported)
        # 真值系生成地面点云，按报告位姿变换到传感器系 —— 模拟"位姿漂移注入"
        world = _world_grid(2.2, 0.05, z=0.0) + np.array([T[0, 3], T[1, 3], 0.0])
        pts = (T_reported[:3, :3].T @ (world - T_reported[:3, 3]).T).T
        d = np.linalg.norm(pts, axis=1)
        keep = (d >= src.min_range) & (d <= src.max_range)
        pts = pts[keep]
        std = src.measurement_std(np.linalg.norm(pts, axis=1))
        radial = pts / np.linalg.norm(pts, axis=1)[:, None]
        noise = (std * rng.standard_normal(len(pts)))[:, None]
        pts = (pts + radial * noise).astype(np.float32)

        mapper.input(pts, stamp, source="lidar")
        mapper.update_variance()
        mapper.update_time()
        mapper.move_to([T_reported[0, 3], T_reported[1, 3], 0.0], T_reported[:3, :3])
    _sync()

    stats = mapper.stats()
    assert stats.coverage > 0.3
    assert abs(stats.mean_height) < 0.08  # 漂移补偿下均值仍贴近地面真值
