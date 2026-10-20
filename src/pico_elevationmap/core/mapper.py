"""ROS-free 建图外观（Phase 1）：包装上游 ElevationMap + 位姿缓冲 + 每源噪声配置。

输入约定：input() 接收传感器系点云（Nx3）与采集时间戳 stamp；位姿由 add_pose()
按时间戳注入 T_map_sensor，插值对齐后拆成 (R, t) 转给上游
``input_pointcloud(points, ["x","y","z"], R, t, pos_noise, ori_noise)``。上游内核在
该调用内完成 1D 高斯/Kalman 式融合、Mahalanobis 门限、GPU raycasting 可见性
清理与漂移补偿。

每源 min/max_range 在包装层按传感器系距离裁剪（上游仅有全局
min_valid_distance / max_height_range）；测量噪声系数经
SensorSourceConfig.upstream_noise_factor 映射到上游全局 sensor_noise_factor
（内核编译期烘焙，多源共享一张图时取 map.sensor_noise_factor，缺省回退到
首个输入源的映射系数）。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field, fields
from pathlib import Path
from typing import Any

import cupy as cp
import numpy as np
import yaml
from elevation_mapping_cupy.elevation_mapping import ElevationMap
from elevation_mapping_cupy.parameter import Parameter

from pico_elevationmap.core.pose_buffer import PoseBuffer
from pico_elevationmap.plugins.chain import (
    CUSTOM_PLUGINS,
    register_custom_plugins,
    validate_plugin_decl,
    write_plugin_config,
)
from pico_elevationmap.sensors.config import SensorSourceConfig, parse_sources

__all__ = ["ElevationMapper", "MapperConfig", "MapStats"]

_DEFAULT_SOURCE = SensorSourceConfig(name="default")

# 上游内建法线层（get_map_with_name_ref 支持，但 exists_layer 不覆盖，单独登记）
_NORMAL_LAYER_NAMES = ("normal_x", "normal_y", "normal_z")


@dataclass
class MapperConfig:
    """建图主配置：YAML 顶层分 map / sources / plugins / sim 四段。

    sim 段只透传存储（由调用方解释，见 scripts/pipeline/run.py），其余三段在此解析。
    sensor_noise_factor 为 None 时回退到首个输入源的 upstream_noise_factor，
    再缺省 0.05（上游默认值）。
    """

    resolution: float = 0.04
    map_length: float = 8.0
    sensor_noise_factor: float | None = None
    mahalanobis_thresh: float = 2.5
    initial_variance: float = 10.0
    max_variance: float = 1.0
    time_variance: float = 0.01
    time_interval: float = 0.1
    min_valid_distance: float = 0.3
    max_height_range: float = 1.0
    max_ray_length: float = 2.0
    enable_visibility_cleanup: bool = True
    enable_drift_compensation: bool = True
    enable_overlap_clearance: bool = True
    weight_file: str = ""  # Phase 1 不加载学习式 traversability 权重
    sources: dict[str, SensorSourceConfig] = field(default_factory=dict)
    plugins: dict[str, dict[str, Any]] = field(default_factory=dict)
    sim: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.sensor_noise_factor is None:
            self.sensor_noise_factor = self.primary_source.upstream_noise_factor
        # 自有 geo_* 插件需要 resolution；缺省注入 map.resolution，避免配置重复
        for name, decl in self.plugins.items():
            if not isinstance(decl, dict):
                continue
            extra = decl.get("extra_params")
            if isinstance(extra, dict) and (decl.get("type", name)) in CUSTOM_PLUGINS:
                extra.setdefault("resolution", self.resolution)

    @classmethod
    def from_yaml(cls, path: str | Path) -> MapperConfig:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if data is None:
            data = {}
        if not isinstance(data, dict):
            raise ValueError(f"{path}: top level must be a mapping")
        unknown_sections = set(data) - {"map", "sources", "plugins", "sim"}
        if unknown_sections:
            raise ValueError(f"{path}: unknown top-level sections {sorted(unknown_sections)}")

        map_section = data.get("map") or {}
        if not isinstance(map_section, dict):
            raise ValueError(f"{path}: 'map' section must be a mapping")
        allowed = {f.name for f in fields(cls)} - {"sources", "plugins", "sim"}
        unknown_keys = set(map_section) - allowed
        if unknown_keys:
            raise ValueError(f"{path}: unknown 'map' keys {sorted(unknown_keys)}")

        sources = parse_sources(data) if "sources" in data else {}
        plugins = validate_plugin_decl(data.get("plugins") or {})

        cfg = cls(sources=sources, plugins=plugins, sim=data.get("sim") or {}, **map_section)
        return cfg

    @property
    def primary_source(self) -> SensorSourceConfig:
        """首个配置的输入源；一个都没配时返回内置 default（perfect 模型）。"""
        if self.sources:
            return next(iter(self.sources.values()))
        return _DEFAULT_SOURCE

    def resolve_source(self, name: str) -> SensorSourceConfig:
        """按名解析输入源；'default' 回退到主源，未知名抛 KeyError。"""
        if name in self.sources:
            return self.sources[name]
        if name == "default":
            return self.primary_source
        raise KeyError(f"unknown source '{name}', configured: {sorted(self.sources)}")


@dataclass(frozen=True)
class MapStats:
    """地图统计（CLI / 测试用）。"""

    valid_cells: int
    total_cells: int
    coverage: float
    mean_height: float
    min_height: float
    max_height: float
    center: tuple[float, float, float]

    def to_dict(self) -> dict[str, Any]:
        return {
            "valid_cells": self.valid_cells,
            "total_cells": self.total_cells,
            "coverage": self.coverage,
            "mean_height": self.mean_height,
            "min_height": self.min_height,
            "max_height": self.max_height,
            "center": list(self.center),
        }


class ElevationMapper:
    """包装上游 ElevationMap 的 ROS-free 建图器（需要 GPU：模块级 import cupy）。"""

    def __init__(
        self,
        config: MapperConfig | None = None,
        *,
        buffer_size: int = 4096,
        plugin_dir: str | Path | None = None,
    ) -> None:
        self.config = config or MapperConfig()
        register_custom_plugins()  # 自有插件（geo_*）alias 进上游包命名空间
        plugin_path = write_plugin_config(self.config.plugins, directory=plugin_dir)
        param = Parameter(
            resolution=self.config.resolution,
            map_length=self.config.map_length,
            sensor_noise_factor=float(self.config.sensor_noise_factor),
            mahalanobis_thresh=self.config.mahalanobis_thresh,
            initial_variance=self.config.initial_variance,
            max_variance=self.config.max_variance,
            time_variance=self.config.time_variance,
            time_interval=self.config.time_interval,
            min_valid_distance=self.config.min_valid_distance,
            max_height_range=self.config.max_height_range,
            max_ray_length=self.config.max_ray_length,
            enable_visibility_cleanup=self.config.enable_visibility_cleanup,
            enable_drift_compensation=self.config.enable_drift_compensation,
            enable_overlap_clearance=self.config.enable_overlap_clearance,
            weight_file=self.config.weight_file,
            plugin_config_file=str(plugin_path),
            subscriber_cfg={},  # Phase 1 不接语义/RGB 通道
            use_chainer=True,  # 仅在使用权重文件时有意义；Phase 1 恒为空
        )
        param.update()
        self._map = ElevationMap(param)
        self._buffer = PoseBuffer(maxlen=buffer_size)

    # ── 几何/参数只读属性 ────────────────────────────────────────────────
    @property
    def resolution(self) -> float:
        return self._map.resolution

    @property
    def cell_n(self) -> int:
        return self._map.cell_n

    @property
    def center(self) -> np.ndarray:
        """地图中心在世界系的位置（xyz）。"""
        return np.asarray(cp.asnumpy(self._map.center))

    # ── 位姿注入 ──────────────────────────────────────────────────────────
    def add_pose(self, stamp: float, matrix: np.ndarray) -> None:
        """注入一个带时间戳的 T_map_sensor（4x4）。"""
        self._buffer.add(stamp, matrix)

    def pose_at(self, stamp: float) -> np.ndarray:
        """按时间戳插值查询 T_map_sensor；越界抛 StampOutOfRangeError。"""
        return self._buffer.interpolate(stamp)

    # ── 数据输入 ──────────────────────────────────────────────────────────
    def input(self, points: np.ndarray, stamp: float, source: str = "default") -> int:
        """融合一帧传感器系点云（Nx3），返回通过量程裁剪后实际参与融合的点数。

        位姿按 stamp 从缓冲插值；点云须为传感器系坐标（传感器原点为基准）。
        """
        pts = np.asarray(points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError(f"points must have shape (N, 3), got {pts.shape}")
        src = self.config.resolve_source(source)
        pose = self._buffer.interpolate(stamp)  # 先校验位姿可用，再看点云
        if len(pts) > 0:
            dist = np.linalg.norm(pts.astype(np.float64), axis=1)
            keep = (dist >= src.min_range) & (dist <= src.max_range)
            pts = pts[keep]
        if len(pts) == 0:
            return 0
        R = cp.asarray(pose[:3, :3], dtype=np.float32)
        t = cp.asarray(pose[:3, 3], dtype=np.float32)
        cloud = cp.asarray(pts)
        self._map.input_pointcloud(cloud, ["x", "y", "z"], R, t, src.position_noise, src.orientation_noise)
        return int(cloud.shape[0])

    # ── 时间传播 / 平移 ──────────────────────────────────────────────────
    def update_variance(self) -> None:
        """按 time_variance 增大有效 cell 的方差（时间衰减）。"""
        self._map.update_variance()

    def update_time(self) -> None:
        """推进 time layer（步长为配置的 time_interval）。"""
        self._map.update_time()

    def move(self, delta_position: Sequence[float]) -> None:
        """按位移平移地图（机器人中心跟随）。"""
        self._map.move(np.asarray(delta_position, dtype=np.float64))

    def move_to(self, position: Sequence[float], R: np.ndarray | None = None) -> None:
        """把地图中心移动到绝对位置（机器人中心跟随）；R 为新的 base 旋转。"""
        rot = np.eye(3) if R is None else np.asarray(R, dtype=np.float64)
        self._map.move_to(np.asarray(position, dtype=np.float64), rot)

    def clear(self) -> None:
        self._map.clear()

    # ── 图层读取 ──────────────────────────────────────────────────────────
    def available_layers(self) -> list[str]:
        """可读图层：7 个基础层 + 上游内建法线层 + 已配置的插件层。"""
        return self._map.list_layers() + [n for n in _NORMAL_LAYER_NAMES if n not in self._map.list_layers()]

    def get_layer(self, name: str) -> np.ndarray:
        """取单个图层（numpy 副本，float32；高度层无效区域为 NaN）。"""
        if name in _NORMAL_LAYER_NAMES:
            buf = np.zeros((self.cell_n - 2, self.cell_n - 2), dtype=np.float32)
            self._map.get_map_with_name_ref(name, buf)
            return buf.copy()
        if not self._map.exists_layer(name):
            raise KeyError(f"unknown layer '{name}', available: {self.available_layers()}")
        return self._map.export_layers([name])[name]

    def get_map(self, layers: Sequence[str] | None = None) -> dict[str, np.ndarray]:
        """取多图层（缺省为全部可读图层），{name: (cell_n-2)^2 float32}。"""
        names = list(layers) if layers is not None else self.available_layers()
        missing = [n for n in names if n not in _NORMAL_LAYER_NAMES and not self._map.exists_layer(n)]
        if missing:
            raise KeyError(f"unknown layers {missing}, available: {self.available_layers()}")
        out = self._map.export_layers([n for n in names if n not in _NORMAL_LAYER_NAMES])
        for n in names:
            if n in _NORMAL_LAYER_NAMES:
                out[n] = self.get_layer(n)
        return out

    # ── 统计 ──────────────────────────────────────────────────────────────
    def stats(self) -> MapStats:
        """当前地图统计：有效 cell 数 / 覆盖率 / 高度均值与范围。"""
        elevation = self.get_layer("elevation")  # NaN = 无效，已含地图中心 z
        valid = np.isfinite(elevation)
        valid_cells = int(valid.sum())
        total = int(elevation.size)
        if valid_cells:
            heights = elevation[valid]
            mean = float(heights.mean())
            min_h = float(heights.min())
            max_h = float(heights.max())
        else:
            mean = min_h = max_h = float("nan")
        center = tuple(float(v) for v in self.center)
        return MapStats(
            valid_cells=valid_cells,
            total_cells=total,
            coverage=valid_cells / total if total else 0.0,
            mean_height=mean,
            min_height=min_h,
            max_height=max_h,
            center=center,
        )
