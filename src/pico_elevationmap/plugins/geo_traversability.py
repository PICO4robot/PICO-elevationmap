"""几何可通行性插件（Phase 2，自有插件）：坡度 + 粗糙度阈值化 traversability 层。

上游 traversability 是学习式滤波（需 weights.dat，本仓库不携带权重文件），
按设计文档 §2.2 的几何可通行性思路实现纯几何版本，输出 1.0（可通行）/
0.0（不可通行），无效 cell 为 NaN。

extra_params（YAML plugins.<name>.extra_params）：
    input_layer_name: 输入高程层（默认 "elevation"，常接 smooth）
    resolution: 地图分辨率（米/cell）
    slope_thresh: rise/run 门限（默认 0.5 ≈ 26.6°）
    roughness_thresh: 局部粗糙度门限（米，默认 0.05）
    roughness_window: 粗糙度统计窗口（cell 数，奇数，默认 5）
"""

from __future__ import annotations

import cupy as cp
from elevation_mapping_cupy.plugins.plugin_manager import PluginBase

from pico_elevationmap.plugins.geo_common import gradient, local_roughness, pick_layer, valid_height


class GeometricTraversability(PluginBase):
    """几何可通行性：slope ≤ 门限 且 roughness ≤ 门限 且有效 → 1，否则 0。"""

    def __init__(
        self,
        cell_n: int = 100,
        input_layer_name: str = "elevation",
        resolution: float = 0.04,
        slope_thresh: float = 0.5,
        roughness_thresh: float = 0.05,
        roughness_window: int = 5,
        **kwargs,
    ):
        super().__init__()
        self.input_layer_name = input_layer_name
        self.resolution = float(resolution)
        self.slope_thresh = float(slope_thresh)
        self.roughness_thresh = float(roughness_thresh)
        self.roughness_window = int(roughness_window)

    def __call__(
        self,
        elevation_map: cp.ndarray,
        layer_names: list[str],
        plugin_layers: cp.ndarray,
        plugin_layer_names: list[str],
        *args,
    ) -> cp.ndarray:
        h = pick_layer(self.input_layer_name, elevation_map, layer_names, plugin_layers, plugin_layer_names)
        valid = valid_height(elevation_map, layer_names, h)
        h_f = cp.where(valid, h, cp.nan)
        gx, gy = gradient(h_f, self.resolution)
        slope = cp.sqrt(gx * gx + gy * gy)
        rough = local_roughness(h, valid, self.roughness_window)
        traversable = (slope <= self.slope_thresh) & (rough <= self.roughness_thresh) & valid
        out = cp.where(valid, traversable.astype(cp.float64), cp.nan)
        return out
