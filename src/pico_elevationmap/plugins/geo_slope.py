"""几何坡度插件（Phase 2，自有插件）：由高程梯度生成 slope 层。

输出为 rise/run（即 tan(slope_angle)），无效/邻接无效 cell 为 NaN。
上游 traversability 管线学习式，坡度层此前没有纯几何来源，本插件补上。

extra_params（YAML plugins.<name>.extra_params）：
    input_layer_name: 输入高程层（默认 "elevation"，常接 min_filter / smooth）
    resolution: 地图分辨率（米/cell），与 map.resolution 保持一致
"""

from __future__ import annotations

import cupy as cp
from elevation_mapping_cupy.plugins.plugin_manager import PluginBase

from pico_elevationmap.plugins.geo_common import gradient, pick_layer, valid_height


class GeometricSlope(PluginBase):
    """坡度层插件：slope = ||∇h||（rise/run）。"""

    def __init__(self, cell_n: int = 100, input_layer_name: str = "elevation", resolution: float = 0.04, **kwargs):
        super().__init__()
        self.input_layer_name = input_layer_name
        self.resolution = float(resolution)

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
        return cp.where(valid, slope, cp.nan).astype(cp.float64)
