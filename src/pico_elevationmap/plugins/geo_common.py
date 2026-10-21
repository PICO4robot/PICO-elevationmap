"""几何插件共享 helper（Phase 2）：坡度/粗糙度计算的公共数值例程。

本模块不含 PluginBase 子类（上游 PluginManager 会把模块内所有 PluginBase
子类都实例化，因此每个插件必须独占一个模块）。
"""

from __future__ import annotations

import cupy as cp


def valid_height(elevation_map: cp.ndarray, layer_names: list[str], h: cp.ndarray) -> cp.ndarray:
    """is_valid > 0.5 且高程有限的 cell mask。"""
    is_valid = elevation_map[layer_names.index("is_valid")]
    return (is_valid > 0.5) & cp.isfinite(h)


def pick_layer(
    name: str,
    elevation_map: cp.ndarray,
    layer_names: list[str],
    plugin_layers: cp.ndarray,
    plugin_layer_names: list[str],
) -> cp.ndarray:
    """从基础层 / 插件层中选输入层；找不到时回退 elevation 并打印提示（上游惯例）。"""
    if name in layer_names:
        return elevation_map[layer_names.index(name)]
    if name in plugin_layer_names:
        return plugin_layers[plugin_layer_names.index(name)]
    print(f"geo plugin: layer '{name}' not found, falling back to 'elevation'")
    return elevation_map[layer_names.index("elevation")]


def gradient(h: cp.ndarray, resolution: float) -> tuple[cp.ndarray, cp.ndarray]:
    """NaN 传播的中心差分梯度；边界 one-sided，避免 roll 绕回。"""
    h = cp.asarray(h, dtype=cp.float64)
    gx = cp.full(h.shape, cp.nan)
    gy = cp.full(h.shape, cp.nan)
    gx[1:-1, :] = (h[2:, :] - h[:-2, :]) / (2.0 * resolution)
    gy[:, 1:-1] = (h[:, 2:] - h[:, :-2]) / (2.0 * resolution)
    gx[0, :] = (h[1, :] - h[0, :]) / resolution
    gx[-1, :] = (h[-1, :] - h[-2, :]) / resolution
    gy[:, 0] = (h[:, 1] - h[:, 0]) / resolution
    gy[:, -1] = (h[:, -1] - h[:, -2]) / resolution
    return gx, gy


def local_roughness(h: cp.ndarray, valid: cp.ndarray, window: int) -> cp.ndarray:
    """box 窗口内高程标准差（NaN 安全；窗口内无有效 cell 返回 NaN）。"""
    import cupyx.scipy.ndimage as ndimage

    v = valid.astype(cp.float64)
    cnt = ndimage.uniform_filter(v, size=window, mode="nearest")
    s = ndimage.uniform_filter(cp.where(valid, h, 0.0), size=window, mode="nearest")
    mean = s / cp.maximum(cnt, 1e-9)
    d2 = ndimage.uniform_filter(cp.where(valid, (h - mean) ** 2, 0.0), size=window, mode="nearest")
    rough = cp.sqrt(d2 / cp.maximum(cnt, 1e-9))
    return cp.where(cnt > 0.0, rough, cp.nan)
