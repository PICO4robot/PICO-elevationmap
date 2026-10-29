"""Map layer rendering primitives: fixed colour bands, hillshade and colour bar."""

from __future__ import annotations

import numpy as np

__all__ = ["render_layer", "render_legend"]


def render_layer(
    layer: np.ndarray,
    *,
    colormap: str = "turbo",
    z_min: float | None = None,
    z_max: float | None = None,
    bands: int = 12,
    resolution: float = 0.03,
    shade: bool = True,
) -> np.ndarray:
    """把地图图层渲染成 BGR 图（NaN 为深灰）。

    ``z_min`` / ``z_max`` 给定时使用绝对高度范围（推荐，颜色跨帧稳定）；
    否则回退到当前帧 2–98 百分位。``bands`` > 1 时做离散分层并画层间线，
    ``shade`` 用地形坡度做明暗，突出高度起伏。
    """
    import cv2

    valid = np.isfinite(layer)
    if z_min is None or z_max is None:
        if valid.any():
            lo, hi = np.percentile(layer[valid], [2.0, 98.0])
        else:
            lo, hi = 0.0, 1.0
    else:
        lo, hi = float(z_min), float(z_max)
    if hi <= lo:
        hi = lo + 1e-6
    filled = np.where(valid, layer, lo)
    norm = np.clip((filled - lo) / (hi - lo), 0.0, 1.0)
    band_idx: np.ndarray | None = None
    if bands > 1:
        band_idx = np.minimum((norm * bands).astype(np.int32), bands - 1)
        norm = (band_idx.astype(np.float32) + 0.5) / bands
    cmap = cv2.COLORMAP_TURBO if colormap == "turbo" else cv2.COLORMAP_VIRIDIS
    img = cv2.applyColorMap((norm * 255.0).astype(np.uint8), cmap)
    if shade:
        gy, gx = np.gradient(filled)
        slope = np.hypot(gx, gy) / max(float(resolution), 1e-6)
        factor = 0.65 + 0.35 / (1.0 + 2.0 * slope)
        img = np.clip(img.astype(np.float32) * factor[..., None], 0, 255).astype(np.uint8)
    if band_idx is not None:
        edges = np.zeros_like(valid)
        edges[1:, :] |= valid[1:, :] & valid[:-1, :] & (band_idx[1:, :] != band_idx[:-1, :])
        edges[:, 1:] |= valid[:, 1:] & valid[:, :-1] & (band_idx[:, 1:] != band_idx[:, :-1])
        img[edges] = (20, 20, 20)
    img[~valid] = (25, 25, 25)
    return img


def render_legend(z_min: float, z_max: float, *, height: int = 267, width: int = 90, bands: int = 12) -> np.ndarray:
    """固定高度范围的颜色条，带米制刻度。"""
    import cv2

    grad = np.linspace(1.0, 0.0, height, dtype=np.float32)[:, None]
    if bands > 1:
        idx = np.minimum((grad * bands).astype(np.int32), bands - 1)
        grad = (idx.astype(np.float32) + 0.5) / bands
    strip = cv2.applyColorMap((grad * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
    strip = np.repeat(strip, width, axis=1)
    panel = np.full((height, width + 66, 3), 25, dtype=np.uint8)
    panel[:, :width] = strip
    span = float(z_max - z_min)
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        y = 26 + int(frac * (height - 34))
        value = z_max - frac * span
        cv2.putText(
            panel,
            f"{value:.2f}",
            (width + 5, y + 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (240, 240, 240),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(panel, "m", (width + 5, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (240, 240, 240), 1, cv2.LINE_AA)
    return panel
