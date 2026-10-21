"""插件链验证（GPU）：inpainting 补洞、geo_slope 坡度值、geo_traversability 陡坡判别。

模块级不 import cupy/mapper —— mapper_mod fixture 懒加载；无 GPU 环境由
gpu fixture + skipif 标记整组跳过。
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


pytestmark = pytest.mark.skipif(not _cupy_cuda_available(), reason="cupy/CUDA not available")


@pytest.fixture(scope="module")
def mapper_mod():
    pytest.importorskip("cupy", reason="cupy not installed")
    from pico_elevationmap.core import mapper as mapper_module

    return mapper_module


def _sync() -> None:
    import cupy as cp

    cp.cuda.Stream.null.synchronize()


def _cam() -> dict[str, SensorSourceConfig]:
    return {"cam": SensorSourceConfig(name="cam", sensor_type="perfect")}


def _world_grid(half: float, spacing: float, z: np.ndarray) -> np.ndarray:
    xs = np.arange(-half, half, spacing)
    X, Y = np.meshgrid(xs, xs)
    return np.stack([X.ravel(), Y.ravel(), np.broadcast_to(z, X.shape).ravel()], axis=1)


def _down_pose(height: float = 1.0) -> np.ndarray:
    T = np.eye(4)
    T[2, 3] = height
    return T


def _row_xs(n: int, length: float, res: float) -> np.ndarray:
    """导出数组 row → 世界 x（center=0：x = L/2 - (row + 1)·res，与 io 导出约定一致）。"""
    return length / 2.0 - (np.arange(n) + 1.0) * res


def test_inpainting_reduces_nans(gpu, mapper_mod, tmp_path) -> None:
    cfg = mapper_mod.MapperConfig(
        resolution=0.05,
        map_length=6.0,
        sources=_cam(),
        plugins={
            "inpainting": {
                "enable": True,
                "layer_name": "inpainted",
                "fill_nan": False,
                "is_height_layer": True,
                "extra_params": {"method": "telea"},
            }
        },
    )
    mapper = mapper_mod.ElevationMapper(cfg, plugin_dir=tmp_path)
    mapper.add_pose(0.0, _down_pose())
    # 只覆盖中心区域 → 外圈是洞
    world = _world_grid(1.2, 0.04, np.array(0.0)) - np.array([0.0, 0.0, 1.0])
    mapper.input(world, stamp=0.0, source="cam")
    _sync()

    elev = mapper.get_layer("elevation")
    inpainted = mapper.get_layer("inpainted")
    nan_before = int(np.isnan(elev).sum())
    nan_after = int(np.isnan(inpainted).sum())
    assert nan_before > 0
    assert nan_after == 0  # inpainting 的职责：把洞全部补上
    # 已有效 cell 保持原值
    valid = np.isfinite(elev)
    np.testing.assert_allclose(inpainted[valid], elev[valid], atol=1e-6)


def _ramp_world(half: float, spacing: float) -> np.ndarray:
    """x > 1 处坡度 0.5 的斜坡（rise/run），其余平地。"""
    xs = np.arange(-half, half, spacing)
    X, Y = np.meshgrid(xs, xs)
    Z = np.where(X > 1.0, 0.5 * np.clip(X - 1.0, 0.0, 1.0), 0.0)
    return np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)


def test_slope_and_traversability_on_ramp(gpu, mapper_mod, tmp_path) -> None:
    cfg = mapper_mod.MapperConfig(
        resolution=0.05,
        map_length=6.0,
        sources=_cam(),
        plugins={
            "geo_slope": {
                "enable": True,
                "layer_name": "slope",
                "fill_nan": False,
                "is_height_layer": False,
                "extra_params": {"input_layer_name": "elevation"},
            },
            "geo_traversability": {
                "enable": True,
                "layer_name": "traversability_geo",
                "fill_nan": False,
                "is_height_layer": False,
                "extra_params": {"input_layer_name": "elevation", "slope_thresh": 0.3, "roughness_thresh": 0.05},
            },
        },
    )
    mapper = mapper_mod.ElevationMapper(cfg, plugin_dir=tmp_path)
    mapper.add_pose(0.0, _down_pose())
    # 采样间距取 cell 的整数分之一（0.025 = res/2），避免点云栅格与地图栅格
    # 混叠导致斜坡面出现锯齿（中心差分对线性面精确）
    mapper.input(_ramp_world(2.4, 0.025) - np.array([0.0, 0.0, 1.0]), stamp=0.0, source="cam")
    _sync()

    layers = mapper.get_map()
    assert {"slope", "traversability_geo"} <= set(layers)
    slope, trav = layers["slope"], layers["traversability_geo"]
    xs = _row_xs(slope.shape[0], cfg.map_length, cfg.resolution)

    # 斜坡区坡度 ≈ 0.5（x∈(1,2) 为斜面，x>2 是 clip 出的平台，窗口取斜面内部；
    # NaN 自动排除边界）
    ramp = (xs > 1.2) & (xs < 1.9)
    ramp_slope = slope[ramp, :].ravel()
    ramp_slope = ramp_slope[np.isfinite(ramp_slope)]
    assert len(ramp_slope) > 0
    assert float(ramp_slope.mean()) == pytest.approx(0.5, abs=0.08)

    # 可通行性：斜坡区绝大多数判 0，平地区绝大多数判 1
    ramp_trav = trav[ramp, :].ravel()
    ramp_trav = ramp_trav[np.isfinite(ramp_trav)]
    assert (ramp_trav < 0.5).mean() > 0.8
    flat = (xs > -2.0) & (xs < 0.0)
    flat_trav = trav[flat, :].ravel()
    flat_trav = flat_trav[np.isfinite(flat_trav)]
    assert len(flat_trav) > 0
    assert (flat_trav > 0.5).mean() > 0.8
