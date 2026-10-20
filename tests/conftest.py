"""共享 pytest fixtures：合成数据 + GPU 可用性自动 skip。

家族约定：测试不依赖外部资产，点云 / 地图数据全部现场合成（确定性 seed）。
GPU 用例使用 ``gpu`` fixture 或 ``requires_gpu`` 标记，cupy / CUDA 不可用时
自动 skip（Phase 3 后端接入后生效；Phase 0 纯 CPU，无 skip）。
"""

from __future__ import annotations

import random

import pytest


def _cupy_cuda_available() -> bool:
    """True only when cupy is importable AND a CUDA device is present."""
    try:
        import cupy as cp
    except ImportError:
        return False
    try:
        return bool(cp.cuda.is_available())
    except Exception:
        return False


requires_gpu = pytest.mark.skipif(
    not _cupy_cuda_available(),
    reason="cupy/CUDA not available",
)


@pytest.fixture(scope="session")
def gpu() -> object:
    """CUDA-enabled cupy module — skips the test when unavailable (Phase 3)."""
    cp = pytest.importorskip("cupy", reason="cupy not installed")
    if not cp.cuda.is_available():
        pytest.skip("CUDA device not available")
    return cp


@pytest.fixture()
def synthetic_pointcloud() -> list[tuple[float, float, float]]:
    """Deterministic flat-ground point cloud: 2 m x 2 m at z=0, (x, y, z) tuples."""
    rng = random.Random(42)
    return [(rng.uniform(-1.0, 1.0), rng.uniform(-1.0, 1.0), 0.0) for _ in range(200)]
