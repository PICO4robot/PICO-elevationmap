"""共享 pytest fixtures：合成数据 + GPU/ROS 可用性自动 skip。

家族约定：测试不依赖外部资产，点云 / 地图数据全部现场合成（确定性 seed）。
GPU 用例使用 ``gpu`` fixture 或 ``requires_gpu`` skipif 条件，cupy / CUDA 不可用时
自动 skip（Phase 1/2 的 GPU 用例在本机 4090 上实际运行）。
ROS 用例使用 ``requires_ros`` 标记，真实 ROS2 环境（rclpy 可导入 + ros2 CLI）
不可用时自动 skip（Phase 4 桥接集成测试用）；桥接逻辑本身由 sys.modules
桩单测，不依赖真实 ROS。
"""

from __future__ import annotations

import importlib.util
import random
import shutil

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


def _ros2_available() -> bool:
    """True only when rclpy 可导入且 ros2 CLI 存在（真实 ROS2 环境）。"""
    return importlib.util.find_spec("rclpy") is not None and shutil.which("ros2") is not None


requires_gpu = pytest.mark.skipif(
    not _cupy_cuda_available(),
    reason="cupy/CUDA not available",
)


def pytest_collection_modifyitems(items: list[pytest.Item]) -> None:
    """requires_ros 标记的用例：无真实 ROS2 时整项跳过（桥接逻辑由桩单测覆盖）。"""
    if _ros2_available():
        return
    skip = pytest.mark.skip(reason="ROS2 not available")
    for item in items:
        if "requires_ros" in item.keywords:
            item.add_marker(skip)


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
