"""PoseBuffer 插值正确性：中点 SE3、单调性约束、越界报错 —— 纯 CPU，无 GPU 依赖。"""

from __future__ import annotations

import numpy as np
import pytest

from pico_elevationmap.core.pose_buffer import PoseBuffer, StampOutOfRangeError


def _rz(deg: float) -> np.ndarray:
    """绕 z 轴旋转矩阵（度）。"""
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _se3(R: np.ndarray, t: tuple[float, float, float]) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def test_exact_stamp_returns_stored_pose() -> None:
    buf = PoseBuffer()
    T = _se3(_rz(30.0), (1.0, 2.0, 0.5))
    buf.add(1.0, T)
    got = buf.interpolate(1.0)
    np.testing.assert_allclose(got, T, atol=1e-12)
    # 返回副本，外部修改不影响缓冲
    got[0, 3] = 99.0
    np.testing.assert_allclose(buf.interpolate(1.0), T, atol=1e-12)


def test_midpoint_translation_and_slerp() -> None:
    buf = PoseBuffer()
    buf.add(0.0, _se3(_rz(0.0), (0.0, 0.0, 1.0)))
    buf.add(2.0, _se3(_rz(180.0), (2.0, 0.0, 1.0)))
    mid = buf.interpolate(1.0)
    np.testing.assert_allclose(mid[:3, 3], [1.0, 0.0, 1.0], atol=1e-12)  # 平移线性
    np.testing.assert_allclose(mid[:3, :3], _rz(90.0), atol=1e-10)  # 旋转 slerp 半程


def test_slerp_takes_short_arc() -> None:
    """+170° → -170° 的中点应是 180°（而非 0° 长弧）。"""
    buf = PoseBuffer()
    buf.add(0.0, _se3(_rz(170.0), (0.0, 0.0, 0.0)))
    buf.add(1.0, _se3(_rz(-170.0), (0.0, 0.0, 0.0)))
    mid = buf.interpolate(0.5)
    np.testing.assert_allclose(mid[:3, :3], _rz(180.0), atol=1e-10)


def test_fractional_translation() -> None:
    buf = PoseBuffer()
    buf.add(0.0, _se3(np.eye(3), (0.0, 0.0, 0.0)))
    buf.add(4.0, _se3(np.eye(3), (4.0, 8.0, -4.0)))
    got = buf.interpolate(3.0)  # u = 0.75
    np.testing.assert_allclose(got[:3, 3], [3.0, 6.0, -3.0], atol=1e-12)
    np.testing.assert_allclose(got[:3, :3], np.eye(3), atol=1e-12)


def test_out_of_range_raises() -> None:
    buf = PoseBuffer()
    buf.add(1.0, np.eye(4))
    buf.add(2.0, np.eye(4))
    with pytest.raises(StampOutOfRangeError):
        buf.interpolate(0.9)
    with pytest.raises(StampOutOfRangeError):
        buf.interpolate(2.1)
    with pytest.raises(ValueError, match="empty"):
        PoseBuffer().interpolate(0.0)


def test_out_of_order_and_duplicate_stamps() -> None:
    buf = PoseBuffer()
    buf.add(1.0, _se3(np.eye(3), (0.0, 0.0, 0.0)))
    with pytest.raises(ValueError, match="out of order"):
        buf.add(0.5, np.eye(4))
    # 同 stamp 覆盖旧值
    buf.add(1.0, _se3(np.eye(3), (9.0, 0.0, 0.0)))
    np.testing.assert_allclose(buf.interpolate(1.0)[:3, 3], [9.0, 0.0, 0.0], atol=1e-12)
    assert len(buf) == 1


def test_bounded_eviction() -> None:
    buf = PoseBuffer(maxlen=4)
    for i in range(6):
        buf.add(float(i), np.eye(4))
    assert len(buf) == 4
    assert buf.coverage == (2.0, 5.0)
    with pytest.raises(StampOutOfRangeError):
        buf.interpolate(1.0)  # 已被逐出


def test_bad_matrix_shape() -> None:
    buf = PoseBuffer()
    with pytest.raises(ValueError, match="4x4"):
        buf.add(0.0, np.eye(3))


def test_coverage_and_latest() -> None:
    buf = PoseBuffer()
    assert buf.coverage is None
    with pytest.raises(StampOutOfRangeError):
        buf.latest()
    T = _se3(np.eye(3), (1.0, 1.0, 1.0))
    buf.add(0.0, np.eye(4))
    buf.add(0.5, T)
    assert buf.coverage == (0.0, 0.5)
    np.testing.assert_allclose(buf.latest(), T, atol=1e-12)
