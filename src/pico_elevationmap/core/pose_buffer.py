"""带时间戳的 SE3 位姿缓冲（Phase 1）：替代 ROS tf 订阅。

维护按时间单调不减的 (stamp, T_map_sensor) 序列；interpolate(stamp) 对平移做
线性插值、对旋转做 slerp（纯 numpy 实现，不引入 scipy）。点云时间戳与位姿
时间戳不一致时由调用方插值对齐；超出缓冲覆盖范围抛出 StampOutOfRangeError，
由调用方决定丢帧或改用最近位姿。
"""

from __future__ import annotations

import bisect

import numpy as np

__all__ = ["PoseBuffer", "StampOutOfRangeError"]


class StampOutOfRangeError(ValueError):
    """请求的 stamp 早于/晚于位姿缓冲的覆盖范围。"""


def _matrix_to_quat(R: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵 → (w, x, y, z) 单位四元数。"""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = np.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = np.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = np.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return q / np.linalg.norm(q)


def _quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """(w, x, y, z) 单位四元数 → 3x3 旋转矩阵。"""
    w, x, y, z = q
    n = w * w + x * x + y * y + z * z
    s = 2.0 / n if n > 0.0 else 0.0
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def _slerp(q0: np.ndarray, q1: np.ndarray, u: float) -> np.ndarray:
    """单位四元数球面插值（u ∈ [0, 1]），始终走短弧。"""
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1 = -q1
        d = -d
    if d > 0.9995:
        # 夹角过小：线性插值后归一化即足够稳定
        q = q0 + u * (q1 - q0)
        return q / np.linalg.norm(q)
    theta = float(np.arccos(np.clip(d, -1.0, 1.0)))
    q = (np.sin((1.0 - u) * theta) * q0 + np.sin(u * theta) * q1) / np.sin(theta)
    return q / np.linalg.norm(q)


class PoseBuffer:
    """有界 (stamp, T_map_sensor) 序列，按时间戳插值查询。"""

    def __init__(self, maxlen: int = 4096) -> None:
        if maxlen < 2:
            raise ValueError(f"maxlen must be >= 2, got {maxlen}")
        self._maxlen = maxlen
        self._stamps: list[float] = []
        self._matrices: list[np.ndarray] = []

    def __len__(self) -> int:
        return len(self._stamps)

    @property
    def maxlen(self) -> int:
        return self._maxlen

    @property
    def coverage(self) -> tuple[float, float] | None:
        """(最早, 最晚) stamp；空缓冲返回 None。"""
        if not self._stamps:
            return None
        return (self._stamps[0], self._stamps[-1])

    def add(self, stamp: float, matrix: np.ndarray) -> None:
        """插入一个位姿。stamp 必须单调不减；同 stamp 覆盖旧值。"""
        T = np.asarray(matrix, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError(f"expected a 4x4 SE3 matrix, got shape {T.shape}")
        if self._stamps and stamp < self._stamps[-1]:
            raise ValueError(f"stamp {stamp} out of order (latest buffered stamp is {self._stamps[-1]})")
        if self._stamps and stamp == self._stamps[-1]:
            self._matrices[-1] = T.copy()
            return
        if len(self._stamps) == self._maxlen:
            del self._stamps[0]
            del self._matrices[0]
        self._stamps.append(float(stamp))
        self._matrices.append(T.copy())

    def latest(self) -> np.ndarray:
        """最新位姿；空缓冲抛 StampOutOfRangeError。"""
        if not self._stamps:
            raise StampOutOfRangeError("pose buffer is empty")
        return self._matrices[-1].copy()

    def interpolate(self, stamp: float) -> np.ndarray:
        """查询 stamp 处的 T_map_sensor：平移线性插值 + 旋转 slerp。"""
        n = len(self._stamps)
        if n == 0:
            raise StampOutOfRangeError("pose buffer is empty")
        first, last = self._stamps[0], self._stamps[-1]
        if stamp < first:
            raise StampOutOfRangeError(
                f"stamp {stamp} is {first - stamp:.6f}s before the oldest buffered pose ({first})"
            )
        if stamp > last:
            raise StampOutOfRangeError(f"stamp {stamp} is {stamp - last:.6f}s after the newest buffered pose ({last})")

        i = bisect.bisect_left(self._stamps, stamp)
        if i < n and self._stamps[i] == stamp:
            return self._matrices[i].copy()
        # self._stamps[i - 1] < stamp < self._stamps[i]
        t0, t1 = self._stamps[i - 1], self._stamps[i]
        u = (stamp - t0) / (t1 - t0)
        T0, T1 = self._matrices[i - 1], self._matrices[i]
        q = _slerp(_matrix_to_quat(T0[:3, :3]), _matrix_to_quat(T1[:3, :3]), u)
        out = np.eye(4, dtype=np.float64)
        out[:3, :3] = _quat_to_matrix(q)
        out[:3, 3] = (1.0 - u) * T0[:3, 3] + u * T1[:3, 3]
        return out

    def clear(self) -> None:
        self._stamps.clear()
        self._matrices.clear()
