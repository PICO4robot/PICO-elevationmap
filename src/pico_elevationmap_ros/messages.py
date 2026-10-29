"""ROS ↔ numpy 消息转换层（Phase 4 桥接包）：零 ROS 依赖，鸭子类型解包。

本模块不 import 任何 rclpy / *_msgs —— 真实消息类型由 node.py 在 ROS 环境内
提供，测试用 SimpleNamespace / 桩消息按属性约定构造。栅格打包为自有最小
GridMessage 数据类（grid_map_msgs/GridMap 的形状等价物，见 dataclass 注释），
node.py 的 _to_ros_gridmap() 负责一处翻译成真消息（仅真实 ROS 环境调用）。

坐标约定与 core.io 导出层一致（Phase 2 定案）：层数组 [row, col] 中
row ↔ x = cx + (n/2 - 1 - row)·res（从 cx + L/2 - res 递减），
col ↔ y 同理；GridMessage.position 为地图中心。
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

__all__ = [
    "GridMessage",
    "layers_to_grid_message",
    "livox_custom_msg_to_array",
    "pointcloud2_to_array",
    "pose_to_se3",
]

# PointField datatype 常量（sensor_msgs/PointField）
_POINTFIELD_FMT = {7: "<f4", 8: "<f8"}


@dataclass
class GridMessage:
    """grid_map_msgs/GridMap 形状的最小数据类（桥内自有，便于 mock 断言）。

    data 与 layers 对齐：每层 (n, n) float32 的 numpy 视图，采用上述
    core.io 行列约定；翻译到真 grid_map_msgs 时的转置/展平由 node 负责。
    """

    layers: list[str]
    basic_layers: list[str]
    resolution: float
    length_x: float
    length_y: float
    position: tuple[float, float, float]  # 地图中心（世界系）
    orientation_wxyz: tuple[float, float, float, float]  # 恒为单位四元数（轴对齐）
    data: list[np.ndarray]
    stamp: float
    frame_id: str


def _stamp_to_float(stamp: Any) -> float:
    """header.stamp → 秒。接受 builtin Time（sec/nanosec）或裸 float。"""
    if hasattr(stamp, "sec"):
        return float(stamp.sec) + float(getattr(stamp, "nanosec", 0.0)) * 1e-9
    return float(stamp)


def pointcloud2_to_array(msg: Any) -> tuple[np.ndarray, float, str]:
    """PointCloud2 → ((N,3) float32, stamp 秒, frame_id)。

    只取 x/y/z（rgb/intensity 等附加字段存在时忽略，核心融合当前不接语义通道）；
    按 xyz 有限性跳过 NaN/Inf 点。
    """
    fields = {f.name: f for f in msg.fields}
    missing = {"x", "y", "z"} - set(fields)
    if missing:
        raise ValueError(f"PointCloud2 missing fields {sorted(missing)}")
    fmts: dict[str, str] = {}
    for name in ("x", "y", "z"):
        datatype = int(fields[name].datatype)
        if datatype not in _POINTFIELD_FMT:
            raise ValueError(f"PointCloud2 field '{name}' unsupported datatype {datatype} (need float32/64)")
        fmts[name] = _POINTFIELD_FMT[datatype]
    dt = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [fmts["x"], fmts["y"], fmts["z"]],
            "offsets": [int(fields["x"].offset), int(fields["y"].offset), int(fields["z"].offset)],
            "itemsize": int(msg.point_step),
        }
    )
    data = msg.data
    buf = data.tobytes() if isinstance(data, np.ndarray) else bytes(data)
    arr = np.frombuffer(buf, dtype=dt, count=int(msg.width) * int(msg.height))
    pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float32)
    finite = np.isfinite(pts).all(axis=1)
    return pts[finite], _stamp_to_float(msg.header.stamp), str(msg.header.frame_id)


def livox_custom_msg_to_array(msg: Any) -> tuple[np.ndarray, float, str]:
    """Livox CustomMsg → ((N,3) float32, stamp 秒, frame_id)。

    ``livox_ros_driver2/msg/CustomMsg`` 的 ``points`` 通常是变长 CustomPoint 数组；
    dtype 化后按字段取 x/y/z，普通对象列表也兼容。附加字段 offset_time /
    reflectivity / tag / line 当前不进入核心融合。
    """
    points = msg.points
    if isinstance(points, np.ndarray) and points.dtype.names:
        missing = {"x", "y", "z"} - set(points.dtype.names)
        if missing:
            raise ValueError(f"Livox CustomMsg points missing fields {sorted(missing)}")
        xyz = np.stack([points["x"], points["y"], points["z"]], axis=1)
    else:
        xyz = np.array([[p.x, p.y, p.z] for p in points], dtype=np.float64).reshape(-1, 3)
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    finite = np.isfinite(xyz).all(axis=1)
    if hasattr(msg, "header"):
        stamp = _stamp_to_float(msg.header.stamp)
        frame_id = str(msg.header.frame_id)
    else:
        stamp = float(getattr(msg, "timebase", 0)) * 1e-9
        frame_id = "livox_frame"
    return xyz[finite], stamp, frame_id


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    n = x * x + y * y + z * z + w * w
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


def pose_to_se3(msg: Any) -> tuple[np.ndarray, float, float, float]:
    """Odometry / PoseWithCovarianceStamped → (T_map_sensor 4x4, stamp, pos_noise, ori_noise)。

    两类消息结构相同：msg.pose 为 PoseWithCovariance（.pose + .covariance），
    因此按鸭子类型统一解包。噪声标量取协方差对角元的均方根：
    pos_noise = sqrt(mean(diag(cov)[:3]))（米），ori_noise 同理（弧度），
    作为上游漂移补偿的触发门限传入 mapper.input。
    """
    pwc = msg.pose
    pose = pwc.pose
    cov = np.asarray(pwc.covariance, dtype=np.float64).reshape(6, 6)
    q = pose.orientation
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _quat_to_matrix(float(q.x), float(q.y), float(q.z), float(q.w))
    T[:3, 3] = [float(pose.position.x), float(pose.position.y), float(pose.position.z)]
    pos_noise = float(np.sqrt(np.mean([cov[0, 0], cov[1, 1], cov[2, 2]])))
    ori_noise = float(np.sqrt(np.mean([cov[3, 3], cov[4, 4], cov[5, 5]])))
    return T, _stamp_to_float(msg.header.stamp), pos_noise, ori_noise


def layers_to_grid_message(
    layers: dict[str, np.ndarray],
    resolution: float,
    center: Sequence[float],
    stamp: float,
    frame_id: str,
    basic_layers: Sequence[str] = ("elevation",),
) -> GridMessage:
    """图层字典 → GridMessage（位置/尺寸按 Phase 2 坐标约定换算）。"""
    if not layers:
        raise ValueError("no layers to pack")
    names = list(layers)
    arrays = [np.asarray(layers[n], dtype=np.float32) for n in names]
    shape = arrays[0].shape
    if len(shape) != 2 or shape[0] != shape[1]:
        raise ValueError(f"layers must be square 2D, got {shape}")
    for n, arr in zip(names, arrays):
        if arr.shape != shape:
            raise ValueError(f"layer '{n}' shape {arr.shape} != {shape}")
    n = shape[0]
    missing_basic = [b for b in basic_layers if b not in layers]
    if missing_basic:
        raise ValueError(f"basic_layers {missing_basic} not in {names}")
    return GridMessage(
        layers=names,
        basic_layers=list(basic_layers),
        resolution=float(resolution),
        length_x=n * float(resolution),
        length_y=n * float(resolution),
        position=(float(center[0]), float(center[1]), float(center[2])),
        orientation_wxyz=(1.0, 0.0, 0.0, 0.0),
        data=arrays,
        stamp=float(stamp),
        frame_id=frame_id,
    )
