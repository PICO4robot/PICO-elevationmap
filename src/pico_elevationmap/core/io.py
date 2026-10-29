"""离线数据 I/O（Phase 2）：点云帧序列、地图 npz、rosbag 读取、PLY/PGM/CSV 写出。

纯 CPU 模块（numpy；rosbags 懒加载），供 run / replay / export 共用。

帧序列 npz（自有格式，``run --save-frames`` 产出、``replay`` 消费）::

    points  (M,3) float32   全部帧传感器系点云拼接
    offsets (F+1,) int64    每帧在 points 中的起止下标
    stamps  (F,) float64    每帧时间戳（秒）
    poses   (F,4,4) float64 T_map_sensor；全 NaN 表示该帧无位姿
    sources (F,) str        输入源名（"default" = 用配置的默认源）

地图 npz（``run`` / ``replay`` 的 ``--save`` 产出、``export`` 消费）：
{layer_name: (n,n) float32} + 元数据 ``resolution`` / ``center``。

rosbag：rosbags 纯 python 包读取 ROS1 .bag / ROS2 .db3/.mcap，无需安装 ROS。
点云取 ``sensor_msgs/msg/PointCloud2``（x/y/z float 字段）；位姿取最新的
``geometry_msgs/msg/PoseStamped``（stamp ≤ 点云 stamp），缺省 None。

PLY/PGM/CSV：零图像依赖的导出写出器。导出网格坐标约定：导出数组
[row, col] 中 row 对应 x（从 +L/2 递减）、col 对应 y（从 +L/2 递减），
与上游 export_layers 的翻转约定一致。
"""

from __future__ import annotations

import bisect
from collections.abc import Iterator, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

__all__ = [
    "PointCloudFrame",
    "frame_files_from_dir",
    "iter_bag_frames",
    "load_frame_file",
    "load_frames",
    "load_map",
    "save_frames",
    "save_map",
    "write_csv",
    "write_pgm",
    "write_ply",
]

# rosbags PointField datatype 常量（sensor_msgs/PointField）
_POINTFIELD_FMT = {7: "<f4", 8: "<f8"}


@dataclass
class PointCloudFrame:
    """一帧传感器系点云 + 时间戳 + 可选位姿。"""

    points: np.ndarray  # (N,3) float32，传感器系
    stamp: float  # 秒
    pose: np.ndarray | None = None  # (4,4) T_map_sensor；None = 未知
    source: str = "default"

    def __post_init__(self) -> None:
        self.points = np.asarray(self.points, dtype=np.float32).reshape(-1, 3)
        if self.pose is not None:
            self.pose = np.asarray(self.pose, dtype=np.float64)
            if self.pose.shape != (4, 4):
                raise ValueError(f"pose must be 4x4, got {self.pose.shape}")


# ── 帧序列 npz ────────────────────────────────────────────────────────────────


def _ensure_parent(path: str | Path) -> Path:
    """确保输出文件的父目录存在（``maps/`` 等目录不再需要调用方预建）。"""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    return target


def save_frames(path: str | Path, frames: Sequence[PointCloudFrame]) -> None:
    """把帧序列写成单个 npz（布局见模块 docstring）；父目录自动创建。"""
    if not frames:
        raise ValueError("no frames to save")
    points = np.concatenate([f.points for f in frames], axis=0)
    offsets = np.concatenate([[0], np.cumsum([len(f.points) for f in frames])])
    stamps = np.asarray([f.stamp for f in frames], dtype=np.float64)
    poses = np.full((len(frames), 4, 4), np.nan, dtype=np.float64)
    for i, f in enumerate(frames):
        if f.pose is not None:
            poses[i] = f.pose
    sources = np.asarray([f.source for f in frames], dtype=str)
    np.savez(_ensure_parent(path), points=points, offsets=offsets, stamps=stamps, poses=poses, sources=sources)


def load_frames(path: str | Path) -> list[PointCloudFrame]:
    """读取 save_frames 写出的帧序列 npz。"""
    with np.load(path) as z:
        points, offsets = z["points"], z["offsets"]
        stamps, poses, sources = z["stamps"], z["poses"], z["sources"]
    frames: list[PointCloudFrame] = []
    for i in range(len(stamps)):
        pose = poses[i] if np.isfinite(poses[i]).all() else None
        frames.append(
            PointCloudFrame(
                points=points[offsets[i] : offsets[i + 1]], stamp=float(stamps[i]), pose=pose, source=str(sources[i])
            )
        )
    return frames


def load_frame_file(path: str | Path) -> PointCloudFrame:
    """读取逐帧 npz 文件（keys: points, stamp, [pose], [source]）。"""
    with np.load(path) as z:
        if "points" not in z or "stamp" not in z:
            raise ValueError(f"{path}: per-frame npz needs 'points' and 'stamp' keys")
        points = z["points"]
        stamp = float(z["stamp"])
        pose = z["pose"] if "pose" in z else None
        source = str(z["source"]) if "source" in z else "default"
    return PointCloudFrame(points=points, stamp=stamp, pose=pose, source=source)


def frame_files_from_dir(directory: str | Path) -> list[Path]:
    """目录内排序后的逐帧 npz 文件列表。"""
    files = sorted(Path(directory).glob("*.npz"))
    if not files:
        raise ValueError(f"{directory}: no .npz frame files found")
    return files


# ── 地图 npz ──────────────────────────────────────────────────────────────────

_MAP_META_KEYS = ("resolution", "center")


def save_map(path: str | Path, layers: dict[str, np.ndarray], resolution: float, center: Sequence[float]) -> None:
    """写出地图图层 + 元数据；父目录自动创建。消费方见 pico_elevationmap/pipeline/export.py。"""
    payload = {name: np.asarray(arr, dtype=np.float32) for name, arr in layers.items()}
    payload["resolution"] = np.float32(resolution)
    payload["center"] = np.asarray(center, dtype=np.float32)
    np.savez(_ensure_parent(path), **payload)


def load_map(path: str | Path) -> tuple[dict[str, np.ndarray], dict[str, Any]]:
    """读回地图 npz → ({layer: array}, {resolution, center})。meta 缺省给 None。"""
    layers: dict[str, np.ndarray] = {}
    meta: dict[str, Any] = {}
    with np.load(path) as z:
        for key in z.files:
            if key in _MAP_META_KEYS:
                meta[key] = z[key]
            else:
                layers[key] = z[key]
    meta["resolution"] = None if "resolution" not in meta else float(meta["resolution"])
    meta["center"] = None if "center" not in meta else [float(v) for v in meta["center"]]
    return layers, meta


# ── rosbag 读取（rosbags 纯 python，无需 ROS）──────────────────────────────────


def _cloud_to_xyz(msg: Any) -> np.ndarray:
    """sensor_msgs/PointCloud2 → (N,3) float32（去非有限点）。"""
    data = msg.data
    if isinstance(data, np.ndarray) and data.dtype.names and {"x", "y", "z"} <= set(data.dtype.names):
        pts = np.stack([data["x"].ravel(), data["y"].ravel(), data["z"].ravel()], axis=1)
    else:
        fields = {f.name: f for f in msg.fields}
        missing = {"x", "y", "z"} - set(fields)
        if missing:
            raise ValueError(f"PointCloud2 missing fields {sorted(missing)}")
        fmts: dict[str, str] = {}
        for name in ("x", "y", "z"):
            datatype = int(fields[name].datatype)
            if datatype not in _POINTFIELD_FMT:
                raise ValueError(f"PointCloud2 field '{name}' has unsupported datatype {datatype} (need float32/64)")
            fmts[name] = _POINTFIELD_FMT[datatype]
        dt = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": [fmts["x"], fmts["y"], fmts["z"]],
                "offsets": [int(fields["x"].offset), int(fields["y"].offset), int(fields["z"].offset)],
                "itemsize": int(msg.point_step),
            }
        )
        buf = data.tobytes() if isinstance(data, np.ndarray) else bytes(data)
        arr = np.frombuffer(buf, dtype=dt, count=int(msg.width) * int(msg.height))
        pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=1)
    pts = pts.astype(np.float32)
    finite = np.isfinite(pts).all(axis=1)
    return pts[finite]


def _pose_to_matrix(msg: Any) -> np.ndarray:
    """geometry_msgs/PoseStamped → (4,4) T_map_sensor。"""
    q = msg.pose.orientation
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = x * x + y * y + z * z + w * w
    s = 2.0 / n if n > 0.0 else 0.0
    wx, wy, wz = s * w * x, s * w * y, s * w * z
    xx, xy, xz = s * x * x, s * x * y, s * x * z
    yy, yz, zz = s * y * y, s * y * z, s * z * z
    T = np.eye(4)
    T[:3, :3] = [
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ]
    T[:3, 3] = [float(msg.pose.position.x), float(msg.pose.position.y), float(msg.pose.position.z)]
    return T


def iter_bag_frames(
    path: str | Path,
    point_topic: str | None = None,
    pose_topic: str | None = None,
) -> Iterator[PointCloudFrame]:
    """从 rosbag（ROS1 .bag / ROS2 .db3/.mcap）逐帧产出 PointCloudFrame。

    点云：所有 sensor_msgs/msg/PointCloud2 连接（可用 point_topic 过滤，取
    topic 名最后一段匹配）。位姿：最新的 geometry_msgs/msg/PoseStamped
    （stamp ≤ 点云 stamp；可用 pose_topic 过滤）。无位姿时 pose=None，
    由 replay 回退单位阵。
    """
    from rosbags.highlevel import AnyReader

    bag_path = Path(path)
    with AnyReader([bag_path]) as reader:
        cloud_conns = [c for c in reader.connections if c.msgtype == "sensor_msgs/msg/PointCloud2"]
        if point_topic is not None:
            cloud_conns = [c for c in cloud_conns if c.topic.split("/")[-1] == point_topic]
        if not cloud_conns:
            raise ValueError(f"{bag_path}: no sensor_msgs/msg/PointCloud2 topic found")
        pose_conns = [c for c in reader.connections if c.msgtype == "geometry_msgs/msg/PoseStamped"]
        if pose_topic is not None:
            pose_conns = [c for c in pose_conns if c.topic.split("/")[-1] == pose_topic]

        # 两遍读取：先建位姿索引（stamp 升序），点云按"最近一个 ≤ 点云 stamp"配对。
        # rosbags 不保证同时间戳消息的写入顺序，单遍"最后见到"会漏首帧配对。
        poses: list[tuple[float, np.ndarray]] = []
        for conn, timestamp, rawdata in reader.messages(connections=pose_conns):
            poses.append((timestamp * 1e-9, _pose_to_matrix(reader.deserialize(rawdata, conn.msgtype))))
        poses.sort(key=lambda p: p[0])
        pose_stamps = [p[0] for p in poses]

        for conn, timestamp, rawdata in reader.messages(connections=cloud_conns):
            msg = reader.deserialize(rawdata, conn.msgtype)
            source = conn.topic.strip("/").split("/")[-1] or "default"
            stamp = timestamp * 1e-9
            i = bisect.bisect_right(pose_stamps, stamp) - 1
            pose = poses[i][1] if i >= 0 else None
            yield PointCloudFrame(points=_cloud_to_xyz(msg), stamp=stamp, pose=pose, source=source)


# ── 导出写出器（PLY / PGM / CSV，零图像依赖）──────────────────────────────────


def grid_coordinates(
    shape: tuple[int, int], resolution: float, center: Sequence[float]
) -> tuple[np.ndarray, np.ndarray]:
    """导出数组的每个 cell 的世界系 (X, Y) 坐标。

    与上游 export_layers 的裁剪+翻转约定严格一致：exported[row, col] 对应
    x = cx + (n_rows/2 - 1 - row)·res（从 cx + L/2 - res 递减到 cx - L/2），
    y 同理（col 轴）。
    """
    n_rows, n_cols = shape
    cx, cy = float(center[0]), float(center[1])
    x_vals = cx + (n_rows / 2.0 - 1.0 - np.arange(n_rows)) * resolution
    y_vals = cy + (n_cols / 2.0 - 1.0 - np.arange(n_cols)) * resolution
    return np.meshgrid(x_vals, y_vals, indexing="ij")


def _height_colormap(h_norm: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """简单三段色标：蓝(低) → 绿(中) → 红(高)，输入已归一化到 [0,1]。"""
    t = np.clip(h_norm, 0.0, 1.0)
    r = np.clip(2.0 * t - 0.5, 0.0, 1.0)
    g = np.clip(1.5 - np.abs(2.0 * t - 1.0), 0.0, 1.0)
    b = np.clip(1.0 - 2.0 * t, 0.0, 1.0)
    return (r * 255).astype(np.uint8), (g * 255).astype(np.uint8), (b * 255).astype(np.uint8)


def write_ply(path: str | Path, layer: np.ndarray, resolution: float, center: Sequence[float]) -> int:
    """把高程层写成带高度色标的 ascii PLY 点云；NaN cell 跳过。返回顶点数。"""
    layer = np.asarray(layer, dtype=np.float64)
    valid = np.isfinite(layer)
    X, Y = grid_coordinates(layer.shape, resolution, center)
    h = layer[valid]
    hmin, hmax = float(h.min()), float(h.max())
    denom = max(hmax - hmin, 1e-9)
    t = (h - hmin) / denom
    r, g, b = _height_colormap(t)
    x, y, z = X[valid], Y[valid], h
    n = int(valid.sum())
    with Path(path).open("w", encoding="utf-8") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"comment pico-elevationmap export hmin={hmin:.4f} hmax={hmax:.4f}\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for i in range(n):
            f.write(f"{x[i]:.4f} {y[i]:.4f} {z[i]:.4f} {int(r[i])} {int(g[i])} {int(b[i])}\n")
    return n


def write_pgm(path: str | Path, layer: np.ndarray) -> tuple[float, float]:
    """把高程层写成 16 位 ascii PGM；NaN 记 0。返回 (hmin, hmax) 供标注。"""
    layer = np.asarray(layer, dtype=np.float64)
    valid = np.isfinite(layer)
    h = layer[valid]
    hmin, hmax = float(h.min()), float(h.max())
    denom = max(hmax - hmin, 1e-9)
    gray = np.zeros(layer.shape, dtype=np.int64)
    gray[valid] = np.rint((layer[valid] - hmin) / denom * 65535.0).astype(np.int64)
    with Path(path).open("w", encoding="utf-8") as f:
        f.write("P2\n")
        f.write(f"# hmin={hmin:.4f} hmax={hmax:.4f} (nan=0)\n")
        f.write(f"{layer.shape[1]} {layer.shape[0]}\n65535\n")
        for row in gray:
            f.write(" ".join(str(int(v)) for v in row) + "\n")
    return hmin, hmax


def write_csv(path: str | Path, layer: np.ndarray) -> None:
    """把图层矩阵写成 CSV（NaN 记空字段——np.savetxt 写 nan 字符串）。"""
    np.savetxt(path, np.asarray(layer, dtype=np.float64), delimiter=",", fmt="%.4f")
