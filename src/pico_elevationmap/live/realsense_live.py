"""D435i 实时建图 demo：深度图 → 点云 → ElevationMapper → 导出。

不依赖 ROS：用 pyrealsense2 直接取流，按深度图内参反投影，自动估计相机相对
地面的姿态（RANSAC 平面），把点云变换到 Z-up 的地图系后送入核心建图器。
适合静态相机/桌面场景的快速验证；真机移动平台仍走 ROS 桥。

用法：
    # 采 30 帧后保存
    uv run python -m pico_elevationmap.live.realsense_live \\
        --config src/pico_elevationmap/configs/realsense_live.yaml \\
        --frames 30 --save maps/realsense_map.npz --export-ply maps/realsense.ply

    # 实时窗口（q / Esc 退出；--frames 0 表示一直跑到退出）
    uv run python -m pico_elevationmap.live.realsense_live \\
        --config src/pico_elevationmap/configs/realsense_live.yaml --view --frames 0
"""

from __future__ import annotations

import argparse
import itertools
import math
import threading
import time
from pathlib import Path

import numpy as np

from pico_elevationmap.core import io
from pico_elevationmap.viz import FoxglovePublisher, SingleWindowViewer, VoxelMap

_PROGRESS_EVERY = 10


def deproject_depth(
    depth: np.ndarray,
    *,
    fx: float,
    fy: float,
    ppx: float,
    ppy: float,
    stride: int = 1,
    min_range: float = 0.15,
    max_range: float = 6.0,
) -> np.ndarray:
    """深度图（米）→ 相机光学系点云 (N,3)，自动过滤无效/超量程点。"""
    if depth.ndim != 2:
        raise ValueError(f"depth must be 2-D, got {depth.shape}")
    if stride < 1:
        raise ValueError(f"stride must be >= 1, got {stride}")
    h, w = depth.shape
    us = np.arange(0, w, stride, dtype=np.float32)
    vs = np.arange(0, h, stride, dtype=np.float32)
    u, v = np.meshgrid(us, vs)
    z = depth[::stride, ::stride].astype(np.float32)
    valid = np.isfinite(z) & (z >= min_range) & (z <= max_range)
    z = z[valid]
    x = (u[valid] - ppx) * z / fx
    y = (v[valid] - ppy) * z / fy
    return np.stack([x, y, z], axis=1).astype(np.float32)


def _ransac_plane(
    points: np.ndarray,
    *,
    thresh: float = 0.015,
    iterations: int = 600,
    seed: int = 0,
    up_hint: np.ndarray | None = None,
    min_up_dot: float = 0.7,
) -> tuple[np.ndarray, float, float] | None:
    """返回 (法向量, d, 内点率)，平面方程 n·p + d = 0；失败返回 None。

    ``up_hint`` 用于排除墙面等非地面主平面：法向量需与 hint 的夹角小于约 45°。
    """
    if len(points) < 100:
        return None
    hint = np.array([0.0, -1.0, 0.0]) if up_hint is None else np.asarray(up_hint, dtype=np.float64)
    hint = hint / np.linalg.norm(hint)
    rng = np.random.default_rng(seed)
    sample = points[rng.choice(len(points), min(6000, len(points)), replace=False)]
    best: tuple[int, np.ndarray, float] | None = None
    for _ in range(iterations):
        p = sample[rng.choice(len(sample), 3, replace=False)]
        n = np.cross(p[1] - p[0], p[2] - p[0])
        norm = float(np.linalg.norm(n))
        if norm < 1e-9:
            continue
        n = n / norm
        d = -float(n @ p[0])
        # 让法向量朝向相机（原点）：d > 0，地面在相机下方
        if d < 0.0:
            n, d = -n, -d
        if float(n @ hint) < min_up_dot:
            continue
        count = int((np.abs(sample @ n + d) < thresh).sum())
        if best is None or count > best[0]:
            best = (count, n, d)
    if best is None:
        return None
    count, n, d = best
    return n, d, count / len(sample)


def estimate_ground(points_opt: np.ndarray, *, max_range: float = 2.5) -> tuple[np.ndarray, float, float]:
    """从近场点云估计地面法向与相机高度；返回 (up, height, inlier_ratio)。

    优先在光学系下半部分（y > 0）用 RANSAC 拟合主平面，并用法向与 -y 的夹角
    排除墙面；失败时回退到光学系 -y 为“上”，高度取下半部分点 median(y) 的粗估。
    """
    dist = np.linalg.norm(points_opt, axis=1)
    near = points_opt[(dist > 0.15) & (dist < max_range)]
    # 地面在相机下方：优先只用光学系下半部分（y > 0），避免墙面成为最大平面
    ground = near[near[:, 1] > 0.0]
    if len(ground) < 100:
        ground = near
    fit = _ransac_plane(ground)
    if fit is not None:
        up, height, ratio = fit
        if ratio >= 0.08 and height > 0.02:
            return up, height, ratio
    up = np.array([0.0, -1.0, 0.0], dtype=np.float64)
    height = float(np.median(ground[:, 1])) if len(ground) else 0.0
    return up, max(height, 0.05), 0.0


def rotation_from_up(up: np.ndarray) -> np.ndarray:
    """构造 R_base_optical：行向量为 base 的 x/y/z 在光学系中的方向。"""
    up = np.asarray(up, dtype=np.float64)
    up = up / np.linalg.norm(up)
    forward = np.array([0.0, 0.0, 1.0])  # 相机光学系前向
    forward = forward - float(forward @ up) * up
    if np.linalg.norm(forward) < 0.1:
        forward = np.array([0.0, 1.0, 0.0]) - float(np.array([0.0, 1.0, 0.0]) @ up) * up
    forward = forward / np.linalg.norm(forward)
    left = np.cross(up, forward)
    return np.stack([forward, left, up], axis=0)


def pose_from_ground(up: np.ndarray, height: float) -> tuple[np.ndarray, np.ndarray]:
    """返回 (R_base_optical, T_map_sensor)，地面在 T 下为 z=0。"""
    R = rotation_from_up(up)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [0.0, 0.0, float(height)]
    return R, T


def rotation_rpy(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    """LiDAR 到地图的旋转：先 roll(x)、再 pitch(y)、最后 yaw(z)。"""
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def transform_lidar_points(
    points: np.ndarray,
    *,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    yaw_deg: float = 0.0,
    offset: tuple[float, float, float] = (0.0, 0.0, 0.0),
    z_offset: float | None = None,
) -> tuple[np.ndarray, np.ndarray, float]:
    """LiDAR 点云 → 地图系；朝下安装用 roll/pitch=180。

    先施加旋转，再用旋转后点云的低分位估计地面 z，因此不依赖 LiDAR 正装还是倒装。
    返回 ``(world_points, lidar_origin, z_offset_used)``。
    """
    pts = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    R = rotation_rpy(roll_deg, pitch_deg, yaw_deg)
    rotated = (R @ pts.T).T
    if z_offset is None:
        ground_z = float(np.percentile(rotated[:, 2], 5.0)) if len(rotated) else 0.0
        z_off = -ground_z
    else:
        z_off = float(z_offset)
    origin = np.asarray(offset, dtype=np.float64).reshape(3) + np.array([0.0, 0.0, z_off])
    world = rotated + origin
    return world, origin, z_off


class LidarSubscriber:
    """rclpy PointCloud2 / Livox CustomMsg 订阅器：只保留最新一帧。"""

    def __init__(self, topic: str, msg_type: str = "pointcloud2") -> None:
        import rclpy
        from rclpy.qos import qos_profile_sensor_data

        from pico_elevationmap_ros import messages

        if msg_type == "custom":
            from livox_ros_driver2.msg import CustomMsg

            self._msg_cls = CustomMsg
            self._parse = messages.livox_custom_msg_to_array
        elif msg_type == "pointcloud2":
            from sensor_msgs.msg import PointCloud2

            self._msg_cls = PointCloud2
            self._parse = messages.pointcloud2_to_array
        else:
            raise ValueError(f"unknown lidar msg_type {msg_type!r} (pointcloud2 | custom)")

        self._rclpy = rclpy
        rclpy.init()
        self.node = rclpy.create_node("pico_elevationmap_lidar_sub")
        self._lock = threading.Lock()
        self._latest: tuple[np.ndarray, float, str] | None = None
        self._seq = 0
        self.node.create_subscription(self._msg_cls, topic, self._callback, qos_profile_sensor_data)

    def _callback(self, msg: object) -> None:
        try:
            points, stamp, frame_id = self._parse(msg)
        except Exception:
            return
        with self._lock:
            self._latest = (points, stamp, frame_id)
            self._seq += 1

    def spin_once(self) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=0.0)

    def latest(self) -> tuple[tuple[np.ndarray, float, str] | None, int]:
        with self._lock:
            return self._latest, self._seq

    def close(self) -> None:
        self.node.destroy_node()
        self._rclpy.shutdown()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="realsense-live",
        description="D435i depth (+ optional ROS2 LiDAR) -> elevation map.",
    )
    parser.add_argument("--config", required=True, type=Path, help="YAML: map / sources / plugins")
    parser.add_argument(
        "--frames", type=int, default=30, help="frames to fuse after warmup; 0 = until quit (requires a view)"
    )
    parser.add_argument("--warmup", type=int, default=15, help="frames to discard before mapping")
    parser.add_argument("--stride", type=int, default=2, help="pixel stride for deprojection")
    parser.add_argument("--lidar-topic", type=str, default=None, help="ROS2 LiDAR topic; enables LiDAR fusion")
    parser.add_argument(
        "--lidar-msg-type",
        choices=("pointcloud2", "custom"),
        default="pointcloud2",
        help="LiDAR message type: sensor_msgs/PointCloud2 or Livox CustomMsg (default pointcloud2)",
    )
    parser.add_argument(
        "--lidar-source", type=str, default="lidar", help="source name in YAML for LiDAR (default lidar)"
    )
    parser.add_argument(
        "--depth-source", type=str, default=None, help="source name in YAML for depth (default primary)"
    )
    parser.add_argument(
        "--lidar-roll", type=float, default=0.0, help="LiDAR roll about x, degrees; use 180 for upside-down"
    )
    parser.add_argument(
        "--lidar-pitch", type=float, default=0.0, help="LiDAR pitch about y, degrees; use 180 for upside-down"
    )
    parser.add_argument(
        "--lidar-yaw", type=float, default=0.0, help="LiDAR yaw about z relative to map x-axis, degrees"
    )
    parser.add_argument(
        "--lidar-offset",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=("X", "Y", "Z"),
        help="LiDAR origin offset in map frame, metres",
    )
    parser.add_argument(
        "--lidar-z-offset",
        type=float,
        default=None,
        help="LiDAR ground-to-map z offset; default auto from 5th percentile",
    )
    parser.add_argument("--view", action="store_true", help="show live elevation/traversability window (q/Esc quits)")
    parser.add_argument("--view-every", type=int, default=1, help="render every N frames (default 1)")
    parser.add_argument("--foxglove", action="store_true", help="publish PointCloud2 topics for Foxglove/RViz")
    parser.add_argument("--z-min", type=float, default=-0.1, help="fixed colorbar lower bound in metres (default -0.1)")
    parser.add_argument("--z-max", type=float, default=1.0, help="fixed colorbar upper bound in metres (default 1.0)")
    parser.add_argument(
        "--color-bands", type=int, default=12, help="discrete height bands (default 12; 1 = continuous)"
    )
    parser.add_argument("--voxel-view", action="store_true", help="show live 3D voxel occupancy window")
    parser.add_argument("--voxel-size", type=float, default=0.05, help="voxel edge length in metres (default 0.05)")
    parser.add_argument("--voxel-alpha", type=float, default=0.55, help="voxel square opacity 0-1 (default 0.55)")
    parser.add_argument("--voxel-min-hits", type=int, default=1, help="minimum hits to draw a voxel (default 1)")
    parser.add_argument(
        "--voxel-hit-log-odds",
        type=float,
        default=0.85,
        help="log-odds increment for a hit (default 0.85)",
    )
    parser.add_argument(
        "--voxel-miss-log-odds",
        type=float,
        default=-0.85,
        help="log-odds increment for a ray passing through (default -0.85)",
    )
    parser.add_argument(
        "--voxel-occ-prob",
        type=float,
        default=0.65,
        help="minimum occupancy probability to draw a voxel (default 0.65)",
    )
    parser.add_argument(
        "--voxel-ttl", type=float, default=2.0, help="seconds before unseen voxels expire; 0 disables (default 2.0)"
    )
    parser.add_argument(
        "--voxel-carve",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="clear stale voxels along current depth rays (default on)",
    )
    parser.add_argument(
        "--voxel-max-rays", type=int, default=1200, help="max rays used for free-space carving (default 1200)"
    )
    parser.add_argument("--voxel-carve-steps", type=int, default=32, help="max samples per carving ray (default 32)")
    parser.add_argument("--voxel-carve-every", type=int, default=3, help="carve every N frames (default 3)")
    parser.add_argument("--voxel-save", type=Path, default=None, help="save last 3D voxel panel PNG")
    parser.add_argument("--view-save", type=Path, default=None, help="save last combined window PNG")
    parser.add_argument("--save", type=Path, default=None, help="save map npz")
    parser.add_argument("--save-frames", type=Path, default=None, help="save frames npz for replay")
    parser.add_argument("--export-ply", type=Path, default=None, help="export elevation layer to PLY")
    parser.add_argument("--export-pgm", type=Path, default=None, help="export elevation layer to PGM")
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    if args.frames <= 0 and not (args.view or args.voxel_view):
        raise SystemExit("--frames must be > 0 unless --view or --voxel-view is set")
    if args.view_every < 1:
        raise SystemExit("--view-every must be >= 1")
    if args.color_bands < 1:
        raise SystemExit("--color-bands must be >= 1")
    if args.z_max <= args.z_min:
        raise SystemExit("--z-max must be > --z-min")
    if args.voxel_size <= 0.0:
        raise SystemExit("--voxel-size must be > 0")
    if not 0.0 < args.voxel_alpha <= 1.0:
        raise SystemExit("--voxel-alpha must be in (0, 1]")
    if args.voxel_min_hits < 1:
        raise SystemExit("--voxel-min-hits must be >= 1")
    if args.voxel_hit_log_odds <= 0.0:
        raise SystemExit("--voxel-hit-log-odds must be > 0")
    if args.voxel_miss_log_odds >= 0.0:
        raise SystemExit("--voxel-miss-log-odds must be < 0")
    if not 0.0 < args.voxel_occ_prob < 1.0:
        raise SystemExit("--voxel-occ-prob must be in (0, 1)")
    if args.voxel_ttl < 0.0:
        raise SystemExit("--voxel-ttl must be >= 0")
    if args.voxel_max_rays < 1:
        raise SystemExit("--voxel-max-rays must be >= 1")
    if args.voxel_carve_steps < 2:
        raise SystemExit("--voxel-carve-steps must be >= 2")
    if args.voxel_carve_every < 1:
        raise SystemExit("--voxel-carve-every must be >= 1")
    view = args.view
    voxel_view = args.voxel_view
    viewer = (
        SingleWindowViewer(
            show_2d=view,
            show_3d=voxel_view,
            z_min=args.z_min,
            z_max=args.z_max,
            bands=args.color_bands,
            voxel_size=args.voxel_size,
            voxel_alpha=args.voxel_alpha,
        )
        if (view or voxel_view)
        else None
    )
    voxel_render = voxel_view or args.voxel_save is not None
    voxel_map = (
        VoxelMap(
            args.voxel_size,
            args.voxel_min_hits,
            ttl=args.voxel_ttl,
            hit_log_odds=args.voxel_hit_log_odds,
            miss_log_odds=args.voxel_miss_log_odds,
            occ_probability=args.voxel_occ_prob,
        )
        if voxel_render
        else None
    )

    import cupy as cp
    import pyrealsense2 as rs

    from pico_elevationmap.core.mapper import ElevationMapper, MapperConfig

    cfg = MapperConfig.from_yaml(args.config)
    depth_source = cfg.resolve_source(args.depth_source) if args.depth_source else cfg.primary_source
    lidar_source = None
    lidar_sub: LidarSubscriber | None = None
    if args.lidar_topic is not None:
        try:
            lidar_source = cfg.resolve_source(args.lidar_source)
        except KeyError as exc:
            raise SystemExit(f"{args.lidar_source!r} not declared in the config sources") from exc
        try:
            lidar_sub = LidarSubscriber(args.lidar_topic, args.lidar_msg_type)
        except ImportError as exc:
            if args.lidar_msg_type == "custom" and (exc.name or "").startswith("livox_ros_driver2"):
                raise SystemExit(
                    "--lidar-msg-type custom needs the Livox ROS 2 driver (livox_ros_driver2). "
                    "Build and source that driver, or use --lidar-msg-type pointcloud2 for "
                    "RoboSense / standard PointCloud2 topics."
                ) from exc
            raise SystemExit(
                "--lidar-topic needs a ROS 2 Python environment with rclpy and sensor_msgs. "
                "Source /opt/ros/<distro>/setup.bash, create a ROS-visible venv with "
                "`uv venv --system-site-packages .venv-ros`, and run "
                "`.venv-ros/bin/python -m pico_elevationmap.live.realsense_live ...`; "
                "see README `D435i + RoboSense Airy`."
            ) from exc
        print(f"[realsense] LiDAR topic {args.lidar_topic!r} ({args.lidar_msg_type}) -> source '{lidar_source.name}'")

    foxglove: FoxglovePublisher | None = None
    if args.foxglove:
        try:
            foxglove = FoxglovePublisher(node=lidar_sub.node if lidar_sub is not None else None)
        except ImportError as exc:
            raise SystemExit(
                "--foxglove needs a ROS 2 Python environment with rclpy, sensor_msgs and "
                "builtin_interfaces. Source /opt/ros/<distro>/setup.bash and use a ROS-visible "
                "venv (`uv venv --system-site-packages .venv-ros`); see README `Foxglove`."
            ) from exc
        print("[realsense] Foxglove topics: /pico_elevationmap/elevation_points, /pico_elevationmap/voxels")

    pipeline = rs.pipeline()
    rs_cfg = rs.config()
    rs_cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    pipeline.start(rs_cfg)
    try:
        for _ in range(max(args.warmup, 1)):
            pipeline.wait_for_frames(timeout_ms=5000)
        frames = pipeline.wait_for_frames(timeout_ms=5000)
        depth_frame = frames.get_depth_frame()
        intr = depth_frame.profile.as_video_stream_profile().intrinsics
        units = float(depth_frame.get_units())
        depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * units
        probe = deproject_depth(
            depth_m,
            fx=intr.fx,
            fy=intr.fy,
            ppx=intr.ppx,
            ppy=intr.ppy,
            stride=args.stride,
            min_range=depth_source.min_range,
            max_range=depth_source.max_range,
        )
        up, height, ratio = estimate_ground(probe)
        R, T = pose_from_ground(up, height)
        print(
            f"[realsense] {probe.shape[0]} pts, ground up={np.round(up, 3).tolist()}, "
            f"height={height:.3f} m, inliers={ratio:.1%}"
        )
        if ratio == 0.0:
            print("[realsense] warning: no ground-like plane found; fallback to level orientation")
            print("[realsense]          raise/tilt the camera so the floor is visible for a useful map")
        print(f"[realsense] R_base_optical=\n{np.round(R, 3)}")

        mapper = ElevationMapper(cfg)
        mapper.move_to([0.0, 0.0, 0.0], np.eye(3))
        times: list[float] = []
        saved_frames: list[io.PointCloudFrame] = []
        last_lidar_seq = 0
        lidar_ground_z: float | None = None
        lidar_frames = 0
        last_lidar_accepted = 0
        last_view_t: float | None = None
        fps = 0.0
        frame_iter = range(args.frames) if args.frames > 0 else itertools.count()
        for i in frame_iter:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            stamp = float(depth_frame.get_timestamp()) / 1000.0
            depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * units
            points = deproject_depth(
                depth_m,
                fx=intr.fx,
                fy=intr.fy,
                ppx=intr.ppx,
                ppy=intr.ppy,
                stride=args.stride,
                min_range=depth_source.min_range,
                max_range=depth_source.max_range,
            )
            camera_world = (R @ points.T).T + T[:3, 3]
            mapper.add_pose(stamp, np.eye(4))

            if lidar_sub is not None and lidar_source is not None:
                lidar_sub.spin_once()
                latest, seq = lidar_sub.latest()
                if latest is not None and seq != last_lidar_seq:
                    lidar_points, _lidar_stamp, frame_id = latest
                    dist = np.linalg.norm(lidar_points, axis=1)
                    keep = (dist >= lidar_source.min_range) & (dist <= lidar_source.max_range)
                    lidar_points = lidar_points[keep]
                    if len(lidar_points):
                        lidar_world, lidar_origin, z_used = transform_lidar_points(
                            lidar_points,
                            roll_deg=args.lidar_roll,
                            pitch_deg=args.lidar_pitch,
                            yaw_deg=args.lidar_yaw,
                            offset=args.lidar_offset,
                            z_offset=lidar_ground_z,
                        )
                        if lidar_ground_z is None:
                            lidar_ground_z = z_used
                            print(f"[realsense] LiDAR frame {frame_id!r}, ground z offset {lidar_ground_z:.3f} m")
                        last_lidar_accepted = mapper.input(lidar_world, stamp, source=lidar_source.name)
                        lidar_frames += 1
                        if voxel_map is not None:
                            voxel_map.add(lidar_world, stamp)
                            if args.voxel_carve and i % args.voxel_carve_every == 0:
                                voxel_map.carve_free(
                                    lidar_origin,
                                    lidar_world,
                                    max_rays=args.voxel_max_rays,
                                    max_steps=args.voxel_carve_steps,
                                )
                    last_lidar_seq = seq

            if voxel_map is not None:
                voxel_map.add(camera_world, stamp)
                if args.voxel_carve and i % args.voxel_carve_every == 0:
                    voxel_map.carve_free(
                        T[:3, 3],
                        camera_world,
                        max_rays=args.voxel_max_rays,
                        max_steps=args.voxel_carve_steps,
                    )
            t0 = time.perf_counter()
            accepted = mapper.input(camera_world, stamp, source=depth_source.name)
            cp.cuda.Stream.null.synchronize()
            times.append((time.perf_counter() - t0) * 1e3)
            mapper.update_variance()
            mapper.update_time()
            if voxel_map is not None:
                voxel_map.prune(stamp)

            if viewer is not None and (i + 1) % args.view_every == 0:
                now = time.perf_counter()
                if last_view_t is not None:
                    inst = 1.0 / max(now - last_view_t, 1e-6)
                    fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst
                last_view_t = now

                elevation = mapper.get_layer("elevation")
                traversability = None
                try:
                    traversability = mapper.get_layer("traversability_geo")
                except KeyError:
                    pass
                valid = np.isfinite(elevation)
                coverage = float(valid.mean())
                if voxel_map is not None:
                    centers, probabilities = voxel_map.centers_with_probabilities()
                else:
                    centers = np.empty((0, 3))
                    probabilities = np.empty((0,))
                center = mapper.center
                target = np.array([center[0], center[1], 0.0])
                canvas = viewer.compose(
                    elevation=elevation,
                    traversability=traversability,
                    voxel_centers=centers,
                    voxel_probabilities=probabilities,
                    target=target,
                    fps=fps,
                    coverage=coverage,
                    resolution=cfg.resolution,
                )
                viewer.show(canvas)
                if viewer.wait_key():
                    print("[realsense] view closed")
                    break

            if foxglove is not None and (i + 1) % args.view_every == 0:
                elevation = mapper.get_layer("elevation")
                if voxel_map is not None:
                    centers, probabilities = voxel_map.centers_with_probabilities()
                else:
                    centers = np.empty((0, 3))
                    probabilities = np.empty((0,))
                foxglove.publish(
                    elevation=elevation,
                    centers=centers,
                    probabilities=probabilities,
                    resolution=cfg.resolution,
                    center=mapper.center,
                    stamp=stamp,
                )

            if args.save_frames is not None:
                saved_frames.append(
                    io.PointCloudFrame(points=points, stamp=stamp, pose=T.copy(), source=depth_source.name)
                )
            if (i + 1) % _PROGRESS_EVERY == 0 or (0 < args.frames == i + 1):
                lidar_text = f", lidar {last_lidar_accepted}" if lidar_sub is not None else ""
                print(
                    f"[realsense] frame {i + 1}/{args.frames}: {len(points)} pts "
                    f"({accepted} fused{lidar_text}), {times[-1]:.2f} ms"
                )

        stats = mapper.stats()
        arr = np.asarray(times)
        print(f"[realsense] timing ms/step: mean {arr.mean():.2f}, min {arr.min():.2f}, max {arr.max():.2f}")
        if lidar_sub is not None:
            print(f"[realsense] LiDAR frames fused: {lidar_frames}")
        print(
            f"[realsense] map stats: coverage {stats.coverage:.1%} ({stats.valid_cells}/{stats.total_cells}), "
            f"height mean {stats.mean_height:.3f} m, range [{stats.min_height:.3f}, {stats.max_height:.3f}] m"
        )

        if args.save is not None:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            layers = mapper.get_map()
            io.save_map(args.save, layers, cfg.resolution, stats.center)
            print(f"[realsense] saved {len(layers)} layers -> {args.save}")
        if args.save_frames is not None:
            args.save_frames.parent.mkdir(parents=True, exist_ok=True)
            io.save_frames(args.save_frames, saved_frames)
            print(f"[realsense] saved {len(saved_frames)} frames -> {args.save_frames}")
        if args.export_ply is not None:
            args.export_ply.parent.mkdir(parents=True, exist_ok=True)
            n = io.write_ply(args.export_ply, mapper.get_layer("elevation"), cfg.resolution, stats.center)
            print(f"[realsense] exported {n} vertices -> {args.export_ply}")
        if args.export_pgm is not None:
            args.export_pgm.parent.mkdir(parents=True, exist_ok=True)
            io.write_pgm(args.export_pgm, mapper.get_layer("elevation"))
            print(f"[realsense] exported PGM -> {args.export_pgm}")
        if args.voxel_save is not None and viewer is not None and viewer.last_voxel_panel is not None:
            args.voxel_save.parent.mkdir(parents=True, exist_ok=True)
            viewer.save_voxel(str(args.voxel_save))
            print(f"[realsense] saved voxel view -> {args.voxel_save}")
        if args.view_save is not None and viewer is not None and viewer.last_canvas is not None:
            args.view_save.parent.mkdir(parents=True, exist_ok=True)
            viewer.save_canvas(str(args.view_save))
            print(f"[realsense] saved combined view -> {args.view_save}")
    finally:
        pipeline.stop()
        if foxglove is not None:
            foxglove.close()
        if lidar_sub is not None:
            lidar_sub.close()
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
