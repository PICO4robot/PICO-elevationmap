"""D435i 实时建图 demo：深度图 → 点云 → ElevationMapper → 导出。

不依赖 ROS：用 pyrealsense2 直接取流，按深度图内参反投影，自动估计相机相对
地面的姿态（RANSAC 平面），把点云变换到 Z-up 的地图系后送入核心建图器。
适合静态相机/桌面场景的快速验证；真机移动平台仍走 ROS 桥。

用法：
    # 采 30 帧后保存
    uv run python -m scripts.tools.realsense_live \\
        --config scripts/tools/configs/realsense_live.yaml \\
        --frames 30 --save maps/realsense_map.npz --export-ply maps/realsense.ply

    # 实时窗口（q / Esc 退出；--frames 0 表示一直跑到退出）
    uv run python -m scripts.tools.realsense_live \\
        --config scripts/tools/configs/realsense_live.yaml --view --frames 0
"""

from __future__ import annotations

import argparse
import itertools
import math
import time
from pathlib import Path

import numpy as np

from pico_elevationmap.core import io

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


class VoxelMap:
    """稀疏体素占用图：世界系点云 → dict[(i,j,k)] 命中计数 + 最近观测时间。

    动态场景的可视化需要“遗忘”机制：
    - ``carve_free`` 从传感器原点沿当前深度射线采样，把射线经过的旧占用格删掉；
    - ``prune`` 按时间戳清理超过 TTL 未再观测的体素，作为遮挡/稀疏射线的兜底。
    """

    def __init__(self, voxel_size: float = 0.05, min_hits: int = 1, ttl: float | None = 2.0) -> None:
        if voxel_size <= 0.0:
            raise ValueError(f"voxel_size must be > 0, got {voxel_size}")
        if min_hits < 1:
            raise ValueError(f"min_hits must be >= 1, got {min_hits}")
        self.voxel_size = float(voxel_size)
        self.min_hits = int(min_hits)
        self.ttl = ttl
        self._counts: dict[tuple[int, int, int], int] = {}
        self._last_seen: dict[tuple[int, int, int], float] = {}

    def add(self, points_world: np.ndarray, stamp: float = 0.0) -> None:
        points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) == 0:
            return
        keys = np.floor(points / self.voxel_size).astype(np.int64)
        uniq, counts = np.unique(keys, axis=0, return_counts=True)
        for key, count in zip(uniq, counts):
            k = (int(key[0]), int(key[1]), int(key[2]))
            self._counts[k] = self._counts.get(k, 0) + int(count)
            self._last_seen[k] = float(stamp)

    def carve_free(
        self,
        origin: np.ndarray,
        points_world: np.ndarray,
        *,
        max_rays: int = 1200,
        max_steps: int = 32,
    ) -> None:
        """沿 origin → 当前点射线标记自由空间，删除射线穿过的旧占用格。

        只采样射线上的体素（不逐点 DDA），够快；端点体素不删，避免把当前
        观测到的障碍物清掉。射线很稀疏时由 ``prune`` 的 TTL 兜底。
        """
        points = np.asarray(points_world, dtype=np.float64).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        if len(points) == 0:
            return
        if len(points) > max_rays:
            points = points[:: int(np.ceil(len(points) / max_rays))]
        origin = np.asarray(origin, dtype=np.float64).reshape(3)
        dist = np.linalg.norm(points - origin, axis=1)
        points = points[dist > self.voxel_size]
        if len(points) == 0:
            return
        steps = max(4, min(int(max_steps), int(np.ceil(float(dist.max()) / self.voxel_size))))
        ts = np.linspace(0.0, 1.0, steps + 2)[1:-1]
        samples = origin + (points[:, None, :] - origin) * ts[None, :, None]
        free_keys = set(map(tuple, np.floor(samples / self.voxel_size).astype(np.int64).reshape(-1, 3)))
        endpoint_keys = set(map(tuple, np.floor(points / self.voxel_size).astype(np.int64)))
        for k in free_keys:
            if k in self._counts and k not in endpoint_keys:
                del self._counts[k]
                self._last_seen.pop(k, None)

    def prune(self, now: float, ttl: float | None = None) -> None:
        """清理超过 TTL 未再观测的体素；ttl=0/None 表示禁用。"""
        limit = self.ttl if ttl is None else ttl
        if limit is None or limit <= 0.0:
            return
        for k, seen in list(self._last_seen.items()):
            if now - seen > limit:
                self._counts.pop(k, None)
                self._last_seen.pop(k, None)

    def __len__(self) -> int:
        return len(self._counts)

    def centers(self) -> np.ndarray:
        keys = [k for k, count in self._counts.items() if count >= self.min_hits]
        if not keys:
            return np.empty((0, 3), dtype=np.float64)
        arr = np.asarray(keys, dtype=np.float64)
        return (arr + 0.5) * self.voxel_size


def render_voxels(
    centers: np.ndarray,
    *,
    z_min: float,
    z_max: float,
    target: np.ndarray,
    yaw: float,
    pitch: float,
    distance: float,
    voxel_size: float = 0.05,
    width: int = 720,
    height: int = 520,
    fov_deg: float = 60.0,
    bands: int = 12,
) -> np.ndarray:
    """把稀疏体素中心渲染成透视 3D 地图（远点先画、近点覆盖）。"""
    import cv2

    img = np.full((height, width, 3), 22, dtype=np.uint8)
    target = np.asarray(target, dtype=np.float64)
    eye = target + float(distance) * np.array(
        [math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)]
    )
    forward = target - eye
    forward /= max(float(np.linalg.norm(forward)), 1e-9)
    up_world = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, up_world)
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)
    view = np.stack([right, up, -forward])
    fx = 0.5 * width / math.tan(math.radians(fov_deg) * 0.5)
    fy = fx

    def project(points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        cam = (np.asarray(points, dtype=np.float64) - eye) @ view.T
        depth = -cam[:, 2]
        u = width * 0.5 + fx * cam[:, 0] / np.maximum(depth, 1e-6)
        v = height * 0.5 - fy * cam[:, 1] / np.maximum(depth, 1e-6)
        return u, v, depth

    # 地面网格（z=0，围绕 target 的 ±3 m）作为空间参照
    grid = 3.0
    for offset in np.arange(-grid, grid + 1e-6, 1.0):
        for p0, p1 in (
            (np.array([target[0] + offset, target[1] - grid, 0.0]), np.array([target[0] + offset, target[1] + grid, 0.0])),
            (np.array([target[0] - grid, target[1] + offset, 0.0]), np.array([target[0] + grid, target[1] + offset, 0.0])),
        ):
            u, v, depth = project(np.stack([p0, p1]))
            if (depth > 0.2).all():
                cv2.line(img, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), (55, 55, 55), 1, cv2.LINE_AA)

    # 坐标轴：x 红、y 绿、z 蓝
    for axis, color in ((np.array([1.0, 0.0, 0.0]), (60, 60, 220)), (np.array([0.0, 1.0, 0.0]), (60, 200, 60)), (np.array([0.0, 0.0, 1.0]), (220, 120, 60))):
        u, v, depth = project(np.stack([target, target + 0.6 * axis]))
        if (depth > 0.2).all():
            cv2.line(img, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), color, 2, cv2.LINE_AA)

    if len(centers) > 0:
        centers = np.asarray(centers, dtype=np.float64)
        if len(centers) > 30000:
            centers = centers[:: int(np.ceil(len(centers) / 30000))]
        u, v, depth = project(centers)
        valid = (depth > 0.2) & np.isfinite(depth)
        valid &= (u >= 0) & (u < width) & (v >= 0) & (v < height)
        palette = cv2.applyColorMap(
            ((np.arange(bands, dtype=np.float32) + 0.5) / bands * 255.0).astype(np.uint8).reshape(-1, 1),
            cv2.COLORMAP_TURBO,
        ).reshape(-1, 3)
        span = max(float(z_max - z_min), 1e-6)
        band = np.clip(((centers[:, 2] - z_min) / span * bands).astype(np.int32), 0, bands - 1)
        radius = max(1, min(6, int(round(voxel_size * fx / max(distance, 1e-6)))))
        for idx in np.argsort(-depth):
            if not valid[idx]:
                continue
            color = tuple(int(c) for c in palette[band[idx]])
            cv2.circle(img, (int(u[idx]), int(v[idx])), radius, color, -1, cv2.LINE_AA)

    cv2.putText(
        img,
        f"voxels {len(centers)} | yaw {math.degrees(yaw):.0f} deg | pitch {math.degrees(pitch):.0f} deg | dist {distance:.1f} m",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (240, 240, 240),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        img,
        "drag rotate | wheel zoom | a/d w/s +/- | q quit",
        (10, 46),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (190, 190, 190),
        1,
        cv2.LINE_AA,
    )
    return img


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="realsense-live",
        description="D435i depth -> elevation map demo (no ROS).",
    )
    parser.add_argument("--config", required=True, type=Path, help="YAML: map / sources / plugins")
    parser.add_argument(
        "--frames", type=int, default=30, help="frames to fuse after warmup; 0 = until quit (requires a view)"
    )
    parser.add_argument("--warmup", type=int, default=15, help="frames to discard before mapping")
    parser.add_argument("--stride", type=int, default=2, help="pixel stride for deprojection")
    parser.add_argument("--view", action="store_true", help="show live elevation/traversability window (q/Esc quits)")
    parser.add_argument("--view-every", type=int, default=1, help="render every N frames (default 1)")
    parser.add_argument("--z-min", type=float, default=-0.1, help="fixed colorbar lower bound in metres (default -0.1)")
    parser.add_argument("--z-max", type=float, default=1.0, help="fixed colorbar upper bound in metres (default 1.0)")
    parser.add_argument("--color-bands", type=int, default=12, help="discrete height bands (default 12; 1 = continuous)")
    parser.add_argument("--voxel-view", action="store_true", help="show live 3D voxel occupancy window")
    parser.add_argument("--voxel-size", type=float, default=0.05, help="voxel edge length in metres (default 0.05)")
    parser.add_argument("--voxel-min-hits", type=int, default=1, help="minimum hits to draw a voxel (default 1)")
    parser.add_argument("--voxel-ttl", type=float, default=2.0, help="seconds before unseen voxels expire; 0 disables (default 2.0)")
    parser.add_argument(
        "--voxel-carve",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="clear stale voxels along current depth rays (default on)",
    )
    parser.add_argument("--voxel-max-rays", type=int, default=1200, help="max rays used for free-space carving (default 1200)")
    parser.add_argument("--voxel-carve-steps", type=int, default=32, help="max samples per carving ray (default 32)")
    parser.add_argument("--voxel-carve-every", type=int, default=3, help="carve every N frames (default 3)")
    parser.add_argument("--voxel-save", type=Path, default=None, help="save last 3D voxel view PNG")
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
    if args.voxel_min_hits < 1:
        raise SystemExit("--voxel-min-hits must be >= 1")
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
    view_state = {
        "yaw": math.radians(45.0),
        "pitch": math.radians(25.0),
        "distance": 6.0,
        "drag": None,
    }
    if view or voxel_view or args.voxel_save is not None:
        import cv2

        if view:
            cv2.namedWindow("PICO elevation map", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("PICO elevation map", 1200, 480)
        if voxel_view:
            cv2.namedWindow("PICO voxel map", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("PICO voxel map", 900, 650)

            def _on_voxel_mouse(event: int, x: int, y: int, flags: int, _param: object) -> None:
                if event == cv2.EVENT_LBUTTONDOWN:
                    view_state["drag"] = (x, y)
                elif event == cv2.EVENT_MOUSEMOVE and view_state["drag"] is not None:
                    if not (flags & cv2.EVENT_FLAG_LBUTTON):
                        view_state["drag"] = None
                        return
                    last_x, last_y = view_state["drag"]
                    view_state["yaw"] += (x - last_x) * 0.01
                    view_state["pitch"] = float(
                        np.clip(view_state["pitch"] + (last_y - y) * 0.01, 0.05, 1.4)
                    )
                    view_state["drag"] = (x, y)
                elif event == cv2.EVENT_LBUTTONUP:
                    view_state["drag"] = None
                elif event == cv2.EVENT_MOUSEWHEEL:
                    step = -0.5 if flags > 0 else 0.5
                    view_state["distance"] = float(np.clip(view_state["distance"] + step, 1.0, 20.0))

            cv2.setMouseCallback("PICO voxel map", _on_voxel_mouse)
    voxel_render = voxel_view or args.voxel_save is not None
    voxel_map = (
        VoxelMap(args.voxel_size, args.voxel_min_hits, ttl=args.voxel_ttl) if voxel_render else None
    )
    last_voxel_canvas: np.ndarray | None = None

    import cupy as cp
    import pyrealsense2 as rs

    from pico_elevationmap.core.mapper import ElevationMapper, MapperConfig

    cfg = MapperConfig.from_yaml(args.config)
    source = cfg.primary_source

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
            min_range=source.min_range,
            max_range=source.max_range,
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
        mapper.move_to([0.0, 0.0, 0.0], R)
        times: list[float] = []
        saved_frames: list[io.PointCloudFrame] = []
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
                min_range=source.min_range,
                max_range=source.max_range,
            )
            if voxel_map is not None:
                world_points = (R @ points.T).T + T[:3, 3]
                voxel_map.add(world_points, stamp)
                if args.voxel_carve and i % args.voxel_carve_every == 0:
                    voxel_map.carve_free(
                        T[:3, 3],
                        world_points,
                        max_rays=args.voxel_max_rays,
                        max_steps=args.voxel_carve_steps,
                    )
                voxel_map.prune(stamp)
            mapper.add_pose(stamp, T)
            t0 = time.perf_counter()
            accepted = mapper.input(points, stamp, source=source.name)
            cp.cuda.Stream.null.synchronize()
            times.append((time.perf_counter() - t0) * 1e3)
            mapper.update_variance()
            mapper.update_time()

            if view and (i + 1) % args.view_every == 0:
                elevation = mapper.get_layer("elevation")
                elev_img = render_layer(
                    elevation,
                    colormap="turbo",
                    z_min=args.z_min,
                    z_max=args.z_max,
                    bands=args.color_bands,
                    resolution=cfg.resolution,
                )
                panels = [elev_img]
                traversability = None
                try:
                    traversability = mapper.get_layer("traversability_geo")
                except KeyError:
                    pass
                if traversability is not None:
                    panels.append(
                        render_layer(
                            traversability,
                            colormap="viridis",
                            z_min=0.0,
                            z_max=1.0,
                            bands=2,
                            shade=False,
                        )
                    )
                legend = render_legend(
                    args.z_min,
                    args.z_max,
                    height=elev_img.shape[0],
                    bands=args.color_bands,
                )
                panels.append(legend)
                canvas = np.hstack(panels)
                scale = 420.0 / elev_img.shape[0]
                canvas = cv2.resize(canvas, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
                now = time.perf_counter()
                if last_view_t is not None:
                    inst = 1.0 / max(now - last_view_t, 1e-6)
                    fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst
                last_view_t = now
                valid = np.isfinite(elevation)
                coverage = float(valid.mean())
                z = elevation[valid]
                z_text = f"z [{z.min():.2f}, {z.max():.2f}] m" if z.size else "z [--]"
                cv2.putText(
                    canvas,
                    f"elevation | {fps:5.1f} fps | coverage {coverage:5.1%} | {z_text}",
                    (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                panel_w = int(elev_img.shape[1] * scale)
                if traversability is not None:
                    cv2.putText(
                        canvas,
                        "traversability_geo",
                        (panel_w + 10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )
                cv2.imshow("PICO elevation map", canvas)

            if voxel_render and (i + 1) % args.view_every == 0:
                assert voxel_map is not None
                centers = voxel_map.centers()
                center = mapper.center
                view_target = np.array([center[0], center[1], 0.0])
                voxel_img = render_voxels(
                    centers,
                    z_min=args.z_min,
                    z_max=args.z_max,
                    target=view_target,
                    yaw=view_state["yaw"],
                    pitch=view_state["pitch"],
                    distance=view_state["distance"],
                    voxel_size=args.voxel_size,
                    bands=args.color_bands,
                )
                voxel_legend = render_legend(
                    args.z_min,
                    args.z_max,
                    height=voxel_img.shape[0],
                    bands=args.color_bands,
                )
                last_voxel_canvas = np.hstack([voxel_img, voxel_legend])
                if voxel_view:
                    cv2.imshow("PICO voxel map", last_voxel_canvas)

            if (view or voxel_view) and (i + 1) % args.view_every == 0:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    print("[realsense] view closed")
                    break
                if voxel_view:
                    if key == ord("a"):
                        view_state["yaw"] -= 0.1
                    elif key == ord("d"):
                        view_state["yaw"] += 0.1
                    elif key == ord("w"):
                        view_state["pitch"] = min(view_state["pitch"] + 0.08, 1.4)
                    elif key == ord("s"):
                        view_state["pitch"] = max(view_state["pitch"] - 0.08, 0.05)
                    elif key in (ord("+"), ord("=")):
                        view_state["distance"] = max(1.0, view_state["distance"] - 0.5)
                    elif key in (ord("-"), ord("_")):
                        view_state["distance"] = min(20.0, view_state["distance"] + 0.5)

            if args.save_frames is not None:
                saved_frames.append(io.PointCloudFrame(points=points, stamp=stamp, pose=T.copy(), source=source.name))
            if (i + 1) % _PROGRESS_EVERY == 0 or (0 < args.frames == i + 1):
                print(f"[realsense] frame {i + 1}/{args.frames}: {len(points)} pts ({accepted} fused), {times[-1]:.2f} ms")

        stats = mapper.stats()
        arr = np.asarray(times)
        print(
            f"[realsense] timing ms/step: mean {arr.mean():.2f}, min {arr.min():.2f}, max {arr.max():.2f}"
        )
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
        if args.voxel_save is not None and last_voxel_canvas is not None:
            args.voxel_save.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(args.voxel_save), last_voxel_canvas)
            print(f"[realsense] saved voxel view -> {args.voxel_save}")
    finally:
        pipeline.stop()
        if view or voxel_view:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
