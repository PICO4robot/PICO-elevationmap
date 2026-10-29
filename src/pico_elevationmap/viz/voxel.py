"""Sparse voxel occupancy map and perspective voxel renderer."""

from __future__ import annotations

import math

import numpy as np

__all__ = ["VoxelMap", "render_voxels"]


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
    alpha: float = 0.55,
) -> np.ndarray:
    """把稀疏体素中心渲染成透视 3D 地图（小方块 + 统一透明度）。"""
    import cv2

    img = np.full((height, width, 3), 22, dtype=np.uint8)
    overlay = np.zeros_like(img)
    mask = np.zeros((height, width), dtype=np.uint8)
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
            (
                np.array([target[0] + offset, target[1] - grid, 0.0]),
                np.array([target[0] + offset, target[1] + grid, 0.0]),
            ),
            (
                np.array([target[0] - grid, target[1] + offset, 0.0]),
                np.array([target[0] + grid, target[1] + offset, 0.0]),
            ),
        ):
            u, v, depth = project(np.stack([p0, p1]))
            if (depth > 0.2).all():
                cv2.line(img, (int(u[0]), int(v[0])), (int(u[1]), int(v[1])), (55, 55, 55), 1, cv2.LINE_AA)

    # 坐标轴：x 红、y 绿、z 蓝
    for axis, color in (
        (np.array([1.0, 0.0, 0.0]), (60, 60, 220)),
        (np.array([0.0, 1.0, 0.0]), (60, 200, 60)),
        (np.array([0.0, 0.0, 1.0]), (220, 120, 60)),
    ):
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
        alpha = float(np.clip(alpha, 0.0, 1.0))
        half_px = np.clip(np.round(0.5 * voxel_size * fx / np.maximum(depth, 1e-6)), 1, 12).astype(np.int32)
        for idx in np.argsort(-depth):
            if not valid[idx]:
                continue
            color = tuple(int(c) for c in palette[band[idx]])
            half = int(half_px[idx])
            cx, cy = int(u[idx]), int(v[idx])
            x0, x1 = max(cx - half, 0), min(cx + half, width - 1)
            y0, y1 = max(cy - half, 0), min(cy + half, height - 1)
            if x0 > x1 or y0 > y1:
                continue
            cv2.rectangle(overlay, (x0, y0), (x1, y1), color, -1, cv2.LINE_AA)
            cv2.rectangle(mask, (x0, y0), (x1, y1), 255, -1)
            if half >= 2:
                border = tuple(max(0, int(c * 0.55)) for c in color)
                cv2.rectangle(overlay, (x0, y0), (x1, y1), border, 1, cv2.LINE_AA)
        if mask.any():
            blended = cv2.addWeighted(img, 1.0 - alpha, overlay, alpha, 0.0)
            covered = mask.astype(bool)
            img[covered] = blended[covered]

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
