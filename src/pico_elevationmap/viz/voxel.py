"""Sparse probabilistic voxel occupancy map and perspective voxel renderer."""

from __future__ import annotations

import math

import numpy as np

__all__ = ["VoxelMap", "render_voxels"]


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, float(value)))


def _probability(log_odds: float) -> float:
    return 1.0 / (1.0 + math.exp(-_clamp(log_odds, -60.0, 60.0)))


class VoxelMap:
    """Sparse probabilistic voxel occupancy map with log-odds updates.

    每个体素保存 log-odds ``l = log(p / (1 - p))``、命中计数和最近观测时间：

    - ``add`` 对命中体素做正向更新；
    - ``carve_free`` 沿传感器射线对穿过的旧体素做负向更新；
    - ``prune`` 仍按 TTL 清理长期未命中的体素，作为稀疏射线/遮挡的兜底；
    - ``centers_with_probabilities`` 返回到过占用概率门限的体素及其概率，
      ``centers`` 只返回位置（兼容旧调用）。
    """

    def __init__(
        self,
        voxel_size: float = 0.05,
        min_hits: int = 1,
        ttl: float | None = 2.0,
        *,
        hit_log_odds: float = 0.85,
        miss_log_odds: float = -0.85,
        min_log_odds: float = -2.0,
        max_log_odds: float = 3.5,
        occ_probability: float = 0.65,
    ) -> None:
        if voxel_size <= 0.0:
            raise ValueError(f"voxel_size must be > 0, got {voxel_size}")
        if min_hits < 1:
            raise ValueError(f"min_hits must be >= 1, got {min_hits}")
        if hit_log_odds <= 0.0:
            raise ValueError(f"hit_log_odds must be > 0, got {hit_log_odds}")
        if miss_log_odds >= 0.0:
            raise ValueError(f"miss_log_odds must be < 0, got {miss_log_odds}")
        if min_log_odds >= max_log_odds:
            raise ValueError(f"min_log_odds must be < max_log_odds, got {min_log_odds} >= {max_log_odds}")
        if not 0.0 < occ_probability < 1.0:
            raise ValueError(f"occ_probability must be in (0, 1), got {occ_probability}")
        self.voxel_size = float(voxel_size)
        self.min_hits = int(min_hits)
        self.ttl = ttl
        self.hit_log_odds = float(hit_log_odds)
        self.miss_log_odds = float(miss_log_odds)
        self.min_log_odds = float(min_log_odds)
        self.max_log_odds = float(max_log_odds)
        self.occ_threshold = math.log(occ_probability / (1.0 - occ_probability))
        self._log_odds: dict[tuple[int, int, int], float] = {}
        self._hits: dict[tuple[int, int, int], int] = {}
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
            hits = int(count)
            self._hits[k] = self._hits.get(k, 0) + hits
            update = self.hit_log_odds * min(hits, 4)
            self._log_odds[k] = _clamp(
                self._log_odds.get(k, 0.0) + update,
                self.min_log_odds,
                self.max_log_odds,
            )
            self._last_seen[k] = float(stamp)

    def carve_free(
        self,
        origin: np.ndarray,
        points_world: np.ndarray,
        *,
        max_rays: int = 1200,
        max_steps: int = 32,
    ) -> None:
        """沿 origin → 当前点射线采样，对穿过的旧占用格做 free 更新。

        只采样射线上的体素（不逐点 DDA），够快；端点体素不做 free 更新，避免
        把当前观测到的障碍物清掉。射线很稀疏时由 ``prune`` 的 TTL 兜底。
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
            if k in self._log_odds and k not in endpoint_keys:
                self._log_odds[k] = _clamp(
                    self._log_odds[k] + self.miss_log_odds,
                    self.min_log_odds,
                    self.max_log_odds,
                )

    def prune(self, now: float, ttl: float | None = None) -> None:
        """清理超过 TTL 未再命中的体素；ttl=0/None 表示禁用。"""
        limit = self.ttl if ttl is None else ttl
        if limit is None or limit <= 0.0:
            return
        for k, seen in list(self._last_seen.items()):
            if now - seen > limit:
                self._log_odds.pop(k, None)
                self._hits.pop(k, None)
                self._last_seen.pop(k, None)

    def __len__(self) -> int:
        return len(self._log_odds)

    def centers_with_probabilities(self) -> tuple[np.ndarray, np.ndarray]:
        keys = [
            k
            for k, log_odds in self._log_odds.items()
            if self._hits.get(k, 0) >= self.min_hits and log_odds >= self.occ_threshold
        ]
        if not keys:
            return np.empty((0, 3), dtype=np.float64), np.empty((0,), dtype=np.float64)
        arr = np.asarray(keys, dtype=np.float64)
        centers = (arr + 0.5) * self.voxel_size
        probabilities = np.asarray([_probability(self._log_odds[k]) for k in keys], dtype=np.float64)
        return centers, probabilities

    def centers(self) -> np.ndarray:
        return self.centers_with_probabilities()[0]


def render_voxels(
    centers: np.ndarray,
    *,
    probabilities: np.ndarray | None = None,
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
    """把稀疏体素中心渲染成透视 3D 地图（小方块，颜色/透明度随概率变化）。

    ``probabilities`` 与 ``centers`` 一一对应；缺省时按 1.0 处理，保持旧行为。
    低概率体素更暗、更透明；高概率体素更亮、更实。
    """
    import cv2

    img = np.full((height, width, 3), 22, dtype=np.uint8)
    overlay = np.zeros_like(img)
    alpha_map = np.zeros((height, width), dtype=np.uint8)
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
        if probabilities is None:
            probs = np.ones(len(centers), dtype=np.float64)
        else:
            probs = np.asarray(probabilities, dtype=np.float64).reshape(-1)
            if len(probs) != len(centers):
                raise ValueError(f"probabilities length {len(probs)} != centers length {len(centers)}")
        if len(centers) > 30000:
            step = int(np.ceil(len(centers) / 30000))
            centers = centers[::step]
            probs = probs[::step]
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
            probability = float(np.clip(probs[idx], 0.0, 1.0))
            base = palette[band[idx]]
            color = tuple(int(c * (0.55 + 0.45 * probability)) for c in base)
            opacity = float(np.clip(alpha * (0.35 + 0.65 * probability), 0.05, 1.0))
            half = int(half_px[idx])
            cx, cy = int(u[idx]), int(v[idx])
            x0, x1 = max(cx - half, 0), min(cx + half, width - 1)
            y0, y1 = max(cy - half, 0), min(cy + half, height - 1)
            if x0 > x1 or y0 > y1:
                continue
            cv2.rectangle(overlay, (x0, y0), (x1, y1), color, -1, cv2.LINE_AA)
            cv2.rectangle(
                alpha_map,
                (x0, y0),
                (x1, y1),
                int(round(opacity * 255.0)),
                -1,
                cv2.LINE_AA,
            )
            if half >= 2:
                border = tuple(max(0, int(c * 0.55)) for c in color)
                cv2.rectangle(overlay, (x0, y0), (x1, y1), border, 1, cv2.LINE_AA)
        if alpha_map.any():
            alpha_f = alpha_map[:, :, None].astype(np.float32) / 255.0
            blended = img.astype(np.float32) * (1.0 - alpha_f) + overlay.astype(np.float32) * alpha_f
            img = np.clip(blended, 0.0, 255.0).astype(np.uint8)

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
