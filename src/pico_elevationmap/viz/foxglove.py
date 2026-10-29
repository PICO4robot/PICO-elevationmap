"""Optional Foxglove/ROS2 PointCloud2 publisher.

Requires a ROS2-visible environment and an external ``foxglove_bridge``; this module
only publishes standard PointCloud2 topics, so it also works with RViz/rqt.
"""

from __future__ import annotations

import numpy as np

from pico_elevationmap.viz.render import render_layer

__all__ = ["FoxglovePublisher"]


class FoxglovePublisher:
    """Publish elevation points and voxel centers as coloured PointCloud2."""

    def __init__(self, *, node: object | None = None, frame_id: str = "map") -> None:
        import rclpy
        from builtin_interfaces.msg import Time
        from sensor_msgs.msg import PointCloud2, PointField

        self._rclpy = rclpy
        self._PointCloud2 = PointCloud2
        self._PointField = PointField
        self._Time = Time
        self.frame_id = frame_id
        self._owns_context = False
        self._owns_node = node is None
        if node is None:
            if not rclpy.ok():
                rclpy.init()
                self._owns_context = True
            node = rclpy.create_node("pico_elevationmap_foxglove")
        self.node = node
        self._elev_pub = self.node.create_publisher(PointCloud2, "/pico_elevationmap/elevation_points", 10)
        self._voxel_pub = self.node.create_publisher(PointCloud2, "/pico_elevationmap/voxels", 10)

    def _cloud(self, xyz: np.ndarray, rgb: np.ndarray, stamp: float) -> object:
        points = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        valid = np.isfinite(points).all(axis=1)
        points = points[valid]
        colors = np.asarray(rgb, dtype=np.uint8).reshape(-1, 3)[valid]
        n = len(points)
        fields = [
            self._PointField(name="x", offset=0, datatype=7, count=1),
            self._PointField(name="y", offset=4, datatype=7, count=1),
            self._PointField(name="z", offset=8, datatype=7, count=1),
            self._PointField(name="rgb", offset=12, datatype=7, count=1),
        ]
        arr = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
        arr["x"], arr["y"], arr["z"] = points[:, 0], points[:, 1], points[:, 2]
        packed = (
            (colors[:, 0].astype(np.uint32) << 16)
            | (colors[:, 1].astype(np.uint32) << 8)
            | colors[:, 2].astype(np.uint32)
        )
        arr["rgb"] = packed.view("<f4")
        msg = self._PointCloud2()
        msg.header.stamp = self._Time(sec=int(stamp), nanosec=int((stamp % 1.0) * 1e9))
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = n
        msg.fields = fields
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = 16 * n
        msg.data = arr.tobytes()
        msg.is_dense = True
        return msg

    def publish(
        self,
        *,
        elevation: np.ndarray,
        centers: np.ndarray,
        resolution: float,
        center: np.ndarray,
        stamp: float,
    ) -> None:
        import cv2

        layer = np.asarray(elevation, dtype=np.float32)
        n = layer.shape[0]
        rows, cols = np.where(np.isfinite(layer))
        if len(rows):
            x = center[0] + (n / 2.0 - 1.0 - rows) * resolution
            y = center[1] + (n / 2.0 - 1.0 - cols) * resolution
            z = layer[rows, cols]
            color = render_layer(layer, z_min=float(np.nanmin(layer)), z_max=float(np.nanmax(layer)))
            bgr = color[rows, cols]
            rgb = bgr[:, ::-1]
            self._elev_pub.publish(self._cloud(np.stack([x, y, z], axis=1), rgb, stamp))

        voxels = np.asarray(centers, dtype=np.float64).reshape(-1, 3)
        voxels = voxels[np.isfinite(voxels).all(axis=1)]
        if len(voxels):
            lo, hi = np.percentile(voxels[:, 2], [2.0, 98.0])
            span = max(float(np.nanmax(voxels[:, 2]) - np.nanmin(voxels[:, 2])), 1e-6)
            norm = np.clip((voxels[:, 2] - lo) / span, 0.0, 1.0)
            palette = cv2.applyColorMap((norm * 255.0).astype(np.uint8), cv2.COLORMAP_TURBO)
            rgb = palette.reshape(-1, 3)[:, ::-1]
            self._voxel_pub.publish(self._cloud(voxels, rgb, stamp))

    def close(self) -> None:
        if self._owns_node:
            self.node.destroy_node()
        if self._owns_context:
            self._rclpy.shutdown()
