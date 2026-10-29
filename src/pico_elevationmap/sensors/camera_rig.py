"""Multi-camera rig configuration and optical-frame transforms.

The rig is deliberately ROS-free: it maps RealSense optical coordinates to a common
base/map frame using per-camera extrinsics. Sensor discovery/streaming lives in the
scripts layer.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

__all__ = ["CameraRig", "CameraSpec"]

# RealSense optical frame -> base frame for a forward-facing, upright camera:
# x_right -> -y, y_down -> -z, z_forward -> +x.
_FORWARD_OPTICAL = np.array(
    [
        [0.0, 0.0, 1.0],
        [-1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float64,
)


def _rotation_rpy(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    r, p, y = np.radians([roll_deg, pitch_deg, yaw_deg])
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


@dataclass(frozen=True)
class CameraSpec:
    """One RealSense camera in a rig."""

    name: str
    source: str
    serial: str
    xyz: tuple[float, float, float]
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    width: int = 640
    height: int = 480
    fps: int = 15

    @classmethod
    def from_dict(cls, name: str, data: dict[str, Any]) -> CameraSpec:
        if not isinstance(data, dict):
            raise ValueError(f"camera '{name}' must be a mapping")
        allowed = {"serial", "source", "xyz", "roll_deg", "pitch_deg", "yaw_deg", "width", "height", "fps"}
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(f"camera '{name}': unknown keys {sorted(unknown)}")
        serial = str(data.get("serial", "")).strip()
        if not serial:
            raise ValueError(f"camera '{name}': serial is required (use rs-enumerate-devices to list serials)")
        xyz = data.get("xyz", (0.0, 0.0, 0.0))
        if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
            raise ValueError(f"camera '{name}': xyz must be [x, y, z]")
        xyz_t = tuple(float(v) for v in xyz)
        width = int(data.get("width", 640))
        height = int(data.get("height", 480))
        fps = int(data.get("fps", 15))
        if width <= 0 or height <= 0 or fps <= 0:
            raise ValueError(f"camera '{name}': width/height/fps must be positive")
        return cls(
            name=name,
            source=str(data.get("source", name)),
            serial=serial,
            xyz=xyz_t,
            roll_deg=float(data.get("roll_deg", 0.0)),
            pitch_deg=float(data.get("pitch_deg", 0.0)),
            yaw_deg=float(data.get("yaw_deg", 0.0)),
            width=width,
            height=height,
            fps=fps,
        )

    @property
    def rotation(self) -> np.ndarray:
        """R_base_optical: optical x/y/z -> base x/y/z (yaw/pitch/roll)."""
        return _rotation_rpy(self.roll_deg, self.pitch_deg, self.yaw_deg) @ _FORWARD_OPTICAL

    def transform(self, points_optical: np.ndarray) -> np.ndarray:
        """Optical-frame points -> base/map frame."""
        points = np.asarray(points_optical, dtype=np.float64).reshape(-1, 3)
        return (self.rotation @ points.T).T + np.asarray(self.xyz, dtype=np.float64)


@dataclass(frozen=True)
class CameraRig:
    cameras: tuple[CameraSpec, ...]

    @classmethod
    def from_config(
        cls, cameras: dict[str, Any], sources: dict[str, Any], *, only: set[str] | None = None
    ) -> CameraRig:
        if not cameras:
            raise ValueError("'cameras' section is empty; declare at least one camera")
        specs: list[CameraSpec] = []
        for name, data in cameras.items():
            if only is not None and name not in only:
                continue
            spec = CameraSpec.from_dict(str(name), data)
            if spec.source not in sources:
                raise ValueError(f"camera '{name}': source '{spec.source}' not in sources")
            specs.append(spec)
        if not specs:
            raise ValueError("no camera selected; check --cameras")
        return cls(tuple(specs))

    def by_name(self, name: str) -> CameraSpec:
        for cam in self.cameras:
            if cam.name == name:
                return cam
        raise KeyError(f"unknown camera '{name}', available: {[c.name for c in self.cameras]}")
