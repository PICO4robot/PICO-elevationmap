"""Small gyro-only rotation tracker for the D435i live demo.

The tracker is intentionally rotation-only: it integrates angular velocity in the
sensor frame and never estimates translation. It is used only when ``--track-gyro``
is explicitly enabled.
"""

from __future__ import annotations

import numpy as np

__all__ = ["GyroTracker", "rotation_from_rotvec"]


def rotation_from_rotvec(rotvec: np.ndarray) -> np.ndarray:
    """Convert a rotation vector to a matrix with Rodrigues' formula."""
    vec = np.asarray(rotvec, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(vec))
    if theta < 1e-12:
        return np.eye(3)
    x, y, z = vec / theta
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]], dtype=np.float64)
    return np.eye(3) + np.sin(theta) * skew + (1.0 - np.cos(theta)) * (skew @ skew)


class GyroTracker:
    """Integrate gyro samples (rad/s, seconds) into a rotation matrix."""

    def __init__(self, initial_rotation: np.ndarray) -> None:
        self.rotation = np.asarray(initial_rotation, dtype=np.float64).reshape(3, 3).copy()
        self._last_stamp: float | None = None
        self._updates = 0

    def update(self, stamp: float, omega: np.ndarray) -> np.ndarray:
        """Integrate one gyro sample and return the current rotation.

        Samples with non-positive or implausibly large time deltas are ignored.
        The rotation is periodically re-orthonormalized to limit numerical drift.
        """
        stamp = float(stamp)
        omega = np.asarray(omega, dtype=np.float64).reshape(3)
        if not np.isfinite(omega).all() or not np.isfinite(stamp):
            return self.rotation
        if self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if 0.0 < dt <= 0.5:
                self.rotation = self.rotation @ rotation_from_rotvec(omega * dt)
                self._updates += 1
                if self._updates % 30 == 0:
                    u, _, vt = np.linalg.svd(self.rotation)
                    self.rotation = u @ vt
        self._last_stamp = stamp
        return self.rotation
