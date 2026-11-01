"""Gyro-only rotation tracking unit tests."""

from __future__ import annotations

import numpy as np

from pico_elevationmap.live.imu import GyroTracker, rotation_from_rotvec


def test_rotation_from_rotvec_zero_is_identity() -> None:
    np.testing.assert_allclose(rotation_from_rotvec(np.zeros(3)), np.eye(3), atol=1e-12)


def test_gyro_tracker_integrates_rotation() -> None:
    tracker = GyroTracker(np.eye(3))
    tracker.update(0.0, np.zeros(3))
    tracker.update(0.1, np.array([0.0, 0.0, 1.0]))
    expected = rotation_from_rotvec(np.array([0.0, 0.0, 0.1]))
    np.testing.assert_allclose(tracker.rotation, expected, atol=1e-10)


def test_gyro_tracker_ignores_bad_time_deltas() -> None:
    tracker = GyroTracker(np.eye(3))
    tracker.update(0.0, np.zeros(3))
    baseline = tracker.rotation.copy()
    tracker.update(0.0, np.array([0.0, 0.0, 1.0]))
    tracker.update(1.0, np.array([0.0, 0.0, 1.0]))
    np.testing.assert_allclose(tracker.rotation, baseline, atol=1e-12)


def test_gyro_tracker_stays_orthonormal() -> None:
    tracker = GyroTracker(np.eye(3))
    for i in range(40):
        tracker.update(i * 0.01, np.array([0.01, 0.02, 0.03]))
    _, singular, _ = np.linalg.svd(tracker.rotation)
    np.testing.assert_allclose(singular, np.ones(3), atol=1e-9)
