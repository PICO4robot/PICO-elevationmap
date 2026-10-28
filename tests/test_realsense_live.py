"""RealSense 深度图 → 点云 / 平面姿态的纯函数测试（无需相机）。"""

from __future__ import annotations

import numpy as np
import pytest

from scripts.tools.realsense_live import (
    VoxelMap,
    deproject_depth,
    estimate_ground,
    pose_from_ground,
    render_layer,
    render_voxels,
    rotation_from_up,
)


def test_deproject_center_and_offset() -> None:
    depth = np.zeros((3, 3), dtype=np.float32)
    depth[1, 1] = 2.0
    depth[1, 2] = 2.0

    points = deproject_depth(depth, fx=100.0, fy=100.0, ppx=1.0, ppy=1.0, min_range=0.1, max_range=10.0)

    assert points.shape == (2, 3)
    np.testing.assert_allclose(points[0], [0.0, 0.0, 2.0], atol=1e-6)
    np.testing.assert_allclose(points[1], [0.02, 0.0, 2.0], atol=1e-6)


def test_deproject_filters_invalid_and_respects_range() -> None:
    depth = np.array([[0.0, 0.05, 12.0], [1.0, np.nan, 2.0]], dtype=np.float32)
    points = deproject_depth(depth, fx=100.0, fy=100.0, ppx=1.0, ppy=0.5, min_range=0.1, max_range=10.0)
    assert points.shape == (2, 3)
    np.testing.assert_allclose(points[:, 2], [1.0, 2.0], atol=1e-6)


def test_deproject_stride_keeps_original_pixel_coordinates() -> None:
    depth = np.full((5, 5), 2.0, dtype=np.float32)
    points = deproject_depth(
        depth, fx=100.0, fy=100.0, ppx=2.0, ppy=2.0, stride=2, min_range=0.1, max_range=10.0
    )
    assert points.shape == (9, 3)
    np.testing.assert_allclose(points[4], [0.0, 0.0, 2.0], atol=1e-6)


def test_rotation_from_up_maps_up_to_z() -> None:
    up = np.array([0.0, -1.0, 0.0])
    R = rotation_from_up(up)
    np.testing.assert_allclose(R @ up, [0.0, 0.0, 1.0], atol=1e-12)
    np.testing.assert_allclose(R @ R.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(R) == pytest.approx(1.0)


def test_estimate_ground_on_horizontal_plane() -> None:
    rng = np.random.default_rng(0)
    xy = rng.uniform(-2.0, 2.0, size=(2000, 2))
    points = np.column_stack([xy[:, 0], np.full(len(xy), 0.5), xy[:, 1]])  # 光学系 y=0.5 平面
    up, height, ratio = estimate_ground(points, max_range=3.0)
    np.testing.assert_allclose(up, [0.0, -1.0, 0.0], atol=0.05)
    assert height == pytest.approx(0.5, abs=0.02)
    assert ratio > 0.8


def test_pose_from_ground_puts_plane_at_zero() -> None:
    R, T = pose_from_ground(np.array([0.0, -1.0, 0.0]), 0.5)
    p = np.array([1.0, 0.5, 2.0])  # 平面 y=0.5 上一点
    p_map = R @ p + T[:3, 3]
    assert p_map[2] == pytest.approx(0.0, abs=1e-12)


def test_render_layer_marks_invalid_cells() -> None:
    layer = np.array([[0.0, np.nan], [1.0, 2.0]], dtype=np.float32)
    img = render_layer(layer, colormap="turbo")
    assert img.shape == (2, 2, 3)
    assert img.dtype == np.uint8
    np.testing.assert_array_equal(img[0, 1], [25, 25, 25])


def test_voxel_map_quantizes_and_applies_min_hits() -> None:
    voxels = VoxelMap(voxel_size=0.1, min_hits=2)
    voxels.add(np.array([[0.01, 0.01, 0.01], [0.02, 0.02, 0.02], [0.11, 0.0, 0.0]]))
    assert len(voxels) == 2
    centers = voxels.centers()
    assert centers.shape == (1, 3)
    np.testing.assert_allclose(centers[0], [0.05, 0.05, 0.05], atol=1e-12)


def test_voxel_map_ignores_nan() -> None:
    voxels = VoxelMap(voxel_size=0.1)
    voxels.add(np.array([[np.nan, 0.0, 0.0], [0.0, 0.0, 0.0]]))
    assert len(voxels) == 1


def test_voxel_map_prunes_unseen_voxels() -> None:
    voxels = VoxelMap(voxel_size=0.1, ttl=2.0)
    voxels.add(np.array([[0.0, 0.0, 0.0]]), stamp=0.0)
    voxels.prune(1.0)
    assert len(voxels) == 1
    voxels.prune(2.5)
    assert len(voxels) == 0


def test_voxel_map_carves_free_space_but_keeps_endpoint() -> None:
    voxels = VoxelMap(voxel_size=0.1, ttl=0)
    voxels.add(np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]), stamp=0.0)
    voxels.carve_free(np.zeros(3), np.array([[2.0, 0.0, 0.0]]), max_rays=1, max_steps=32)
    centers = voxels.centers()
    assert centers.shape == (1, 3)
    np.testing.assert_allclose(centers[0], [2.05, 0.05, 0.05], atol=1e-12)


def test_render_voxels_returns_image() -> None:
    centers = np.array([[0.0, 0.0, 0.0], [0.2, 0.1, 0.3]], dtype=np.float64)
    img = render_voxels(
        centers,
        z_min=-0.1,
        z_max=1.0,
        target=np.zeros(3),
        yaw=0.8,
        pitch=0.4,
        distance=3.0,
        voxel_size=0.05,
    )
    assert img.shape == (520, 720, 3)
    assert img.dtype == np.uint8
    assert img.std() > 0.0
