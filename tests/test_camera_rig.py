"""Multi-camera rig config and camera-optical → base transform tests."""

from __future__ import annotations

import numpy as np
import pytest

from pico_elevationmap.core.mapper import MapperConfig
from pico_elevationmap.sensors.camera_rig import CameraRig, CameraSpec


def _front(**kwargs: object) -> CameraSpec:
    data = {
        "serial": "front-serial",
        "source": "cam_front",
        "xyz": [0.0, 0.0, 0.0],
        "width": 640,
        "height": 480,
        "fps": 15,
    }
    data.update(kwargs)
    return CameraSpec.from_dict("front", data)


def test_front_camera_optical_axes() -> None:
    spec = _front()
    points = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])
    world = spec.transform(points)
    np.testing.assert_allclose(world[0], [0.0, -1.0, 0.0], atol=1e-12)  # right -> -y
    np.testing.assert_allclose(world[1], [0.0, 0.0, -1.0], atol=1e-12)  # down -> -z
    np.testing.assert_allclose(world[2], [1.0, 0.0, 0.0], atol=1e-12)  # forward -> +x


def test_back_and_side_yaw() -> None:
    back = CameraSpec.from_dict("back", {"serial": "b", "source": "cam_back", "yaw_deg": 180.0})
    np.testing.assert_allclose(back.transform(np.array([[0.0, 0.0, 1.0]]))[0], [-1.0, 0.0, 0.0], atol=1e-12)
    left = CameraSpec.from_dict("left", {"serial": "l", "source": "cam_left", "yaw_deg": 90.0})
    np.testing.assert_allclose(left.transform(np.array([[0.0, 0.0, 1.0]]))[0], [0.0, 1.0, 0.0], atol=1e-12)
    right = CameraSpec.from_dict("right", {"serial": "r", "source": "cam_right", "yaw_deg": -90.0})
    np.testing.assert_allclose(right.transform(np.array([[0.0, 0.0, 1.0]]))[0], [0.0, -1.0, 0.0], atol=1e-12)


def test_pitch_tilts_forward_down() -> None:
    spec = _front(pitch_deg=30.0)
    forward = spec.transform(np.array([[0.0, 0.0, 1.0]]))[0]
    assert forward[0] == pytest.approx(np.cos(np.deg2rad(30.0)))
    assert forward[2] == pytest.approx(-np.sin(np.deg2rad(30.0)))


def test_quad_config_parses() -> None:
    cfg = MapperConfig.from_yaml("src/pico_elevationmap/configs/quad_camera.yaml")
    rig = CameraRig.from_config(cfg.cameras, cfg.sources)
    assert [c.name for c in rig.cameras] == ["front", "back", "left", "right"]
    assert {c.source for c in rig.cameras} == {"cam_front", "cam_back", "cam_left", "cam_right"}


def test_camera_subset_and_validation() -> None:
    cfg = MapperConfig.from_yaml("src/pico_elevationmap/configs/quad_camera.yaml")
    rig = CameraRig.from_config(cfg.cameras, cfg.sources, only={"front"})
    assert [c.name for c in rig.cameras] == ["front"]
    with pytest.raises(ValueError, match="unknown keys"):
        CameraSpec.from_dict("bad", {"serial": "x", "source": "cam_front", "bogus": 1})
    with pytest.raises(ValueError, match="serial is required"):
        CameraSpec.from_dict("bad", {"source": "cam_front"})
