"""SensorSourceConfig：YAML 加载校验、噪声系数到上游参数的映射 —— 纯 CPU。"""

from __future__ import annotations

import textwrap

import numpy as np
import pytest

from pico_elevationmap.sensors.config import (
    PERFECT_NOISE_FACTOR,
    SensorSourceConfig,
    load_sources,
    parse_sources,
)

_VALID_YAML = textwrap.dedent(
    """\
    sources:
      lidar:
        sensor_type: laser
        noise_coefficient: 0.01
        min_range: 0.5
        max_range: 30.0
      depth_cam:
        sensor_type: depth_camera
        noise_coefficient: 0.002
        min_range: 0.3
        max_range: 8.0
      sim_gt:
        sensor_type: perfect
    """
)


def test_parse_valid_yaml(tmp_path) -> None:
    path = tmp_path / "sources.yaml"
    path.write_text(_VALID_YAML, encoding="utf-8")
    sources = load_sources(path)
    assert set(sources) == {"lidar", "depth_cam", "sim_gt"}
    lidar = sources["lidar"]
    assert lidar.min_range == 0.5 and lidar.max_range == 30.0


def test_laser_factor_is_coeff_squared() -> None:
    src = SensorSourceConfig(name="l", sensor_type="laser", noise_coefficient=0.01, max_range=30.0)
    # std = c·d ⇒ var = c²·d²，上游 factor 取 c²（精确匹配）
    assert src.upstream_noise_factor == pytest.approx(1e-4)
    np.testing.assert_allclose(src.measurement_std(np.array([2.0])), [0.02])


def test_depth_camera_factor_is_range_upper_bound() -> None:
    src = SensorSourceConfig(name="d", sensor_type="depth_camera", noise_coefficient=0.002, max_range=8.0)
    # var = c²·d⁴ 以内核 factor·d² 上界近似：factor = (c·r_max)²
    assert src.upstream_noise_factor == pytest.approx((0.002 * 8.0) ** 2)
    np.testing.assert_allclose(src.measurement_std(np.array([2.0])), [0.002 * 4.0])


def test_perfect_source() -> None:
    src = SensorSourceConfig(name="p", sensor_type="perfect")
    assert src.upstream_noise_factor == PERFECT_NOISE_FACTOR
    assert src.measurement_std(np.array([1.0, 5.0])).shape == (2,)
    assert float(src.measurement_std(np.array([5.0]))[0]) == 0.0


@pytest.mark.parametrize(
    "kwargs, match",
    [
        ({"sensor_type": "stereo"}, "unknown sensor_type"),
        ({"sensor_type": "laser", "noise_coefficient": 0.0}, "noise_coefficient > 0"),
        ({"sensor_type": "perfect", "noise_coefficient": 0.1}, "perfect sensors"),
        ({"sensor_type": "laser", "noise_coefficient": 0.1, "min_range": 0.0}, "min_range"),
        ({"sensor_type": "laser", "noise_coefficient": 0.1, "min_range": 5.0, "max_range": 5.0}, "max_range"),
        ({"sensor_type": "laser", "noise_coefficient": -0.1}, "non-negative"),
    ],
)
def test_validation_errors(kwargs, match: str) -> None:
    kwargs.setdefault("noise_coefficient", 0.1)
    with pytest.raises(ValueError, match=match):
        SensorSourceConfig(name="bad", **kwargs)


def test_unknown_yaml_keys_rejected() -> None:
    with pytest.raises(ValueError, match="unknown keys"):
        parse_sources({"sources": {"s": {"sensor_type": "perfect", "typo_key": 1}}})


def test_missing_or_malformed_sections() -> None:
    with pytest.raises(ValueError, match="'sources'"):
        parse_sources({"map": {}})
    with pytest.raises(ValueError, match="empty"):
        parse_sources({"sources": {}})
    with pytest.raises(ValueError, match="mapping"):
        parse_sources({"sources": {"s": "laser"}})


def test_unknown_fields_in_yaml(tmp_path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        "sources:\n  s:\n    sensor_type: perfect\n    resolution: 0.04\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unknown keys"):
        load_sources(path)
