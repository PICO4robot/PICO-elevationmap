"""Scaffold smoke tests: importable packages, CLI dispatcher, version."""

from __future__ import annotations

import importlib
import importlib.metadata
from importlib import resources

import pytest

import pico_elevationmap.cli as cli

SUBCOMMANDS = ("run", "replay", "bench", "export")
# Phase 2: run / replay / export 已实现；bench 待 Phase 3（torch 后端）仍无实现模块
STUB_SUBCOMMANDS = ("bench",)
PACKAGES = (
    "pico_elevationmap",
    "pico_elevationmap.cli",
    "pico_elevationmap.core",
    "pico_elevationmap.fusion",
    "pico_elevationmap.plugins",
    "pico_elevationmap.backends",
    "pico_elevationmap.sensors",
    "pico_elevationmap.viz",
    "pico_elevationmap.pipeline",
    "pico_elevationmap.live",
    "pico_elevationmap.configs",
)


def test_packages_importable() -> None:
    for name in PACKAGES:
        importlib.import_module(name)


def test_bundled_configs_discoverable() -> None:
    config_dir = resources.files("pico_elevationmap.configs")
    assert config_dir.is_dir()
    names = {entry.name for entry in config_dir.iterdir() if entry.name.endswith(".yaml")}
    assert {"run_sim.yaml", "ros_camera_lidar.yaml"} <= names


def test_cli_lists_subcommands(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setattr("sys.argv", ["pico-elevationmap", "--help"])
    with pytest.raises(SystemExit) as exc_info:
        cli.main()
    assert exc_info.value.code == 0
    out = capsys.readouterr().out
    for cmd in SUBCOMMANDS:
        assert cmd in out


@pytest.mark.parametrize("cmd", STUB_SUBCOMMANDS)
def test_cli_stubs_exit_nonzero(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], cmd: str) -> None:
    monkeypatch.setattr("sys.argv", ["pico-elevationmap", cmd, "--config", "dummy.yaml"])
    with pytest.raises(SystemExit) as exc_info:
        cli.main()
    assert exc_info.value.code == 1
    err = capsys.readouterr().err
    assert "not implemented" in err
    assert "Phase 3" in err


def test_cli_run_help_exit_zero(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    """已实现子命令的 --help 在无 GPU 环境也必须可用（GPU 栈为懒加载）。"""
    expected_flag = {"run": "--config", "replay": "--input", "export": "--input"}
    for cmd, flag in expected_flag.items():
        monkeypatch.setattr("sys.argv", ["pico-elevationmap", cmd, "--help"])
        with pytest.raises(SystemExit) as exc_info:
            cli.main()
        assert exc_info.value.code == 0
        assert flag in capsys.readouterr().out


def test_cli_unknown_command_exits_2(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setattr("sys.argv", ["pico-elevationmap", "bogus"])
    with pytest.raises(SystemExit) as exc_info:
        cli.main()
    assert exc_info.value.code == 2
    assert "unknown command" in capsys.readouterr().err


def test_version() -> None:
    assert importlib.metadata.version("pico-elevationmap") == "0.4.0"


def test_synthetic_pointcloud_fixture(synthetic_pointcloud: list[tuple[float, float, float]]) -> None:
    assert len(synthetic_pointcloud) == 200
    xs = [p[0] for p in synthetic_pointcloud]
    assert min(xs) >= -1.0
    assert max(xs) <= 1.0
