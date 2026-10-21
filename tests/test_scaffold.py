"""Phase 0 scaffold smoke tests: packages importable, CLI dispatcher, version."""

from __future__ import annotations

import importlib
import importlib.metadata

import pytest

import scripts.cli as cli

SUBCOMMANDS = ("run", "replay", "bench", "export")
PACKAGES = (
    "pico_elevationmap",
    "pico_elevationmap.core",
    "pico_elevationmap.fusion",
    "pico_elevationmap.plugins",
    "pico_elevationmap.backends",
    "pico_elevationmap.sensors",
    "scripts",
)


def test_packages_importable() -> None:
    for name in PACKAGES:
        importlib.import_module(name)


def test_cli_lists_subcommands(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setattr("sys.argv", ["pico-elevationmap", "--help"])
    with pytest.raises(SystemExit) as exc_info:
        cli.main()
    assert exc_info.value.code == 0
    out = capsys.readouterr().out
    for cmd in SUBCOMMANDS:
        assert cmd in out


@pytest.mark.parametrize("cmd", SUBCOMMANDS)
def test_cli_stubs_exit_zero(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str], cmd: str
) -> None:
    monkeypatch.setattr("sys.argv", ["pico-elevationmap", cmd, "--config", "dummy.yaml"])
    with pytest.raises(SystemExit) as exc_info:
        cli.main()
    assert exc_info.value.code == 0
    out = capsys.readouterr().out
    assert "not implemented yet" in out
    assert "Phase" in out


def test_cli_unknown_command_exits_2(monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]) -> None:
    monkeypatch.setattr("sys.argv", ["pico-elevationmap", "bogus"])
    with pytest.raises(SystemExit) as exc_info:
        cli.main()
    assert exc_info.value.code == 2
    assert "unknown command" in capsys.readouterr().err


def test_version() -> None:
    assert importlib.metadata.version("pico-elevationmap") == "0.1.0"


def test_synthetic_pointcloud_fixture(synthetic_pointcloud: list[tuple[float, float, float]]) -> None:
    assert len(synthetic_pointcloud) == 200
    xs = [p[0] for p in synthetic_pointcloud]
    assert min(xs) >= -1.0
    assert max(xs) <= 1.0
