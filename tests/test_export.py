"""export 管线验证：npz → PLY / PGM / CSV / 多层导出 —— 纯 CPU。"""

from __future__ import annotations

import numpy as np
import pytest

from pico_elevationmap.core import io
from pico_elevationmap.pipeline import export as export_mod


def _map_npz(tmp_path) -> object:
    path = tmp_path / "map.npz"
    elev = np.full((4, 5), 0.5, dtype=np.float32)
    elev[0, 0] = np.nan
    elev[3, 4] = 2.0
    layers = {"elevation": elev, "variance": np.ones((4, 5), dtype=np.float32)}
    io.save_map(path, layers, resolution=0.1, center=[0.5, 0.5, 0.0])
    return path


def _argv(tmp_path, *extra: str) -> list[str]:
    # 直接调用 export.main()（不经 cli 分发），argv 不含子命令名
    return ["pico-elevationmap export", "--input", str(_map_npz(tmp_path)), *extra]


def test_export_list(tmp_path, monkeypatch, capsys) -> None:
    monkeypatch.setattr("sys.argv", _argv(tmp_path, "--list"))
    export_mod.main()
    out = capsys.readouterr().out
    assert "elevation" in out and "variance" in out and "resolution=0.1" in out


def test_export_ply(tmp_path, monkeypatch, capsys) -> None:
    out_path = tmp_path / "out.ply"
    monkeypatch.setattr("sys.argv", _argv(tmp_path, "--output", str(out_path), "--format", "ply"))
    export_mod.main()
    assert "19 vertices" in capsys.readouterr().out  # 20 cells - 1 NaN
    lines = out_path.read_text(encoding="utf-8").splitlines()
    assert lines[0] == "ply"


def test_export_pgm(tmp_path, monkeypatch, capsys) -> None:
    out_path = tmp_path / "out.pgm"
    monkeypatch.setattr("sys.argv", _argv(tmp_path, "--output", str(out_path)))  # 后缀推断格式
    export_mod.main()
    assert "h ∈ [0.500, 2.000]" in capsys.readouterr().out
    text = out_path.read_text(encoding="utf-8")
    assert text.startswith("P2")


def test_export_csv_multi_layer(tmp_path, monkeypatch, capsys) -> None:
    out_path = tmp_path / "out.csv"
    monkeypatch.setattr("sys.argv", _argv(tmp_path, "--output", str(out_path), "--layers", "elevation,variance"))
    export_mod.main()
    out = capsys.readouterr().out
    assert "out_elevation.csv" in out and "out_variance.csv" in out
    assert np.loadtxt(tmp_path / "out_elevation.csv", delimiter=",").shape == (4, 5)


def test_export_unknown_layer_raises(tmp_path, monkeypatch) -> None:
    monkeypatch.setattr("sys.argv", _argv(tmp_path, "--format", "csv", "--layers", "nope"))
    with pytest.raises(KeyError, match="nope"):
        export_mod.main()


def test_export_missing_meta_raises(tmp_path, monkeypatch) -> None:
    path = tmp_path / "bare.npz"
    np.savez(path, elevation=np.zeros((3, 3), dtype=np.float32))
    monkeypatch.setattr("sys.argv", ["pico-elevationmap export", "--input", str(path), "--format", "csv"])
    with pytest.raises(ValueError, match="resolution"):
        export_mod.main()
