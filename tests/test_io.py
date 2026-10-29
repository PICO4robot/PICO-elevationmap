"""io 模块 round-trip：帧序列、地图 npz、PLY/PGM/CSV 写出 —— 纯 CPU。"""

from __future__ import annotations

import numpy as np
import pytest

from pico_elevationmap.core import io


def _frames() -> list[io.PointCloudFrame]:
    rng = np.random.default_rng(0)
    return [
        io.PointCloudFrame(points=rng.normal(size=(10, 3)).astype(np.float32), stamp=0.0, pose=None, source="cam"),
        io.PointCloudFrame(points=rng.normal(size=(7, 3)).astype(np.float32), stamp=0.1, pose=np.eye(4), source="cam"),
        io.PointCloudFrame(points=rng.normal(size=(5, 3)).astype(np.float32), stamp=0.2, pose=None, source="lidar"),
    ]


def test_frames_round_trip(tmp_path) -> None:
    path = tmp_path / "frames.npz"
    frames = _frames()
    io.save_frames(path, frames)
    loaded = io.load_frames(path)
    assert len(loaded) == 3
    for orig, got in zip(frames, loaded):
        assert got.stamp == pytest.approx(orig.stamp)
        assert got.source == orig.source
        np.testing.assert_array_equal(got.points, orig.points)
        if orig.pose is None:
            assert got.pose is None
        else:
            np.testing.assert_allclose(got.pose, orig.pose)


def test_per_frame_dir(tmp_path) -> None:
    for i, f in enumerate(_frames()):
        payload = {"points": f.points, "stamp": f.stamp}
        if f.pose is not None:
            payload["pose"] = f.pose
        np.savez(tmp_path / f"frame_{i:04d}.npz", **payload)
    files = io.frame_files_from_dir(tmp_path)
    assert len(files) == 3
    frame = io.load_frame_file(files[0])
    assert len(frame.points) == 10 and frame.pose is None


def test_frame_dir_empty_raises(tmp_path) -> None:
    with pytest.raises(ValueError, match="no .npz"):
        io.frame_files_from_dir(tmp_path)


def test_map_round_trip(tmp_path) -> None:
    path = tmp_path / "map.npz"
    layers = {"elevation": np.full((4, 4), 0.5, dtype=np.float32), "variance": np.ones((4, 4), dtype=np.float32)}
    io.save_map(path, layers, resolution=0.1, center=[1.0, 2.0, 0.0])
    got_layers, meta = io.load_map(path)
    assert set(got_layers) == set(layers)
    np.testing.assert_array_equal(got_layers["elevation"], layers["elevation"])
    assert meta["resolution"] == pytest.approx(0.1)
    assert meta["center"] == [pytest.approx(1.0), pytest.approx(2.0), pytest.approx(0.0)]


def _small_layer() -> np.ndarray:
    layer = np.full((3, 4), 0.25, dtype=np.float32)
    layer[0, 0] = np.nan  # 一个洞
    layer[2, 3] = 1.5
    return layer


def test_write_ply(tmp_path) -> None:
    path = tmp_path / "out.ply"
    n = io.write_ply(path, _small_layer(), resolution=0.1, center=[0.0, 0.0, 0.0])
    assert n == 11  # 12 cells - 1 NaN
    text = path.read_text(encoding="utf-8").splitlines()
    assert text[0] == "ply"
    vertex_lines = [ln for ln in text if ln and not ln[0].isalpha()]
    assert len(vertex_lines) == n
    # 高度极值进入色标范围（注释行记录）
    assert any("hmin=0.25" in ln or "hmin=" in ln for ln in text[:6])


def test_write_pgm(tmp_path) -> None:
    path = tmp_path / "out.pgm"
    hmin, hmax = io.write_pgm(path, _small_layer())
    assert (hmin, hmax) == (pytest.approx(0.25), pytest.approx(1.5))
    tokens = []
    for ln in path.read_text(encoding="utf-8").splitlines():
        if ln.startswith("#"):
            continue
        tokens.extend(ln.split())
    assert tokens[0] == "P2"
    assert tokens[1:4] == ["4", "3", "65535"]  # 宽 高 maxval
    vals = [int(t) for t in tokens[4:]]
    assert len(vals) == 12
    assert min(vals) == 0 and max(vals) == 65535  # NaN=0，hmax 满量程


def test_write_csv(tmp_path) -> None:
    path = tmp_path / "out.csv"
    io.write_csv(path, _small_layer())
    arr = np.loadtxt(path, delimiter=",")
    assert arr.shape == (3, 4)


def test_grid_coordinates_convention() -> None:
    X, Y = io.grid_coordinates((2, 2), resolution=1.0, center=[0.0, 0.0, 0.0])
    # exported[row]: x = cx + (n/2 - 1 - row)·res → {0, -1}
    np.testing.assert_allclose(X[:, 0], [0.0, -1.0])
    np.testing.assert_allclose(Y[0, :], [0.0, -1.0])


def test_save_map_creates_parent_directories(tmp_path) -> None:
    path = tmp_path / "new" / "nested" / "map.npz"
    layers = {"elevation": np.zeros((2, 2), dtype=np.float32)}
    io.save_map(path, layers, resolution=0.1, center=[0.0, 0.0, 0.0])
    assert path.is_file()
    loaded, meta = io.load_map(path)
    assert set(loaded) == {"elevation"}
    assert meta["resolution"] == pytest.approx(0.1)


def test_save_frames_creates_parent_directories(tmp_path) -> None:
    path = tmp_path / "new" / "nested" / "frames.npz"
    io.save_frames(path, _frames())
    assert path.is_file()
    assert len(io.load_frames(path)) == 3
