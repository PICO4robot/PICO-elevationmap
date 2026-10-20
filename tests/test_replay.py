"""replay 管线验证：npz 帧序列与 rosbag 的端到端回放（GPU）+ bag 解析（CPU）。

bag 由 rosbags 自带 writer 现场生成（ROS1 .bag，PointCloud2 + PoseStamped），
无需安装 ROS。
"""

from __future__ import annotations

import numpy as np
import pytest

from pico_elevationmap.core import io


def _cupy_cuda_available() -> bool:
    try:
        import cupy as cp

        return bool(cp.cuda.is_available())
    except Exception:
        return False


pytestmark = pytest.mark.skipif(not _cupy_cuda_available(), reason="cupy/CUDA not available")


@pytest.fixture(scope="module")
def mapper_mod():
    pytest.importorskip("cupy", reason="cupy not installed")
    from pico_elevationmap.core import mapper as mapper_module

    return mapper_module


def _ground_frames(n_frames: int = 3) -> list[io.PointCloudFrame]:
    """平地网格点云，传感器位姿 (0,0,1) 俯视。"""
    xs = np.arange(-1.0, 1.0, 0.05)
    X, Y = np.meshgrid(xs, xs)
    world = np.stack([X.ravel(), Y.ravel(), np.zeros(X.size)], axis=1)
    sensor = (world - np.array([0.0, 0.0, 1.0])).astype(np.float32)
    pose = np.eye(4)
    pose[2, 3] = 1.0
    return [
        io.PointCloudFrame(points=sensor, stamp=i * 0.1, pose=pose.copy(), source="cam")
        for i in range(n_frames)
    ]


# ── rosbag 生成 helper（写 ROS1 .bag，PointCloud2 xyz float32 + PoseStamped）───


def _write_rosbag1(path, frames: list[io.PointCloudFrame]) -> None:
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    PointCloud2 = ts.types["sensor_msgs/msg/PointCloud2"]
    PointField = ts.types["sensor_msgs/msg/PointField"]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    PoseStamped = ts.types["geometry_msgs/msg/PoseStamped"]
    Pose = ts.types["geometry_msgs/msg/Pose"]
    Point = ts.types["geometry_msgs/msg/Point"]
    Quaternion = ts.types["geometry_msgs/msg/Quaternion"]
    fields = [
        PointField(name="x", offset=0, datatype=7, count=1),
        PointField(name="y", offset=4, datatype=7, count=1),
        PointField(name="z", offset=8, datatype=7, count=1),
    ]

    def sec_of(stamp: float) -> int:
        return int(stamp)

    def nsec_of(stamp: float) -> int:
        return int(round((stamp - int(stamp)) * 1e9))

    with Writer(path) as writer:
        cloud_conn = writer.add_connection("/robot/points", "sensor_msgs/msg/PointCloud2", typestore=ts)
        pose_conn = writer.add_connection("/robot/pose", "geometry_msgs/msg/PoseStamped", typestore=ts)
        for i, frame in enumerate(frames):
            pose = frame.pose if frame.pose is not None else np.eye(4)
            pmsg = PoseStamped(
                header=Header(seq=i, stamp=Time(sec=sec_of(frame.stamp), nanosec=0), frame_id="map"),
                pose=Pose(
                    position=Point(float(pose[0, 3]), float(pose[1, 3]), float(pose[2, 3])),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                ),
            )
            # 位姿先于同 stamp 的点云写入：iter_bag_frames 取"最近一个 ≤ 点云 stamp"的位姿
            writer.write(pose_conn, int(frame.stamp * 1e9), ts.serialize_ros1(pmsg, pmsg.__msgtype__))
            pts = frame.points.astype(np.float32)
            data = np.zeros(len(pts), dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
            data["x"], data["y"], data["z"] = pts[:, 0], pts[:, 1], pts[:, 2]
            msg = PointCloud2(
                header=Header(seq=i, stamp=Time(sec=sec_of(frame.stamp), nanosec=nsec_of(frame.stamp)), frame_id="map"),
                height=1,
                width=len(pts),
                fields=fields,
                is_bigendian=False,
                point_step=12,
                row_step=12 * len(pts),
                data=data.view(np.uint8),
                is_dense=True,
            )
            writer.write(cloud_conn, int(frame.stamp * 1e9), ts.serialize_ros1(msg, msg.__msgtype__))


# ── CPU：bag 解析（不触 GPU）───────────────────────────────────────────────────


def test_bag_parse_round_trip_cpu(tmp_path) -> None:
    frames = _ground_frames(2)
    bag = tmp_path / "test.bag"
    _write_rosbag1(bag, frames)
    got = list(io.iter_bag_frames(bag))
    assert len(got) == 2
    for orig, g in zip(frames, got):
        assert g.stamp == pytest.approx(orig.stamp)
        assert len(g.points) == len(orig.points)
        np.testing.assert_allclose(g.points, orig.points, atol=1e-6)
        assert g.pose is not None  # bag 里写了 pose
        np.testing.assert_allclose(g.pose[:3, 3], orig.pose[:3, 3], atol=1e-6)
        assert g.source == "points"  # topic 末段 /robot/points


# ── GPU：端到端 replay ──────────────────────────────────────────────────────────


def _replay_args(input_path, config_path, tmp_path, extra: list[str]) -> list[str]:
    # 直接调用 replay.main()（不经 cli 分发），argv 不含子命令名
    return [
        "pico-elevationmap replay",
        "--config",
        str(config_path),
        "--input",
        str(input_path),
        "--save",
        str(tmp_path / "map.npz"),
        *extra,
    ]


def _write_config(tmp_path, mapper_mod) -> object:
    path = tmp_path / "replay.yaml"
    path.write_text(
        "map:\n  resolution: 0.05\n  map_length: 4.0\nsources:\n  cam:\n    sensor_type: perfect\n",
        encoding="utf-8",
    )
    return path


def test_replay_npz_end_to_end(gpu, mapper_mod, tmp_path, monkeypatch, capsys) -> None:
    import scripts.pipeline.replay as replay

    frames_path = tmp_path / "frames.npz"
    io.save_frames(frames_path, _ground_frames(3))
    config_path = _write_config(tmp_path, mapper_mod)
    monkeypatch.setattr(
        "sys.argv", _replay_args(frames_path, config_path, tmp_path, ["--export-ply", str(tmp_path / "map.ply")])
    )
    replay.main()

    out = capsys.readouterr().out
    assert "map stats" in out and "coverage" in out
    layers, meta = io.load_map(tmp_path / "map.npz")
    assert "elevation" in layers
    assert meta["resolution"] == pytest.approx(0.05)
    ply_head = (tmp_path / "map.ply").read_text(encoding="utf-8").splitlines()[:8]
    assert ply_head[0] == "ply"
    assert any(ln.startswith("element vertex") for ln in ply_head)


def test_replay_bag_end_to_end(gpu, mapper_mod, tmp_path, monkeypatch, capsys) -> None:
    import scripts.pipeline.replay as replay

    bag = tmp_path / "test.bag"
    _write_rosbag1(bag, _ground_frames(2))
    config_path = _write_config(tmp_path, mapper_mod)
    monkeypatch.setattr("sys.argv", _replay_args(bag, config_path, tmp_path, []))
    replay.main()
    out = capsys.readouterr().out
    assert "frame 2/2" in out
    layers, _ = io.load_map(tmp_path / "map.npz")
    elev = layers["elevation"]
    assert np.isfinite(elev).sum() > 0
    assert abs(float(np.nanmean(elev))) < 0.02  # 平地真值 0


def test_replay_max_frames(gpu, mapper_mod, tmp_path, monkeypatch, capsys) -> None:
    import scripts.pipeline.replay as replay

    frames_path = tmp_path / "frames.npz"
    io.save_frames(frames_path, _ground_frames(5))
    config_path = _write_config(tmp_path, mapper_mod)
    monkeypatch.setattr(
        "sys.argv", _replay_args(frames_path, config_path, tmp_path, ["--max-frames", "2"])
    )
    replay.main()
    out = capsys.readouterr().out
    assert "frame 2/2" in out
