"""ROS2 桥接验证：消息转换（纯 CPU）、GridMessage 打包、节点接线（sys.modules 桩）。

转换层 messages.py 零 ROS 依赖，直接以 SimpleNamespace 鸭子类型消息测试；
node.py 由 tests/ros_stubs.py 在 sys.modules 层打桩 rclpy/消息包，mapper 走
依赖注入 Mock —— 全部 CPU 可跑。真实 ROS2 集成测试见 requires_ros 标记用例
（本机无 ROS，自动 skip）。
"""

from __future__ import annotations

import importlib
from types import SimpleNamespace
from unittest import mock

import numpy as np
import pytest

from pico_elevationmap.core import io
from pico_elevationmap.core.pose_buffer import StampOutOfRangeError
from pico_elevationmap.sensors.config import SensorSourceConfig
from pico_elevationmap_ros import messages
from pico_elevationmap_ros.config import RosConfig
from tests import ros_stubs


@pytest.fixture(scope="module")
def node_mod():
    """打桩后导入 node 模块（首次导入即绑定桩，后续复用）。"""
    ros_stubs.install()
    return importlib.import_module("pico_elevationmap_ros.node")


# ── 消息转换：PointCloud2 → numpy ──────────────────────────────────────────────


def _pointcloud2_msg(points: np.ndarray, stamp_sec=12, stamp_nsec=500_000_000, frame_id="lidar"):
    """按 ROS 形状构造 PointCloud2（xyz float32，12 字节步长）。"""
    n = len(points)
    data = np.zeros(n, dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4")])
    data["x"], data["y"], data["z"] = points[:, 0], points[:, 1], points[:, 2]
    fields = [
        SimpleNamespace(name="x", offset=0, datatype=7),
        SimpleNamespace(name="y", offset=4, datatype=7),
        SimpleNamespace(name="z", offset=8, datatype=7),
    ]
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=stamp_sec, nanosec=stamp_nsec), frame_id=frame_id),
        height=1,
        width=n,
        fields=fields,
        point_step=12,
        row_step=12 * n,
        data=data.view(np.uint8),
    )


def test_pointcloud2_to_array_skips_nans() -> None:
    pts = np.array([[1.0, 2.0, 3.0], [np.nan, 0.0, 0.0], [4.0, 5.0, 6.0]], dtype=np.float32)
    arr, stamp, frame_id = messages.pointcloud2_to_array(_pointcloud2_msg(pts))
    assert arr.shape == (2, 3)
    assert arr.dtype == np.float32
    np.testing.assert_allclose(arr[0], [1.0, 2.0, 3.0])
    assert stamp == pytest.approx(12.5)
    assert frame_id == "lidar"


def test_pointcloud2_missing_field_raises() -> None:
    msg = _pointcloud2_msg(np.zeros((1, 3), dtype=np.float32))
    msg.fields = [f for f in msg.fields if f.name != "z"]
    with pytest.raises(ValueError, match="missing fields"):
        messages.pointcloud2_to_array(msg)


# ── 消息转换：Odometry / PoseWithCovarianceStamped → SE3 + 噪声 ────────────────


def _odom_msg(cov_diag=(0.01, 0.01, 0.01, 0.04, 0.04, 0.04)):
    cov = np.zeros(36, dtype=np.float64)
    for i, v in zip((0, 7, 14, 21, 28, 35), cov_diag):
        cov[i] = v
    pose = SimpleNamespace(
        position=SimpleNamespace(x=1.0, y=2.0, z=3.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )
    pwc = SimpleNamespace(pose=pose, covariance=cov)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=7, nanosec=250_000_000), frame_id="odom"),
        pose=pwc,
    )


def test_pose_to_se3_and_cov_noise() -> None:
    T, stamp, pos_noise, ori_noise = messages.pose_to_se3(_odom_msg())
    np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0], atol=1e-12)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-12)
    assert stamp == pytest.approx(7.25)
    assert pos_noise == pytest.approx(0.1)  # sqrt(0.01)
    assert ori_noise == pytest.approx(0.2)  # sqrt(0.04)


def test_pose_to_se3_with_rotation() -> None:
    msg = _odom_msg()
    msg.pose.pose.orientation = SimpleNamespace(x=0.0, y=0.0, z=np.sin(np.pi / 4), w=np.cos(np.pi / 4))
    T, *_ = messages.pose_to_se3(msg)
    np.testing.assert_allclose(T[:3, :3], [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]], atol=1e-12)


# ── GridMessage 打包 ────────────────────────────────────────────────────────────


def test_layers_to_grid_message_packing() -> None:
    res, center = 0.1, (1.0, 1.0, 0.0)
    layers = {"elevation": np.zeros((3, 3), dtype=np.float32), "variance": np.ones((3, 3), dtype=np.float32)}
    layers["elevation"][0, 0] = 0.77  # marker：row0/col0
    grid = messages.layers_to_grid_message(layers, res, center, stamp=5.0, frame_id="map")
    assert grid.layers == ["elevation", "variance"]
    assert grid.basic_layers == ["elevation"]
    assert grid.length_x == pytest.approx(0.3)
    assert grid.length_y == pytest.approx(0.3)
    assert grid.position == (1.0, 1.0, 0.0)
    assert grid.resolution == pytest.approx(res)
    # marker cell 的世界坐标：row ↔ x = cx + (n/2-1-row)·res → (1.05, 1.05)
    X, Y = io.grid_coordinates((3, 3), res, center)
    assert X[0, 0] == pytest.approx(1.05) and Y[0, 0] == pytest.approx(1.05)
    assert grid.data[0][0, 0] == pytest.approx(0.77)
    with pytest.raises(ValueError, match="basic_layers"):
        messages.layers_to_grid_message(layers, res, center, 0.0, "map", basic_layers=("nope",))


def test_ros_config_validation() -> None:
    sources = {"lidar": SensorSourceConfig(name="lidar", sensor_type="perfect")}
    cfg = RosConfig.from_dict(
        {
            "point_topics": {"lidar": "/points"},
            "pose_type": "pose_cov",
            "publish_fps": 10.0,
            "publish_layers": ["elevation", "slope"],
        },
        sources=sources,
    )
    assert cfg.pose_type == "pose_cov" and cfg.publish_fps == 10.0
    with pytest.raises(ValueError, match="undeclared sources"):
        RosConfig.from_dict({"point_topics": {"cam": "/cam"}}, sources=sources)
    with pytest.raises(ValueError, match="pose_type"):
        RosConfig.from_dict({"point_topics": {"lidar": "/p"}, "pose_type": "tf"}, sources=sources)
    with pytest.raises(ValueError, match="unknown keys"):
        RosConfig.from_dict({"tf_tree": True}, sources=sources)
    with pytest.raises(ValueError, match="at least one"):
        RosConfig.from_dict({}, sources=sources)


# ── 节点接线（打桩 rclpy，注入 Mock mapper）─────────────────────────────────────


def _node_config(save_path: str):
    return SimpleNamespace(
        resolution=0.05,
        sources={"lidar": SensorSourceConfig(name="lidar", sensor_type="perfect")},
        ros={
            "map_frame": "map",
            "point_topics": {"lidar": "/points"},
            "pose_topic": "/odom",
            "pose_type": "odometry",
            "publish_fps": 10.0,
            "publish_layers": ["elevation", "slope"],
            "save_path": save_path,
        },
    )


def test_node_wiring_and_callbacks(node_mod, tmp_path) -> None:
    mapper = mock.Mock()
    mapper.center = np.array([0.5, 0.5, 0.0])
    mapper.get_map.return_value = {
        "elevation": np.zeros((4, 4), dtype=np.float32),
        "slope": np.ones((4, 4), dtype=np.float32),
    }
    node = node_mod.ElevationMappingNode(config=_node_config(str(tmp_path / "save.npz")), mapper=mapper)

    # 订阅/服务/定时器注册
    topics = sorted(s.topic for s in node.subscriptions)
    assert topics == ["/odom", "/points"]
    assert len(node.timers) == 1 and node.timers[0].period == pytest.approx(0.1)
    assert sorted(s.name for s in node.services) == ["~/clear", "~/save"]

    # 位姿回调 → add_pose（协方差噪声被记录，供点云回调传入）
    pose_sub = next(s for s in node.subscriptions if s.topic == "/odom")
    pose_sub.callback(_odom_msg())
    mapper.add_pose.assert_called_once()
    stamp, T = mapper.add_pose.call_args.args
    assert stamp == pytest.approx(7.25)
    assert T.shape == (4, 4)

    # 点云回调 → input（source 与协方差噪声透传）
    cloud_sub = next(s for s in node.subscriptions if s.topic == "/points")
    pts = np.array([[0.0, 0.0, -1.0], [0.1, 0.0, -1.0]], dtype=np.float32)
    cloud_sub.callback(_pointcloud2_msg(pts, stamp_sec=7, stamp_nsec=300_000_000))
    mapper.input.assert_called_once()
    cargs, ckwargs = mapper.input.call_args
    np.testing.assert_allclose(cargs[0], pts, atol=1e-6)  # 转换后的点
    assert cargs[1] == pytest.approx(7.3)  # stamp
    assert ckwargs["source"] == "lidar"
    assert ckwargs["position_noise"] == pytest.approx(0.1)
    assert ckwargs["orientation_noise"] == pytest.approx(0.2)

    # 位姿时间范围外的点云被丢弃并计数
    mapper.input.side_effect = StampOutOfRangeError("test")
    cloud_sub.callback(_pointcloud2_msg(pts, stamp_sec=99, stamp_nsec=0))
    assert node._drops == 1
    mapper.input.side_effect = None

    # 定时发布：get_map 取配置图层，_publish_grid 收到 GridMessage
    node._publish_grid = mock.Mock()
    node.timers[0].callback()
    mapper.get_map.assert_called_with(["elevation", "slope"])
    grid = node._publish_grid.call_args.args[0]
    assert isinstance(grid, messages.GridMessage)
    assert grid.layers == ["elevation", "slope"] and grid.frame_id == "map"

    # clear / save 服务
    clear_srv = next(s for s in node.services if s.name == "~/clear")
    response = ros_stubs.Trigger.Response()
    clear_srv.callback(None, response)
    assert response.success and mapper.clear.called

    save_srv = next(s for s in node.services if s.name == "~/save")
    mapper.stats.return_value = SimpleNamespace(center=(0.5, 0.5, 0.0))
    response = ros_stubs.Trigger.Response()
    save_srv.callback(None, response)
    assert response.success
    layers, meta = io.load_map(tmp_path / "save.npz")
    assert set(layers) == {"elevation", "slope"}
    assert meta["resolution"] == pytest.approx(0.05)


def test_node_missing_config_raises(node_mod) -> None:
    with pytest.raises(ValueError, match="config"):
        node_mod.ElevationMappingNode()  # 无 config_path、无注入 config、桩参数默认 ""


# ── 真实 ROS2 集成（本机无 ROS，预期 skip）──────────────────────────────────────


@pytest.mark.requires_ros
def test_ros2_integration_spin_once(tmp_path) -> None:
    """真实 ROS2 环境的烟测：起一个节点构造/销毁周期。需要 source ROS2 + GPU。"""
    rclpy = importlib.import_module("rclpy")
    node_mod = importlib.import_module("pico_elevationmap_ros.node")
    config = tmp_path / "ros.yaml"
    config.write_text(
        "map:\n  resolution: 0.05\n  map_length: 4.0\nsources:\n  lidar:\n    sensor_type: perfect\n"
        "ros:\n  point_topics:\n    lidar: /points\n  publish_fps: 2.0\n",
        encoding="utf-8",
    )
    rclpy.init()
    try:
        node = node_mod.ElevationMappingNode(config_path=str(config))
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
    finally:
        rclpy.shutdown()
