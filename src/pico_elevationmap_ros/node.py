"""ElevationMappingNode：rclpy 节点，把 ElevationMapper 接入 ROS2（Phase 4）。

ROS import 只存在于本文件（grid_map_msgs 例外：仅在真实环境调用的
_to_ros_gridmap() 与 _grid_publisher() 内懒加载，以便在没有 grid-map 包的
机器上也能用本节点做纯点云融合）。核心包保持零 ROS 依赖。

运行方式（真实 ROS2 环境，先 source /opt/ros/<distro>/setup.bash）：

    # ament/控制台脚本均可（本 wheel 安装了 pico-elevationmap-ros 入口）
    pico-elevationmap-ros --config /abs/path/config.yaml
    # 或把 config 作为 ROS 参数：
    ros2 run pico_elevationmap_ros mapping_node --ros-args -p config:=/abs/path/config.yaml

config YAML = 核心四段（map/sources/plugins/sim）+ ros 段（见
pico_elevationmap_ros.config.RosConfig 模块 docstring）。

服务：~/clear（std_srvs/Trigger 清空地图）、~/save（Trigger，存到 ros.save_path）。
订阅：ros.point_topics 各点云（sensor_msgs/PointCloud2）+ ros.pose_topic
（Odometry 或 PoseWithCovarianceStamped，位姿协方差对角推导的噪声标量传入
融合，作为漂移补偿触发门限）。
发布：~/grid_map（grid_map_msgs/GridMap，ros.publish_fps 定时、ros.publish_layers
指定的图层）；grid_map_msgs 缺失时发布跳过并每 5s 警告一次。

测试策略：rclpy 与消息包由 tests/ros_stubs.py 在 sys.modules 层打桩，
mapper 经依赖注入传入 Mock（node 本体不 import GPU 栈）。
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

from pico_elevationmap.core import io
from pico_elevationmap.core.pose_buffer import StampOutOfRangeError
from pico_elevationmap_ros import messages
from pico_elevationmap_ros.config import RosConfig

_POSE_MSG_TYPES = {"odometry": Odometry, "pose_cov": PoseWithCovarianceStamped}


class ElevationMappingNode(Node):
    """ROS2 桥接节点：点云/位姿订阅 → ElevationMapper → GridMap 发布。"""

    def __init__(
        self,
        config_path: str | None = None,
        *,
        config: Any | None = None,
        mapper: Any | None = None,
    ) -> None:
        """config_path 从 YAML 加载（真实运行）；config= 直接注入已解析配置
        （测试与自定义装配）；mapper= 注入 ElevationMapper 实例/Mock
        （缺省在内部懒加载真实 mapper —— 该路径需要 GPU）。"""
        super().__init__("elevation_mapping")
        if config is None:
            from pico_elevationmap.core.mapper import MapperConfig

            config_path = config_path or self.declare_parameter("config", "").value
            if not config_path:
                raise ValueError("config yaml path required (CLI --config or -p config:=...)")
            config = MapperConfig.from_yaml(config_path)
        self._cfg = config
        self._ros = RosConfig.from_dict(self._cfg.ros, sources=self._cfg.sources)
        if mapper is None:
            from pico_elevationmap.core.mapper import ElevationMapper

            mapper = ElevationMapper(self._cfg)
        self._mapper = mapper

        self._drops = 0
        self._last_pos_noise: float | None = None
        self._last_ori_noise: float | None = None
        self._grid_warned = False

        self.create_subscription(
            _POSE_MSG_TYPES[self._ros.pose_type], self._ros.pose_topic, self._on_pose, 10
        )
        self._cloud_subscriptions = {
            source: self.create_subscription(PointCloud2, topic, self._make_cloud_cb(source), 10)
            for source, topic in self._ros.point_topics.items()
        }
        self._grid_pub = None  # 首次发布时懒建（grid_map_msgs 可能不存在）
        self.create_timer(1.0 / self._ros.publish_fps, self._on_timer)
        self.create_service(Trigger, "~/clear", self._on_clear)
        self.create_service(Trigger, "~/save", self._on_save)
        self.get_logger().info(
            f"elevation mapping node up: sources {sorted(self._ros.point_topics)}, "
            f"pose {self._ros.pose_topic} ({self._ros.pose_type}), "
            f"publish {list(self._ros.publish_layers)} @ {self._ros.publish_fps} fps"
        )

    # ── 订阅回调 ──────────────────────────────────────────────────────────

    def _on_pose(self, msg: Any) -> None:
        T, stamp, pos_noise, ori_noise = messages.pose_to_se3(msg)
        self._mapper.add_pose(stamp, T)
        self._last_pos_noise = pos_noise
        self._last_ori_noise = ori_noise

    def _make_cloud_cb(self, source: str):
        def _cb(msg: Any) -> None:
            points, stamp, _frame_id = messages.pointcloud2_to_array(msg)
            try:
                self._mapper.input(
                    points,
                    stamp,
                    source=source,
                    position_noise=self._last_pos_noise,
                    orientation_noise=self._last_ori_noise,
                )
            except StampOutOfRangeError:
                self._drops += 1
                if self._drops == 1 or self._drops % 100 == 0:
                    self.get_logger().warn(
                        f"dropped {self._drops} cloud frame(s): pose buffer does not cover their stamps"
                    )

        return _cb

    # ── 定时发布 / 服务 ──────────────────────────────────────────────────

    def _on_timer(self) -> None:
        try:
            layers = self._mapper.get_map(list(self._ros.publish_layers))
        except KeyError as exc:
            if not self._grid_warned:
                self.get_logger().warn(f"publish skipped: {exc}")
                self._grid_warned = True
            return
        center = self._mapper.center
        grid = messages.layers_to_grid_message(
            layers,
            resolution=float(self._cfg.resolution),
            center=center,
            stamp=time.time(),
            frame_id=self._ros.map_frame,
            basic_layers=(self._ros.publish_layers[0],),
        )
        self._publish_grid(grid)

    def _publish_grid(self, grid: messages.GridMessage) -> None:
        """GridMessage → 真 grid_map_msgs/GridMap 并发布（仅真实 ROS 路径）。"""
        pub = self._grid_publisher()
        if pub is None:
            if not self._grid_warned:
                self.get_logger().warn("grid_map_msgs not available, skipping GridMap publish")
                self._grid_warned = True
            return
        pub.publish(_to_ros_gridmap(grid))

    def _grid_publisher(self) -> Any | None:
        if self._grid_pub is None:
            try:
                from grid_map_msgs.msg import GridMap
            except ImportError:
                return None
            self._grid_pub = self.create_publisher(GridMap, "~/grid_map", 10)
        return self._grid_pub

    def _on_clear(self, _request: Any, response: Any) -> Any:
        self._mapper.clear()
        response.success = True
        response.message = "elevation map cleared"
        return response

    def _on_save(self, _request: Any, response: Any) -> Any:
        stats = self._mapper.stats()
        layers = self._mapper.get_map()
        io.save_map(self._ros.save_path, layers, float(self._cfg.resolution), stats.center)
        response.success = True
        response.message = f"saved {len(layers)} layers -> {Path(self._ros.save_path).absolute()}"
        return response


def _to_ros_gridmap(grid: messages.GridMessage) -> Any:
    """GridMessage → grid_map_msgs/GridMap（真实 ROS 环境专用，单点翻译）。

    grid_map 的 data 为 column-major Float32MultiArray：内部行（x 递减）转置
    后按列展平；坐标约定见 messages 模块 docstring。
    """
    from grid_map_msgs.msg import GridMap
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

    n = grid.data[0].shape[0]
    msg = GridMap()
    msg.header.frame_id = grid.frame_id
    msg.layers = grid.layers
    msg.basic_layers = grid.basic_layers
    msg.info.resolution = grid.resolution
    msg.info.length_x = grid.length_x
    msg.info.length_y = grid.length_y
    msg.info.pose.position.x, msg.info.pose.position.y, msg.info.pose.position.z = grid.position
    w, x, y, z = grid.orientation_wxyz
    msg.info.pose.orientation.w = w
    msg.info.pose.orientation.x = x
    msg.info.pose.orientation.y = y
    msg.info.pose.orientation.z = z
    msg.data = []
    for layer in grid.data:
        arr = Float32MultiArray()
        arr.layout = MultiArrayLayout(
            dim=[
                MultiArrayDimension(label="column_index", size=n, stride=n * n),
                MultiArrayDimension(label="row_index", size=n, stride=n),
            ],
            data_offset=0,
        )
        # 我们的 [row(x), col(y)] → grid_map column-major（外层为 x）
        arr.data = np.ascontiguousarray(layer.T, dtype=np.float32).ravel().tolist()
        msg.data.append(arr)
    return msg


def main() -> None:
    """控制台脚本入口：pico-elevationmap-ros --config <yaml>（需 ROS2 环境）。"""
    import sys

    rclpy.init()
    try:
        config_arg = next((a.split("=", 1)[1] for a in sys.argv[1:] if a.startswith("--config")), None)
        node = ElevationMappingNode(config_arg)
        rclpy.spin(node)
    finally:
        rclpy.shutdown()
