"""ROS 桥配置：YAML `ros:` 段校验（Phase 4）。

核心 MapperConfig 对 ros: 段只做透传存储，本模块负责严格校验 —— 与家族
其余配置一致：未知键、类型错误、引用未配置的输入源都抛 ValueError。

YAML 示例::

    ros:
      map_frame: map
      point_topics:            # 输入源名 → 点云 topic（键必须在 sources 段声明）
        lidar: /ouster/points
        depth: /camera/depth/points
      pose_topic: /odometry    # Odometry 或 PoseWithCovarianceStamped
      pose_type: odometry      # odometry | pose_cov
      publish_fps: 5.0
      publish_layers: [elevation, traversability_geo]
      save_path: /tmp/elevation_map.npz
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, ClassVar

from pico_elevationmap.sensors.config import SensorSourceConfig

__all__ = ["RosConfig"]


@dataclass(frozen=True)
class RosConfig:
    """ROS 桥运行参数（全部可选，默认值如下）。"""

    POSE_TYPES: ClassVar[tuple[str, ...]] = ("odometry", "pose_cov")

    map_frame: str = "map"
    point_topics: dict[str, str] = field(default_factory=dict)
    pose_topic: str = "/pose"
    pose_type: str = "odometry"
    publish_fps: float = 5.0
    publish_layers: tuple[str, ...] = ("elevation",)
    save_path: str = "elevation_map_save.npz"

    @classmethod
    def from_dict(
        cls, data: dict[str, Any] | None, *, sources: dict[str, SensorSourceConfig]
    ) -> RosConfig:
        if data is None:
            data = {}
        if not isinstance(data, dict):
            raise ValueError("'ros' section must be a mapping")
        allowed = {"map_frame", "point_topics", "pose_topic", "pose_type", "publish_fps", "publish_layers", "save_path"}
        unknown = set(data) - allowed
        if unknown:
            raise ValueError(f"'ros' section: unknown keys {sorted(unknown)}")

        cfg = cls(
            map_frame=str(data.get("map_frame", "map")),
            point_topics=dict(data.get("point_topics") or {}),
            pose_topic=str(data.get("pose_topic", "/pose")),
            pose_type=str(data.get("pose_type", "odometry")),
            publish_fps=float(data.get("publish_fps", 5.0)),
            publish_layers=tuple(data.get("publish_layers") or ("elevation",)),
            save_path=str(data.get("save_path", "elevation_map_save.npz")),
        )
        cfg._validate(sources)
        return cfg

    def _validate(self, sources: dict[str, SensorSourceConfig]) -> None:
        if not self.point_topics:
            raise ValueError("'ros.point_topics' must map at least one source to a topic")
        unknown_sources = set(self.point_topics) - set(sources)
        if unknown_sources:
            raise ValueError(
                f"'ros.point_topics' references undeclared sources {sorted(unknown_sources)} "
                f"(configured: {sorted(sources)})"
            )
        for name, topic in self.point_topics.items():
            if not isinstance(topic, str) or not topic.startswith("/"):
                raise ValueError(f"'ros.point_topics.{name}': topic must be an absolute path, got {topic!r}")
        if self.pose_type not in self.POSE_TYPES:
            raise ValueError(f"'ros.pose_type' must be one of {self.POSE_TYPES}, got '{self.pose_type}'")
        if not self.pose_topic.startswith("/"):
            raise ValueError(f"'ros.pose_topic' must be an absolute path, got {self.pose_topic!r}")
        if self.publish_fps <= 0.0:
            raise ValueError(f"'ros.publish_fps' must be > 0, got {self.publish_fps}")
        if not self.publish_layers or not all(isinstance(n, str) for n in self.publish_layers):
            raise ValueError("'ros.publish_layers' must be a non-empty list of layer names")
