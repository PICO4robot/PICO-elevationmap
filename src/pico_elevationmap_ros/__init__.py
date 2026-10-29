"""pico_elevationmap_ros — 可选 ROS2 桥接包（Phase 4）。

核心包 pico_elevationmap 保持零 ROS 依赖；本包把 ElevationMapper 接入 ROS2：
messages.py（零 ROS 的鸭子类型消息转换层）、config.py（ros: 段配置校验）、
node.py（rclpy 节点，ROS import 只存在于该文件）。

设计约束见 docs/design.md §5/§6（历史设计稿）；当前发布/许可证姿态见
docs/open-source-readiness.md。ROS 只是消费方，grid_map_msgs 翻译集中在
node.py 一处。
"""

from __future__ import annotations
