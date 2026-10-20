"""高程图核心数据结构（Phase 1/2 已落地）。

- ``pose_buffer``：带时间戳的 SE3 位姿缓冲（平移线性插值 + 旋转 slerp），
  替代 ROS tf 订阅，按点云时间戳对齐位姿；
- ``mapper``：ROS-free 建图外观 ElevationMapper，包装上游
  elevation_mapping_cupy 的 ElevationMap（融合 / Mahalanobis 门限 /
  raycasting 可见性清理 / 漂移补偿），每源量程裁剪 + 噪声系数映射 +
  法线层导出 + 自有 geo_* 插件接线；
- ``io``：离线帧序列 / 地图 npz / rosbag（rosbags 纯 python）读写与
  PLY / PGM / CSV 导出写出器，供 run / replay / export 共用。
"""

from __future__ import annotations
