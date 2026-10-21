"""高程图核心数据结构（Phase 1）。

封装多层栅格 + 环形缓冲区地图（grid_map 式零拷贝平移）、SE3 位姿缓冲
（带协方差、按时间戳线性插值对齐点云），替代上游的 tf 订阅耦合。
"""

from __future__ import annotations
