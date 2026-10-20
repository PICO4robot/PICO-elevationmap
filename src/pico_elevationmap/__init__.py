"""pico_elevationmap — PICO 高程建图框架核心包（ROS-free）。

多层栅格高程图 + 可插拔融合策略 + 后处理插件链 + cupy / torch 双后端；
各子包按 Phase 1-3 逐包落地。家族约定：顶层包只放 docstring，不做 re-export。
"""

from __future__ import annotations
