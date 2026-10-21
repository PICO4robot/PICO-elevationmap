"""建图后端抽象（Phase 3）。

cupy 后端：实机实时建图（包装 cupy_core）；torch 后端：mjlab 仿真中
N 个并行环境各自维护一张图，批量输出作为 PICO-rl 的 observation。
"""

from __future__ import annotations
