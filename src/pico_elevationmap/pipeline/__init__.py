"""Pipeline entry points（run / replay / bench / export），由 pico_elevationmap.cli 懒加载。

Phase 2 已落地 run（合成场景实时建图）、replay（npz 帧序列 / rosbag 离线回放）、
export（地图 npz → PLY / PGM / CSV）；bench 待 Phase 3 的 torch 批量后端。
"""

from __future__ import annotations
