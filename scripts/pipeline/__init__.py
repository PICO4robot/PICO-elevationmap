"""Pipeline entry points（run / replay / bench / export），由 scripts.cli 懒加载。

Phase 1 仅实现 run（合成场景实时建图 demo）；replay / bench / export 仍走
cli.py 的 stub 回退。
"""

from __future__ import annotations
