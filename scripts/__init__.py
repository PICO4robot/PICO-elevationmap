"""Scripts package — ``scripts/cli.py`` 是唯一的 lazy-import CLI dispatcher。

Phase 0 仅含 cli.py；pipeline/（run / replay / bench / export 入口）与
tools/ 随各 Phase 落地，由 cli.py 的 _COMMANDS 表懒加载。
"""

from __future__ import annotations
