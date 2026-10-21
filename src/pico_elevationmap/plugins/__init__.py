"""后处理插件链（Phase 2）。

inpaint / smooth / normal / traversability 等纯函数式 map → map 插件，
YAML 声明 + 动态 import，零样板。
"""

from __future__ import annotations
