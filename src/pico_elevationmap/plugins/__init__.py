"""后处理插件链（Phase 1 已接线，插件阵容在 Phase 2 扩充）。

上游 PluginManager 直接复用（YAML 声明 + 动态 import 上游插件类）；
``chain`` 负责校验声明结构并落盘 plugin_config.yaml 供 ElevationMap 加载。
真正的计算链在上游 PluginBase.__call__（纯函数式 map → map）。
"""

from __future__ import annotations
