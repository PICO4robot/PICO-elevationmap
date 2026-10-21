"""后处理插件链（Phase 2 已落地）。

- ``chain``：校验插件声明、落盘 plugin_config.yaml 给上游 PluginManager，
  并把自有插件 alias 进上游包命名空间（register_custom_plugins）；
- ``geo_slope`` / ``geo_traversability``：自有几何插件（坡度层 / 几何可通行性，
  不携带上游学习权重），遵循上游 PluginBase 接口，经同一条链加载；
- ``geo_common``：坡度 / 粗糙度共享数值例程（无 PluginBase 子类）。

上游插件（inpainting / smooth_filter / min_filter / ...）直接按 YAML 声明启用。
"""

from __future__ import annotations
