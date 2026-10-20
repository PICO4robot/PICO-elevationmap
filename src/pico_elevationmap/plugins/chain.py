"""插件链装配（Phase 2）：校验插件声明、落盘 YAML、注册自有插件。

上游 elevation_mapping_cupy.plugins.PluginManager 直接复用 —— 它从 YAML 读取
插件声明（enable / type / layer_name / fill_nan / is_height_layer / extra_params），
并以相对 import 从 elevation_mapping_cupy.plugins 包内加载插件类。自有插件
（geo_slope / geo_traversability）通过 register_custom_plugins() 在加载前
alias 进 sys.modules，从而进入同一条插件链，无需 fork 上游包。
"""

from __future__ import annotations

import importlib
import sys
import tempfile
from pathlib import Path
from typing import Any

import yaml

__all__ = ["CUSTOM_PLUGINS", "register_custom_plugins", "validate_plugin_decl", "write_plugin_config"]

# 自有插件：插件名（YAML key）-> 实现模块。每个插件独占一个模块 —— 上游
# PluginManager 会把模块内所有 PluginBase 子类都实例化。
CUSTOM_PLUGINS = {
    "geo_slope": "pico_elevationmap.plugins.geo_slope",
    "geo_traversability": "pico_elevationmap.plugins.geo_traversability",
}

_REQUIRED_KEYS = {"enable", "layer_name", "extra_params"}


def register_custom_plugins() -> None:
    """把自有插件模块 alias 成 elevation_mapping_cupy.plugins.<name>。

    上游 PluginManager.init 用相对 import 从自家包加载插件；sys.modules
    命中后返回的是我们的模块，其中的 PluginBase 子类（与上游同一基类）
    随之被实例化。幂等，可重复调用。
    """
    import elevation_mapping_cupy.plugins  # noqa: F401  —— 确保目标包已加载

    for plugin_name, module_name in CUSTOM_PLUGINS.items():
        alias = f"elevation_mapping_cupy.plugins.{plugin_name}"
        if alias not in sys.modules:
            sys.modules[alias] = importlib.import_module(module_name)


def validate_plugin_decl(decl: Any) -> dict[str, dict[str, Any]]:
    """校验上游格式的插件声明字典，原样返回；结构问题抛 ValueError。"""
    if not isinstance(decl, dict):
        raise ValueError("'plugins' section must be a mapping of plugin name -> config")
    for name, cfg in decl.items():
        if not isinstance(name, str) or not isinstance(cfg, dict):
            raise ValueError(f"plugin '{name}': entry must be a mapping")
        missing = _REQUIRED_KEYS - set(cfg)
        if missing:
            raise ValueError(f"plugin '{name}': missing keys {sorted(missing)} (upstream plugin config format)")
        if not isinstance(cfg["enable"], bool):
            raise ValueError(f"plugin '{name}': 'enable' must be a boolean")
        if not isinstance(cfg["extra_params"], dict):
            raise ValueError(f"plugin '{name}': 'extra_params' must be a mapping")
    return decl


def write_plugin_config(decl: dict[str, dict[str, Any]], directory: str | Path | None = None) -> Path:
    """把插件声明写入 directory/plugin_config.yaml 并返回路径。

    上游 ElevationMap 构造时按 plugin_config_file 加载该文件（文件必须存在，
    即使不启用任何插件），故空声明也落盘为 "{}"，而不是省略文件。
    """
    decl = validate_plugin_decl(decl)
    out_dir = Path(directory) if directory is not None else Path(tempfile.mkdtemp(prefix="pico_em_plugins_"))
    out_dir.mkdir(parents=True, exist_ok=True)
    path = out_dir / "plugin_config.yaml"
    path.write_text(yaml.safe_dump(dict(decl), sort_keys=False, default_flow_style=False), encoding="utf-8")
    return path
