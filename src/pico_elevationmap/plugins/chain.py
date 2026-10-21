"""插件链装配（Phase 1）：校验插件声明并落盘 YAML，供上游 PluginManager 加载。

上游 elevation_mapping_cupy.plugins.PluginManager 可直接复用 —— 它从 YAML 读取
插件声明（enable / type / layer_name / fill_nan / is_height_layer / extra_params），
动态 import elevation_mapping_cupy.plugins 包下的插件类（min_filter / smooth_filter /
inpainting / erosion / ...）。本模块只负责把程序内的声明字典做结构校验后写成
该 YAML；真正的计算链在上游 PluginBase.__call__（纯函数式 map → map）。
"""

from __future__ import annotations

import tempfile
from pathlib import Path
from typing import Any

import yaml

__all__ = ["validate_plugin_decl", "write_plugin_config"]

_REQUIRED_KEYS = {"enable", "layer_name", "extra_params"}


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
