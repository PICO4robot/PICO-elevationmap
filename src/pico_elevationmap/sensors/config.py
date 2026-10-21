"""每输入源噪声模型配置（Phase 1）：YAML 声明、校验、映射到上游噪声参数。

设计文档 §7.3：三种传感器本质是不同的噪声参数化 —— 激光误差随距离线性增长
（std ∝ d），深度相机随距离平方（std ∝ d²），仿真理想深度无噪声。不做继承
体系，每源一组 YAML 配置项。

上游融合内核只暴露一个二次型系数（var = sensor_noise_factor · d²，d 为传感器
系下点到原点距离），映射规则：

- perfect：极小正系数（1e-4）——零方差会让 Mahalanobis 门限除零；
- laser：std = c·d ⇒ var = c²·d²，系数取 c²，精确匹配；
- depth_camera：std = c·d² ⇒ var = c²·d⁴，内核无法精确表达，取 c²·r_max²
  作为操作量程内的方差上界（宁可高估测量方差，不低估）。

min/max_range 在包装层按传感器系距离逐源裁剪（上游仅有全局
min_valid_distance / max_height_range）。
"""

from __future__ import annotations

from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, ClassVar

import numpy as np
import yaml

__all__ = ["SensorSourceConfig", "load_sources", "parse_sources"]

# perfect 源进入上游融合的最小非零系数（避免 Mahalanobis 除零）
PERFECT_NOISE_FACTOR = 1e-4


@dataclass(frozen=True)
class SensorSourceConfig:
    """单路输入源的噪声模型与量程裁剪。

    Args:
        name: 输入源标识（YAML key）。
        sensor_type: "laser" | "depth_camera" | "perfect"。
        noise_coefficient: 测距标准差系数 c —— laser σ = c·d，depth_camera σ = c·d²。
        min_range / max_range: 量程裁剪（米，按传感器系距离）。
        position_noise / orientation_noise: 上报给上游的位姿噪声水平，
            仅用于触发漂移补偿门限，不参与逐点方差。
    """

    SENSOR_TYPES: ClassVar[tuple[str, ...]] = ("laser", "depth_camera", "perfect")

    name: str
    sensor_type: str = "perfect"
    noise_coefficient: float = 0.0
    min_range: float = 0.2
    max_range: float = 30.0
    position_noise: float = 0.0
    orientation_noise: float = 0.0

    def __post_init__(self) -> None:
        if self.sensor_type not in self.SENSOR_TYPES:
            raise ValueError(
                f"source '{self.name}': unknown sensor_type '{self.sensor_type}', "
                f"expected one of {self.SENSOR_TYPES}"
            )
        if not np.isfinite(self.noise_coefficient) or self.noise_coefficient < 0.0:
            raise ValueError(f"source '{self.name}': noise_coefficient must be a non-negative finite number")
        if self.sensor_type == "perfect" and self.noise_coefficient > 0.0:
            raise ValueError(f"source '{self.name}': perfect sensors take noise_coefficient 0")
        if self.sensor_type != "perfect" and self.noise_coefficient <= 0.0:
            raise ValueError(f"source '{self.name}': {self.sensor_type} sensors need noise_coefficient > 0")
        if not np.isfinite(self.min_range) or self.min_range <= 0.0:
            raise ValueError(f"source '{self.name}': min_range must be > 0")
        if not np.isfinite(self.max_range) or self.max_range <= self.min_range:
            raise ValueError(f"source '{self.name}': max_range must be > min_range")

    @property
    def upstream_noise_factor(self) -> float:
        """映射到上游 Parameter.sensor_noise_factor 的二次型系数。"""
        if self.sensor_type == "perfect":
            return PERFECT_NOISE_FACTOR
        if self.sensor_type == "laser":
            return self.noise_coefficient**2
        # depth_camera：var = c²·d⁴ 的上游最近似（操作量程内上界，见模块 docstring）
        return (self.noise_coefficient * self.max_range) ** 2

    def measurement_std(self, distance: np.ndarray) -> np.ndarray:
        """仿真噪声注入用：各点径向测距标准差。"""
        d = np.asarray(distance, dtype=np.float64)
        if self.sensor_type == "perfect":
            return np.zeros_like(d)
        if self.sensor_type == "laser":
            return self.noise_coefficient * d
        return self.noise_coefficient * d * d


def parse_sources(data: Any) -> dict[str, SensorSourceConfig]:
    """从 YAML 顶层 'sources' 段解析 {name: SensorSourceConfig}。"""
    if not isinstance(data, dict) or not isinstance(data.get("sources"), dict):
        raise ValueError("config must be a mapping with a 'sources' section")
    raw = data["sources"]
    if not raw:
        raise ValueError("'sources' section is empty")
    allowed = {f.name for f in fields(SensorSourceConfig)} - {"name"}
    out: dict[str, SensorSourceConfig] = {}
    for name, cfg in raw.items():
        if not isinstance(name, str) or not isinstance(cfg, dict):
            raise ValueError(f"source '{name}': entry must be a mapping")
        unknown = set(cfg) - allowed
        if unknown:
            raise ValueError(f"source '{name}': unknown keys {sorted(unknown)}")
        try:
            out[name] = SensorSourceConfig(name=name, **cfg)
        except TypeError as exc:
            raise ValueError(f"source '{name}': {exc}") from exc
    return out


def load_sources(path: str | Path) -> dict[str, SensorSourceConfig]:
    """从 YAML 文件加载输入源配置。"""
    text = Path(path).read_text(encoding="utf-8")
    return parse_sources(yaml.safe_load(text))
