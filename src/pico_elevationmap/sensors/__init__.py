"""输入源噪声配置（Phase 1 已落地）。

``config``：每路输入源一组 YAML 配置项（laser / depth_camera / perfect 的
std 参数化、量程裁剪、上报位姿噪声），并把噪声系数映射到上游融合内核的
二次型 sensor_noise_factor（映射规则见模块 docstring）。
"""

from __future__ import annotations
