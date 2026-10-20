"""输入源噪声配置（Phase 1）。

每路输入源（LiDAR / 深度相机 / 仿真理想深度）一组 YAML 噪声参数
（激光误差随距离线性增长、结构光随距离平方增长等），不做继承体系。
"""

from __future__ import annotations
