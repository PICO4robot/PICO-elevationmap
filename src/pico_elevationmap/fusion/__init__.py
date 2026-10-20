"""可插拔融合策略注册表（Phase 1）。

bayesian / average / class_* 等策略统一注册，代理到
elevation_mapping_cupy_core；Mahalanobis 门限决定融合 / 覆盖 / 丢弃。
"""

from __future__ import annotations
