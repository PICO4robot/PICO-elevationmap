# PICO-elevationmap 文档索引

| 文档 | 内容 | 状态 |
|---|---|---|
| [design.md](design.md) | 总体设计方案：上游开源项目分析（leggedrobotics/ANYbotics）、PICO 家族约定、目标架构（ROS-free 核心 + CuPy/torch 双后端 + 可选 ROS2 桥）、仓库结构、传感器方案、工程规范、Phase 0–4 路线图、风险 | v0.2，Phase 0/1/2/4 已实施；Phase 3 暂缓 |
| [development.md](development.md) | 开发环境搭建与验证：前置条件、uv 安装、CUDA headers、GitHub 访问、Quickstart、pytest/ruff、ROS2 可选环境、常见问题 | v0.1，已在 Ubuntu 24.04 + RTX 4060 Laptop 实测 |
| [deploy-hardware.md](deploy-hardware.md) | 真实硬件部署与验证：硬件配置（最小/完整两档）、连接拓扑、工程要点（时间同步/标定/带宽/供电/自机过滤）、四阶段验证节奏、`requires_ros` 测试环境要求、DGX Spark 平台评估 | v0.2，未实施 |
| [semantic-extension.md](semantic-extension.md) | 语义扩展（Phase 5 提案）：MEM 原理、三条语义注入路径、现状与改动清单、价值场景、排期建议 | v0.1 提案，待真机验证后立项 |
| [concepts.md](concepts.md) | 概念辨析：BEV 与 elevation map 的关系与合流趋势 | v0.1 |

根目录 [README.md](../README.md) 承担分支门户与快速上手；开发环境搭建、CUDA
headers、GitHub 访问与踩坑处理见 [development.md](development.md)。

## 文档维护约定

- 设计决策的讨论结论沉淀为 docs/ 下的独立文档，不要只留在对话里；
- 每份文档头部标注版本、日期、状态；实施后回填"已实施"及对应分支/版本；
- design.md 为总体权威文档，专题方案（部署、语义等）独立成文并在本索引登记。
