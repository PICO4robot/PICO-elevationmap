# PICO-elevationmap 文档索引

| 文档 | 内容 | 状态 |
|---|---|---|
| [design.md](design.md) | 总体设计方案：上游开源项目分析（leggedrobotics/ANYbotics）、PICO 家族约定、目标架构（ROS-free 核心 + CuPy/torch 双后端 + 可选 ROS2 桥）、仓库结构、传感器方案、工程规范、Phase 0–4 路线图、风险 | v0.2，Phase 0/1/2/4 已实施；Phase 3 暂缓 |
| [development.md](development.md) | 开发环境搭建与验证：前置条件、uv 安装、CUDA headers、GitHub 访问、Quickstart、RealSense D435i 直连、pytest/ruff、ROS2 可选环境、常见问题 | v0.2，已在 Ubuntu 24.04 + RTX 4060 Laptop + D435i 实测；2026-10-30 重跑 CPU 测试 |
| [deploy-hardware.md](deploy-hardware.md) | 真实硬件部署与验证：硬件配置（最小/完整两档）、连接拓扑、工程要点（时间同步/标定/带宽/供电/自机过滤）、四阶段验证节奏、`requires_ros` 测试环境要求、DGX Spark 平台评估 | v0.2，未实施 |
| [sensor-profiles.md](sensor-profiles.md) | 双传感器方案：仅深度相机 vs 深度相机 + LiDAR 的需求、现状评估、main 兼容决策、配置 profile、深度图投影、多源限制、实施计划与风险 | v0.3，M1/M2/M3 原型已跑通（D435i + Airy），标定/时间同步待完善 |
| [api.md](api.md) | 库 API：公共导入路径、`MapperConfig` → `ElevationMapper.add_pose/input/get_map` 示例、I/O 与可选的 ROS 转换层 | v0.1，与 0.4.2 代码同步 |
| [open-source-readiness.md](open-source-readiness.md) | 开源发布就绪：MIT 许可证、第三方 MIT/BSD 声明、支持矩阵、发布检查清单、已知限制与非生产就绪范围 | v0.4，MIT 已定、GitHub-only 发布；PyPI lock 已重建、wheel 依赖仍待处理，CI 实跑已通过 |
| [pypi-release.md](pypi-release.md) | PyPI 发布与 wheel 依赖解析：git-only 依赖的问题、fork 发布方案与备选方案 | v0.1 方案说明，尚未执行 |
| [decisions.md](decisions.md) | 设计决策记录：可视化后端选型（自研 viewer、Foxglove、Rerun） | v0.1，D001 已记录 |
| [semantic-extension.md](semantic-extension.md) | 语义扩展（Phase 5 提案）：MEM 原理、三条语义注入路径、现状与改动清单、价值场景、排期建议 | v0.1 提案，待真机验证后立项 |
| [concepts.md](concepts.md) | 概念辨析：BEV 与 elevation map 的关系与合流趋势 | v0.1 |

根目录 [README.md](../README.md) 承担项目介绍与快速上手；开发环境搭建、CUDA
headers、GitHub 访问与踩坑处理见 [development.md](development.md)；未完成事项见
[TODO.md](../TODO.md)。

## 文档语言

README 使用英文入口，技术文档以中文为主；欢迎补充英文翻译或双语版本。对外
发布前，若目标受众包含非中文用户，应至少提供英文 `README`、`docs/api.md` 和
`docs/open-source-readiness.md` 摘要。英文摘要现已提供在
[en/README.md](en/README.md)（quickstart / API / readiness）。

## 文档维护约定

- 设计决策的讨论结论沉淀为 docs/ 下的独立文档，不要只留在对话里；
- 每份文档头部标注版本、日期、状态；实施后回填"已实施"及对应分支/版本；
- [design.md](design.md) 是历史总体设计稿；当前发布姿态、许可证与支持矩阵以
  [open-source-readiness.md](open-source-readiness.md) 为准；专题方案（部署、
  语义等）独立成文并在本索引登记。
