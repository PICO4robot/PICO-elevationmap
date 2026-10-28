# 语义扩展设计（Phase 5 提案）

> 版本：v0.1（提案）　日期：2026-10-28　状态：未立项，待真机验证后排期
> 上游依据：MEM（Erni et al., *Multi-Modal Elevation Mapping*, IROS 2023），
> 已内置于 elevation_mapping_cupy（`SemanticMap` + 语义融合插件）。

## 1. 原理

地图本是多层结构（elevation / variance / time / …），语义是新增一类 layer：

```
cell (i,j) = { elevation, variance, ..., class_prob: [平地 0.7, 草地 0.2, 台阶 0.1] }
```

语义层与几何层共享同一套栅格机制（环形缓冲、随机器人移动、时间衰减），
因此**几何与语义永远空间对齐**——这是相比独立 BEV 语义图的核心优势。

## 2. 三条注入路径（上游均支持）

| 路径 | 输入 | 说明 |
|---|---|---|
| 点云自带通道 | 点云除 xyz 外带 class/intensity 字段 | LiDAR 分割网络在点云侧输出类别，随点云入图逐 cell Bayesian 融合 |
| 图像投影 | 语义分割图 + 相机内外参 + 位姿 | RGB 过分割网络 → 像素级类别概率投到栅格（`image_exponential` 等策略）；对"头部深度相机"方案最自然——深度与语义同机采集、天然对齐 |
| 特征图层 | 高维特征（DINO / CLIP 类） | 不限于离散类别，供下游网络消费 |

## 3. 现状与差距

上游 `elevation_mapping_cupy_core` 自带 `SemanticMap` 与语义融合插件，
但本仓库 Phase 1 接入时刻意关闭：`subscriber_cfg={}` 跳过语义层，点云只收 xyz。

启用所需的改动清单：

1. **配置 schema 扩展**：`sources` 允许声明语义通道（点云第 K 通道）或图像源
   （内外参 + topic）；`MapperConfig` 增加对应严格校验；
2. **图像输入路径**：新增"深度图 + 内外参 → 点云"前置投影（接实机深度相机
   本来就要补，见 deploy-hardware.md §6），语义图走同一套投影；
3. **分割模型依赖**：引入 torch（与 deferred 的 Phase 3 共用依赖）+ 轻量分割网络
   （机载用轻量模型，离线 replay 可用大模型）；权重走家族资产管理惯例；
4. **语义 traversability 插件**：把现有纯几何 `geo_traversability`
   （坡度 + 粗糙度阈值）升级为"几何 ∧ 语义"联合判定——
   例："坡度合格但类别为碎石/湿滑 → 降权"；
5. **测试**：合成带语义通道的点云 fixture，验证 Bayesian 融合收敛与图层对齐。

## 4. 价值场景（按近期价值排序）

- **地形自适应步态**：规划器/策略知道脚下是"草地/台阶/碎石/湿滑"而非仅
  "高 0.15 m"——几何上不可分的地形（草地 vs 硬地）语义可分；
- **导航代价图**：语义层直接作为 costmap 软约束（"优先平地，可踩草地，避开碎石"）；
- **RL observation（远期）**：与 Phase 3（torch 批量后端）合流，语义 layer 作为
  BEV 张量额外通道喂给策略网络。

## 5. 排期建议

列为 **Phase 5**，排在真机验证（ROS2 桥 + 深度相机投影前置）之后：
图像投影与深度图投影是同一套基础设施，一起做完语义链路即通。
依赖关系：Phase 5 与 Phase 3 共享 torch 依赖，两者可合并排期。
