# 概念辨析：BEV 与 elevation map

> 版本：v0.1　日期：2026-10-28
> 定位：常见概念问题的权威回答，供团队对齐术语。

## BEV ⊃ elevation map

**BEV（Bird's Eye View，鸟瞰图）** 是一种表示范式：把机器人周围场景投影到
自上而下的二维栅格，每个格子存"某些通道"——几何量、语义类别或学习特征均可。
自动驾驶的 BEV 感知（BEVFormer、LSS 等）即此范式：多路相机 → 神经网络 →
BEV 特征栅格。

**Elevation map 是 BEV 的一种具体实现**：同为自上而下栅格，但主通道明确为
**地面高度**（加方差、时间戳等辅助层），并用手工设计的概率融合
（Kalman 式更新 + 可见性清理）构建，而非学习得到。

说"一张 elevation map"时，已隐含三个约定：BEV 视角 + 高度通道 + 概率建图方法。

## 并排对比

| 维度 | Elevation map（本仓库） | 学习式 BEV（自动驾驶语境） |
|---|---|---|
| 格子内容 | 高度、方差、坡度、traversability——可解释几何量 | 高维特征向量 / 语义 logits——不可直接解释 |
| 构建方式 | 手工概率融合（点云 + 位姿 → 逐 cell 更新） | 神经网络端到端编码（多为多相机） |
| 输入 | 深度点云 + 6DoF 位姿（必需） | 纯视觉多视角为主，位姿需求弱 |
| 消费者 | 规划器 / 控制器 / foothold 选择（白盒可查） | 下游预测 / 规划网络（黑盒张量） |
| 误差模型 | 显式（每 cell 方差，漂移补偿） | 隐式（网络自行学习） |
| 典型尺度 | 机器人中心局部 4–20 m，厘米级 | 车体周围 50–100 m，分米级 |

## 两者正在合流

1. **MEM 多模态高程图**（Erni et al., IROS 2023）：把 RGB / 语义特征融进
   elevation map 的额外 layer——用几何地图的骨架承载学习式 BEV 的内容
   （见 semantic-extension.md）；
2. **RL 的 heightfield observation**：腿足 RL 常用的"机器人周围高度采样阵列"
   就是极简 elevation map；本仓库 deferred 的 Phase 3 torch 批量后端输出形态
   即 `(N_env, layers, H, W)` 的 BEV 张量，可直接作策略网络输入；
3. 本仓库的派生 layer（slope、traversability_geo）已经是"几何版多通道 BEV
   特征图"，与学习式 BEV 的差别只在通道是算出来的还是学出来的。

## 对 PICO 的策略含义

- 短期（实机导航 / 落足）：elevation map 是正确答案——可解释、有不确定度、
  无需训练数据；
- 远期（端到端策略）：不推翻重来，而是沿 MEM 思路往地图加学习式 layer，
  或用 torch 批量后端把地图喂给策略——elevation map 作为 BEV 的**几何底座**
  继续存在，学习特征作为增量层叠加。
