# PICO-elevationmap 设计方案

> 版本：v0.2（草案）　日期：2026-10-28　状态：历史设计稿
>
> ⚠️ **历史设计文档**：本文记录 2026-10 的设计推演，其中 CI、torch 后端、
> ROS 打包与许可证等状态可能已过时。当前发布/许可证姿态、支持矩阵与已知限制
> 以 [open-source-readiness.md](open-source-readiness.md) 为准。

本文档给出 PICO-elevationmap 的总体设计：上游开源项目分析、需要遵循的 PICO 家族约定、
目标架构、模块划分、传感器方案、工程规范与实施路线图。

---

## 1. 背景与目标

为 PICO 系列机器人（四足 / 人形）提供一套 **2.5D 高程建图（elevation mapping）框架**：

- 实时：实机上以传感器频率增量维护以机器人为中心的局部高程图；
- 可训练：仿真中批量建图，作为 PICO-rl 的 RL observation；
- 符合 PICO-xxx 家族组织特点：纯 Python、零 ROS 依赖的核心库，ROS 仅作为可选桥接。

设计上不从零造轮子，而是基于 leggedrobotics / ANYbotics 的成熟开源成果做**包装与重构**。

## 2. 上游开源项目分析

### 2.1 项目版图

| 项目 | 语言 | License | ROS | 状态 | 定位 |
|---|---|---|---|---|---|
| [elevation_mapping](https://github.com/ANYbotics/elevation_mapping)（规范仓库在 ANYbotics） | C++ | BSD-3 | 仅 ROS1 | 维护停滞 | 经典 CPU 版（Fankhauser et al., RA-L 2018） |
| [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) | Python + C++（pybind11） | MIT | `main`=ROS1，**`ros2` 分支=现役**（Jazzy + CUDA 12） | 活跃 | GPU 版，事实标准（Miki et al., IROS 2022） |
| [elevation_mapping_cupy_core](https://github.com/leggedrobotics/elevation_mapping_cupy_core) | 纯 Python | MIT | **无 ROS** | 2026 年新抽出 | 去 ROS 的 pip 核心库 |
| [grid_map](https://github.com/ANYbotics/grid_map) | C++ | BSD-3 | core 零 ROS | 活跃 | 底层多层栅格图 |
| [traversability_estimation](https://github.com/leggedrobotics/traversability_estimation) | C++ | BSD | ROS1 | 被学习式方法取代 | 几何可通行性 |

### 2.2 核心算法精髓（两代项目共有）

- **机器人中心局部 2.5D 栅格图**，地图随机器人平移；grid_map 用二维环形缓冲区实现零拷贝移动；
- 每个 cell 维护**高度 + 方差**，新测量做 1D 高斯 / Kalman 式融合；**Mahalanobis 距离门限**
  （默认 2.5）决定融合 / 覆盖 / 丢弃；高于估计的测量加 multi-height 噪声以适应动态障碍；
- cupy 版升级为**双分布**（下表面 elevation + 上表面 upper_bound），Bayesian 可插拔融合策略；
- **GPU raycasting 可见性清理**：从传感器原点 raycast 移除被遮挡的"幽灵"cell，比 CPU 版快几个数量级；
- **漂移补偿**：对比当前点云与累积地图，检测状态估计漂移并就地修正地图；
- 后处理**插件链**：inpainting / smooth / 法线 / 学习式 traversability（小型 MLP），均为纯函数式
  `map → map`，YAML 声明 + 动态 import，零样板。

### 2.3 值得保留的抽象

1. 多层栅格 + 环形缓冲的数据结构（grid_map_core 证明它可以零 ROS 依赖）；
2. **fusion 策略 + 后处理插件**双插件点；
3. `ElevationMap.input(points, R, t, pos_noise, ori_noise)` 的显式位姿调用形式
   （cupy_core 已示范，用于替代 tf 订阅）。

### 2.4 需要重新设计的 ROS 耦合点

| ROS 耦合点 | 替代方案 |
|---|---|
| tf（odom↔base↔sensor） | 显式 SE3 + 带时间戳的位姿缓冲（线性插值对齐点云时间戳） |
| grid_map_msgs/GridMap 消息与服务 | 零拷贝 layer 视图 / 数据类返回 |
| ros/filters 插件链 | 自带 PluginManager（沿用 cupy 版模式） |
| PoseWithCovarianceStamped / rosbag | 普通回调与数据类；离线 replay 走自家管线 |
| catkin / ament | pyproject + pip/uv |

### 2.5 上游主要局限

- 强依赖 NVIDIA GPU（cupy-cuda12x，无 CPU fallback）；
- 单一正方形局部图，无多地图 / 回环；
- 方差模型为启发式；
- Python + pybind11 边界工程复杂度高。

## 3. PICO 家族约定（必须遵循）

调研自 PICO-robot / PICO-rl / PICO-dataset 三个 sibling 仓库：

1. **纯 Python ≥ 3.10，核心零 ROS 依赖**；硬件 / 仿真接口统一走 PICO-robot 的
   MJCF/URDF 资产 + MuJoCo（mjlab）；
2. sibling checkout 互相关联，`$PICO_{ROBOT,DATASET,CKPT,LOG}_DIR` 环境变量解析；
   GitHub org `PICO4robot`，远端小写 `pico-*`，默认分支 `main`；
3. 打包 setuptools + pyproject.toml；依赖管理 **uv**（uv.lock + dependency-groups +
   `tool.uv.sources` 锁 git rev，参照 PICO-rl 锁 mjlab 的写法）；
4. 代码风格 **ruff**（line-length 120、双引号、select `E,W,F,I,UP`、target py310）；
   中英混写注释；`from __future__ import annotations`；
5. **src layout** 多包结构（参照 PICO-rl：框架层 / 任务层 / 资产层单向依赖）；
   `src/pico_elevationmap/cli.py` 唯一 lazy-import CLI dispatcher；
6. pytest 测试，tests/ 镜像 src 布局，合成数据 fixture（无外部资产时自动 skip）；
7. CI：单文件 `.github/workflows/ci.yml`，push/PR 触发，CPU lint / test / build；
   不依赖 GPU 或私有 sibling 检出；
8. SemVer 写在 pyproject `version`；conventional commits（本仓库 scope 用 `(map)`）；
   需要消费 sibling 资产时用 ASSETS.lock + check_asset_pins.py 锁 commit；
9. License：MIT（2026-10-30 起）；发布姿态见
   [open-source-readiness.md](open-source-readiness.md)。

## 4. 核心决策：基于 cupy_core 包装，而非从零重写

[elevation_mapping_cupy_core](https://github.com/leggedrobotics/elevation_mapping_cupy_core)
（MIT、pip 可装、纯 Python 无 ROS）已把最难的部分（CUDA kernel 集、`ElevationMap` /
`SemanticMap` / `PluginManager` / `Parameter`）从 ROS 解耦。三条路线对比：

| 路线 | 优点 | 缺点 |
|---|---|---|
| **A. 依赖 cupy_core + 包装（采纳）** | 算法成熟、跟随上游修 bug；家族已有锁 git 依赖先例 | 上游 API 变动需跟随 |
| B. vendor 复制内核再改 | 完全可控 | 失去上游更新 |
| C. 从零重写 | 无外部依赖 | 重复造 CUDA kernel，成本高 |

采纳 **A**：`tool.uv.sources` 把 `elevation-mapping-cupy-core` 锁到具体 git rev。
PICO-elevationmap 的差异化价值在于：**配置体系、插件层、I/O 桥接、与 PICO-rl 的
observation 集成**；torch 批量后端设计为差异化方向，但 Phase 3 暂缓。定制化需求
积累到一定程度后再评估部分 vendor。

License 合规：cupy_core（MIT）、elevation_mapping（BSD-3）的版权与许可证声明必须
保留；最终许可证方案（OSI / source-available / 双许可）尚未决定，见
[open-source-readiness.md](open-source-readiness.md) 与
[THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)。

## 5. 目标架构

```
传感器点云 ─┐
           ├─→ 输入源管理（每源 YAML 噪声配置）─→ 位姿缓冲（时间戳对齐）
语义图像 ───┘                    │
                                 ▼
                    融合内核（fusion，可插拔策略）
                                 │
                    高程图核心（多层栅格 + 环形缓冲，双分布 + 方差）
                                 │
              ┌──────────────────┼───────────────────┐
              ▼                  ▼                   ▼
       raycast 清理        漂移补偿            后处理插件链
       （可见性）                              （inpaint/smooth/法线/traversability）
                                 │
              ┌──────────────────┼───────────────────┐
              ▼                  ▼                   ▼
        实机 ROS2 桥       RL observation        离线 replay/导出
       （可选子包）        （torch 批量后端）     （PICO-dataset）
```

三条设计主线：

1. **ROS-free 核心 + 可选桥接**：输入统一为
   `input(points, T_map_sensor, stamp, noise)` 数据类；ROS2 node 只是消费方，
   放在独立子包 / extras 中；
2. **显式位姿注入替代 tf**：维护带时间戳的位姿缓冲，线性插值对齐点云时间戳；
   位姿协方差直接进入融合公式；保留 cupy 版的漂移补偿配方；
3. **双后端（最大差异化价值）**：
   - `cupy` 后端：实机实时建图（包装 cupy_core）；
   - `torch` 后端：mjlab 仿真中 N 个并行环境各自维护一张图，批量输出作为
     PICO-rl 的 observation——Phase 3 设计目标，当前暂缓；与 PICO-rl 已有的
     PyTorch 依赖天然衔接。

## 6. 仓库结构

```
PICO-elevationmap/
├── pyproject.toml              # setuptools + uv，version = 0.4.2，license = MIT
├── uv.lock
├── README.md
├── docs/
│   ├── README.md               # 文档索引
│   ├── design.md               # 本文档
│   ├── development.md          # 开发环境搭建与验证
│   ├── deploy-hardware.md      # 真实硬件部署
│   ├── semantic-extension.md   # 语义扩展（Phase 5 提案）
│   └── concepts.md             # BEV / elevation map 概念
├── src/
│   ├── pico_elevationmap/      # 框架层（核心，零 ROS）
│   │   ├── core/               # 地图数据结构封装、SE3 位姿缓冲、时间戳对齐
│   │   ├── fusion/             # 融合策略注册表（bayesian / average / class_*，代理到 cupy_core）
│   │   ├── plugins/            # 后处理插件链：YAML 配置 + 动态 import
│   │   ├── backends/           # 后端抽象：cupy（GPU 实时）| torch（批量，RL 用）
│   │   ├── sensors/            # 每输入源一个噪声模型配置（YAML）
│   │   ├── cli.py              # 唯一 lazy-import dispatcher
│   │   ├── pipeline/           # run / replay / export（bench 待 Phase 3）
│   │   ├── live/               # D435i 单相机 / 四相机直连 demo
│   │   └── configs/            # 示例 YAML（package data）
│   ├── pico_elevationmap_ros/  # 可选 ROS2 桥接（独立 extras）
│   └── pico_elevationmap_sim/  # mjlab / MuJoCo 集成（Phase 3，未实现）
├── scripts/setup_env.sh        # 一键环境安装
├── tests/                      # pytest，镜像 src 布局，合成数据 fixture
└── .github/workflows/ci.yml    # push/PR CPU gate（lint + test + build）
```

命名约定：包 `pico_elevationmap`，distribution `pico-elevationmap`，
CLI `pico-elevationmap`（dispatcher 模式）。

## 7. 传感器方案

### 7.1 必需输入

1. **深度点云**（一路或多路，每路独立噪声系数）：
   - 旋转激光雷达（Ouster / Velodyne / Livox）：360° 视野、测距远，近地面有盲区；
   - 深度相机（RealSense D435/D455、Azure Kinect、ZED）：近距稠密、带 RGB，怕强光；
   - 仿真理想深度（MuJoCo / mjlab 渲染）；
   - 参考 ANYmal 实机做法：**深度相机补近处脚下 + LiDAR 管中远距**的多源组合。
2. **6DoF 位姿（带协方差）**：LIO / VIO / 腿部里程计（编码器 + IMU）/ 仿真真值；
   必须与点云时间戳对齐；位姿不确定性显式进入融合方差。

### 7.2 可选输入

- RGB / 语义图像 + 相机内外参（MEM 多模态：语义概率融成额外 layer）；
- 本体感知排除区配置（过滤自机点云，实机必备）。

### 7.3 噪声模型

原项目三个 SensorProcessor 子类（StructuredLight / Laser / Stereo）的本质是
**不同的噪声参数化**：激光误差随距离线性增长，深度相机随距离平方增长，双目更大。
新框架退化为"每输入源一组 YAML 配置项"，不做继承体系。

### 7.4 PICO 落地顺序

1. **仿真先行**：mjlab 渲染深度 + 真值位姿 + 可调注入噪声，打通 torch 批量后端
   与 RL observation 链路（Phase 3，当前暂缓）；
2. **实机最小方案**：头部单深度相机 + 腿部里程计（覆盖脚下 0.3–6 m，支撑楼梯/台阶）；
3. **实机完整方案**：深度相机 + 顶部 360° LiDAR 双输入源 + LIO 位姿。

两种实机形态的兼容策略（`main` 单实现 + 配置 profile + 短期 feature branch）见
[sensor-profiles.md](sensor-profiles.md)。

## 8. 工程规范

- **依赖**：uv 管理；`cupy-cuda12x` + numpy 2.x（lock 当前 2.5.4）；
  `elevation-mapping-cupy-core` 锁 git rev；CuPy 的 NVRTC 需要 CUDA 12.x
  headers（`cupy-cuda12x[ctk]` 或 `CUDA_PATH`，见
  [development.md](development.md#3-cuda-运行时与-headers)）；
  torch 待 Phase 3 引入后再用 explicit index（照 PICO-rl 写法）；
- **ruff**：`[tool.ruff]` 段照抄 PICO-rl（120 列、双引号、`E,W,F,I,UP`）；
- **测试**：pytest，tests/ 镜像 src；CI 无 GPU，只跑 CPU 测试并由 conftest 自动
  skip GPU/ROS 用例，不依赖 mock 后端；GPU 路径需在带 CUDA 的机器上单独验证；
- **版本**：SemVer 从 0.1.0 起步，写 pyproject `version`（当前 0.4.2）；
  conventional commits，scope 用 `(map)`；
- **资产联动**：仿真 / 测试消费 PICO-robot MJCF 模型时，照抄 ASSETS.lock +
  check_asset_pins.py 机制；
- **CI**：单文件 push/PR 触发，ubuntu-24.04，CPU lint / test / build；GPU/ROS 用例
  自动 skip；不再依赖 `PICO_READ_PAT` 或 osmesa；
- **开发环境**：前置条件、CUDA headers、GitHub 访问与排障见
  [development.md](development.md)；
- **License**：MIT；第三方 MIT/BSD 声明见
  [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)。

## 9. 实施路线图

| 阶段 | 内容 | 产出 |
|---|---|---|
| Phase 0 — 骨架 | pyproject（uv + setuptools + ruff）、src layout 空包、README、.gitignore、CI、version 0.1.0 | 可安装的空框架 |
| Phase 1 — 核心管线 | 接入 cupy_core；位姿缓冲 + 传感器 YAML + 基础 run 管线；合成点云 pytest 验证融合 / 清理 / 插件链 | 单机可跑通建图 |
| Phase 2 — 插件与 replay | 移植所需插件（inpaint / smooth / normal / traversability）；从 PICO-dataset / rosbag 离线 replay 建图并导出 | 离线评估能力 |
| Phase 3 — torch 批量后端 + RL 集成 | mjlab 仿真深度渲染 → 批量地图 → 注册为 pico_tasks observation | 与 PICO-rl 联调 |
| Phase 4 — 可选 ROS2 桥 | `pico_elevationmap_ros`，grid_map_msgs 转换只存在于桥内 | 实机部署 |

状态：Phase 0 / 1 / 2 / 4 已合入 `main`；Phase 3（torch 批量后端）暂缓，`bench`
子命令仍为 stub。

## 10. 风险与开放问题

- **GPU 依赖**：cupy_core 无 CPU fallback；CI / 开发机无 GPU 时只跑 CPU 测试并
  skip GPU 用例，不声称 GPU 路径已通过 CI；
- **上游 API 稳定性**：cupy_core 是 2026 年新抽出的库，锁 git rev + 定期人工升级
  （符合家族"dependency bumps are manual PRs"惯例）；
- **CUDA 环境**：CuPy 后端依赖 CUDA 12.x 运行时与 headers；驱动 13.x 向后兼容
  CUDA 12.x（已在 RTX 4060 Laptop + 驱动 595.99 / CUDA 13.2 验证）。无系统
  toolkit 的机器需安装 `cupy-cuda12x[ctk]` 或设置 `CUDA_PATH`，见
  [development.md](development.md#3-cuda-运行时与-headers)；aarch64 / DGX Spark
  （sm_121 + CUDA 13）迁移仍需单独验证；
- **torch 后端的取舍**：批量后端是重写融合 kernel（工作量大）还是 N 个 cupy 实例循环
  （简单但难上 batch 梯度），需在 Phase 3 前做 spike 验证；
- **多地图 / 回环**：上游均不支持，本期不做，列为远期开放问题。

## 参考

- Fankhauser et al., *Probabilistic Terrain Mapping for Mobile Robots with Uncertain Localization*, RA-L 2018
- Miki et al., *Elevation Mapping for Locomotion and Navigation using GPU*, IROS 2022
- Erni et al., *MEM: Multi-Modal Elevation Mapping for Robotics and Learning*, IROS 2023
- https://github.com/leggedrobotics/elevation_mapping_cupy （ros2 分支）
- https://github.com/leggedrobotics/elevation_mapping_cupy_core
- https://github.com/ANYbotics/elevation_mapping
- https://github.com/ANYbotics/grid_map
