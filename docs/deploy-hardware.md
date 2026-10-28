# 真实硬件部署与验证方案

> 版本：v0.1　日期：2026-10-28
> 目标：把 PICO-elevationmap（cupy GPU 核心 + 可选 ROS2 桥）部署到真实机器人并验证建图效果。

## 1. 硬件配置

### 1.1 核心三件套

| 部件 | 最小方案（先跑通） | 完整方案 |
|---|---|---|
| 机载计算 | Jetson Orin NX/AGX（16 GB+，aarch64 + CUDA） | x86 小主机 + RTX 4060 级独显，或 DGX Spark（见 §5） |
| 深度相机 | RealSense D435 ×1（头部 `camera_link`） | D435i / D455（带 IMU 更佳） |
| 激光雷达 | 可不要 | Livox Mid-360（躯干 `lidar_link`，性价比 + ROS2 驱动成熟）或 Ouster OS0 |
| 位姿来源 | 机器人关节编码器 + IMU 的腿部里程计 | 腿部里程计，或升级 FAST-LIO（Mid-360 内置 IMU，直接出带协方差位姿） |

### 1.2 机器人本体选型

- **Unitree Go2 起步最省事**：自带 4D LiDAR L1 + SDK 完善，无需外挂传感器；
- Fourier charming 等自研人形：`camera_link`（挂 `head_yaw_link`）/ `lidar_link`
  （挂 `torso_link`）挂载点已在模型中预留（见 PICO-robot 仓 `fourier_charming/v100`），
  但传感器支架、供电、走线需自行解决；`model.yaml` 中传感器 `model` 字段尚待填写。

### 1.3 计算平台权衡

- **Jetson Orin**：cupy 有 aarch64 wheel 可跑，但算力紧张——建议降地图分辨率
  （0.06–0.08 m/格）并裁剪图层；
- **DGX Spark**：性能过剩，适合基站 / 开发机；部署细节与生态风险见 §5；
- **首验最稳路径**：x86 小主机 + RTX 4060/4070 背挂，或直接 tether 到工作站——
  验证阶段可靠性优先。

## 2. 连接拓扑

```
                        ┌─────────────────────────────┐
 RealSense D435 ──USB3──▶│                             │
 (点云/深度图+IMU)        │                             │
                         │   机载计算机 / 工作站        │
 Livox Mid-360 ──以太网──▶│  Ubuntu 24.04 + ROS2 Jazzy  │
 (点云+内置IMU)           │  ├─ realsense-ros 驱动      │
                         │  ├─ livox_ros_driver2       │
 机器人本体 ─────以太网──▶│  ├─ robot SDK bridge        │
 (关节状态/IMU/里程计)     │  │   (JointStates→腿部里程计)│
                         │  ├─ FAST-LIO (可选)         │
                         │  └─ pico-elevationmap-ros   │
                         └─────────────┬───────────────┘
                                       │ DDS（同网段）
                                       ▼
                              监控端 RViz2 / 录 rosbag
```

## 3. 工程要点（实机翻车重灾区）

1. **时间同步是第一要务**：位姿缓冲靠时间戳对齐点云与位姿。机载计算机做 ROS
   时间源；LiDAR 走 PTP 或驱动软件时间戳；机器人 SDK 数据打本机接收时间。
   时间戳错位 >50 ms 时快速转身地图直接糊。
2. **外参标定**：`camera_link` / `lidar_link` 到 `base_link` 的外参写入 URDF/TF；
   PICO-robot MJCF 中的挂载点坐标转换为 ROS static TF 即可。
3. **带宽**：D435 点云独占一个 USB3 控制器口；Mid-360 用千兆网口；WiFi 监控只开
   1–2 Hz 图层发布，勿在无线链路上跑原始点云。
4. **供电**：传感器与计算机经独立 DC-DC 从机器人电池取电，勿用逻辑电源
   （电机启停会拉崩电压）。
5. **自机过滤**：机器人自身几何（腿、机身）必须配置排除区，否则自机点云会建进地图。

## 4. 验证节奏（每步有明确退出标准）

| 阶段 | 配置 | 验证目标 |
|---|---|---|
| 1. 台架 | 传感器固定三脚架 + 工作站，手持行走 | replay 链路：录 rosbag → 离线 `pico-elevationmap replay` → 对照卷尺实测（台阶高度误差 < 2 cm） |
| 2. 手持行走 | 同上，绕场走圈（含楼梯） | 位姿漂移下地图连贯性；drift compensation 效果 |
| 3. 上机静态 | 传感器 + 计算机装机器人，机器人站立，人推箱子造地形变化 | ROS2 桥实时链路：端到端延迟 < 100 ms；目标平台上的单帧耗时实测 |
| 4. 上机行走 | 完整配置 | 平地 → 台阶 → 斜坡；traversability 图层与规划/控制接口联调 |

阶段 1 门槛最低：一台 RealSense + 现有 GPU 工作站即可，录制用 `ros2 bag record`，
回放走本仓库纯 Python rosbags 路径（无需装 ROS）。

### 4.1 真机 ROS 集成测试

`tests/` 中带 `requires_ros` 标记的集成测试需要在装了 ROS2 的机器上跑：

- GPU：NVIDIA，驱动支持 CUDA 12.x+（核显不行，cupy_core 无 CPU fallback）；
- OS：Ubuntu 24.04 + ROS2 Jazzy（`rclpy`、`sensor_msgs`、`geometry_msgs`、
  `nav_msgs`、`std_srvs`、`grid_map_msgs`，即 `ros-jazzy-grid-map-msgs`）；
- Python：3.12 + uv；venv 需能看到系统 ROS 包
  （`--system-site-packages` 或 source ROS 后用系统 Python）。

## 5. DGX Spark 平台评估（GB10 Grace-Blackwell）

结论：**可行且性能过剩，但属生态早期平台**，适合基站 / 开发机，不适合跑仿真。

- 规格：20 核 ARM（aarch64）+ sm_121 GPU（48 SM）+ 128 GB 统一内存；
  算力约 RTX 4090 的 1/3，折算本仓库工作负载（4090 实测 2.5 ms/step）约
  8–10 ms/step，30–100 Hz 实时建图无压力；统一内存省一次 CPU→GPU 搬运。
- **CUDA 版本线**：sm_121a 需 **CUDA 13**（CUDA 12.9 的 ptxas 不认），迁移时用
  `cupy-cuda13x`（CuPy 官方为 CUDA 12.x/13.x 均提供 x86_64 / aarch64 wheel）；
  上游 kernel 为运行时 NVRTC 编译，换 CUDA 大版本只改依赖声明。
- **aarch64 生态**：cupy / numpy / scipy / opencv / shapely 均有 aarch64 wheel，
  rosbags 纯 Python，ROS2 Jazzy 有官方 arm64 deb——链路通，但部分库
  （如 PyTorch aarch64 wheel arch list 止于 sm_120，靠 PTX JIT 兼容）需预留调试时间。
- **不要在 Spark 上跑仿真**：Isaac Sim 的 PhysX GPU 在 GB10 有已知问题，
  MuJoCo/mjlab 在 ARM Linux 非主流；仿真训练留在 x86 工作站。

## 6. 传感器方案速查（详见 design.md §7）

- 必需：一路或多路深度点云 + 带协方差的 6DoF 位姿（时间戳对齐）；
- 推荐组合：头部深度相机（脚下 0.3–6 m 稠密，看台阶）+ 躯干 360° LiDAR（中远距）；
- 噪声模型：激光误差随距离线性增长，深度相机随距离平方增长；
  配置为每源 YAML（`laser | depth_camera | perfect`）；
- 待补：实机深度相机输出的是深度图，需"深度图 + 内外参 → 点云"前置投影
  （与语义扩展共用基础设施，见 semantic-extension.md）。
