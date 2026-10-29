# 双传感器方案：仅深度相机 vs 深度相机 + LiDAR

> 版本：v0.3　日期：2026-10-29　状态：方案建议，M1/M2/M3 原型已跑通（D435i + Airy），精确标定/时间同步待完善
> 关联：[design.md](design.md) §7、[deploy-hardware.md](deploy-hardware.md) §1/§6

## 1. 需求

PICO-elevationmap 需要支持两种部署形态，共用同一套建图核心：

| Profile | 传感器 | 位姿来源 | 典型场景 |
|---|---|---|---|
| `camera_only` | 头部深度相机（RealSense D435/D455 等） | 腿部里程计 / IMU / VIO | 低成本、轻量机器人；先跑通 |
| `camera_lidar` | 深度相机 + 躯干 360° LiDAR（Livox Mid-360 / Ouster 等） | LIO 或腿部里程计 | 全地形、远距与遮挡场景 |

### 1.1 目标

- 同一 commit、同一套核心代码支持两种 profile，切换只改 YAML/launch；
- 两种 profile 共用 `ElevationMapper`、插件链、replay/export 与测试基础设施；
- 摄像头方案既能消费驱动输出的点云，也能消费深度图 + 相机内参；
- 文档、测试、CI 同时覆盖两种组合。

### 1.2 非目标

- 纯 RGB 单目深度估计 / 学习式 BEV（输入不是几何深度，需另立方案）；
- 多地图 / 回环（上游不支持，见 design.md §10）；
- 语义融合（Phase 5，见 semantic-extension.md）。

## 2. 现状评估

已经具备的基础：

- 核心支持多输入源：`MapperConfig.sources`、`primary_source`、按名 `resolve_source`
  （`src/pico_elevationmap/core/mapper.py:70-124`）；
- 每源独立噪声模型与量程：`laser` / `depth_camera` / `perfect`
  （`src/pico_elevationmap/sensors/config.py`）；
- ROS 桥按“源名 → 点云 topic”映射：`ros.point_topics`
  （`src/pico_elevationmap_ros/config.py:10-13`），天然支持 N 路 PointCloud2；
- `design.md` §7.4 的落地顺序本来就是“先单深度相机，再深度相机 + LiDAR”。

真正的缺口：

1. **深度图 → 点云**：实机深度相机通常输出 `sensor_msgs/Image` + `CameraInfo`，
   而不是 PointCloud2；当前 ROS 桥只订阅 PointCloud2（`deploy-hardware.md` §6 已标为待补）。
2. **多源噪声的全局限制**：上游内核的 `sensor_noise_factor` 是全局参数，当前取
   主输入源的映射值（`mapper.py:9-13, 75-77`）；两源共享一张图时，逐源方差并未真正
   独立生效，需要用保守系数或后续扩展。
3. **深度图 rosbag replay**：`io.iter_bag_frames` 目前以 PointCloud2 为主，
   直接回放 Image 话题需要扩展。

## 3. 方案选择

### 3.1 建议

**在 `main` 上做兼容性开发，不拆两个长期分支。**

- `main` 保持唯一实现；两种形态由 **配置 profile** 表达；
- 具体能力用短期 topic branch 开发，完成后合回 `main`：
  `feat/config-profiles`、`feat/depth-image-projection`、`feat/lidar-multisource`；
- 不在核心代码里散落 `if camera_only`；差异收敛在配置与传感器适配层。

### 3.2 备选方案

| 方案 | 评价 |
|---|---|
| 两个长期分支（`camera-only` / `camera+lidar`） | 否决。核心修复、测试、文档要维护两份，配置 schema 易漂移，合并冲突集中爆发 |
| Fork / vendor 两套代码 | 否决。同一套算法与插件，重复维护没有收益 |
| `main` + 配置 profile + 短期 feature branch | **采纳**。差异在输入层，核心不分叉 |
| 插件/适配器边界（若后续差异变大） | 作为 `main` 方案的演进方向，而非另起分支 |

### 3.3 何时重新评估

只有出现以下情况才考虑独立发布分支或 tag，而不是日常开发分支：

- 纯 RGB 单目方案，需要引入单目深度估计 / 学习式 BEV，输入语义与几何管线不同；
- 两个方案依赖互斥的硬件 SDK 或算法路线；
- 两个方案需要完全独立的发布节奏。

## 4. 设计

### 4.1 分层与数据流

```
depth image + CameraInfo + extrinsics ─┐
                                      ├─→ 点云 (N,3) + stamp + source
LiDAR PointCloud2 ────────────────────┘
                                              │
                                              ▼
                                   ElevationMapper（零 ROS）
                                              │
                                  插件链 / replay / export
```

- 核心仍只接收传感器系点云，不 import ROS；
- 深度图投影、PointCloud2 解包、TF/外参查表都在 ROS 桥或传感器适配层完成；
- 两个 profile 的 source 名保持稳定：`depth`（必需）、`lidar`（可选）。

### 4.2 配置 profile

**M1（不改核心）**：直接使用 RealSense 驱动输出的点云 topic，摄像头方案零代码即可跑：

```yaml
# profiles/camera_only.yaml
sources:
  depth:
    sensor_type: depth_camera
    noise_coefficient: 0.002
    min_range: 0.4
    max_range: 6.0
ros:
  point_topics:
    depth: /camera/depth/color/points
  pose_topic: /leg_odom
  pose_type: odometry
```

```yaml
# profiles/camera_lidar.yaml
sources:
  depth:
    sensor_type: depth_camera
    noise_coefficient: 0.002
    min_range: 0.4
    max_range: 6.0
  lidar:
    sensor_type: laser
    noise_coefficient: 0.005
    min_range: 0.5
    max_range: 30.0
ros:
  point_topics:
    depth: /camera/depth/color/points
    lidar: /livox/lidar
  pose_topic: /lio/odom
  pose_type: odometry
```

**M2（深度图直连）**：在 `ros` 段增加可选的深度图输入，和 `point_topics` 二选一：

```yaml
ros:
  depth_images:
    depth:
      image: /camera/depth/image_rect_raw
      camera_info: /camera/depth/camera_info
```

约束：每个 source 只能出现在 `point_topics` 或 `depth_images` 之一；两者都未配置的
source 在启动时报错，避免静默丢数据。

### 4.3 深度图 → 点云

给定深度图 `D`、内参 `fx, fy, cx, cy`：

```
Z = D[v, u] / depth_scale
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
```

处理要点：

- 过滤 `Z <= 0`、非有限值，以及 `min_range`/`max_range` 之外的点；
- 用外参把点从相机光学系变换到 `base_link`/传感器系，再交给 `mapper.input()`；
- 时间戳取 `Image.header.stamp`，与位姿缓冲的 stamp 对齐；
- 可选 `stride` 降采样（如 2×2），控制单帧点数与 CPU 开销；
- 先做 numpy 向量化，保持 ROS-free 的纯函数，便于单测。

### 4.4 多源融合与已知限制

- 每源量程裁剪与位姿噪声已支持；
- 上游 `sensor_noise_factor` 是内核级全局参数，当前只能取一个值。两源同时工作时：
  - M1/M2 先用保守系数（例如取较小噪声系数，避免过度信任某一源），并在 profile 中显式注释；
  - M3 评估逐源方差方案（包装层缩放、或上游参数扩展），不把“每源独立噪声”当成已实现能力；
- 时间同步是双源方案的第一风险：LiDAR 走 PTP/驱动时间戳，相机与位姿打同一时钟，
  详细要求见 `deploy-hardware.md` §3。

### 4.5 测试与验收

- 单元测试：已知内参 + 合成深度图 → 期望点云；无效深度、外参、stride；
- 配置测试：两个 profile 都能被 `MapperConfig.from_yaml` / `RosConfig.from_dict` 解析；
- 集成测试：同一组 mapper 用例参数化跑单源/双源；ROS 桥用桩测深度图转换；
- replay：M4 扩展 Image 话题回放，或先用“录好点云”的 bag 验证；
- 验收标准：
  - 同一 commit 切换 profile 即可运行两种方案；
  - `uv run pytest` 覆盖两种 profile；
  - 摄像头方案在 M1 用驱动点云跑通，M2 用深度图跑通；
  - 双源方案在台架/手持阶段完成时间同步与外参验证。

### 4.6 原型验证（2026-10-28）

- `src/pico_elevationmap/live/realsense_live.py` 已用 D435i 直连验证 depth → point cloud →
  `ElevationMapper` 的完整链路；
- 实现方式：`pyrealsense2` 取 640×480@30 深度图，按内参反投影；在光学系下半
  部分（y > 0）用 RANSAC 估计主平面并排除墙面，构造 Z-up 地图系，不经过 ROS；
- 实测：`--stride 2` 约 3.7–7.2 万点/帧，稳定后约 8–11 ms/step；桌面场景 8 m 地图
  覆盖率 6.1%，走廊场景 3.0%（可见地面面积小），抬高相机并下俯后应改善；
- `tests/test_realsense_live.py` 覆盖反投影、无效值、stride、旋转与地面估计；
- 待办：把同一投影函数接到 ROS 侧 `depth_images` schema，并补 Image 话题 bag replay。

### 4.7 RoboSense Airy 双源验证（2026-10-29）

- 硬件：D435i + RoboSense Airy（Ethernet，LiDAR `192.168.1.200`，主机
  `192.168.1.102/24`）；
- 驱动：`rslidar_sdk` ROS2，`lidar_type: RSAIRY`，topic `/rslidar_points`，
  实测约 8–10 Hz、6.5–7.1 万点/帧；
- 运行：`realsense_live.py --lidar-topic /rslidar_points`，使用
  `.venv-ros`（`--system-site-packages`）同时看到 rclpy 与项目依赖；
- 结果：30 个相机帧融合了 15 个 LiDAR 帧，12 m 地图覆盖率 6.3%
  （3634/57600 cells），3D 体素视图约 2.9 万格；相机建图均值约 4.6 ms/step
  （不含 ROS/LiDAR 接收与渲染）；
- 限制：当前用相机 stamp 作为统一时间戳，未做 LiDAR/相机硬件时间同步；外参做了
  旋转后自动地面 z 对齐 + 手动 `--lidar-roll/--lidar-pitch/--lidar-yaw/--lidar-offset`
  （倒装可用 roll 或 pitch=180°），精确标定待做；上游全局 `sensor_noise_factor`
  仍由主源（深度）决定。
- Livox Mid-360 代码路径已补：`livox_mid360.yaml`、`--lidar-msg-type custom`
  （CustomMsg）、`launch/bringup.launch.py`（`msg_MID360_launch.py`）；本机没有
  Livox 硬件，CustomMsg 解析由单元测试覆盖，真机待验证。

## 5. 实施计划

| 里程碑 | 分支 | 内容 | 退出标准 |
|---|---|---|---|
| M1 配置 profile | `feat/config-profiles` | 两个 YAML、ROS 配置说明、文档与解析测试 | 两 profile 在同一 commit 解析通过；摄像头方案用驱动点云跑通 |
| M2 深度图投影 | `feat/depth-image-projection` | `depth_images` schema、Image+CameraInfo→点云、单测 | 摄像头方案不依赖驱动点云；单测覆盖投影/无效值/外参 |
| M3 双源融合 | `feat/lidar-multisource` | LiDAR source、双源参数、时间同步/噪声评估 | **原型已跑通（D435i + Airy）**；精确标定/时间同步待完善 |
| M4 replay 与收尾 | `feat/replay-image-bags`（可选） | Image 话题 bag 回放、文档回填 | 两种 profile 的 replay 路径可用 |

所有分支保持短期，合入 `main` 后删除；不建立长期 `camera-only` / `camera-lidar` 分支。

## 6. 风险与开放问题

- **全局噪声系数**：双源共享一个 `sensor_noise_factor`，逐源方差尚未实现；
- **深度图投影开销**：640×480 单帧约 30 万点，需 stride/量程裁剪，避免 CPU 成为瓶颈；
- **外参与时序**：相机-LiDAR-本体外参、相机与 LiDAR 时间戳对齐是实机翻车重灾区；
- **遮挡与盲区**：仅相机时脚下遮挡/强光；加 LiDAR 后近地盲区与自机点云过滤；
- **驱动差异**：realsense-ros 与 Livox/Ouster 的 topic 名、frame_id、点云字段不同，
  profile 需按硬件固化；
- **Jetson 算力**：双源点数增加，需按 `deploy-hardware.md` §1.3 降分辨率/裁图层；
- **纯 RGB 单目**：不在本方案范围；若要做，需重新评估是否独立分支。

## 7. 参考

- [design.md](design.md) §7（传感器方案与落地顺序）
- [deploy-hardware.md](deploy-hardware.md) §1（硬件配置）、§3（时间同步/外参）、§6（待补投影）
- `src/pico_elevationmap/core/mapper.py`（多源配置与全局噪声限制）
- `src/pico_elevationmap_ros/config.py`（`point_topics` / `RosConfig`）
- `src/pico_elevationmap/core/io.py`（bag replay 当前以 PointCloud2 为主）
