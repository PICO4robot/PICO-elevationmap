# PICO-elevationmap

[![CI](https://github.com/PICO4robot/PICO-elevationmap/actions/workflows/ci.yml/badge.svg)](https://github.com/PICO4robot/PICO-elevationmap/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.12](https://img.shields.io/badge/python-3.12-blue.svg)](pyproject.toml)

PICO-elevationmap is a ROS-free 2.5D elevation mapping library for PICO quadrupeds and
humanoids, split out of an internal PICO monorepo.

What sets it apart from the upstream ROS packages: poses come from a timestamped SE3
buffer instead of tf, sensor noise and the postprocessing plugin chain are YAML-driven,
and the same map can be replayed offline from npz sequences or rosbags — no ROS graph
required. Documentation index: [docs/README.md](docs/README.md) (framework design,
development environment, hardware deployment plan, API, release readiness, semantic
extension proposal, concepts; English summaries in [docs/en/README.md](docs/en/README.md)).

## Support matrix

| Area | Status |
|---|---|
| Platform | x86_64 Linux, NVIDIA GPU, CUDA 12.x headers |
| Python | 3.12 (CI target); 3.10/3.11 are not supported until a CI matrix exists |
| aarch64 / Jetson | Experimental; no CI coverage |
| ROS2 bridge | Optional; requires a sourced ROS2 environment. No ament package yet — use `uv run pico-elevationmap-ros --config ...` |
| RealSense / LiDAR | Optional; D435i direct-SDK and ROS2 LiDAR paths are prototype quality |
| Four-camera D435i rig | Config + single-camera subset tested; four-device hardware unverified |
| torch backend / `bench` | Deferred (Phase 3) |
| License | MIT (see [LICENSE](LICENSE)) |
| Production readiness | Not production-ready; see [docs/open-source-readiness.md](docs/open-source-readiness.md) |

The core library and CPU tests install and run without a GPU; mapping itself needs a
CUDA device at runtime.

## Layout

```
src/pico_elevationmap/       # 核心包（零 ROS 依赖）
  core/                      # pose_buffer（SE3 位姿缓冲）、mapper（建图外观）、io（帧/地图/rosbag/导出）
  sensors/                   # 每输入源噪声模型 YAML（laser / depth_camera / perfect）
  plugins/                   # 插件链装配 + 自有 geo_slope / geo_traversability
  fusion/                    # 融合策略代理到上游（占位，跟随 cupy_core）
  backends/                  # 后端抽象占位（torch 批量后端为 deferred Phase 3）
  cli.py                     # 唯一 lazy-import dispatcher（`pico-elevationmap` console script）
  pipeline/                  # run / replay / bench / export 入口模块
  live/                      # D435i 单相机 / 四相机直连 demo（可选 extra）
  configs/                   # 示例 YAML（作为 package data 打入 wheel）
src/pico_elevationmap_ros/   # 可选 ROS2 桥接（rclpy 依赖仅存在于该包 node.py）
scripts/setup_env.sh         # 一键源码环境安装
tests/                       # pytest，镜像 src；GPU 用例由 conftest 的 requires_gpu 条件自动 skip，ROS 集成用例由 requires_ros 条件自动 skip（均非 pytest marker）
docs/                        # 设计、开发环境、硬件部署、API、开源发布、语义扩展、概念
```

## Install

Prerequisites and troubleshooting (CUDA headers, GitHub access, ROS2 venv) are in
[docs/development.md](docs/development.md). Mapping needs an NVIDIA GPU at runtime;
installation itself is CPU-friendly.

```bash
git clone https://github.com/PICO4robot/PICO-elevationmap.git
cd PICO-elevationmap

bash scripts/setup_env.sh   # one-click: source sync + CUDA headers + RealSense + checks
```

`setup_env.sh` syncs from the source checkout with `--extra cuda --extra realsense`,
verifies NVRTC and the RealSense SDK, and runs pytest. Add `--ros` to also create
the ROS-visible `.venv-ros`, `--skip-tests` to skip pytest, or `--dry-run` to preview.
Equivalent manual steps:

```bash
uv sync              # core + dev tools (pytest / ruff)
uv sync --extra cuda --extra realsense  # driver-only CUDA headers + D435i direct SDK
uv sync --extra ros  # ROS semantic marker; system ROS2 provides rclpy / grid_map_msgs
```

On a driver-only machine, make sure NVRTC can find CUDA 12.x headers before the first
run (`uv sync --extra cuda`, `cupy-cuda12x[ctk]`, or `CUDA_PATH`); see
[development.md §3](docs/development.md#3-cuda-运行时与-headers).

Wheels bundle the example YAML configs under `src/pico_elevationmap/configs/` (including
`ros_camera_lidar.yaml`). `launch/bringup.launch.py` remains **repo-only** and is
not packaged into the wheel; copy it from a source checkout if needed. To locate the
bundled configs in an installed wheel:

```bash
python -c "from importlib.resources import files; print(files('pico_elevationmap.configs'))"
```

The wheel's `elevation-mapping-cupy-core` dependency resolves from a git revision, so a
plain `pip install dist/*.whl` needs `--no-deps`; use `bash scripts/setup_env.sh` or a
source `uv sync` for a fully resolved install. The project is GitHub-only (no PyPI
release), see [docs/open-source-readiness.md](docs/open-source-readiness.md).

## Quickstart

```bash
# 合成场景实时建图 demo（平地 + 方台 + 台阶，圆周轨迹）
uv run pico-elevationmap run --config src/pico_elevationmap/configs/run_sim.yaml \
    --save maps/map.npz --save-frames maps/frames.npz

# 离线回放：帧序列 / 逐帧目录 / rosbag（.bag/.db3/.mcap，无需安装 ROS）
uv run pico-elevationmap replay --config src/pico_elevationmap/configs/run_sim.yaml \
    --input maps/frames.npz --save maps/replay_map.npz --export-ply maps/map.ply

# 导出已保存地图：PLY（点云）/ PGM（16 位高程图）/ CSV
uv run pico-elevationmap export --input maps/replay_map.npz --layers elevation --output maps/elevation.pgm
```

`--save` / `--save-frames` 会自动创建父目录；`maps/` 已在 `.gitignore` 中。

配置 YAML 分 `map`（分辨率/门限等）、`sources`（每源噪声与量程）、`plugins`
（上游 inpainting/smoothing + 自有 geo_*）、`sim`（run 的仿真参数）、`ros`
（桥接参数）五段，键名拼写与取值越界都会报明确错误；示例见
`src/pico_elevationmap/configs/run_sim.yaml`。

## Visual proof

![D435i live elevation/traversability and 3D voxel view](https://raw.githubusercontent.com/PICO4robot/PICO-elevationmap/main/docs/assets/realsense_window.png)

D435i live session: fixed absolute colour bands plus traversability (top) and the
accumulated 3D voxel view (bottom). This is prototype-quality output from a static
camera, not a calibrated deployment; see the RealSense section below. Images live in
`docs/assets/`; the README uses absolute raw URLs so PyPI renders them.

## Development

```bash
env -u PYTHONPATH uv run pytest -q
uv run ruff check .
uv run ruff format --check .
bash -n scripts/setup_env.sh
```

GPU and ROS integration cases skip automatically when the corresponding hardware or
environment is absent; the suite does not hardcode a pass count. Open work is tracked
in [TODO.md](TODO.md). Environment setup,
CUDA headers, GitHub access and troubleshooting are in
[docs/development.md](docs/development.md).

## RealSense live (optional)

D435i 直连建图，不依赖 ROS：

```bash
uv sync --extra cuda --extra realsense

# 实时 2D + 3D：同一个窗口，上半高程/traversability/色标，下半 3D 体素
uv run python -m pico_elevationmap.live.realsense_live \
    --config src/pico_elevationmap/configs/realsense_live.yaml \
    --view --voxel-view --frames 0 \
    --z-min -0.1 --z-max 1.0 --color-bands 12 \
    --voxel-size 0.05 --voxel-save maps/realsense_voxel.png \
    --view-save maps/realsense_window.png

# 或采 30 帧后保存
uv run python -m pico_elevationmap.live.realsense_live \
    --config src/pico_elevationmap/configs/realsense_live.yaml \
    --frames 30 --save maps/realsense_map.npz \
    --save-frames maps/realsense_frames.npz \
    --export-ply maps/realsense.ply --export-pgm maps/realsense.pgm
```

实时 2D 颜色用**固定绝对高度范围** `--z-min/--z-max`（默认 -0.1/1.0 m），
12 个高度带 + 坡度明暗 + 层间线 + 右侧米制色标，同色即同高；`--color-bands 1`
可切回连续色带。3D 体素窗口按 `--voxel-size`（默认 5 cm）维护概率占用格：每个
体素保存 log-odds，命中增加、射线穿过减少；只绘制概率高于 `--voxel-occ-prob`
（默认 0.65）的体素，小方块的颜色亮度和透明度随概率变化（`--voxel-alpha`
为最大透明度，默认 0.55）。先点击 3D 窗口获得焦点：鼠标左键拖拽旋转、滚轮缩放；
也可用 `a/d` 偏航、`w/s` 俯仰、`+/-` 缩放，`q` 退出。动态障碍物离开后，默认每
3 帧沿当前深度射线做 free 更新，并用 `--voxel-ttl`（默认 2 s）清理未再命中的体素；
`--no-voxel-carve` 可关闭射线清除。`--voxel-hit-log-odds`、`--voxel-miss-log-odds`
可调命中/穿透证据强度。当前脚本假设静态相机；接上里程计后 3D 视图会跟随地图中心移动。

脚本会按深度图内参反投影、自动估计近场主平面并构造 Z-up 地图系。相机抬高并
下俯 20–30°、让地面进入视野时效果最好；桌面掠射视角的覆盖率会偏低。

## D435i + RoboSense Airy (optional)

Airy 通过以太网接入，默认 LiDAR IP `192.168.1.200`，主机网卡配
`192.168.1.102/24`。用 RoboSense `rslidar_sdk` 的 ROS2 驱动发布
`/rslidar_points`（`lidar_type: RSAIRY`），再用带系统 ROS 可见性的 venv
运行双源脚本：

```bash
# 终端 A：启动 Airy 驱动
source /opt/ros/jazzy/setup.bash
source /path/to/rslidar_ws/install/setup.bash
ros2 run rslidar_sdk rslidar_sdk_node --ros-args \
    -p config_path:=$PWD/src/pico_elevationmap/configs/airy_rslidar.yaml

# 终端 B：双源建图
uv venv --system-site-packages .venv-ros
UV_PROJECT_ENVIRONMENT=.venv-ros uv sync --extra cuda --extra realsense

source /opt/ros/jazzy/setup.bash
source /path/to/rslidar_ws/install/setup.bash

.venv-ros/bin/python -m pico_elevationmap.live.realsense_live \
    --config src/pico_elevationmap/configs/realsense_lidar.yaml \
    --lidar-topic /rslidar_points --frames 30 \
    --save maps/dual_map.npz --export-ply maps/dual.ply
```

`--lidar-z-offset` 缺省时按旋转后点云 5% 分位估计地面；`--lidar-yaw`、
`--lidar-roll`、`--lidar-pitch` 和 `--lidar-offset X Y Z` 用于手工外参。
默认假设 LiDAR z 轴朝上；**倒装时用 `--lidar-roll 180`（绕 x 翻转）或
`--lidar-pitch 180`（绕 y 翻转）**，具体取决于实际翻转轴。当前验证使用相机
时间戳做统一对齐，精确的相机-LiDAR 时间同步和外参标定仍待补。

![D435i + RoboSense Airy 3D voxel view](https://raw.githubusercontent.com/PICO4robot/PICO-elevationmap/main/docs/assets/dual_voxel.png)

D435i + Airy dual-source voxel view (prototype; extrinsics/time sync still manual).

## Four-camera D435i rig (optional)

`src/pico_elevationmap/live/quad_camera_live.py` 支持前后左右四台 D435i，各相机使用独立
serial 和显式外参，点云统一变换到 base 系（x 前、y 左、z 上）后送入同一张地图。

```bash
# 查看已连接的 RealSense 序列号
uv run python -m pico_elevationmap.live.quad_camera_live --list

# 编辑 quad_camera.yaml，替换 REPLACE_*_SERIAL
uv run python -m pico_elevationmap.live.quad_camera_live \
    --config src/pico_elevationmap/configs/quad_camera.yaml \
    --view --voxel-view --frames 0
```

`quad_camera.yaml` 中每台相机包含 `serial`、`source`、`xyz`、`roll_deg`、
`pitch_deg`、`yaw_deg`、`width`、`height`、`fps`。默认外参是前/后/左/右四个
朝向，pitch 向下 20°；`xyz` 为相机在 base 系的位置。只验证单路时可加
`--cameras front`。

四台 640×480@15 需要足够的 USB3 带宽和多控制器/供电 HUB；掉流时先降到
`fps: 10` 或 `480x360`。本仓库只在单台 D435i 上验证了单路 rig 路径，四台
同时工作的硬件验证仍待完成。

## ROS2 launch (bringup)

`launch/bringup.launch.py` 用一条命令拉起 LiDAR 驱动 + D435i + 建图窗口。先
`source` ROS2 和对应驱动工作区，并准备好 `.venv-ros`。该 launch 文件是
**repo-only 示例，不包含在 wheel 中**；wheel 用户需从源码 checkout 获取。
若 `.venv-ros/bin/python` 不存在，launch 会先给出可操作的错误（可用
`python:=/path/to/python` 覆盖），而不是等到 spawn 才失败。

```bash
# Livox Mid-360（CustomMsg）
ros2 launch ./launch/bringup.launch.py \
    lidar:=livox lidar_topic:=/livox/lidar lidar_msg_type:=custom \
    config:=$PWD/src/pico_elevationmap/configs/livox_mid360.yaml

# RoboSense Airy（PointCloud2）
ros2 launch ./launch/bringup.launch.py \
    lidar:=airy lidar_topic:=/rslidar_points lidar_msg_type:=pointcloud2 \
    config:=$PWD/src/pico_elevationmap/configs/realsense_lidar.yaml

# 只跑 D435i
ros2 launch ./launch/bringup.launch.py \
    lidar:=none config:=$PWD/src/pico_elevationmap/configs/realsense_live.yaml
```

默认 `view:=true voxel_view:=true frames:=0`，`foxglove:=false`；按 `q` 退出建图后
launch 会一并关闭驱动；`start_delay:=3.0` 是驱动启动到建图启动的默认等待，
`warmup:=15` 是建图前丢弃的相机帧数（短帧数采集 LiDAR 时可调大）。Livox 默认
`livox_launch:=msg_MID360_launch.py`（CustomMsg）；如果驱动改成 PointCloud2，
把 `lidar_msg_type` 改为 `pointcloud2` 并使用对应的 PointCloud2 launch 文件。

## Foxglove (optional)

`--foxglove` 会把高程点和体素中心发布为标准 `sensor_msgs/PointCloud2`
（带 rgb 颜色），配合 ROS2 `foxglove_bridge` 可在 Foxglove Studio 中看 3D：

```bash
source /opt/ros/jazzy/setup.bash
.venv-ros/bin/python -m pico_elevationmap.live.realsense_live \
    --config src/pico_elevationmap/configs/realsense_lidar.yaml \
    --lidar-topic /rslidar_points \
    --foxglove --view --voxel-view --frames 0

# 另一个终端
ros2 run foxglove_bridge foxglove_bridge
```

Topic：

- `/pico_elevationmap/elevation_points`
- `/pico_elevationmap/voxels`

Foxglove Studio 连接 `ws://<host>:8765` 后，在 3D 面板添加这两个 PointCloud2
即可。`foxglove_bridge` 需要单独安装/编译；它只负责桥接，不改变建图逻辑。
`--foxglove` 也可以和 `lidar:=none` 的相机-only 模式一起用。

## ROS2 bridge (optional)

`pico_elevationmap_ros` 把 `ElevationMapper` 接入 ROS2：订阅 N 路 PointCloud2 +
一路 Odometry/PoseWithCovarianceStamped（位姿协方差推导的噪声传入融合触发
漂移补偿），按 `publish_fps` 定时发布 grid_map_msgs/GridMap（图层可配），
并提供 `~/clear`、`~/save`（Trigger）服务。真实环境运行（需先 source ROS2）：

```bash
source /opt/ros/jazzy/setup.bash
uv run pico-elevationmap-ros --config src/pico_elevationmap/configs/ros_camera_lidar.yaml
uv run pico-elevationmap-ros --config=src/pico_elevationmap/configs/ros_camera_lidar.yaml   # 等价写法
```

若使用带系统 ROS 可见性的 `.venv-ros`，在命令前加
`UV_PROJECT_ENVIRONMENT=.venv-ros`。示例配置
`src/pico_elevationmap/configs/ros_camera_lidar.yaml` 已包含 `ros:` 段（`point_topics`
键与 `sources` 一致、`pose_topic: /odom`），并随 wheel 一起打包。

本仓库暂无 ament `package.xml`，因此不提供 `ros2 run pico_elevationmap_ros ...`
入口；请使用上面的 console script。`uv run pico-elevationmap-ros --help` 在未
source ROS 的环境也可用；缺少 ROS 依赖时启动会给出可操作的错误提示，而不是
import traceback。

核心包不 import 任何 ROS 组件；桥接逻辑（消息转换/打包/接线）由 sys.modules
桩单测覆盖，真实 ROS2 集成用例以 `requires_ros` 标记、无 ROS 时自动 skip。
系统 ROS2 包、`grid_map_msgs` 与 `--system-site-packages` venv 的配置见
[development.md §7](docs/development.md#7-可选ros2-桥接开发)。

## License

PICO-elevationmap is released under the [MIT License](LICENSE).

Built on and wrapping these upstream open-source projects — many thanks to their authors.
See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for the upstream MIT/BSD notices.

- [elevation_mapping_cupy_core](https://github.com/leggedrobotics/elevation_mapping_cupy_core) (MIT) — CUDA fusion kernels, `ElevationMap`, plugin manager
- [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) (MIT, Miki et al. IROS 2022) — GPU elevation mapping
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) (BSD-3, Fankhauser et al. RA-L 2018) — original probabilistic terrain mapping
- [grid_map](https://github.com/ANYbotics/grid_map) (BSD-3) — grid map data structure and messages
