# 开发环境搭建与验证

> 版本：v0.2　日期：2026-10-30　状态：已在 Ubuntu 24.04 + RTX 4060 Laptop 实测；
> 2026-10-30 重跑 CPU/无 ROS 测试
> 目标：从零构建 PICO-elevationmap 的开发环境，跑通建图 demo、测试与 lint。

## 1. 前置条件

| 项 | 要求 | 说明 |
|---|---|---|
| OS | Ubuntu 24.04（x86_64） | 支持矩阵以 x86_64 为基线；aarch64 / Jetson 为实验性，暂无 CI 覆盖 |
| Python | 3.12（uv 管理） | CI 只测 3.12；3.10 / 3.11 在 CI matrix 出现前不作为受支持版本 |
| uv | 最新稳定版 | `uv --version` |
| GPU | NVIDIA，驱动支持 CUDA 12.x+ | 另需 CUDA 12.x headers；不必装完整 toolkit，见 §3 |
| 网络 | 可访问 GitHub 与 PyPI/镜像 | upstream 是 git 依赖；见 §4 |
| 磁盘 | 建议 ≥ 4 GB 空闲 | 核心 venv 约 0.8 GB；装 `[ctk]` 会再增加 1–2 GB |

`nvidia-smi` 右上角的 **CUDA Version 是驱动支持的最高 CUDA 版本，不是已安装的
CUDA toolkit**。驱动 13.x 可以运行 CUDA 12.x 程序（向后兼容），本项目继续使用
`cupy-cuda12x` 即可；只有 CUDA 13 专属平台（如 DGX Spark 的 sm_121）才需要
`cupy-cuda13x`，见 [deploy-hardware.md](deploy-hardware.md) §5。

本机参考环境：Ubuntu 24.04.5、Python 3.12.3、uv 0.11.16、RTX 4060 Laptop 8 GB、
驱动 595.99（显示 CUDA 13.2）；CuPy 报告 runtime 12.9、driver 13.2，建图正常。

## 2. 安装

```bash
git clone https://github.com/PICO4robot/PICO-elevationmap.git
cd PICO-elevationmap
uv sync
```

`uv sync` 默认安装核心包、`pytest` 与 `ruff`（`dependency-groups.dev`）。
`uv sync --extra ros` 只是语义标记，不会安装 ROS2——ROS 由系统提供（见 §7）。

## 3. CUDA 运行时与 headers

CuPy 14 的 wheel 不含 NVRTC 所需的 CUDA headers。只装驱动、没有 CUDA toolkit 的
机器，在首次编译 kernel 时会报：

```
RuntimeError: Failed to find CUDA headers. Please install CUDA toolkit headers
(e.g., pip install cupy-cuda12x[ctk]) or specify CUDA_PATH environment variable.
```

任选一种方式补齐 **CUDA 12.x headers**：

**A. 安装项目自带的轻量 CUDA extra（推荐）**

```bash
uv sync --extra cuda
# 等价于只装 headers + nvrtc（约 100 MB，比完整 ctk 轻很多）：
# uv pip install 'cuda-toolkit[cudart,nvrtc]==12.*'
```

这也适用于已经 `uv sync` 过的环境；它会补上 `cuda-toolkit[cudart,nvrtc]`。
若想一次装全 CuPy 的 toolkit 组件，也可以 `uv pip install 'cupy-cuda12x[ctk]'`，
但会额外拉取 cublas / cufft / cusolver 等大包。

**B. 复用已有的 CUDA 12.x toolkit，设置 `CUDA_PATH`**

```bash
export CUDA_PATH=/usr/local/cuda-12.8   # 目录下需有 include/ 与 lib64/
```

注意：本项目依赖 `cupy-cuda12x`，headers 也必须来自 **CUDA 12.x**；不要让
`CUDA_PATH` 指向 CUDA 13 的 toolkit。`nvidia-smi` 显示 CUDA 13.x 只代表驱动版本，
不影响这里的选择。

验证 NVRTC 能编译 kernel：

```bash
uv run python - <<'PY'
import cupy as cp

k = cp.RawKernel('extern "C" __global__ void k(){}', 'k')
k((1,), (1,), ())
print("CUDA NVRTC OK:", cp.cuda.runtime.getDeviceProperties(0)["name"].decode())
PY
```

## 4. GitHub 访问

`elevation-mapping-cupy-core` 以 git rev 锁在 GitHub 上，`uv sync` 会执行
`git fetch https://github.com/...`。若 HTTPS 连不上（例如本机报
`Failed to connect to github.com port 443`），可临时让 git 走 SSH：

```bash
GIT_CONFIG_COUNT=1 \
GIT_CONFIG_KEY_0=url.git@github.com:.insteadOf \
GIT_CONFIG_VALUE_0=https://github.com/ \
uv sync
```

前提是 `ssh -T git@github.com` 可用。也可以用代理，或把改写写进全局
`~/.gitconfig`（会影响所有 GitHub HTTPS 仓库，按需选择）。同步成功后依赖已进入
uv 缓存，后续 `uv run` 不再访问 GitHub。

## 5. 快速验证

```bash
uv run pico-elevationmap run --config scripts/tools/configs/run_sim.yaml \
    --save maps/map.npz --save-frames maps/frames.npz
uv run pico-elevationmap replay --config scripts/tools/configs/run_sim.yaml \
    --input maps/frames.npz --save maps/replay_map.npz --export-ply maps/map.ply
uv run pico-elevationmap export --input maps/replay_map.npz \
    --layers elevation --output maps/elevation.pgm
```

`--save` / `--save-frames` 会自动创建父目录；`maps/` 已在 `.gitignore` 中。

期望：`run` 打印 60 步耗时与地图统计；`replay` / `export` 在 `maps/` 下生成
npz、PLY 与 PGM。RTX 4060 Laptop 实测约 **5 ms/step**（0.04 m 分辨率、约 4.6 万点/帧，仅供参考）。

### 5.1 RealSense D435i 直连（可选）

不需要 ROS，用 `pyrealsense2` 直接取深度流并建图：

```bash
uv sync --extra cuda --extra realsense

# 实时 2D + 3D 体素；q / Esc 退出
uv run python -m scripts.tools.realsense_live \
    --config scripts/tools/configs/realsense_live.yaml \
    --view --voxel-view --frames 0 \
    --z-min -0.1 --z-max 1.0 --color-bands 12 --voxel-size 0.05

# 或采 30 帧后保存
uv run python -m scripts.tools.realsense_live \
    --config scripts/tools/configs/realsense_live.yaml \
    --frames 30 --save maps/realsense_map.npz \
    --save-frames maps/realsense_frames.npz \
    --export-ply maps/realsense.ply --export-pgm maps/realsense.pgm
```

脚本流程：640×480@30 深度图 → 按内参反投影为光学系点云 → RANSAC 估计近场
主平面（地面/桌面）并构造 Z-up 地图系 → 逐帧送入 `ElevationMapper` → 窗口
刷新 / 导出 npz、PLY、PGM。`--save-frames` 的帧序列可直接用同一配置 `replay`。

实时颜色约定：

- `--z-min/--z-max` 是**固定绝对高度范围**（默认 -0.1 / 1.0 m），同色即同高；
- `--color-bands 12` 做 12 个离散高度带，带间画线，并叠加坡度明暗和右侧米制色标；
- 高度超过 `z-max` 会被夹到最高色；场景更高时调大 `--z-max`；
- `--color-bands 1` 切回连续色带。

3D 体素视图：

- `--view` 只看 2D；`--voxel-view` 只看 3D；两个一起用会在**同一个窗口**里
  显示上半 2D（高程 + traversability + 色标）和下半 3D 体素；
- `--voxel-size` 控制体素边长（默认 5 cm），渲染为带透明度的小方块；
  `--voxel-alpha`（默认 0.55）调整透明度；
- 点击窗口获得焦点：鼠标左键拖拽旋转、滚轮缩放；也可用 `a/d` 偏航、
  `w/s` 俯仰、`+/-` 缩放、`q` 退出；
- 动态障碍物：默认每 3 帧沿当前深度射线清空自由空间；`--voxel-ttl`（默认 2 s）
  清理未再观测的体素，`--no-voxel-carve` 关闭射线清空；`--voxel-carve-every`、
  `--voxel-max-rays`、`--voxel-carve-steps` 可调清理频率/开销；
- 视图 target 跟随 `mapper.center`；接上里程计/位姿后，地图和视角会随运动前移；
- `--voxel-save` 保存最后一帧 3D 面板；`--view-save` 保存完整合并窗口。

本机 D435i（fw 5.17.0.10）实测：`--stride 2` 约 3.7–7.2 万点/帧（取决于有效
深度比例），稳定后约 8–11 ms/step。桌面视角下桌面接近掠射，地图覆盖率会偏低；
把相机抬高并下俯 20–30°、让地面进入视野，覆盖率会明显改善。

### 5.2 D435i + RoboSense Airy 双源

Airy 默认 IP 为 `192.168.1.200`，主机网卡配到同网段（如 `192.168.1.102/24`）：

```bash
nmcli connection add type ethernet ifname <usb-eth-iface> con-name lidar-airy \
    ipv4.method manual ipv4.addresses 192.168.1.102/24 ipv4.never-default yes
nmcli connection up lidar-airy
ping -c 3 192.168.1.200
```

RoboSense 驱动用 ROS2 `rslidar_sdk`（`lidar_type: RSAIRY`，发布
`/rslidar_points`）。Ubuntu 上先准备 `libpcap-dev`、`libyaml-cpp-dev`、PCL 等依赖，
然后在独立工作区编译：

```bash
colcon build --packages-select rslidar_msg rslidar_sdk
source install/setup.bash
ros2 run rslidar_sdk rslidar_sdk_node --ros-args -p config_path:=/abs/path/scripts/tools/configs/airy_rslidar.yaml
```

双源脚本需要能看到系统 ROS 包，单独建一个带 `--system-site-packages` 的 venv：

```bash
uv venv --system-site-packages .venv-ros
UV_PROJECT_ENVIRONMENT=.venv-ros uv sync --extra cuda --extra realsense

source /opt/ros/jazzy/setup.bash
source /path/to/rslidar_ws/install/setup.bash
.venv-ros/bin/python -m scripts.tools.realsense_live \
    --config scripts/tools/configs/realsense_lidar.yaml \
    --lidar-topic /rslidar_points --frames 30 \
    --save maps/dual_map.npz --export-ply maps/dual.ply
```

本机实测：Airy 约 8–10 Hz、6.5–7.1 万点/帧；30 个相机帧融合 15 个 LiDAR 帧，
12 m 地图覆盖率 6.3%，3D 体素视图约 2.9 万格。当前用相机 stamp 统一对齐，
LiDAR 时间戳未参与位姿插值；外参用 `--lidar-roll` / `--lidar-pitch` /
`--lidar-yaw` / `--lidar-offset` 手工微调。默认 LiDAR z 轴朝上；**倒装时加
`--lidar-roll 180`（绕 x 翻转）或 `--lidar-pitch 180`（绕 y 翻转）**，自动地面
z 估计会在旋转后重新计算。精确标定与硬件时间同步仍需补测。

### 5.3 ROS2 launch

`launch/bringup.launch.py` 负责启动驱动 + 建图，`start_delay` 默认 3 s，
建图进程退出时 launch 会关闭驱动：

```bash
# Livox Mid-360（CustomMsg）
ros2 launch ./launch/bringup.launch.py \
    lidar:=livox lidar_topic:=/livox/lidar lidar_msg_type:=custom \
    config:=$PWD/scripts/tools/configs/livox_mid360.yaml

# Airy（PointCloud2）
ros2 launch ./launch/bringup.launch.py \
    lidar:=airy lidar_topic:=/rslidar_points lidar_msg_type:=pointcloud2 \
    config:=$PWD/scripts/tools/configs/realsense_lidar.yaml
```

常用参数：`view`、`voxel_view`、`frames`、`lidar_roll/pitch/yaw`、
`lidar_offset`、`voxel_alpha`、`foxglove`、`save_map`、`voxel_save`、`view_save`。Livox 驱动需先
build/source `livox_ros_driver2`；若驱动输出 PointCloud2，把
`lidar_msg_type` 改成 `pointcloud2` 并使用对应 launch 文件。launch 只负责进程
编排，不解决 PTP/时间同步、QoS 和外参标定。

### 5.4 Foxglove

```bash
source /opt/ros/jazzy/setup.bash
.venv-ros/bin/python -m scripts.tools.realsense_live \
    --config scripts/tools/configs/realsense_lidar.yaml \
    --lidar-topic /rslidar_points \
    --foxglove --view --voxel-view --frames 0
```

发布两个带 rgb 的 `sensor_msgs/PointCloud2`：

- `/pico_elevationmap/elevation_points`
- `/pico_elevationmap/voxels`

另一个终端运行 `ros2 run foxglove_bridge foxglove_bridge`，Foxglove Studio 连接
`ws://<host>:8765` 后在 3D 面板添加这两个 topic。`foxglove_bridge` 需要单独安装；
它只做桥接，不改变建图逻辑。

## 6. 测试与 lint

```bash
env -u PYTHONPATH uv run pytest -q          # 本机基线：全部通过，GPU/ROS 用例自动 skip（无 ROS）
uv run ruff check .
```

- GPU 用例在 cupy / CUDA 不可用时自动 skip；CPU 用例始终可跑。
- 若当前 shell 已 `source` ROS，pytest 会额外加载 ROS 的 `launch_pytest` 插件，
  可能因缺少 `lark` 等依赖在收集阶段失败。核心测试建议在未 source ROS 的 shell
  中运行，或加 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。
- `requires_ros` 用例只应在完整 ROS 环境单独运行，见 §7。

## 7. 可选：ROS2 桥接开发

前置：Ubuntu 24.04 + ROS2 Jazzy，并安装 `rclpy`、`sensor_msgs`、`geometry_msgs`、
`nav_msgs`、`std_srvs` 与 `ros-jazzy-grid-map-msgs`：

```bash
sudo apt install ros-jazzy-grid-map-msgs
```

让 uv venv 看到系统 ROS 包，然后同步：

```bash
export UV_PROJECT_ENVIRONMENT=.venv-ros
uv venv --system-site-packages --allow-existing .venv-ros
source /opt/ros/jazzy/setup.bash
uv sync --extra ros
```

真实运行（本仓库暂无 ament `package.xml`，使用 wheel 安装的 console script）：

```bash
source /opt/ros/jazzy/setup.bash
uv run pico-elevationmap-ros --config scripts/tools/configs/ros_camera_lidar.yaml
uv run pico-elevationmap-ros --config=scripts/tools/configs/ros_camera_lidar.yaml
```

`--config path` 与 `--config=path` 两种写法都支持。`uv run pico-elevationmap-ros --help`
未 source ROS 也可用；缺少 rclpy / 消息包时启动会给出可操作的错误提示，而不是
import traceback。

真实 ROS 集成测试：

```bash
uv run pytest -q -m requires_ros
```

说明：

- `grid_map_msgs` 缺失时节点仍可做点云融合，但会跳过 GridMap 发布并周期性告警；
  需要发布 `/grid_map` 时必须安装。
- `tests/ros_stubs.py` 会把 `rclpy` / 消息包桩写入 `sys.modules` 且不清理。因此
  在装了 ROS 的环境里跑**全量** pytest 时，`requires_ros` 用例可能看到桩而失败；
  请像上面一样单独跑 `-m requires_ros`，或修复桩的清理逻辑后再合并跑。

## 8. 常见问题

| 现象 | 处理 |
|---|---|
| `uv sync` 卡在 `git fetch` / `curl 28` | 按 §4 配 SSH 或代理；确认 `ssh -T git@github.com` |
| `Failed to find CUDA headers` | 按 §3 安装 `cupy-cuda12x[ctk]` 或设置 `CUDA_PATH`（CUDA 12.x） |
| `FileNotFoundError: maps/...` | `io.save_map` / `io.save_frames` 会自动创建父目录；PLY/PGM/CSV 导出请确保输出目录存在 |
| pytest 收集阶段报 ROS 插件 / `lark` 错误 | 未 source ROS 的 shell 中跑核心测试，或 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` |
| `numpy<2` 的旧说明 | 已过时；当前 lock 为 numpy 2.x，测试通过 |
| 无 NVIDIA GPU | 可安装、可跑 CPU 测试；`run` / `replay` 需要 GPU，无 GPU 时无法建图 |

## 9. 已验证组合

> 最近一次完整硬件验证：2026-10-28（Ubuntu 24.04.5 + RTX 4060 Laptop + D435i）。
> 2026-10-30 重新运行了 `env -u PYTHONPATH uv run pytest -q`（CPU / 无 ROS 路径），
> 结果：全部通过，GPU/ROS 用例自动 skip。

| 项 | 值 |
|---|---|
| OS / Python / uv | Ubuntu 24.04.5 / 3.12.3 / 0.11.16 |
| GPU / 驱动 | RTX 4060 Laptop 8 GB / 595.99（CUDA 13.2） |
| CuPy | 14.2.0（cupy-cuda12x），runtime 12.9，NVRTC 用 CUDA 12.9 headers（`cuda-toolkit[cudart,nvrtc]`） |
| `env -u PYTHONPATH uv run pytest -q` | 2026-10-30 重跑：全部通过；GPU/ROS 用例自动 skip（无 ROS） |
| `run` | 2026-10-28 验证：60 步，约 5.02 ms/step，覆盖率 78.7% |
| RealSense | 2026-10-28 验证：D435i fw 5.17.0.10，640×480@30，stride 2 约 3.7–7.2 万点/帧，约 8–11 ms/step |
