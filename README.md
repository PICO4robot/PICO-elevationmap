# PICO-elevationmap

Elevation mapping for PICO robots, split out of the [PICO](https://github.com/PICO4robot/PICO) monorepo.

ROS-free 2.5D elevation mapping framework for PICO quadrupeds / humanoids: a CuPy GPU
core wrapping [elevation_mapping_cupy_core](https://github.com/leggedrobotics/elevation_mapping_cupy_core),
with pluggable fusion, a YAML-driven postprocessing plugin chain (including our own
geometric slope / traversability plugins), offline replay from npz sequences or rosbags,
and an optional ROS2 bridge. Full framework design: [docs/design.md](docs/design.md).

This repository is developed on topic branches; `main` only serves as the entry point
summarizing what each branch contains.

## Branches

| Branch | Status | Contents |
|---|---|---|
| [`design/framework`](https://github.com/PICO4robot/pico-elevationmap/tree/design/framework) | Merged to main | Framework design proposal ([docs/design.md](docs/design.md)): analysis of the leggedrobotics/ANYbotics elevation-mapping projects, target architecture (ROS-free core + CuPy/torch dual backends), repo layout, sensor plan, and roadmap. |
| [`feat/phase0-scaffold`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase0-scaffold) | Merged to main | Installable package skeleton: src-layout `pico_elevationmap` (core/fusion/plugins/backends/sensors stubs), `pico-elevationmap` CLI dispatcher, uv + ruff + pytest, CI. No GPU/ROS deps yet. |
| [`feat/phase1-core-pipeline`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase1-core-pipeline) | Merged to main | Core mapping pipeline on GPU: wraps `elevation_mapping_cupy_core` (git-pinned) — SE3 pose buffer (replaces tf), per-source YAML sensor noise configs, `ElevationMapper` facade, plugin chain, real `pico-elevationmap run` demo with synthetic scenes. Verified on RTX 4090: 2.5 ms/step, 34 tests green. |
| [`feat/phase2-plugins-replay`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase2-plugins-replay) | Merged to main | Plugin chain (inpaint / smooth / geometric slope+traversability as own plugins), offline `replay` (npz frames, ROS1/ROS2 bags via pure-python rosbags), `export` to PLY/PGM/CSV. 52 tests green. |
| [`feat/phase4-ros2-bridge`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase4-ros2-bridge) | Merged to main | Optional ROS2 bridge package `pico_elevationmap_ros` (core stays zero-ROS): PointCloud2/Odometry→numpy conversion, GridMap packing, node with pub/sub + clear/save services. Mock-verified (no ROS2 on dev machine). Phase 3 (torch batch backend / PICO-rl integration) deferred. |

## Layout

```
src/pico_elevationmap/       # 核心包（零 ROS 依赖）
  core/                      # pose_buffer（SE3 位姿缓冲）、mapper（建图外观）、io（帧/地图/rosbag/导出）
  sensors/                   # 每输入源噪声模型 YAML（laser / depth_camera / perfect）
  plugins/                   # 插件链装配 + 自有 geo_slope / geo_traversability
  fusion/                    # 融合策略代理到上游（占位，跟随 cupy_core）
  backends/                  # 后端抽象占位（torch 批量后端为 deferred Phase 3）
src/pico_elevationmap_ros/   # 可选 ROS2 桥接（rclpy 依赖仅存在于该包 node.py）
scripts/                     # cli.py 唯一 lazy-import dispatcher + pipeline/（run/replay/export）+ tools/configs/
tests/                       # pytest，镜像 src；GPU 用例 requires_gpu 自动 skip，ROS 集成用例 requires_ros 自动 skip
```

## Install

Core (GPU required at runtime for mapping; install itself is CPU-friendly):

```bash
uv sync            # 核心：pico_elevationmap + pico-elevationmap CLI
uv sync --extra ros  # 语义标记：rclpy / grid_map_msgs 由系统 ROS2 提供（pip 无法安装）
```

## Quickstart

```bash
# 合成场景实时建图 demo（平地 + 方台 + 台阶，圆周轨迹）
uv run pico-elevationmap run --config scripts/tools/configs/run_sim.yaml \
    --save maps/map.npz --save-frames maps/frames.npz

# 离线回放：帧序列 / 逐帧目录 / rosbag（.bag/.db3/.mcap，无需安装 ROS）
uv run pico-elevationmap replay --config scripts/tools/configs/run_sim.yaml \
    --input maps/frames.npz --save maps/replay_map.npz --export-ply maps/map.ply

# 导出已保存地图：PLY（点云）/ PGM（16 位高程图）/ CSV
uv run pico-elevationmap export --input maps/replay_map.npz --layers elevation --output maps/elevation.pgm
```

配置 YAML 分 `map`（分辨率/门限等）、`sources`（每源噪声与量程）、`plugins`
（上游 inpainting/smoothing + 自有 geo_*）、`sim`（run 的仿真参数）、`ros`
（桥接参数）五段，键名拼写与取值越界都会报明确错误；示例见
`scripts/tools/configs/run_sim.yaml`。

## ROS2 bridge (optional)

`pico_elevationmap_ros` 把 `ElevationMapper` 接入 ROS2：订阅 N 路 PointCloud2 +
一路 Odometry/PoseWithCovarianceStamped（位姿协方差推导的噪声传入融合触发
漂移补偿），按 `publish_fps` 定时发布 grid_map_msgs/GridMap（图层可配），
并提供 `~/clear`、`~/save`（Trigger）服务。真实环境运行（需先 source ROS2）：

```bash
ros2 run pico_elevationmap_ros mapping_node --ros-args -p config:=/abs/path/config.yaml
# 或（本 wheel 的控制台脚本，同样要求 ROS2 环境已 source）
pico-elevationmap-ros --config /abs/path/config.yaml
```

核心包不 import 任何 ROS 组件；桥接逻辑（消息转换/打包/接线）由 sys.modules
桩单测覆盖，真实 ROS2 集成用例以 `requires_ros` 标记、无 ROS 时自动 skip。

## Sibling repositories

- [PICO-robot](https://github.com/PICO4robot/pico-robot) — robot model assets (URDF/MJCF)
- [PICO-rl](https://github.com/PICO4robot/pico-rl) — reinforcement learning framework
- [PICO-dataset](https://github.com/PICO4robot/pico-dataset) — dataset registry

## License

Proprietary (PICO). Built on and wrapping these upstream open-source projects — many
thanks to their authors:

- [elevation_mapping_cupy_core](https://github.com/leggedrobotics/elevation_mapping_cupy_core) (MIT) — CUDA fusion kernels, `ElevationMap`, plugin manager
- [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) (MIT, Miki et al. IROS 2022) — GPU elevation mapping
- [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) (BSD-3, Fankhauser et al. RA-L 2018) — original probabilistic terrain mapping
- [grid_map](https://github.com/ANYbotics/grid_map) (BSD-3) — grid map data structure and messages
