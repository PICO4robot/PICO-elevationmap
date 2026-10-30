# PICO-elevationmap

[English](README.md) | [简体中文](README.zh-CN.md)

[![CI](https://github.com/PICO4robot/PICO-elevationmap/actions/workflows/ci.yml/badge.svg)](https://github.com/PICO4robot/PICO-elevationmap/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.12](https://img.shields.io/badge/python-3.12-blue.svg)](pyproject.toml)

PICO-elevationmap 是面向 PICO 四足/人形机器人的 ROS-free 2.5D 高程建图库。位姿走带时间戳的
SE3 位姿缓冲而不是 tf；传感器噪声与插件链由 YAML 驱动；地图可以离线回放 npz 帧序列或 rosbag，
不需要 ROS 图。

## 状态

公开源码 alpha、GitHub-only 发布、MIT 许可；**非生产就绪**。完整发布状态与已知限制见
[docs/open-source-readiness.md](docs/open-source-readiness.md)。

| 项目 | 状态 |
|---|---|
| 平台 | x86_64 Linux、NVIDIA GPU、CUDA 12.x headers |
| Python | 3.12（CI 目标） |
| ROS2 桥接 | 可选；需要已 source 的 ROS2；暂无 ament 包 |
| RealSense / LiDAR | 可选；D435i 与 RoboSense Airy 为原型质量 |
| 四相机 D435i rig | 配置 + 单路子集已验证；四台整机未验证 |
| torch 后端 / `bench` | 暂缓（Phase 3） |

## 安装

```bash
git clone https://github.com/PICO4robot/PICO-elevationmap.git
cd PICO-elevationmap
bash scripts/setup_env.sh
```

`setup_env.sh` 从源码同步 CUDA + RealSense extras，验证 NVRTC 与 RealSense SDK，并跑 pytest；
可加 `--ros` 建 `.venv-ros`、`--skip-tests` 跳过测试、`--dry-run` 预览。wheel 的
`elevation-mapping-cupy-core` 依赖仍来自 git 源，普通 `pip install dist/*.whl` 需要 `--no-deps`；
源码安装是当前主路径。

## 快速开始

```bash
# 合成场景：平地 + 方台 + 台阶，圆周轨迹
uv run pico-elevationmap run --config src/pico_elevationmap/configs/run_sim.yaml \
    --save maps/map.npz --save-frames maps/frames.npz

# 离线回放：npz 帧序列 / 逐帧目录 / rosbag（.bag/.db3/.mcap，无需安装 ROS）
uv run pico-elevationmap replay --config src/pico_elevationmap/configs/run_sim.yaml \
    --input maps/frames.npz --save maps/replay_map.npz --export-ply maps/map.ply

# 导出已保存地图：PLY / PGM（16 位高程）/ CSV
uv run pico-elevationmap export --input maps/replay_map.npz --layers elevation --output maps/elevation.pgm
```

## 文档

- [docs/README.md](docs/README.md) — 完整中文文档索引
- [docs/en/README.md](docs/en/README.md) — 英文 Quickstart / API / readiness 摘要
- [docs/development.md](docs/development.md) — 环境、D435i、LiDAR、ROS2 launch、Foxglove 细节
- [docs/api.md](docs/api.md) — 公共导入路径与示例
- [TODO.md](TODO.md) — 未完成事项

可选实时链路（细节见上方文档）：

- **D435i 单相机**：`uv run python -m pico_elevationmap.live.realsense_live --config src/pico_elevationmap/configs/realsense_live.yaml --view --voxel-view --frames 0`
- **D435i + RoboSense Airy**：见 [§5.2](docs/development.md) 与 `realsense_lidar.yaml`
- **四相机 D435i**：见 [§5.5](docs/development.md) 与 `quad_camera.yaml`
- **ROS2 launch / Foxglove**：见 [§5.3 / §5.4](docs/development.md)

![D435i 实时高程、traversability 与概率体素视图](https://raw.githubusercontent.com/PICO4robot/PICO-elevationmap/main/docs/assets/realsense_window.png)

静态 D435i 的原型质量输出，不是标定后的部署效果。

## 开发

```bash
env -u PYTHONPATH uv run pytest -q
uv run ruff check .
uv run ruff format --check .
bash -n scripts/setup_env.sh
```

## 许可证

MIT — 见 [LICENSE](LICENSE) 与 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) 中的上游声明。
