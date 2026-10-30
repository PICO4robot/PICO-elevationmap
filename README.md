# PICO-elevationmap

[English](README.md) | [简体中文](README.zh-CN.md)

[![CI](https://github.com/PICO4robot/PICO-elevationmap/actions/workflows/ci.yml/badge.svg)](https://github.com/PICO4robot/PICO-elevationmap/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.12](https://img.shields.io/badge/python-3.12-blue.svg)](pyproject.toml)

PICO-elevationmap is a ROS-free 2.5D elevation mapping library for PICO quadrupeds and
humanoids. Poses come from a timestamped SE3 pose buffer instead of tf; sensor noise
and the plugin chain are YAML-driven; maps can be replayed offline from npz sequences
or rosbags without a ROS graph.

## Status

Public source alpha, GitHub-only release, MIT licensed. **Not production-ready.**
Full release status and known limitations: [docs/open-source-readiness.md](docs/open-source-readiness.md).

| Area | Status |
|---|---|
| Platform | x86_64 Linux, NVIDIA GPU, CUDA 12.x headers |
| Python | 3.12 (CI target) |
| ROS2 bridge | Optional; sourced ROS2 required; no ament package |
| RealSense / LiDAR | Optional; D435i and RoboSense Airy paths are prototype quality |
| Four-camera D435i rig | Config + single-camera subset tested; four-device hardware unverified |
| torch backend / `bench` | Deferred (Phase 3) |

## Install

```bash
git clone https://github.com/PICO4robot/PICO-elevationmap.git
cd PICO-elevationmap
bash scripts/setup_env.sh
```

`setup_env.sh` syncs from source with the CUDA and RealSense extras, verifies NVRTC and
the RealSense SDK, and runs pytest. Add `--ros` for a ROS-visible `.venv-ros`,
`--skip-tests` to skip pytest, or `--dry-run` to preview. The wheel's
`elevation-mapping-cupy-core` dependency is git-only, so plain `pip install dist/*.whl`
needs `--no-deps`; source install is the supported path.

## Quickstart

```bash
# Synthetic scene: flat ground + box + steps, circular trajectory
uv run pico-elevationmap run --config src/pico_elevationmap/configs/run_sim.yaml \
    --save maps/map.npz --save-frames maps/frames.npz

# Offline replay: npz frames / frame dirs / rosbag (.bag/.db3/.mcap, no ROS install)
uv run pico-elevationmap replay --config src/pico_elevationmap/configs/run_sim.yaml \
    --input maps/frames.npz --save maps/replay_map.npz --export-ply maps/map.ply

# Export a saved map: PLY / PGM (16-bit elevation) / CSV
uv run pico-elevationmap export --input maps/replay_map.npz --layers elevation --output maps/elevation.pgm
```

## Documentation

- [docs/en/README.md](docs/en/README.md) — English quickstart, API and readiness summaries
- [docs/README.md](docs/README.md) — full Chinese documentation index
- [docs/development.md](docs/development.md) — environment, D435i, LiDAR, ROS2 launch and Foxglove details
- [docs/api.md](docs/api.md) — public import paths and examples
- [TODO.md](TODO.md) — open work

Optional live paths (details in the docs above):

- **D435i live**: `uv run python -m pico_elevationmap.live.realsense_live --config src/pico_elevationmap/configs/realsense_live.yaml --view --voxel-view --frames 0`
- **D435i + RoboSense Airy**: see [§5.2](docs/development.md) and `realsense_lidar.yaml`
- **Four-camera D435i**: see [§5.5](docs/development.md) and `quad_camera.yaml`
- **ROS2 launch / Foxglove bridge**: see [§5.3 / §5.4](docs/development.md)

![D435i live elevation, traversability and probability voxel view](https://raw.githubusercontent.com/PICO4robot/PICO-elevationmap/main/docs/assets/realsense_window.png)

Prototype-quality output from a static D435i; not a calibrated deployment.

## Development

```bash
env -u PYTHONPATH uv run pytest -q
uv run ruff check .
uv run ruff format --check .
bash -n scripts/setup_env.sh
```

## License

MIT — see [LICENSE](LICENSE) and [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for
the upstream notices.
