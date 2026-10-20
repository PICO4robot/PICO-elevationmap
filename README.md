# PICO-elevationmap

Elevation mapping for PICO robots, split out of the [PICO](https://github.com/PICO4robot/PICO) monorepo.

This repository is developed on topic branches; `main` only serves as the entry point
summarizing what each branch contains.

## Branches

| Branch | Status | Contents |
|---|---|---|
| [`design/framework`](https://github.com/PICO4robot/pico-elevationmap/tree/design/framework) | Active | Framework design proposal ([docs/design.md](https://github.com/PICO4robot/pico-elevationmap/blob/design/framework/docs/design.md)): analysis of the leggedrobotics/ANYbotics elevation-mapping projects, target architecture (ROS-free core + CuPy/torch dual backends), repo layout, sensor plan, and roadmap. |
| [`feat/phase0-scaffold`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase0-scaffold) | Phase 0 done | Installable package skeleton: src-layout `pico_elevationmap` (core/fusion/plugins/backends/sensors stubs), `pico-elevationmap` CLI dispatcher, uv + ruff + pytest, CI. No GPU/ROS deps yet. |
| [`feat/phase1-core-pipeline`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase1-core-pipeline) | Phase 1 done | Core mapping pipeline on GPU: wraps `elevation_mapping_cupy_core` (git-pinned) — SE3 pose buffer (replaces tf), per-source YAML sensor noise configs, `ElevationMapper` facade, plugin chain, real `pico-elevationmap run` demo with synthetic scenes. Verified on RTX 4090: 2.5 ms/step, 34 tests green. |
| [`feat/phase2-plugins-replay`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase2-plugins-replay) | Phase 2 done | Plugin chain (inpaint / smooth / geometric slope+traversability as own plugins), offline `replay` (npz frames, ROS1/ROS2 bags via pure-python rosbags), `export` to PLY/PGM/CSV. 52 tests green. |
| [`feat/phase4-ros2-bridge`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase4-ros2-bridge) | Phase 4 done | Optional ROS2 bridge package `pico_elevationmap_ros` (core stays zero-ROS): PointCloud2/Odometry→numpy conversion, GridMap packing, node with pub/sub + clear/save services. Mock-verified (no ROS2 on dev machine). Phase 3 (torch batch backend / PICO-rl integration) deferred. |

## Sibling repositories

- [PICO-robot](https://github.com/PICO4robot/pico-robot) — robot model assets (URDF/MJCF)
- [PICO-rl](https://github.com/PICO4robot/pico-rl) — reinforcement learning framework
- [PICO-dataset](https://github.com/PICO4robot/pico-dataset) — dataset registry

## License

Proprietary. Upstream open-source projects referenced by the design
(MIT / BSD-3) are credited in the design document.
