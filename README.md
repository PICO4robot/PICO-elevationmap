# PICO-elevationmap

Elevation mapping for PICO robots, split out of the [PICO](https://github.com/PICO4robot/PICO) monorepo.

This repository is developed on topic branches; `main` only serves as the entry point
summarizing what each branch contains.

## Branches

| Branch | Status | Contents |
|---|---|---|
| [`design/framework`](https://github.com/PICO4robot/pico-elevationmap/tree/design/framework) | Active | Framework design proposal ([docs/design.md](https://github.com/PICO4robot/pico-elevationmap/blob/design/framework/docs/design.md)): analysis of the leggedrobotics/ANYbotics elevation-mapping projects, target architecture (ROS-free core + CuPy/torch dual backends), repo layout, sensor plan, and roadmap. |
| [`feat/phase0-scaffold`](https://github.com/PICO4robot/pico-elevationmap/tree/feat/phase0-scaffold) | Phase 0 done | Installable package skeleton: src-layout `pico_elevationmap` (core/fusion/plugins/backends/sensors stubs), `pico-elevationmap` CLI dispatcher, uv + ruff + pytest, CI. No GPU/ROS deps yet. |

## Sibling repositories

- [PICO-robot](https://github.com/PICO4robot/pico-robot) — robot model assets (URDF/MJCF)
- [PICO-rl](https://github.com/PICO4robot/pico-rl) — reinforcement learning framework
- [PICO-dataset](https://github.com/PICO4robot/pico-dataset) — dataset registry

## License

Proprietary. Upstream open-source projects referenced by the design
(MIT / BSD-3) are credited in the design document.
