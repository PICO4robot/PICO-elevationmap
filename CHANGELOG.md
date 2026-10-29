# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
The project is pre-1.0: minor releases may contain breaking changes.

## [Unreleased]

### Added

- `docs/open-source-readiness.md`: release checklist, third-party MIT/BSD notice
  requirements, support matrix and known limitations.
- `LICENSE` (MIT) and a `License :: OSI Approved :: MIT License` classifier; the
  project is now distributed under MIT.
- `docs/api.md` and `src/pico_elevationmap/py.typed`: documented public import paths
  and library usage examples.
- Community files: `CONTRIBUTING.md`, `CODE_OF_CONDUCT.md`, `SECURITY.md`,
  GitHub issue forms and a pull-request template.
- `docs/assets/` screenshots for the D435i and D435i + LiDAR voxel views.
- `THIRD_PARTY_NOTICES.md` with the upstream MIT/BSD notices.
- `scripts/tools/configs/ros_camera_lidar.yaml`: runnable camera + LiDAR ROS bridge
  example, with a parsing test for `MapperConfig` + `RosConfig`.
- Bundled `scripts.tools/configs/*.yaml` package data, a config-discovery test and a
  CI wheel-install smoke check.
- Tag-triggered `.github/workflows/release.yml` for GitHub Releases (no PyPI publish).
- Unit tests for ROS CLI argument parsing and for save-parent directory creation.
- `CHANGELOG.md` and `THIRD_PARTY_NOTICES.md` now ship in the wheel via `data-files`;
  `MANIFEST.in` includes docs and launch examples in the sdist.

### Changed

- `pico-elevationmap-ros` now imports safely without ROS: `--help` works, and
  missing ROS dependencies produce an actionable error instead of an import traceback.
- The ROS console script uses `argparse`, accepting both `--config path` and
  `--config=path`; README/development commands use `uv run pico-elevationmap-ros`.
- `io.save_map` and `io.save_frames` create missing parent directories, so the
  README quickstart no longer requires `mkdir -p maps`.
- CI now runs on push/PR with `uv lock --check`, `uv sync --frozen`, ruff, pytest,
  `uv build`, a wheel-install smoke check and `twine check`; Python 3.12 is documented.
- Package metadata now includes README, neutral authors, project URLs, keywords,
  classifiers and the MIT license; the description reflects the cupy-only backend.
- README now states the support matrix, removes private/dead PICO links, uses absolute
  raw image URLs for PyPI rendering, and documents that `launch/` is repo-only.
- `docs/design.md` is marked as a historical design document and its CI/license/torch
  claims are reconciled with the current release posture.
- `docs/open-source-readiness.md` now tracks the Aliyun-locked lockfile, wheel config
  packaging, `scripts` namespace collision, missing ROS example and unexecuted CI.

### Fixed

- Removed the unsupported `ros2 run pico_elevationmap_ros ...` claim; the repository
  has no ament `package.xml`, so the console-script invocation is documented instead.
- `pico-elevationmap bench` now exits non-zero with an explicit “not implemented
  (Phase 3)” message instead of reporting success.
- `launch/bringup.launch.py` now fails fast with an actionable error when the
  configured `.venv-ros` Python is missing, while keeping `python:=` configurable.

## [0.4.0] - unreleased

Version 0.4.0 is the current pre-release version in `pyproject.toml`; no git tag has
been created yet. The release workflow and readiness checklist target tag `v0.4.0`.
Summary of the work on `main`:

- ROS-free core wrapping `elevation_mapping_cupy_core` with a timestamped SE3 pose
  buffer instead of tf.
- Per-source YAML sensor-noise models and a YAML-driven postprocessing plugin chain,
  including the project's own geometric slope and traversability plugins.
- Offline replay from npz frame sequences, frame directories and ROS1/ROS2 rosbags
  (pure-Python `rosbags`), plus PLY/PGM/CSV export.
- Synthetic-scene `run` demo and optional RealSense D435i / LiDAR live pipelines.
- Optional ROS2 bridge package (`pico_elevationmap_ros`) with PointCloud2/pose
  conversion, GridMap packing and clear/save services.
- torch batch backend and `bench` remain deferred to Phase 3.
