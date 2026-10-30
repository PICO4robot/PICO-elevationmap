# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).
The project is pre-1.0: minor releases may contain breaking changes.

## [0.4.2] - 2026-10-30

### Changed

- Adopted the Developer Certificate of Origin 1.1 (`DCO`); `CONTRIBUTING.md` and the
  PR template now require `git commit -s` sign-off.
- Corrected the `SECURITY.md` scope paths for the `pico_elevationmap` package layout.
- Replaced the Code of Conduct contact TBD with the PICO4robot maintainer profiles and
  best-effort reporting expectations.
- Linked `docs/pypi-release.md` from the wheel-dependency limitation and TODO.

## [0.4.1] - 2026-10-30

### Changed

- GitHub Actions `actions/checkout` and `astral-sh/setup-uv` were updated to v7 via
  Dependabot; CI and release workflows now use Node 24-compatible action versions.
- Dependabot now ignores CUDA major-version updates, so it cannot widen the CUDA 12.x
  constraint to an incompatible CUDA 13 toolchain.
- Added `TODO.md` and recorded the `v0.4.0` public-release status in the readiness docs.

### Fixed

- Restored the explicit `cuda-toolkit[cudart,nvrtc]>=12,<13` constraint after the
  Dependabot PR that proposed `>=12,<14` and failed CI (`cupy-cuda12x` requires
  CUDA 12.x headers).

## [0.4.0] - 2026-10-30

First public GitHub release (source alpha; no PyPI publishing).

### Added

- `docs/en/`: English Quickstart, API and open-source readiness summaries.
- `scripts/setup_env.sh`: one-click source environment setup (CUDA + RealSense
  extras, NVRTC/RealSense verification, optional `.venv-ros`, pytest) with a
  per-command SSH fallback when GitHub HTTPS is unreachable.
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
- `src/pico_elevationmap/configs/ros_camera_lidar.yaml`: runnable camera + LiDAR ROS bridge
  example, with a parsing test for `MapperConfig` + `RosConfig`.
- `src/pico_elevationmap/live/quad_camera_live.py`,
  `src/pico_elevationmap/configs/quad_camera.yaml` and
  `pico_elevationmap.sensors.camera_rig`: four-camera D435i rig with per-camera serials
  and optical-to-base extrinsics; single-camera subset tested.
- Bundled `pico_elevationmap.configs` package data, a config-discovery test and a
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
- CI now runs on push/PR with `uv lock --check`, `uv sync --frozen`, ruff check,
  ruff format check, `bash -n` for the setup script, pytest, `uv build`, a wheel-install
  smoke check and `twine check`; Python 3.12 is documented.
- `uv.lock` was regenerated against PyPI; the Aliyun mirror index was removed so clean
  checkouts resolve from the default index.
- The top-level `scripts` package was moved into `pico_elevationmap` (`cli.py`,
  `pipeline/`, `live/`, `configs/`), removing the generic top-level package name from
  the wheel; `scripts/setup_env.sh` remains a repo-level helper.
- `launch/bringup.launch.py` now exposes `warmup` so short LiDAR captures can wait for
  the driver to publish frames.
- The README now documents the wheel's `--no-deps` install limitation and links the
  English documentation summaries.
- Package metadata now includes README, neutral authors, project URLs, keywords,
  classifiers and the MIT license; the description reflects the cupy-only backend.
- README now states the support matrix, removes private/dead PICO links, uses absolute
  raw image URLs for PyPI rendering, and documents that `launch/` is repo-only.
- `docs/design.md` is marked as a historical design document and its CI/license/torch
  claims are reconciled with the current release posture.
- `docs/open-source-readiness.md` now tracks the PyPI-locked lockfile, wheel config
  packaging, the resolved `scripts` namespace collision, missing ROS example and CI
  status.

### Fixed

- Removed the unsupported `ros2 run pico_elevationmap_ros ...` claim; the repository
  has no ament `package.xml`, so the console-script invocation is documented instead.
- `pico-elevationmap bench` now exits non-zero with an explicit “not implemented
  (Phase 3)” message instead of reporting success.
- `launch/bringup.launch.py` now fails fast with an actionable error when the
  configured `.venv-ros` Python is missing, while keeping `python:=` configurable.

### Highlights

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
