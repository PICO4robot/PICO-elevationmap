# TODO

Project status: public source alpha, GitHub-only release (`v0.4.2`), MIT licensed.
This file tracks the work that is **not** complete; release posture and known
limitations are documented in [docs/open-source-readiness.md](docs/open-source-readiness.md).

## 1. Hardware validation

- [ ] **Four-camera D435i rig (P1)**: run all four cameras simultaneously.
  - Verify USB3 bandwidth, power/HUB stability and per-camera extrinsics.
  - Acceptance: at least 10 minutes at 640×480@15 with no dropped camera, four
    non-empty point clouds per step, and a map coverage comparable to the single-camera
    subset. Current code: `src/pico_elevationmap/live/quad_camera_live.py`,
    `src/pico_elevationmap/configs/quad_camera.yaml`.
- [ ] **Livox Mid-360 (P1)**: validate the real driver and both message paths.
  - Test `livox_ros_driver2` CustomMsg and PointCloud2; confirm topic/QoS and LiDAR
    frame timing with `src/pico_elevationmap/configs/livox_mid360.yaml`.
  - Acceptance: a reproducible short capture with non-zero LiDAR frames fused.
- [ ] **Camera-LiDAR calibration and time sync (P1)**: replace the manual extrinsics
  (`--lidar-roll/pitch/yaw`, `--lidar-offset`) with a documented calibration
  procedure and hardware time sync (PTP/gPTP, GPS/PPS or equivalent).
  - Acceptance: calibration artefacts, repeatable residuals, and a replayable capture.
- [ ] **ROS2 bridge in a real graph (P1)**: run `pico_elevationmap_ros` against
  `tests/` `requires_ros` cases and `launch/bringup.launch.py` in a sourced ROS2 graph
  with `ros_camera_lidar.yaml`.
  - Acceptance: PointCloud2/pose input, GridMap output and clear/save services exercised
    end-to-end.

## 2. CI and test coverage

- [ ] **GPU CI (P1)**: add a self-hosted or GPU runner that executes the CUDA/cupy
  tests; currently CI skips them.
- [ ] **ROS2 CI (P1)**: run `requires_ros` integration tests in a ROS2 container or
  runner.
- [ ] **Python matrix (P1)**: add 3.10/3.11 to CI before advertising them in
  `requires-python`.
- [ ] **Coverage gate (P2)**: add pytest coverage reporting and a threshold; prioritize
  `pico_elevationmap/viz/viewer.py`, `viz/foxglove.py` and the `geo_*` plugins.
- [ ] **Shell script checks (P2)**: add `shellcheck` for `scripts/setup_env.sh`.

## 3. Packaging and release

- [ ] **Wheel dependency resolution (P1)**: `elevation-mapping-cupy-core` is git-only,
  so `pip install dist/*.whl` needs `--no-deps`. Recommended path: publish a fork of the
  upstream MIT core to PyPI under a unique distribution name from the owner's account,
  then depend on that package; see [docs/pypi-release.md](docs/pypi-release.md). The
  GitHub-only source install (`scripts/setup_env.sh`) remains the supported path until then.
- [x] **Dependabot PR review**: PR #1 (actions group) merged as actions v7; PR #2 and #3
  (`cuda-toolkit>=12,<14`) closed; `.github/dependabot.yml` now ignores CUDA
  major-version updates and `cuda-toolkit` stays on 12.x.
- [x] **Post-release docs**: `v0.4.2` released with the DCO, security/conduct and
  Dependabot documentation updates; `v0.4.1` remains the previous release.

## 4. Community and documentation

- [x] **CLA/DCO policy (P1)**: DCO 1.1 adopted (`git commit -s`), documented in
  `CONTRIBUTING.md`, the PR template and [DCO](DCO).
- [ ] **English documentation (P1)**: expand `docs/en/` beyond the current summaries
  (full development, deployment and sensor-profile translations if the audience needs
  them).
- [ ] **Repository metadata (P2)**: add `CODEOWNERS`, `.gitattributes` and optionally
  `CITATION.cff`; consider enabling GitHub Discussions.
- [ ] **ament package (P2)**: only if `ros2 run` / `ros2 launch` must become a
  first-class ROS2 entry point; currently `launch/` is repo-only.

## 5. Not planned

- **Rerun**: not adopted. Keep the custom OpenCV viewer default and Foxglove for ROS;
  see [docs/decisions.md](docs/decisions.md) D001. Revisit only if time-travel or
  multi-sensor synchronization becomes a requirement.
