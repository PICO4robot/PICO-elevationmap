# TODO

Project status: public source alpha, GitHub-only release (`v0.4.0`), MIT licensed.
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
  so `pip install dist/*.whl` needs `--no-deps`. Resolve before any PyPI release; the
  GitHub-only source install (`scripts/setup_env.sh`) remains the supported path.
- [ ] **Dependabot PR review**: PR #2 relaxes `cuda-toolkit==12.*` to `>=12,<14`
  (CI fails; do not merge). Review/merge the actions update PR only after confirming
  the Node 20 deprecation warning is gone.
- [ ] **Post-release docs**: tag `v0.4.0` points to `f479b20`; later commits
  (`dependabot.yml`, release-status docs) are on `main`. Decide whether a follow-up
  `v0.4.1` release is needed.

## 4. Community and documentation

- [ ] **CLA/DCO policy (P1)**: decide and document the contribution terms in
  `CONTRIBUTING.md`.
- [ ] **English documentation (P1)**: expand `docs/en/` beyond the current summaries
  (full development, deployment and sensor-profile translations if the audience needs
  them).
- [ ] **Repository metadata (P2)**: add `CODEOWNERS`, `.gitattributes` and optionally
  `CITATION.cff`; consider enabling GitHub Discussions.
- [ ] **ament package (P2)**: only if `ros2 run` / `ros2 launch` must become a
  first-class ROS2 entry point; currently `launch/` is repo-only.
