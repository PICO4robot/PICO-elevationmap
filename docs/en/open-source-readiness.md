# Open-source readiness (English summary)

Status: **source alpha, GitHub-only release (no PyPI publishing), MIT licensed.**
Not production-ready. Full checklist and Chinese details:
[../open-source-readiness.md](../open-source-readiness.md).

## Support matrix

| Area | Status |
|---|---|
| Platform | x86_64 Linux (Ubuntu 24.04 baseline); NVIDIA GPU with CUDA 12.x headers |
| Python | 3.12 (CI target); 3.10/3.11 are not supported until a CI matrix exists |
| aarch64 / Jetson | Experimental; no CI coverage |
| ROS2 bridge | Optional; requires a sourced ROS2 install; no ament package yet |
| RealSense D435i | Optional extra; direct-SDK path is prototype quality |
| Four-camera D435i rig | Config + single-camera subset tested; four-device hardware unverified |
| LiDAR (RoboSense Airy / Livox) | Optional; vendor ROS2 drivers; extrinsics/time sync are manual |
| torch backend / `bench` | Deferred (Phase 3) |

## Release checklist status

Resolved: MIT license, third-party notices, community files, CI on push/PR, release
workflow, PyPI-based `uv.lock`, wheel config packaging, removal of the generic
top-level `scripts` package.

Remaining: source-install path is required for the wheel's git-only
`elevation-mapping-cupy-core` dependency; real ROS2/LiDAR/four-camera hardware
validation; GPU runner coverage; English documentation (this directory is the start);
CLA/DCO policy.

## Known limitations

- Wheel dependency resolution: `pip install dist/*.whl` needs `--no-deps`; use
  `bash scripts/setup_env.sh` or a source `uv sync` instead.
- No ament package: use `uv run pico-elevationmap-ros --config ...`; `launch/` is
  repo-only.
- CI does not run GPU, real ROS2 or four-camera hardware paths.
- Time sync and camera-LiDAR extrinsics are manual/approximate; no calibrated
  deployment is claimed.
