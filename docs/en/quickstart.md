# Quickstart (English)

## Install

```bash
git clone https://github.com/PICO4robot/PICO-elevationmap.git
cd PICO-elevationmap
bash scripts/setup_env.sh
```

`setup_env.sh` syncs the source checkout with the CUDA and RealSense extras, verifies
NVRTC and the RealSense SDK, and runs pytest. Add `--ros` to also create the
ROS-visible `.venv-ros`, `--skip-tests` to skip pytest, `--dry-run` to preview.

The wheel's `elevation-mapping-cupy-core` dependency resolves from a git revision, so
`pip install dist/*.whl` needs `--no-deps`; the source install above is the supported
path. The project is GitHub-only and does not publish to PyPI.

## Synthetic demo

```bash
uv run pico-elevationmap run --config src/pico_elevationmap/configs/run_sim.yaml \
    --save maps/map.npz --save-frames maps/frames.npz
```

## D435i live mapping

```bash
uv sync --extra cuda --extra realsense

# Single window: elevation/traversability/colour bands on top, 3D voxel view below
uv run python -m pico_elevationmap.live.realsense_live \
    --config src/pico_elevationmap/configs/realsense_live.yaml \
    --view --voxel-view --frames 0
```

The live script estimates the ground plane with RANSAC and streams depth points into
the mapper without ROS. See the root README for the four-camera rig and LiDAR options.

## Replay and export

```bash
uv run pico-elevationmap replay --config src/pico_elevationmap/configs/run_sim.yaml \
    --input maps/frames.npz --save maps/replay_map.npz --export-ply maps/map.ply
uv run pico-elevationmap export --input maps/replay_map.npz --layers elevation --output maps/elevation.pgm
```

Replay accepts npz frame sequences, per-frame directories and ROS1/ROS2 rosbags
(`.bag` / `.db3` / `.mcap`) through the pure-Python `rosbags` package; no ROS install
is required.

## ROS2 launch (optional)

```bash
source /opt/ros/jazzy/setup.bash
source /path/to/rslidar_ws/install/setup.bash
ros2 launch ./launch/bringup.launch.py \
    lidar:=airy lidar_topic:=/rslidar_points lidar_msg_type:=pointcloud2 \
    config:=$PWD/src/pico_elevationmap/configs/realsense_lidar.yaml
```

`launch/` is repo-only (not packaged into the wheel). Drivers and mapper processes are
orchestrated by launch; time sync and extrinsics remain manual.

## Troubleshooting

- `Failed to find CUDA headers`: install the CUDA extra (`uv sync --extra cuda`) or set
  `CUDA_PATH` to a CUDA 12.x toolkit. The driver's CUDA 13.x capability does not
  require a CUDA 13 toolkit.
- GitHub HTTPS blocked: `setup_env.sh` retries with a per-command SSH URL rewrite.
- No GPU: CPU tests and `--help` still work; mapping paths skip.
