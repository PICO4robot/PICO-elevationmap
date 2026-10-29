# Library API (English)

`pico_elevationmap` deliberately does not re-export at the top level:
`MapperConfig` / `ElevationMapper` pull in CuPy and the upstream CUDA kernels, while
`import pico_elevationmap` itself should stay lightweight. Import from the following
modules explicitly.

| Module | Public objects |
|---|---|
| `pico_elevationmap.core.mapper` | `MapperConfig`, `ElevationMapper`, `MapStats` |
| `pico_elevationmap.core.pose_buffer` | `PoseBuffer`, `StampOutOfRangeError` |
| `pico_elevationmap.core.io` | `PointCloudFrame`, `save_map`, `load_map`, `save_frames`, `load_frames`, `load_frame_file`, `frame_files_from_dir`, `iter_bag_frames`, `write_ply`, `write_pgm`, `write_csv` |
| `pico_elevationmap.sensors.config` | `SensorSourceConfig`, `parse_sources`, `load_sources` |
| `pico_elevationmap.plugins.chain` | plugin declaration validation and registration |
| `pico_elevationmap_ros.messages` | ROS-free PointCloud2 / Odometry → numpy conversion |
| `pico_elevationmap_ros.node` | optional ROS2 node (requires a sourced ROS2 environment) |

## Minimal mapping example

```python
import numpy as np

from pico_elevationmap.core.mapper import ElevationMapper, MapperConfig

cfg = MapperConfig(resolution=0.05, map_length=4.0)
mapper = ElevationMapper(cfg)

# Timestamped T_map_sensor (4x4); send points with the same stamp
mapper.add_pose(stamp=0.0, matrix=np.eye(4, dtype=np.float64))
points = np.array([[0.0, 0.0, -1.0], [0.1, 0.0, -1.0]], dtype=np.float32)
mapper.input(points, stamp=0.0, source="default")

layers = mapper.get_map(["elevation"])  # (cell_n-2)^2 float32, NaN = invalid
stats = mapper.stats()
print(layers["elevation"].shape, stats.coverage, stats.center)
```

Key points:

- Call `add_pose` before `input`; `input` interpolates the pose at `stamp` and raises
  `StampOutOfRangeError` when the stamp is outside the buffered range.
- `points` are in the **sensor frame**; the pose transforms them into the map frame.
- `source` must be declared in the YAML `sources` section; `"default"` falls back to
  the primary source.
- Plugin layers (for example `traversability_geo`) exist only when the plugin is
  enabled in the config.

## From YAML

```python
from pico_elevationmap.core.mapper import MapperConfig

cfg = MapperConfig.from_yaml("src/pico_elevationmap/configs/run_sim.yaml")
print(cfg.resolution, sorted(cfg.sources), sorted(cfg.plugins))
```

`MapperConfig.from_yaml` validates the top-level sections (`map`, `sources`, `plugins`,
`sim`, `ros`) and the `map` keys; unknown keys, wrong types and undeclared sources raise
`ValueError`.

## Map I/O

```python
from pico_elevationmap.core import io

io.save_map("maps/map.npz", layers, mapper.resolution, stats.center)
frames = [io.PointCloudFrame(points=np.zeros((1, 3), dtype=np.float32), stamp=0.0)]
io.save_frames("maps/frames.npz", frames)
```

`save_map` / `save_frames` create parent directories; PLY/PGM/CSV writers do not.
rosbag reading uses `iter_bag_frames` from the pure-Python `rosbags` package.

For the full Chinese API reference, see [../api.md](../api.md).
