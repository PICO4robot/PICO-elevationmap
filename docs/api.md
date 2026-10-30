# 库 API 使用

> 版本：v0.1　日期：2026-10-30　状态：与 0.4.1 代码同步
> 关联：[concepts.md](concepts.md)、[development.md](development.md)、[open-source-readiness.md](open-source-readiness.md)

## 1. 导入约定

`pico_elevationmap` 顶层包刻意不做 re-export：`MapperConfig` / `ElevationMapper`
会拉入 cupy 和上游 CUDA 内核，而 `import pico_elevationmap` 本身应保持轻量。
公共 API 从下列子模块显式导入：

| 模块 | 公共对象 |
|---|---|
| `pico_elevationmap.core.mapper` | `MapperConfig`、`ElevationMapper`、`MapStats` |
| `pico_elevationmap.core.pose_buffer` | `PoseBuffer`、`StampOutOfRangeError` |
| `pico_elevationmap.core.io` | `PointCloudFrame`、`save_map`、`load_map`、`save_frames`、`load_frames`、`load_frame_file`、`frame_files_from_dir`、`iter_bag_frames`、`write_ply`、`write_pgm`、`write_csv` |
| `pico_elevationmap.sensors.config` | `SensorSourceConfig`、`parse_sources`、`load_sources` |
| `pico_elevationmap.plugins.chain` | 插件声明校验、自有插件注册与 plugin_config 写出 |
| `pico_elevationmap_ros.messages` | 零 ROS 依赖的 PointCloud2 / Odometry → numpy 转换 |
| `pico_elevationmap_ros.node` | 可选 ROS2 节点（需要已 source 的 ROS2 环境） |

这些路径是本轮文档化的稳定入口；顶层 `__init__.py` 的“只放 docstring”约定
保持不变，避免 import 顶层包时意外加载 GPU 栈。

## 2. 最小建图示例

`MapperConfig` 可直接构造，也可用 `MapperConfig.from_yaml(path)` 读取与 CLI 相同的
YAML。下面用默认 `perfect` 源和单位位姿跑一帧传感器系点云（需要 NVIDIA GPU）：

```python
import numpy as np

from pico_elevationmap.core.mapper import ElevationMapper, MapperConfig

cfg = MapperConfig(resolution=0.05, map_length=4.0)
mapper = ElevationMapper(cfg)

# 1) 注入带时间戳的 T_map_sensor（4x4）；同一个 stamp 再送点云
T = np.eye(4, dtype=np.float64)
mapper.add_pose(stamp=0.0, matrix=T)

# 2) 传感器系点云 (N,3)，单位位姿下等价于地图系点云
points = np.array(
    [[0.0, 0.0, -1.0], [0.1, 0.0, -1.0], [0.0, 0.1, -1.0]],
    dtype=np.float32,
)
accepted = mapper.input(points, stamp=0.0, source="default")
print("fused points:", accepted)

# 3) 取图层；返回 (cell_n-2)^2 的 float32 数组，无效 cell 为 NaN
layers = mapper.get_map(["elevation"])
stats = mapper.stats()
print(layers["elevation"].shape, stats.coverage, stats.center)
```

要点：

- `add_pose` 先于 `input`；`input` 会按 `stamp` 从位姿缓冲插值，越界时抛
  `StampOutOfRangeError`（ROS 桥会丢弃该帧并计数）。
- `points` 必须是**传感器系**坐标；位姿负责把点变换到地图系。
- `source` 必须是 YAML `sources` 中声明的源名；`"default"` 回退到主源。
- `mapper.get_map()` 不传参数时返回全部可读图层；也可传层名列表。
- 插件层（如 `traversability_geo`）只有在该 YAML 启用了对应插件时才存在。

## 3. 从 YAML 配置

```python
from pico_elevationmap.core.mapper import MapperConfig

cfg = MapperConfig.from_yaml("src/pico_elevationmap/configs/run_sim.yaml")
print(cfg.resolution, sorted(cfg.sources), sorted(cfg.plugins))
```

`MapperConfig.from_yaml` 会校验顶层段（`map` / `sources` / `plugins` / `sim` /
`ros`）和 `map` 段键名；未知键、类型错误或引用未声明源都会抛 `ValueError`。

## 4. 位姿缓冲

需要手动管理位姿时可直接使用 `PoseBuffer`：

```python
import numpy as np

from pico_elevationmap.core.pose_buffer import PoseBuffer, StampOutOfRangeError

buffer = PoseBuffer(maxlen=128)
buffer.add(0.0, np.eye(4))
buffer.add(0.1, np.eye(4))
T = buffer.interpolate(0.05)
try:
    buffer.interpolate(1.0)
except StampOutOfRangeError:
    print("stamp outside buffered pose range")
```

## 5. 地图 I/O

```python
import numpy as np

from pico_elevationmap.core import io

layers = mapper.get_map(["elevation"])
stats = mapper.stats()

io.save_map("maps/map.npz", layers, mapper.resolution, stats.center)
loaded, meta = io.load_map("maps/map.npz")

frames = [io.PointCloudFrame(points=np.zeros((1, 3), dtype=np.float32), stamp=0.0)]
io.save_frames("maps/frames.npz", frames)
loaded_frames = io.load_frames("maps/frames.npz")
```

`save_map` / `save_frames` 会自动创建父目录；PLY/PGM/CSV 写出器（`write_ply` /
`write_pgm` / `write_csv`）不会创建目录，调用方需保证输出目录存在。rosbag 读取用
`iter_bag_frames`，由纯 Python 的 `rosbags` 提供，不要求安装 ROS。

## 6. 可选 ROS 转换层

`pico_elevationmap_ros.messages` 不 import 任何 ROS 包，可直接用于单元测试或自定义
订阅逻辑：

```python
from pico_elevationmap_ros import messages

points, stamp, frame_id = messages.pointcloud2_to_array(msg)
T, stamp, pos_noise, ori_noise = messages.pose_to_se3(odom_msg)
grid = messages.layers_to_grid_message(
    layers,
    resolution=0.05,
    center=stats.center,
    stamp=stamp,
    frame_id="map",
)
```

真实 ROS2 节点从 `pico_elevationmap_ros.node` 导入；没有 ROS 时 `--help` 仍可用，
实例化节点会抛出带安装/`source` 指引的 `RuntimeError`。ROS2 环境准备见
[development.md §7](development.md#7-可选ros2-桥接开发)。
