"""`pico-elevationmap replay`：离线帧序列 / rosbag 回放建图（Phase 2）。

输入（--input）：
- 帧序列 npz（``run --save-frames`` 的产出，io.save_frames 单文件格式）
- 目录（其内排序后的逐帧 npz：points + stamp + [pose] + [source]）
- rosbag（ROS1 .bag / ROS2 .db3/.mcap，rosbags 纯 python 解析，无需 ROS；
  点云取 PointCloud2，位姿取最新 PoseStamped，缺省单位阵并警告一次）

逐帧注入 ElevationMapper；帧位姿缺失时该帧用单位阵（地图坐标即传感器坐标，
仅适合粗评）。结束打印地图统计，可 ``--save`` 地图 npz / ``--export-ply``
导出高程层点云。GPU 栈懒加载，``--help`` 在无 GPU 机器可用。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from pico_elevationmap.core import io

_BAG_SUFFIXES = {".bag", ".db3", ".mcap"}
_PROGRESS_EVERY = 10


def _load_input(path: Path, point_topic: str | None, pose_topic: str | None) -> list[io.PointCloudFrame]:
    if path.is_dir():
        return [io.load_frame_file(f) for f in io.frame_files_from_dir(path)]
    if path.suffix.lower() in _BAG_SUFFIXES:
        return list(io.iter_bag_frames(path, point_topic=point_topic, pose_topic=pose_topic))
    return io.load_frames(path)  # 单文件帧序列 npz


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="pico-elevationmap replay",
        description="Offline mapping from frame npz sequence or rosbag (Phase 2).",
    )
    parser.add_argument("--config", required=True, type=Path, help="YAML: map / sources / plugins")
    parser.add_argument("--input", required=True, type=Path, help="frames npz | frame dir | rosbag")
    parser.add_argument("--max-frames", type=int, default=None, help="only replay the first N frames")
    parser.add_argument("--point-topic", type=str, default=None, help="rosbag: pointcloud topic basename filter")
    parser.add_argument("--pose-topic", type=str, default=None, help="rosbag: pose topic basename filter")
    parser.add_argument("--save", type=Path, default=None, help="optional npz dump of all map layers")
    parser.add_argument("--export-ply", type=Path, default=None, help="optional PLY export of the elevation layer")
    args = parser.parse_args()

    from pico_elevationmap.core.mapper import ElevationMapper, MapperConfig

    cfg = MapperConfig.from_yaml(args.config)
    frames = _load_input(args.input, args.point_topic, args.pose_topic)
    if args.max_frames is not None:
        frames = frames[: args.max_frames]
    if not frames:
        raise ValueError(f"{args.input}: no frames loaded")

    mapper = ElevationMapper(cfg)
    warned_no_pose = False
    warned_sources: set[str] = set()
    for i, frame in enumerate(frames):
        pose = frame.pose
        if pose is None:
            if not warned_no_pose:
                print("[replay] warning: frames without pose, falling back to identity T_map_sensor")
                warned_no_pose = True
            pose = np.eye(4)
        mapper.add_pose(frame.stamp, pose)
        source = frame.source
        if source != "default" and source not in cfg.sources:
            # bag topic 名通常与配置的输入源名不一致：回退主源并提示一次
            if source not in warned_sources:
                print(f"[replay] warning: source '{source}' not configured, using primary '{cfg.primary_source.name}'")
                warned_sources.add(source)
            source = cfg.primary_source.name
        accepted = mapper.input(frame.points, frame.stamp, source=source)
        if (i + 1) % _PROGRESS_EVERY == 0 or i + 1 == len(frames):
            print(f"[replay] frame {i + 1}/{len(frames)}: {len(frame.points)} pts ({accepted} fused)")

    stats = mapper.stats()
    print(
        f"[replay] map stats: coverage {stats.coverage:.1%} ({stats.valid_cells}/{stats.total_cells}), "
        f"height mean {stats.mean_height:.3f} m, range [{stats.min_height:.3f}, {stats.max_height:.3f}] m, "
        f"center {stats.center}"
    )

    if args.save is not None:
        layers = mapper.get_map()
        io.save_map(args.save, layers, cfg.resolution, stats.center)
        print(f"[replay] saved {len(layers)} layers -> {args.save}")
    if args.export_ply is not None:
        n = io.write_ply(args.export_ply, mapper.get_layer("elevation"), cfg.resolution, stats.center)
        print(f"[replay] exported {n} vertices -> {args.export_ply}")
