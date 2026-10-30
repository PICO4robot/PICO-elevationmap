"""Four-camera D435i rig -> single ElevationMapper / single-window visualization.

This is the multi-camera counterpart of ``realsense_live.py``. Each camera has an
explicit optical-to-base extrinsic in the YAML ``cameras`` section; points are
transformed to the common base frame before fusion. Static/bench use is the supported
case; moving platforms still need per-camera time sync and odometry.
"""

from __future__ import annotations

import argparse
import itertools
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from pico_elevationmap.core import io
from pico_elevationmap.live.realsense_live import deproject_depth
from pico_elevationmap.sensors.camera_rig import CameraRig, CameraSpec
from pico_elevationmap.viz import SingleWindowViewer, VoxelMap

_PROGRESS_EVERY = 10


@dataclass
class _Stream:
    spec: CameraSpec
    pipeline: object
    intr: object
    units: float


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="quad-camera-live",
        description="Four RealSense cameras -> one elevation map and one window.",
    )
    parser.add_argument("--config", type=Path, default=None, help="YAML with map/sources/cameras/plugins")
    parser.add_argument("--cameras", type=str, default=None, help="comma-separated rig names to use (default: all)")
    parser.add_argument("--list", action="store_true", help="list RealSense serial numbers and exit")
    parser.add_argument("--frames", type=int, default=30, help="frames to fuse; 0 = until quit (requires a view)")
    parser.add_argument("--warmup", type=int, default=10, help="frames to discard per camera")
    parser.add_argument("--stride", type=int, default=2, help="pixel stride for deprojection")
    parser.add_argument("--view", action="store_true", help="show combined 2D/3D window")
    parser.add_argument("--voxel-view", action="store_true", help="show 3D voxel panel (combined with --view)")
    parser.add_argument("--view-every", type=int, default=1, help="render every N frames")
    parser.add_argument("--z-min", type=float, default=-0.1)
    parser.add_argument("--z-max", type=float, default=1.0)
    parser.add_argument("--color-bands", type=int, default=12)
    parser.add_argument("--voxel-size", type=float, default=0.05)
    parser.add_argument("--voxel-alpha", type=float, default=0.55)
    parser.add_argument("--voxel-min-hits", type=int, default=1)
    parser.add_argument("--voxel-hit-log-odds", type=float, default=0.85)
    parser.add_argument("--voxel-miss-log-odds", type=float, default=-0.85)
    parser.add_argument("--voxel-occ-prob", type=float, default=0.65)
    parser.add_argument("--voxel-ttl", type=float, default=2.0)
    parser.add_argument("--voxel-carve", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--voxel-max-rays", type=int, default=1200)
    parser.add_argument("--voxel-carve-steps", type=int, default=32)
    parser.add_argument("--voxel-carve-every", type=int, default=3)
    parser.add_argument("--save", type=Path, default=None, help="save map npz")
    parser.add_argument("--save-frames", type=Path, default=None, help="save all camera frames for replay")
    parser.add_argument("--export-ply", type=Path, default=None)
    parser.add_argument("--export-pgm", type=Path, default=None)
    parser.add_argument("--view-save", type=Path, default=None, help="save last combined window PNG")
    parser.add_argument("--voxel-save", type=Path, default=None, help="save last 3D voxel panel PNG")
    return parser.parse_args()


def _list_devices(rs: object) -> None:
    devices = rs.context().query_devices()
    print(f"RealSense devices: {len(devices)}")
    for dev in devices:
        print(
            f"  {dev.get_info(rs.camera_info.name)}  serial={dev.get_info(rs.camera_info.serial_number)}  "
            f"fw={dev.get_info(rs.camera_info.firmware_version)}"
        )


def _start_streams(rs: object, rig: CameraRig, warmup: int) -> list[_Stream]:
    devices = {d.get_info(rs.camera_info.serial_number): d for d in rs.context().query_devices()}
    available = sorted(devices)
    streams: list[_Stream] = []
    try:
        for spec in rig.cameras:
            if spec.serial not in devices:
                raise SystemExit(f"camera '{spec.name}': serial {spec.serial!r} not found; available: {available}")
            cfg = rs.config()
            cfg.enable_device(spec.serial)
            cfg.enable_stream(rs.stream.depth, spec.width, spec.height, rs.format.z16, spec.fps)
            pipeline = rs.pipeline()
            profile = pipeline.start(cfg)
            depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            for _ in range(max(warmup, 0)):
                pipeline.wait_for_frames(timeout_ms=5000)
            streams.append(
                _Stream(
                    spec,
                    pipeline,
                    depth_profile.get_intrinsics(),
                    float(profile.get_device().first_depth_sensor().get_depth_scale()),
                )
            )
            print(
                f"[quad] camera {spec.name}: serial={spec.serial} source={spec.source} "
                f"{spec.width}x{spec.height}@{spec.fps}"
            )
    except Exception:
        for stream in streams:
            stream.pipeline.stop()
        raise
    return streams


def _pose(spec: CameraSpec) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = spec.rotation
    T[:3, 3] = np.asarray(spec.xyz, dtype=np.float64)
    return T


def main() -> None:
    args = _parse_args()
    import pyrealsense2 as rs

    if args.list:
        _list_devices(rs)
        return
    if args.config is None:
        raise SystemExit("--config is required unless --list is used")
    if args.frames <= 0 and not (args.view or args.voxel_view):
        raise SystemExit("--frames must be > 0 unless --view or --voxel-view is set")
    if args.view_every < 1:
        raise SystemExit("--view-every must be >= 1")
    if args.z_max <= args.z_min:
        raise SystemExit("--z-max must be > --z-min")
    if not 0.0 < args.voxel_alpha <= 1.0:
        raise SystemExit("--voxel-alpha must be in (0, 1]")
    if args.voxel_min_hits < 1:
        raise SystemExit("--voxel-min-hits must be >= 1")
    if args.voxel_hit_log_odds <= 0.0:
        raise SystemExit("--voxel-hit-log-odds must be > 0")
    if args.voxel_miss_log_odds >= 0.0:
        raise SystemExit("--voxel-miss-log-odds must be < 0")
    if not 0.0 < args.voxel_occ_prob < 1.0:
        raise SystemExit("--voxel-occ-prob must be in (0, 1)")
    if args.voxel_ttl < 0.0:
        raise SystemExit("--voxel-ttl must be >= 0")

    from pico_elevationmap.core.mapper import ElevationMapper, MapperConfig

    cfg = MapperConfig.from_yaml(args.config)
    only = None
    if args.cameras:
        only = {name.strip() for name in args.cameras.split(",") if name.strip()}
    rig = CameraRig.from_config(cfg.cameras, cfg.sources, only=only)
    print(f"[quad] rig cameras: {[c.name for c in rig.cameras]}")
    streams = _start_streams(rs, rig, args.warmup)

    viewer = (
        SingleWindowViewer(
            show_2d=args.view,
            show_3d=args.voxel_view,
            z_min=args.z_min,
            z_max=args.z_max,
            bands=args.color_bands,
            voxel_size=args.voxel_size,
            voxel_alpha=args.voxel_alpha,
        )
        if (args.view or args.voxel_view)
        else None
    )
    voxel_map = (
        VoxelMap(
            args.voxel_size,
            args.voxel_min_hits,
            ttl=args.voxel_ttl,
            hit_log_odds=args.voxel_hit_log_odds,
            miss_log_odds=args.voxel_miss_log_odds,
            occ_probability=args.voxel_occ_prob,
        )
        if (args.voxel_view or args.voxel_save is not None)
        else None
    )

    mapper = ElevationMapper(cfg)
    mapper.move_to([0.0, 0.0, 0.0], np.eye(3))
    all_frames: list[io.PointCloudFrame] = []
    times: list[float] = []
    last_view_t: float | None = None
    fps = 0.0
    frame_iter = range(args.frames) if args.frames > 0 else itertools.count()
    try:
        for i in frame_iter:
            t0 = time.perf_counter()
            cameras: list[tuple[_Stream, np.ndarray, float]] = []
            for stream in streams:
                frames = stream.pipeline.wait_for_frames(timeout_ms=5000)
                depth_frame = frames.get_depth_frame()
                if not depth_frame:
                    continue
                stamp = float(depth_frame.get_timestamp()) / 1000.0
                depth = np.asanyarray(depth_frame.get_data()).astype(np.float32) * stream.units
                points = deproject_depth(
                    depth,
                    fx=stream.intr.fx,
                    fy=stream.intr.fy,
                    ppx=stream.intr.ppx,
                    ppy=stream.intr.ppy,
                    stride=args.stride,
                    min_range=cfg.resolve_source(stream.spec.source).min_range,
                    max_range=cfg.resolve_source(stream.spec.source).max_range,
                )
                cameras.append((stream, points, stamp))
            if not cameras:
                continue

            # 四台 D435i 的硬件时间戳不在同一时钟域；静态/台架验证用主机单调时钟
            # 作为统一 stamp，避免不同相机时钟导致的 PoseBuffer 乱序。
            stamp = time.monotonic()
            mapper.add_pose(stamp, np.eye(4))
            fused = 0
            for stream, points, _ in cameras:
                source = cfg.resolve_source(stream.spec.source)
                world = stream.spec.transform(points)
                fused += mapper.input(world, stamp, source=source.name)
                if voxel_map is not None:
                    voxel_map.add(world, stamp)
                    if args.voxel_carve and i % args.voxel_carve_every == 0:
                        voxel_map.carve_free(
                            stream.spec.xyz,
                            world,
                            max_rays=args.voxel_max_rays,
                            max_steps=args.voxel_carve_steps,
                        )
                if args.save_frames is not None:
                    all_frames.append(
                        io.PointCloudFrame(points=points, stamp=stamp, pose=_pose(stream.spec), source=source.name)
                    )
            mapper.update_variance()
            mapper.update_time()
            if voxel_map is not None:
                voxel_map.prune(stamp)
            times.append((time.perf_counter() - t0) * 1e3)

            if viewer is not None and (i + 1) % args.view_every == 0:
                now = time.perf_counter()
                if last_view_t is not None:
                    inst = 1.0 / max(now - last_view_t, 1e-6)
                    fps = inst if fps == 0.0 else 0.9 * fps + 0.1 * inst
                last_view_t = now
                elevation = mapper.get_layer("elevation")
                traversability = None
                try:
                    traversability = mapper.get_layer("traversability_geo")
                except KeyError:
                    pass
                valid = np.isfinite(elevation)
                if voxel_map is not None:
                    centers, probabilities = voxel_map.centers_with_probabilities()
                else:
                    centers = np.empty((0, 3))
                    probabilities = np.empty((0,))
                center = mapper.center
                canvas = viewer.compose(
                    elevation=elevation,
                    traversability=traversability,
                    voxel_centers=centers,
                    voxel_probabilities=probabilities,
                    target=np.array([center[0], center[1], 0.0]),
                    fps=fps,
                    coverage=float(valid.mean()),
                    resolution=cfg.resolution,
                )
                viewer.show(canvas)
                if viewer.wait_key():
                    break

            if (i + 1) % _PROGRESS_EVERY == 0 or (0 < args.frames == i + 1):
                print(f"[quad] frame {i + 1}/{args.frames}: fused {fused} pts, {times[-1]:.2f} ms")

        stats = mapper.stats()
        print(
            f"[quad] map stats: coverage {stats.coverage:.1%} ({stats.valid_cells}/{stats.total_cells}), "
            f"height range [{stats.min_height:.3f}, {stats.max_height:.3f}] m"
        )
        if args.save is not None:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            layers = mapper.get_map()
            io.save_map(args.save, layers, cfg.resolution, stats.center)
            print(f"[quad] saved {len(layers)} layers -> {args.save}")
        if args.save_frames is not None:
            args.save_frames.parent.mkdir(parents=True, exist_ok=True)
            io.save_frames(args.save_frames, all_frames)
            print(f"[quad] saved {len(all_frames)} frames -> {args.save_frames}")
        if args.export_ply is not None:
            args.export_ply.parent.mkdir(parents=True, exist_ok=True)
            n = io.write_ply(args.export_ply, mapper.get_layer("elevation"), cfg.resolution, stats.center)
            print(f"[quad] exported {n} vertices -> {args.export_ply}")
        if args.export_pgm is not None:
            args.export_pgm.parent.mkdir(parents=True, exist_ok=True)
            io.write_pgm(args.export_pgm, mapper.get_layer("elevation"))
            print(f"[quad] exported PGM -> {args.export_pgm}")
        if viewer is not None:
            if args.voxel_save is not None and viewer.last_voxel_panel is not None:
                args.voxel_save.parent.mkdir(parents=True, exist_ok=True)
                viewer.save_voxel(str(args.voxel_save))
                print(f"[quad] saved voxel view -> {args.voxel_save}")
            if args.view_save is not None and viewer.last_canvas is not None:
                args.view_save.parent.mkdir(parents=True, exist_ok=True)
                viewer.save_canvas(str(args.view_save))
                print(f"[quad] saved combined view -> {args.view_save}")
    finally:
        for stream in streams:
            stream.pipeline.stop()
        if viewer is not None:
            viewer.close()


if __name__ == "__main__":
    main()
