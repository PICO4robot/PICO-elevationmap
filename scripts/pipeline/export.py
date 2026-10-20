"""`pico-elevationmap export`：已保存地图 npz → PLY / PGM / CSV（Phase 2）。

PLY：单层（默认 elevation）带高度色标的 ascii 点云，NaN cell 跳过；
PGM：16 位灰度高程图，头注释记录真实高程范围（nan=0）；
CSV：原始矩阵（多层时逐层一个文件，文件名加层名后缀）。
零图像依赖。``--list`` 只打印图层与元数据。
"""

from __future__ import annotations

import argparse
from pathlib import Path

from pico_elevationmap.core import io

_FORMATS = ("ply", "pgm", "csv")


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="pico-elevationmap export",
        description="Export a saved map npz to PLY / PGM / CSV (Phase 2).",
    )
    parser.add_argument("--input", required=True, type=Path, help="map npz from run/replay --save")
    parser.add_argument("--output", type=Path, default=None, help="output file (default: <input>.<fmt>)")
    parser.add_argument("--format", choices=_FORMATS, default=None, help="default: infer from --output suffix")
    parser.add_argument("--layers", type=str, default="elevation", help="comma-separated layer names")
    parser.add_argument("--list", action="store_true", help="list layers and metadata, then exit")
    args = parser.parse_args()

    layers, meta = io.load_map(args.input)
    if args.list:
        print(f"layers: {sorted(layers)}")
        print(f"meta: resolution={meta['resolution']} center={meta['center']}")
        return

    names = [n.strip() for n in args.layers.split(",") if n.strip()]
    missing = [n for n in names if n not in layers]
    if missing:
        raise KeyError(f"layers {missing} not in {args.input} (have {sorted(layers)})")
    if not names:
        raise ValueError("--layers is empty")

    fmt = args.format
    if fmt is None:
        if args.output is None or args.output.suffix.lstrip(".") not in _FORMATS:
            parser.error("--format needed when --output suffix is not one of ply/pgm/csv")
        fmt = args.output.suffix.lstrip(".")
    if fmt in ("ply", "pgm") and len(names) != 1:
        parser.error(f"{fmt} exports a single layer, got {names}")
    if meta["resolution"] is None or meta["center"] is None:
        raise ValueError(f"{args.input}: missing resolution/center metadata (need a run/replay --save map)")

    out = args.output or args.input.with_suffix(f".{fmt}")
    for name in names:
        target = out if len(names) == 1 else out.with_name(f"{out.stem}_{name}{out.suffix}")
        layer = layers[name]
        if fmt == "ply":
            n = io.write_ply(target, layer, meta["resolution"], meta["center"])
            print(f"[export] {name}: {n} vertices -> {target}")
        elif fmt == "pgm":
            hmin, hmax = io.write_pgm(target, layer)
            print(f"[export] {name}: {layer.shape[1]}x{layer.shape[0]} PGM, h ∈ [{hmin:.3f}, {hmax:.3f}] -> {target}")
        else:
            io.write_csv(target, layer)
            print(f"[export] {name}: {layer.shape} CSV -> {target}")
