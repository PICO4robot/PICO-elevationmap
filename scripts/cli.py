"""Unified thin CLI for PICO-elevationmap: ``pico-elevationmap <command> [args...]``.

Pure dispatcher — mirrors PICO-rl ``scripts/cli.py``: ``_COMMANDS`` is the single
source of truth for command → entry module, lazy-imported via ``importlib`` so
heavy deps (cupy / torch / ROS bridges) only load for the chosen command. Entry
points stay directly runnable, e.g. ``python -m scripts.cli run ...``.

`run`, `replay` and `export` resolve to their real pipeline modules; `bench`
remains a Phase 3 stub and exits non-zero until a bench pipeline exists.
"""

from __future__ import annotations

import importlib
import sys

# command → (entry module with main(), one-line description, roadmap phase)
_COMMANDS: dict[str, tuple[str, str, str]] = {
    "run": ("scripts.pipeline.run", "Run live elevation mapping from a sensor stream", "Phase 1"),
    "replay": ("scripts.pipeline.replay", "Replay recorded data and rebuild maps offline", "Phase 2"),
    "bench": ("scripts.pipeline.bench", "Benchmark fusion / plugin / backend throughput", "Phase 3"),
    "export": ("scripts.pipeline.export", "Export built maps to file (PICO-dataset formats)", "Phase 2"),
}

# 明确标记尚无实现模块的命令；不依赖 import 失败来判断，避免把安装损坏
# 误报成 "not implemented"。
_NOT_IMPLEMENTED: dict[str, str] = {"bench": "Phase 3"}


def _print_usage() -> None:
    print("usage: pico-elevationmap <command> [args...]\n")
    print("commands:")
    for name, (_, desc, phase) in _COMMANDS.items():
        print(f"  {name:<12} {desc}  [{phase}]")
    print("\nAll args after <command> are forwarded verbatim, e.g.:")
    print("  pico-elevationmap run --config scripts/tools/configs/run_sim.yaml")


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help"):
        _print_usage()
        sys.exit(0 if len(sys.argv) >= 2 else 2)

    cmd = sys.argv[1]
    if cmd not in _COMMANDS:
        print(f"pico-elevationmap: unknown command '{cmd}' (see `pico-elevationmap --help`)", file=sys.stderr)
        sys.exit(2)

    if cmd in _NOT_IMPLEMENTED:
        print(f"pico-elevationmap {cmd}: not implemented ({_NOT_IMPLEMENTED[cmd]})", file=sys.stderr)
        sys.exit(1)

    module_path, _, phase = _COMMANDS[cmd]
    sys.argv = [f"pico-elevationmap {cmd}", *sys.argv[2:]]
    try:
        module = importlib.import_module(module_path)
    except ModuleNotFoundError as exc:
        if exc.name is None or not module_path.startswith(exc.name):
            raise  # missing dep inside an existing module — surface the real error
        print(f"pico-elevationmap {cmd}: not implemented ({phase})", file=sys.stderr)
        sys.exit(1)
    module.main()


if __name__ == "__main__":
    main()
