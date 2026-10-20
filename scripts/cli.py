"""Unified thin CLI for PICO-elevationmap: ``pico-elevationmap <command> [args...]``.

Pure dispatcher — mirrors PICO-rl ``scripts/cli.py``: ``_COMMANDS`` is the single
source of truth for command → entry module, lazy-imported via ``importlib`` so
heavy deps (cupy / torch / ROS bridges) only load for the chosen command. Entry
points stay directly runnable, e.g. ``python -m scripts.cli run ...``.

Phase 1: ``run`` resolves to the real ``scripts/pipeline/run.py`` (synthetic-scene
live mapping demo); the remaining commands (replay / bench / export) still hit the
stub fallback below — the lazy-import dispatch is shared by both paths.
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

    module_path, _, phase = _COMMANDS[cmd]
    sys.argv = [f"pico-elevationmap {cmd}", *sys.argv[2:]]
    try:
        module = importlib.import_module(module_path)
    except ModuleNotFoundError as exc:
        if exc.name is None or not module_path.startswith(exc.name):
            raise  # missing dep inside an existing module — surface the real error
        print(f"pico-elevationmap {cmd}: not implemented yet ({phase})")
        sys.exit(0)
    module.main()


if __name__ == "__main__":
    main()
