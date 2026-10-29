#!/usr/bin/env bash
# One-click development environment for PICO-elevationmap (Linux + uv).
# Installs from a source checkout, so the git-only upstream dependency is resolved by
# uv instead of a wheel's Requires-Dist. Does not modify global git config.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WITH_CUDA=1
WITH_REALSENSE=1
WITH_ROS=0
RUN_TESTS=1
DRY_RUN=0

usage() {
  cat <<'EOF'
Usage: bash scripts/setup_env.sh [options]

Options:
  --no-cuda          do not install the cuda-toolkit[cudart,nvrtc] extra
  --no-realsense     do not install pyrealsense2
  --ros              also create .venv-ros with system-site-packages
  --skip-tests       skip pytest (still runs import/GPU checks)
  --dry-run          print commands without executing them
  -h, --help         show this help

The script uses `uv sync --frozen` from the source tree. If GitHub HTTPS is
unreachable it retries with a per-command SSH URL rewrite; it does not change
global git config.
EOF
}

for arg in "$@"; do
  case "$arg" in
    --no-cuda) WITH_CUDA=0 ;;
    --no-realsense) WITH_REALSENSE=0 ;;
    --ros) WITH_ROS=1 ;;
    --skip-tests) RUN_TESTS=0 ;;
    --dry-run) DRY_RUN=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $arg" >&2; usage; exit 2 ;;
  esac
done

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required. Install it from https://docs.astral.sh/uv/ and rerun." >&2
  exit 1
fi

EXTRAS=()
if (( WITH_CUDA )); then EXTRAS+=(--extra cuda); fi
if (( WITH_REALSENSE )); then EXTRAS+=(--extra realsense); fi

run() {
  if (( DRY_RUN )); then
    printf '  '
    printf '%q ' "$@"
    printf '\n'
  else
    "$@"
  fi
}

GIT_ENV=()
if ! timeout 10 git ls-remote https://github.com/leggedrobotics/elevation_mapping_cupy_core.git HEAD >/dev/null 2>&1; then
  if timeout 10 env GIT_SSH_COMMAND="ssh -o BatchMode=yes -o ConnectTimeout=6" \
      git ls-remote git@github.com:leggedrobotics/elevation_mapping_cupy_core.git HEAD >/dev/null 2>&1; then
    echo "GitHub HTTPS unavailable; using SSH URL rewrite for this script only."
    GIT_ENV=(
      "GIT_CONFIG_COUNT=1"
      "GIT_CONFIG_KEY_0=url.git@github.com:.insteadOf"
      "GIT_CONFIG_VALUE_0=https://github.com/"
    )
  else
    echo "Cannot reach GitHub over HTTPS or SSH; configure a proxy or SSH and retry." >&2
    exit 1
  fi
fi

echo "== uv sync (source install) =="
run env "${GIT_ENV[@]}" uv sync --frozen "${EXTRAS[@]}"

if (( WITH_ROS )); then
  if [ ! -f /opt/ros/jazzy/setup.bash ]; then
    echo "ROS Jazzy not found at /opt/ros/jazzy/setup.bash; skipping .venv-ros." >&2
  else
    echo "== ROS-visible venv (.venv-ros) =="
    run uv venv --system-site-packages --allow-existing --python 3.12 .venv-ros
    run env "${GIT_ENV[@]}" UV_PROJECT_ENVIRONMENT="$ROOT/.venv-ros" \
      uv sync --frozen "${EXTRAS[@]}"
    echo "Run in this shell before ROS commands: source /opt/ros/jazzy/setup.bash"
  fi
fi

if (( DRY_RUN )); then
  echo "Dry run complete; no changes were made."
  exit 0
fi

echo "== verify imports / NVRTC =="
uv run python - <<'PY'
import sys
try:
    import cupy as cp
except Exception as exc:  # pragma: no cover - environment dependent
    print(f"warning: cupy import failed: {exc}")
    sys.exit(0)
print(f"cupy {cp.__version__}")
if not cp.cuda.is_available():
    print("warning: no CUDA device; GPU mapping tests will skip")
    sys.exit(0)
print("CUDA device:", cp.cuda.runtime.getDeviceProperties(0)["name"].decode())
k = cp.RawKernel('extern "C" __global__ void k(){}', 'k')
k((1,), (1,), ())
print("NVRTC OK")
PY

if (( WITH_REALSENSE )); then
  echo "== verify RealSense SDK =="
  uv run python - <<'PY'
try:
    import pyrealsense2 as rs
except Exception as exc:
    print(f"warning: pyrealsense2 import failed: {exc}")
else:
    print("RealSense devices:", len(rs.context().query_devices()))
PY
fi

if (( RUN_TESTS )); then
  echo "== pytest =="
  env -u PYTHONPATH uv run pytest -q
fi

if (( WITH_ROS )) && [ -f "$ROOT/.venv-ros/bin/activate" ]; then
  echo "Done. Activate with: source .venv-ros/bin/activate"
else
  echo "Done. Activate with: source .venv/bin/activate"
fi
