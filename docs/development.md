# 开发环境搭建与验证

> 版本：v0.1　日期：2026-10-28　状态：已在 Ubuntu 24.04 + RTX 4060 Laptop 实测
> 目标：从零构建 PICO-elevationmap 的开发环境，跑通建图 demo、测试与 lint。

## 1. 前置条件

| 项 | 要求 | 说明 |
|---|---|---|
| OS | Ubuntu 24.04（x86_64 或 aarch64） | CI 与实机方案均以 24.04 为基线 |
| Python | 3.12（uv 管理） | `.python-version` 已固定 3.12；pyproject 支持 3.10–3.12 |
| uv | 最新稳定版 | `uv --version` |
| GPU | NVIDIA，驱动支持 CUDA 12.x+ | 另需 CUDA 12.x headers；不必装完整 toolkit，见 §3 |
| 网络 | 可访问 GitHub 与 PyPI/镜像 | upstream 是 git 依赖；见 §4 |
| 磁盘 | 建议 ≥ 4 GB 空闲 | 核心 venv 约 0.8 GB；装 `[ctk]` 会再增加 1–2 GB |

`nvidia-smi` 右上角的 **CUDA Version 是驱动支持的最高 CUDA 版本，不是已安装的
CUDA toolkit**。驱动 13.x 可以运行 CUDA 12.x 程序（向后兼容），本项目继续使用
`cupy-cuda12x` 即可；只有 CUDA 13 专属平台（如 DGX Spark 的 sm_121）才需要
`cupy-cuda13x`，见 [deploy-hardware.md](deploy-hardware.md) §5。

本机参考环境：Ubuntu 24.04.5、Python 3.12.3、uv 0.11.16、RTX 4060 Laptop 8 GB、
驱动 595.99（显示 CUDA 13.2）；CuPy 报告 runtime 12.9、driver 13.2，建图正常。

## 2. 安装

```bash
git clone git@github.com:PICO4robot/PICO-elevationmap.git
cd PICO-elevationmap
uv sync
```

`uv sync` 默认安装核心包、`pytest` 与 `ruff`（`dependency-groups.dev`）。
`uv sync --extra ros` 只是语义标记，不会安装 ROS2——ROS 由系统提供（见 §7）。

## 3. CUDA 运行时与 headers

CuPy 14 的 wheel 不含 NVRTC 所需的 CUDA headers。只装驱动、没有 CUDA toolkit 的
机器，在首次编译 kernel 时会报：

```
RuntimeError: Failed to find CUDA headers. Please install CUDA toolkit headers
(e.g., pip install cupy-cuda12x[ctk]) or specify CUDA_PATH environment variable.
```

任选一种方式补齐 **CUDA 12.x headers**：

**A. 安装 CuPy 的 CUDA 12 toolkit extra（推荐，版本自洽；下载量较大）**

```bash
uv pip install 'cupy-cuda12x[ctk]'
```

**B. 复用已有的 CUDA 12.x toolkit，设置 `CUDA_PATH`**

```bash
export CUDA_PATH=/usr/local/cuda-12.8   # 目录下需有 include/ 与 lib64/
```

注意：本项目依赖 `cupy-cuda12x`，headers 也必须来自 **CUDA 12.x**；不要让
`CUDA_PATH` 指向 CUDA 13 的 toolkit。`nvidia-smi` 显示 CUDA 13.x 只代表驱动版本，
不影响这里的选择。

验证 NVRTC 能编译 kernel：

```bash
uv run python - <<'PY'
import cupy as cp

k = cp.RawKernel('extern "C" __global__ void k(){}', 'k')
k((1,), (1,), ())
print("CUDA NVRTC OK:", cp.cuda.runtime.getDeviceProperties(0)["name"].decode())
PY
```

## 4. GitHub 访问

`elevation-mapping-cupy-core` 以 git rev 锁在 GitHub 上，`uv sync` 会执行
`git fetch https://github.com/...`。若 HTTPS 连不上（例如本机报
`Failed to connect to github.com port 443`），可临时让 git 走 SSH：

```bash
GIT_CONFIG_COUNT=1 \
GIT_CONFIG_KEY_0=url.git@github.com:.insteadOf \
GIT_CONFIG_VALUE_0=https://github.com/ \
uv sync
```

前提是 `ssh -T git@github.com` 可用。也可以用代理，或把改写写进全局
`~/.gitconfig`（会影响所有 GitHub HTTPS 仓库，按需选择）。同步成功后依赖已进入
uv 缓存，后续 `uv run` 不再访问 GitHub。

## 5. 快速验证

```bash
mkdir -p maps   # run/replay 的 --save 不会自动创建父目录
uv run pico-elevationmap run --config scripts/tools/configs/run_sim.yaml \
    --save maps/map.npz --save-frames maps/frames.npz
uv run pico-elevationmap replay --config scripts/tools/configs/run_sim.yaml \
    --input maps/frames.npz --save maps/replay_map.npz --export-ply maps/map.ply
uv run pico-elevationmap export --input maps/replay_map.npz \
    --layers elevation --output maps/elevation.pgm
```

期望：`run` 打印 60 步耗时与地图统计；`replay` / `export` 在 `maps/` 下生成
npz、PLY 与 PGM。`maps/` 已在 `.gitignore` 中。RTX 4060 Laptop 实测约
**5 ms/step**（0.04 m 分辨率、约 4.6 万点/帧，仅供参考）。

## 6. 测试与 lint

```bash
uv run pytest -q          # 本机基线：60 passed, 1 skipped（无 ROS）
uv run ruff check .
```

- GPU 用例在 cupy / CUDA 不可用时自动 skip；CPU 用例始终可跑。
- 若当前 shell 已 `source` ROS，pytest 会额外加载 ROS 的 `launch_pytest` 插件，
  可能因缺少 `lark` 等依赖在收集阶段失败。核心测试建议在未 source ROS 的 shell
  中运行，或加 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`。
- `requires_ros` 用例只应在完整 ROS 环境单独运行，见 §7。

## 7. 可选：ROS2 桥接开发

前置：Ubuntu 24.04 + ROS2 Jazzy，并安装 `rclpy`、`sensor_msgs`、`geometry_msgs`、
`nav_msgs`、`std_srvs` 与 `ros-jazzy-grid-map-msgs`：

```bash
sudo apt install ros-jazzy-grid-map-msgs
```

让 uv venv 看到系统 ROS 包，然后同步：

```bash
uv venv --system-site-packages --allow-existing
source /opt/ros/jazzy/setup.bash
uv sync --extra ros
```

真实 ROS 集成测试：

```bash
uv run pytest -q -m requires_ros
```

说明：

- `grid_map_msgs` 缺失时节点仍可做点云融合，但会跳过 GridMap 发布并周期性告警；
  需要发布 `/grid_map` 时必须安装。
- `tests/ros_stubs.py` 会把 `rclpy` / 消息包桩写入 `sys.modules` 且不清理。因此
  在装了 ROS 的环境里跑**全量** pytest 时，`requires_ros` 用例可能看到桩而失败；
  请像上面一样单独跑 `-m requires_ros`，或修复桩的清理逻辑后再合并跑。

## 8. 常见问题

| 现象 | 处理 |
|---|---|
| `uv sync` 卡在 `git fetch` / `curl 28` | 按 §4 配 SSH 或代理；确认 `ssh -T git@github.com` |
| `Failed to find CUDA headers` | 按 §3 安装 `cupy-cuda12x[ctk]` 或设置 `CUDA_PATH`（CUDA 12.x） |
| `FileNotFoundError: maps/...` | 先 `mkdir -p maps`，或把 `--save` 指向已存在的目录 |
| pytest 收集阶段报 ROS 插件 / `lark` 错误 | 未 source ROS 的 shell 中跑核心测试，或 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` |
| `numpy<2` 的旧说明 | 已过时；当前 lock 为 numpy 2.x，测试通过 |
| 无 NVIDIA GPU | 可安装、可跑 CPU 测试；`run` / `replay` 需要 GPU，无 GPU 时无法建图 |

## 9. 已验证组合（2026-10-28）

| 项 | 值 |
|---|---|
| OS / Python / uv | Ubuntu 24.04.5 / 3.12.3 / 0.11.16 |
| GPU / 驱动 | RTX 4060 Laptop 8 GB / 595.99（CUDA 13.2） |
| CuPy | 14.2.0（cupy-cuda12x），runtime 12.9，NVRTC 用 CUDA 12.8 headers |
| `pytest -q` | 60 passed, 1 skipped（无 ROS） |
| `run` | 60 步，约 5.02 ms/step，覆盖率 78.7% |
