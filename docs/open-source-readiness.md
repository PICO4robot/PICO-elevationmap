# 开源就绪评估与发布清单

> 版本：v0.3　日期：2026-10-30　状态：MIT 已定；GitHub-only 发布，技术阻塞待清
> 关联：[README.md](../README.md)、[development.md](development.md)、[api.md](api.md)、
> [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)

## 1. 结论

项目已选择 **MIT License**（2026-10-30），发布形态为 **GitHub 仓库 + GitHub
Release**，不发布 PyPI。代码、测试与文档具备对外协作的基础；P0 技术阻塞已清理，
剩余为 P1 级事项：

- `uv.lock` 已改用 PyPI 重建，锁文件不再包含 Aliyun 镜像地址。
- 顶层 `scripts` 包已并入 `pico_elevationmap`，wheel 不再占用通用顶层包名。
- CI 已在 GitHub Actions 上实际执行并通过（run `40971256320`）；本轮改动合并后
  需再跑一次。
- wheel 的 `elevation-mapping-cupy-core` 依赖仍来自 git 源，普通 pip 安装不带
  `--no-deps` 会失败；GitHub-only 发布以源码安装为主路径。
- 真实 ROS2 / LiDAR / 四相机硬件链路仍需现场验证；英文文档仍待补充。

### 1.1 当前 P0/P1 状态

| 项目 | 状态 |
|---|---|
| `uv.lock` 绑定 Aliyun 镜像 | **已解决**：已移除 Aliyun index 并用 PyPI 重新生成，`uv lock --check` / `uv sync --frozen` 通过。 |
| 许可证 | **已解决**：MIT；`LICENSE`、`pyproject.toml`、README、design/readiness 已同步。 |
| CI 执行 | **已执行**：GitHub Actions run `40971256320`（Python 3.12）在 push 后通过 lock/sync/ruff/pytest/build/wheel smoke/twine；本轮改动合并后需再跑一次。 |
| wheel 配置打包 | **已解决**：`pico_elevationmap/configs/*.yaml` 作为 package-data 打入 wheel，并已在本机构建验证。 |
| wheel 依赖解析 | `uv pip install dist/*.whl`（不带 `--no-deps`）当前会因 `elevation-mapping-cupy-core` 只在 git 源而失败；CI smoke 使用 `--no-deps` 只验证 wheel 载荷。GitHub-only 发布以 `setup_env.sh` 源码安装为主路径。 |
| `scripts` 顶层命名冲突 | **已解决**：CLI / pipeline / live demo / configs 已移入 `pico_elevationmap`，顶层 `scripts` 目录只保留 `setup_env.sh`。 |
| ROS 示例与真实 ROS 验证 | `src/pico_elevationmap/configs/ros_camera_lidar.yaml` 已通过解析测试；真实 ROS2 环境仍需单独验证。 |
| 私有/失效链接 | README 已移除 PICO monorepo 与 sibling 仓库链接；历史文档中仍可能引用内部仓库。 |
| 英文文档 | **待补**：README 为英文入口，`docs/` 仍以中文为主；至少需英文 Quickstart / API / readiness 摘要。 |

## 2. 许可证（已定：MIT）

项目采用 **MIT License**，`LICENSE` 已添加；`pyproject.toml` 使用
`license = { file = "LICENSE" }` 并声明 `License :: OSI Approved :: MIT License`，
README、design 与本节已同步。

选择理由：MIT 是 GitHub 上最常见的宽松许可证之一，与上游 MIT/BSD 依赖兼容，
法律负担低，适合当前“先开放协作、再考虑商业化”的阶段。若未来需要专利授权或
双许可证，应在单独 PR 中重新评估，并同步所有法律与文档文本。

仍待定：

1. CLA / DCO 政策；
2. 贡献者对其贡献的授权确认；
3. 如果未来发布到 PyPI，需重新核对 `license` 元数据与 wheel/sdist 中的
   `LICENSE` / `THIRD_PARTY_NOTICES.md`。

## 3. 第三方许可证与声明

本项目包装/依赖以下上游项目。若对外分发源码或 wheel，必须保留其版权声明与
许可证文本，并在发布物中提供第三方声明。第三方通知文件已整理在
[THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)；发布前仍需核对上游 LICENSE
原文，而不是只依赖本表。

| 上游项目 | 许可证 | 用途 |
|---|---|---|
| [elevation_mapping_cupy_core](https://github.com/leggedrobotics/elevation_mapping_cupy_core) | MIT | CUDA 融合内核、`ElevationMap`、插件管理 |
| [elevation_mapping_cupy](https://github.com/leggedrobotics/elevation_mapping_cupy) | MIT | GPU elevation mapping（Miki et al., IROS 2022） |
| [elevation_mapping](https://github.com/ANYbotics/elevation_mapping) | BSD-3-Clause | 原始概率地形建图（Fankhauser et al., RA-L 2018） |
| [grid_map](https://github.com/ANYbotics/grid_map) | BSD-3-Clause | 栅格地图数据结构与消息 |

注意：

- 上游许可证文本以其仓库为准；发布前应重新核对 LICENSE 文件，而不是只依赖本表。
- `cupy-cuda12x`、`numpy`、`PyYAML`、`rosbags` 等 Python 依赖也有各自的许可证，
  生成第三方声明时应一并纳入。
- MIT 与上游 MIT/BSD 依赖兼容；无论未来是否改为 source-available 或双许可，
  第三方声明都不能省略。

## 4. 支持矩阵

| 项 | 状态 |
|---|---|
| 操作系统 | x86_64 Linux（Ubuntu 24.04 为基线） |
| GPU | NVIDIA GPU；CUDA 12.x headers（NVRTC） |
| Python | 3.12（CI 只测 3.12） |
| Python 3.10 / 3.11 | 暂不支持；`pyproject` 的 `requires-python` 范围较宽只是兼容上游依赖，等 CI matrix 建立后再承诺 |
| aarch64 / Jetson | 实验性；无 CI 覆盖，DGX Spark 等平台需单独验证 |
| ROS2 桥接 | 可选；需要系统 ROS2（Jazzy 实测）、`rclpy` 与消息包；本仓库**没有 ament `package.xml`**，使用 `uv run pico-elevationmap-ros` console script；示例配置见 `src/pico_elevationmap/configs/ros_camera_lidar.yaml` |
| RealSense D435i | 可选 extra；直连 SDK 路径为原型质量 |
| 四相机 D435i rig | 配置/单路 rig 路径已验证；四台同时工作、USB 带宽与逐相机外参标定未验证 |
| LiDAR（RoboSense Airy / Livox） | 可选；依赖厂商 ROS2 驱动，外参与时间同步仍需人工配置。Airy + D435i 双源路径已于 2026-10-30 复验（30 帧、7 个 LiDAR 帧）；Livox Mid-360 仍未在本机验证 |
| torch 批量后端 / `bench` | 暂缓（Phase 3），当前为占位/stub |
| 生产环境 | 不就绪，见 §6 |

## 5. 发布检查清单

- [x] 许可证已定为 MIT：`LICENSE`、`pyproject.toml`、README 与 design/readiness 已同步
- [x] 添加并核对 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md)，核对上游 MIT/BSD 文本
- [x] 重新生成不绑定 Aliyun 镜像的 `uv.lock`，并让 `uv lock --check` 通过
- [ ] `uv sync --frozen` 在干净环境可复现（待本轮 CI）
- [x] `env -u PYTHONPATH uv run pytest -q` 通过（GPU/ROS 用例按环境 skip）
- [x] `uv run ruff check .` 与 `uv run ruff format --check .` 通过
- [x] `uv build` 成功，`uvx twine check dist/*` 通过
- [x] 在临时 venv 安装 wheel，运行 `pico-elevationmap --help`，并确认
      `pico_elevationmap.configs/*.yaml` 可从 wheel 中发现（CI smoke 已在 run
      `40971256320` 执行，本轮再次本地验证）
- [x] CI workflow 已在 GitHub Actions 上实际执行并通过（run `40971256320`，Python 3.12）
- [x] wheel 内含 `py.typed`；`CHANGELOG.md` 与 `THIRD_PARTY_NOTICES.md` 通过
      `data-files` 进入 wheel，sdist 通过 `MANIFEST.in` 包含文档与 launch 示例
- [x] 确认 `docs/assets/` 未打入不必要的大文件
- [x] 确认 `launch/` 仍为 repo-only 示例，未被打入 wheel（除非明确打包）
- [x] 更新 `CHANGELOG.md` 与版本号；确认 `docs/` 链接有效（21 个 Markdown 文件，0 个失效本地链接）
- [x] 确认 README 不含私有/失效链接；内部 sibling 仓库已移除或标注
- [x] 确认仓库不含 `.venv*`、缓存、`maps/`、`*.npz` 等生成物或超过 5 MB 的二进制
- [x] 在至少一台 x86_64 + NVIDIA GPU 机器上跑通 README Quickstart
- [ ] 如需 ROS2 发布，单独验证 `requires_ros` 用例与 `uv run pico-elevationmap-ros --help`
- [ ] 打 tag `v0.4.0`（与 `pyproject.toml` / CHANGELOG 版本一致；执行前由所有者确认）
- [ ] 确认 tag 触发的 `.github/workflows/release.yml` 构建 / twine 检查 / GitHub Release 成功（该 workflow 不发布到 PyPI，也不改变许可证）
- [ ] 发布说明中写明支持矩阵、已知限制与“非生产就绪”状态

## 6. 已知限制与非生产就绪范围

以下内容**不应被理解为生产可用**：

- **未做真实硬件标定**：D435i + LiDAR 的外参、时间同步、相机-LiDAR 统一时间戳
  仍是手工/近似流程；`--lidar-z-offset` 等参数需要现场调整。
- **四相机 D435i rig 未做四机验证**：`pico_elevationmap/live/quad_camera_live.py` 和
  `src/pico_elevationmap/configs/quad_camera.yaml` 已实现并只在本机单台 D435i 的
  `--cameras front` 子集上验证；四台同时工作时的 USB3 带宽、供电、逐相机外参与
  时间同步仍需实测。
- **ROS2 / LiDAR 验证范围**：RoboSense Airy + D435i 双源路径已于 2026-10-30 在 ROS2
  Jazzy 下复验（30 帧相机、7 个 LiDAR 帧）；Livox Mid-360 仍无本机硬件验证。时间同步
  与相机-LiDAR 外参仍是手工流程，不构成标定精度验证。
- **ROS2 桥接只做过桩测试**：消息转换、GridMap 打包与节点接线由 `tests/ros_stubs.py`
  覆盖；真实 ROS2 集成用例标记为 `requires_ros`，本机无 ROS 时自动 skip。示例配置
  `src/pico_elevationmap/configs/ros_camera_lidar.yaml` 已通过解析测试，但尚未在真实
  ROS 图中验证。
- **无 ament 包**：不能 `ros2 run pico_elevationmap_ros ...`；目前只有 Python
  console script，ROS2 launch 通过 `launch/bringup.launch.py` 直接启动脚本。
- **`launch/` 是 repo-only**：`launch/bringup.launch.py` 不会自动进入 wheel；
  wheel 用户需要从源码 checkout 获取，或自行复制到 ROS2 包中。
- **GPU 测试依赖机器**：CI 不跑 CUDA 用例；没有 GPU 时相关测试 skip，不能据此
  宣称 GPU 路径已在 CI 中验证。
- **CI 覆盖面有限**：GitHub Actions 的 Python 3.12 push run 已通过，但 CI 不跑
  GPU、真实 ROS 或四相机硬件；四相机提交后仍需再跑一次，且不能据此宣称这些
  硬件路径已验证。
- **Python 3.10/3.11 未测**：当前 CI 只有 3.12；不要把 `requires-python` 范围当作
  支持承诺。
- **aarch64 / Jetson 未验证**：CUDA 13 / sm_121 等平台需要额外处理，见
  [deploy-hardware.md](deploy-hardware.md)。
- **torch 后端 / `bench` 未实现**：`bench` 子命令仍是 Phase 3 stub，现在会以非零
  状态退出；不要在工作流中依赖它。
- **wheel 依赖解析（P1）**：wheel 的 `Requires-Dist` 只是未固定版本的
  `elevation-mapping-cupy-core`，而该依赖当前仅通过 `tool.uv.sources` 的 git rev
  提供；因此不带 `--no-deps` 的 `uv pip install dist/*.whl` 会失败。GitHub-only
  发布以 `bash scripts/setup_env.sh` 的**源码安装**为主路径；CI wheel smoke 用
  `--no-deps` 检查 wheel 载荷与 CLI，不宣称“可直接从 PyPI 安装”。
- **`scripts/` 顶层命名风险（已解决）**：CLI / pipeline / live demo / configs 已移入
  `pico_elevationmap`；顶层 `scripts/` 目录只保留 `setup_env.sh`，不再作为 Python
  包进入 wheel。
- **英文文档不足（P1）**：README 是英文入口，但 `docs/` 仍以中文为主；公开面向国际
  用户前，至少补英文 Quickstart、API 与 readiness 摘要。
- **私有/失效链接**：README 已移除 PICO monorepo 与 sibling 仓库链接；其他历史
  文档（如 `docs/design.md`）仍可能引用内部仓库，已用历史文档 banner 标注。
- **默认输出路径**：ROS 节点默认 `save_path` 为 `elevation_map_save.npz`；该文件
  已在 `.gitignore` 中，避免误提交。
- **许可证**：MIT 已定；CLA/DCO 与贡献者授权政策仍待定，不要把它与许可证选择混为一谈。

## 7. 建议的下一步（P1）

1. 补齐英文 Quickstart / API / readiness 摘要，并在 README 中加入英文索引。
2. 确认 CLA/DCO 与贡献者授权政策；许可证已定为 MIT，无需重新选择，但需在
   `CONTRIBUTING.md` 中写清贡献条款。
3. 建立 CI matrix（至少 3.12；再评估 3.10/3.11）与 GPU 专用 runner/自托管测试。
4. 补真实 ROS2 集成测试与 ament 包（若确实需要 `ros2 run`/`ros2 launch` 生态入口），
   并在真实 ROS 图中运行 `ros_camera_lidar.yaml`。
5. 完成 D435i + LiDAR 的标定与时间同步流程，补充可复现的实机验证记录。
6. 完成四相机 D435i 整机的 USB 带宽、供电与逐相机外参验证。
