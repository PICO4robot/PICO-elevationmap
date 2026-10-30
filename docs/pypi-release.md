# PyPI 发布与 wheel 依赖解析

> 版本：v0.1　日期：2026-10-30　状态：方案说明，尚未执行
> 关联：[open-source-readiness.md](open-source-readiness.md)、[development.md](development.md)

## 1. 问题

当前 wheel 的 `Requires-Dist` 包含 `elevation-mapping-cupy-core`，但该 distribution
不在 PyPI；`tool.uv.sources` 只对 uv 生效，不会写进 wheel 元数据。因此：

```bash
pip install dist/*.whl          # 会失败：No matching distribution found
pip install --no-deps dist/*.whl  # 可装 wheel，但缺运行依赖
```

PyPI 不接受 `Requires-Dist: package @ git+https://...` 这类直接 URL 依赖，所以不能
只改 pyproject 解决。

## 2. 推荐方案：发布上游 core 的 fork

上游 `elevation_mapping_cupy_core` 是 MIT 许可，可以合法 fork 并重新分发。建议用
**唯一 distribution 名**，避免占用上游名称或造成混淆：

```text
pico-elevation-mapping-cupy-core
```

`import` 名仍是上游的 `elevation_mapping_cupy`，无需改代码导入。

步骤（使用你自己的 PyPI 账号）：

```bash
# 1) fork https://github.com/leggedrobotics/elevation_mapping_cupy_core
git clone https://github.com/<your-account>/elevation_mapping_cupy_core.git
cd elevation_mapping_cupy_core

# 2) 只改 distribution 名，保留 MIT LICENSE 与上游版权声明
#    pyproject.toml: name = "pico-elevation-mapping-cupy-core"
#    可选：version = "0.1.0"

# 3) 构建并检查
uv build
uvx twine check dist/*

# 4) 上传（需要 PyPI API token）
uvx twine upload dist/* -u __token__ -p "$PYPI_TOKEN"
```

然后在本仓库：

```toml
[project]
dependencies = [
  "pico-elevation-mapping-cupy-core>=0.1,<0.2",
  ...
]

# 删除 [tool.uv.sources] 里的 git 项；uv 会改从 PyPI 解析
```

再执行 `uv lock`、`uv sync --frozen`、`uv build`，wheel 的 `Requires-Dist` 就能被
普通 `pip install` 正常解析。

注意：

- 不要直接把 fork 以 `elevation-mapping-cupy-core` 为名上传；那会占用上游名称，除非
  上游维护者明确同意。
- 保留上游 `LICENSE` 与版权声明，并在 `THIRD_PARTY_NOTICES.md` 说明 fork 来源。
- fork 的版本策略要写清：上游更新时需要手动同步并重新发布。

## 3. 备选方案

| 方案 | 优点 | 缺点 |
|---|---|---|
| 上游官方发布到 PyPI | 最干净，用户直接 `pip install` | 取决于上游维护者排期 |
| 本仓库 fork 并发布 | 可控，MIT 允许 | 需要维护 fork 与版本同步 |
| 把上游 core 源码 vendor 进 wheel | 无外部依赖 | wheel 变大，许可证/升级复杂，不推荐 |
| 继续 GitHub-only + `--no-deps` | 当前可用 | 不能宣称普通 `pip install` |

## 4. 当前状态

- 本仓库仍以 GitHub-only + `scripts/setup_env.sh` 源码安装为主路径。
- wheel 依赖解析在 PyPI 发布前仍是一个 P1 阻塞项，见
  [TODO.md](../TODO.md) 与 [open-source-readiness.md](open-source-readiness.md)。
