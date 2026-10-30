# 设计决策记录

> 版本：v0.1　日期：2026-10-30　状态：持续记录
> 关联：[README.md](README.md)、[open-source-readiness.md](open-source-readiness.md)、[TODO.md](../TODO.md)

## D001：可视化后端保留自研 viewer，不引入 Rerun

- **日期**：2026-10-30
- **决策**：
  - 默认继续使用 `src/pico_elevationmap/viz/` 的自研 OpenCV viewer（2D 俯视 + 3D 体素单窗口）。
  - Foxglove 继续作为 ROS 图/在线调试的可选后端。
  - **不引入 `rerun-sdk`**，也不把 Rerun 列入 `viz` extra。
- **理由**：
  - `rerun-sdk` Linux x86_64 wheel 约 170 MB，且需要额外 viewer 进程；对机载/边缘部署偏重。
  - 当前 viewer 复用已有 OpenCV/numpy，满足 2D/3D、旋转/缩放和单窗口需求。
  - Rerun 的时间回溯/多传感器同步能力更强，但当前阶段不是刚需；官方也说明大量实体/百万级点云时可能变慢。
- **影响**：
  - 不再把 Rerun 作为待办项；如果未来需要时间回溯、多传感器同步或更强的 3D 调试，再重新评估。
  - 维护重点放在自研 viewer 的性能、交互和测试，以及 Foxglove 的 ROS 集成上。
