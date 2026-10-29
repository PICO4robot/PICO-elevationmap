# Contributing to PICO-elevationmap

Thanks for taking the time to contribute. This repository is a ROS-free 2.5D
elevation mapping library; it is pre-1.0 and not production-ready yet. Small,
focused changes with tests and docs are the easiest to review.

## Before you start

- Read [docs/open-source-readiness.md](docs/open-source-readiness.md) for the current
  support matrix and known limitations.
- Search existing issues and pull requests before opening a new one.
- For a large change, open a feature request first so the scope can be agreed.

## Development setup

Python 3.12 and [uv](https://docs.astral.sh/uv/) are required. Mapping also needs an
NVIDIA GPU with CUDA 12.x headers; CPU tests and `--help` paths do not.

```bash
git clone https://github.com/PICO4robot/PICO-elevationmap.git
cd PICO-elevationmap
uv sync
```

See [docs/development.md](docs/development.md) for CUDA headers, RealSense and
optional ROS2 setup.

## Tests and lint

Run these before opening a pull request:

```bash
env -u PYTHONPATH uv run pytest -q
uv run ruff check .
```

GPU and ROS integration tests skip automatically when the corresponding hardware or
environment is absent. Do not add hardcoded pass counts to docs. If you have a GPU or
a sourced ROS2 environment, run the relevant tests there and say so in the PR.

## Style

- Follow the existing src layout, Chinese/English documentation style and ruff
  settings (`line-length = 120`, `E`, `W`, `F`, `I`, `UP`).
- Keep the core package ROS-free. ROS imports belong in `pico_elevationmap_ros`;
  the console script must remain importable without ROS.
- Add tests for behavior changes, especially CLI parsing, I/O and config validation.
- Do not commit `.venv/`, `.venv-ros/`, caches, `maps/`, generated `*.npz`, or
  binary data larger than 5 MB.

## Commit messages

Use conventional commits with the `(map)` scope, matching the repository history:

```text
fix(map): parse --config=path in ROS CLI
test(map): cover save-map parent directories
docs(map): add release readiness checklist
```

## Pull requests

- Keep each PR scoped to one concern; include tests and docs where relevant.
- Describe how you verified the change (commands, hardware, ROS distro, GPU).
- The project is MIT-licensed; by contributing you agree that your contribution is
  provided under the MIT License. Do not change the license in an unrelated PR.
- A CLA or DCO policy has **not** been decided (TBD). Do not add a sign-off
  requirement or CLA wording on your own.

By participating, you agree to follow the [Code of Conduct](CODE_OF_CONDUCT.md).
Security issues should follow [SECURITY.md](SECURITY.md), not a public issue.
