# Security Policy

## Supported versions

This project is pre-1.0 and **not production-ready**. Security fixes are currently
provided on the `main` branch for the latest 0.4.x version only; older versions and
unreleased branches are unsupported.

## Reporting a vulnerability

Please **do not open a public issue** for a security problem.

Use GitHub's private vulnerability reporting for this repository:

1. Open the [Security Advisories page](https://github.com/PICO4robot/PICO-elevationmap/security/advisories/new).
2. Describe the issue, affected version/commit, and a minimal reproduction.
3. Include the environment (OS, Python, GPU/driver, ROS distro) if relevant.

If private reporting is unavailable, contact the repository maintainers through their
GitHub profiles in the [PICO4robot organization](https://github.com/PICO4robot).
Do not include credentials, tokens or personal data in a public issue or pull request.

## Scope

In scope:

- the Python packages `pico_elevationmap` and `pico_elevationmap_ros`;
- the `pico-elevationmap` and `pico-elevationmap-ros` console scripts;
- configuration parsing and file I/O in `scripts/`.

Out of scope:

- vulnerabilities in upstream dependencies or vendor drivers (report those upstream);
- issues caused by running generated code or untrusted rosbag/config files without
  appropriate isolation;
- GPU driver/CUDA installation problems.

## What to expect

This is a volunteer, pre-1.0 project. We will acknowledge reports on a best-effort
basis, investigate, and coordinate disclosure after a fix is available. There is no
bug bounty program and no guaranteed response time.
