## Summary

<!-- What changed, and why? Keep this focused on one concern. -->

## Area

- [ ] Core mapper / pose buffer / I/O
- [ ] Plugins / fusion
- [ ] ROS 2 bridge
- [ ] RealSense / LiDAR tooling
- [ ] Packaging / CI / docs

## Verification

- [ ] `env -u PYTHONPATH uv run pytest -q`
- [ ] `uv run ruff check .`
- [ ] `uv run ruff format --check .`
- [ ] Commits include a DCO sign-off (`git commit -s`)
- [ ] Tested on hardware (describe GPU / ROS distro / sensors below, if relevant)
- [ ] Docs updated for user-visible changes

```text
<!-- Paste the relevant test output or hardware notes here. -->
```

## Notes

- The project is MIT-licensed and contributions use the Developer Certificate of
  Origin 1.1 (`git commit -s`). Do not change the license or add CLA wording in an
  unrelated PR.
- Do not include `.venv*/`, caches, `maps/`, generated `*.npz`, or binary data > 5 MB.
- Security issues should follow `SECURITY.md`, not this public PR.
