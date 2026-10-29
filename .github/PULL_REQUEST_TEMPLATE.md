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
- [ ] Tested on hardware (describe GPU / ROS distro / sensors below, if relevant)
- [ ] Docs updated for user-visible changes

```text
<!-- Paste the relevant test output or hardware notes here. -->
```

## Notes

- The project is MIT-licensed; CLA/DCO policy is still **TBD**. Do not change the
  license or add a sign-off/CLA requirement in an unrelated PR.
- Do not include `.venv*/`, caches, `maps/`, generated `*.npz`, or binary data > 5 MB.
- Security issues should follow `SECURITY.md`, not this public PR.
