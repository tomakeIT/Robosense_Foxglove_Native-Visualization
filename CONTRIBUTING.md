# Contributing Guide

Thank you for improving this project.

## Development Setup

```bash
cd rslidar_sdk
mkdir -p build && cd build
cmake ..
cmake --build . -j4
```

## Code and Config Changes

- Keep runtime free of ROS/ROS2 dependencies.
- Keep default configuration in `rslidar_sdk/config/config.yaml` runnable on localhost.
- Prefer small, focused changes with clear rationale.
- Update `README.md` / `README_CN.md` when behavior or configuration changes.

## Pull Request Checklist

- [ ] Build succeeds locally (`cmake --build`).
- [ ] Updated docs for any user-facing change.
- [ ] No secrets, credentials, or machine-specific paths committed.
- [ ] Changes are tested with online LiDAR or PCAP playback.

## Commit Message Recommendation

Use clear prefixes such as `feat:`, `fix:`, `docs:`, `refactor:`, `build:`.
