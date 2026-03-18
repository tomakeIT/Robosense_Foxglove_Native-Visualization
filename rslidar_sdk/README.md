# rslidar_sdk (Foxglove Native Edition)

[中文文档](README_CN.md)

## Overview

This project is a ROS-free RoboSense LiDAR runtime:

- Decode LiDAR packets with `rs_driver`.
- Publish point cloud directly from C++ to Foxglove WebSocket.
- Remove ROS/ROS2/RViz runtime dependency.

## Architecture

1. `rs_driver` receives MSOP/DIFOP packets (online or PCAP).
2. `rslidar_sdk_node` converts decoded points to Foxglove `PointCloud`.
3. Built-in C++ WebSocket server streams topics to Foxglove Studio.

## Supported Platforms

- macOS: `arm64`, `x86_64`
- Linux: `aarch64`, `x86_64`

Note: 32-bit Linux ARM is not included in bundled Foxglove SDK artifacts.

## Data Sources

- `common.msg_source: 1` - online LiDAR
- `common.msg_source: 3` - PCAP file

## Dependencies

- `yaml-cpp`
- `libpcap` (required for PCAP playback)
- Foxglove C++ SDK (automatically downloaded by CMake)

## Build

```bash
cd rslidar_sdk
mkdir -p build
cd build
cmake ..
cmake --build . -j4
```

## Run

### Option A: one command (recommended)

```bash
cd rslidar_sdk
./scripts/start_foxglove.sh ./build/rslidar_sdk_node
```

### Option B: run binary directly

```bash
cd rslidar_sdk
./build/rslidar_sdk_node ./config/config.yaml
```

## Foxglove Connection

- URL: `ws://127.0.0.1:8765`
- Default topic: `/rslidar_points`

## Configuration Guide

Main file: `config/config.yaml`

- `common`
  - `msg_source`: input mode (online or PCAP).
- `lidar[].driver`
  - packet ports, lidar type, split strategy, timestamp policy.
- `foxglove`
  - WebSocket endpoint, topic, frame id, queue policy, preallocation knobs.

## Tuning Profiles

### Lowest Latency

- small `num_blks_split` (for example `90` or `120`)
- `low_latency_drop_old_frames: true`
- `max_pending_frames: 1~2`

### Lowest Data Loss

- larger `num_blks_split` (for example `180` or `240`)
- `low_latency_drop_old_frames: false`
- `max_pending_frames: 0` (or a larger value)
- larger `socket_recv_buf` and preallocation values

## Open Source Notes

- License: `LICENSE`
- Upstream driver docs are preserved in `rslidar_sdk/src/rs_driver/doc/`
- This fork focuses on Foxglove-native visualization and cross-platform runtime
