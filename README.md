# RoboSense Foxglove Native Visualization

ROS-free RoboSense LiDAR visualization stack with native C++ Foxglove WebSocket publishing.

## What This Repository Provides

- Native `rslidar_sdk_node` that decodes RoboSense packets and streams point cloud directly to Foxglove.
- No runtime ROS/ROS2/RViz dependency.
- Cross-platform CMake build support for:
  - macOS (`arm64`, `x86_64`)
  - Linux (`aarch64`, `x86_64`)

## Project Layout

- `rslidar_sdk/` - Main SDK integration, C++ executable, runtime config, docs.

## Quick Start

```bash
cd rslidar_sdk
mkdir -p build && cd build
cmake ..
cmake --build . -j4
../scripts/start_foxglove.sh ./rslidar_sdk_node
```

Then open Foxglove Studio and connect to:

- `ws://127.0.0.1:8765`

## Documentation

- English guide: `rslidar_sdk/README.md`
- 中文文档: `rslidar_sdk/README_CN.md`

## License

BSD-3-Clause style license inherited from RoboSense SDK:

- `rslidar_sdk/LICENSE`
