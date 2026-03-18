# 使用 Foxglove 查看点云（无 ROS）

本文档说明如何在不依赖 ROS/ROS2 的情况下，用 Foxglove Studio 查看 `rslidar_sdk` 实时点云。

## 1. 准备依赖

- `yaml-cpp`
- `libpcap`（如需 PCAP 输入）
- 支持平台：macOS(`arm64`/`x86_64`)、Linux(`aarch64`/`x86_64`)

Foxglove C++ SDK 会在 CMake 配置阶段自动下载。

## 2. 编译 C++ 节点

```bash
cd rslidar_sdk
mkdir -p build
cd build
cmake ..
make -j4
```

## 3. 启动（推荐一键）

```bash
cd rslidar_sdk
./scripts/start_foxglove.sh ./build/rslidar_sdk_node
```

也可以直接运行节点：

```bash
cd rslidar_sdk
./build/rslidar_sdk_node ./config/config.yaml
```

## 4. Foxglove Studio 连接

在 Foxglove Studio 中选择 Open connection，填入：

- `ws://127.0.0.1:8765`

默认点云 topic：

- `/rslidar_points`

## 5. 常见问题

- 如果看不到点云，请先确认雷达数据端口和 `config.yaml` 中 `msop_port/difop_port` 一致。
- 如果是多雷达，节点会自动发布多个 topic（例如 `/rslidar_points_1`）。
- 若端口冲突，可在 `config.yaml` 中修改 `foxglove.port`。
- 如果延迟高：减小 `num_blks_split`，并开启 `low_latency_drop_old_frames`。
- 如果丢失多：增大 `socket_recv_buf`，关闭 `low_latency_drop_old_frames`，并增大 `max_pending_frames`。
