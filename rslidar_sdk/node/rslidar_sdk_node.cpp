/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)
*********************************************************************************************************************/

#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <yaml-cpp/yaml.h>

#include <foxglove/error.hpp>
#include <foxglove/schemas.hpp>
#include <foxglove/server.hpp>

#include "msg/rs_msg/lidar_point_cloud_msg.hpp"
#include "rs_driver/api/lidar_driver.hpp"
#include "rs_driver/macro/version.hpp"
#include "rs_driver/utility/sync_queue.hpp"
#include "utility/yaml_reader.hpp"

using namespace robosense::lidar;

namespace
{
std::atomic<bool> g_running(true);

void sigHandler(int)
{
  g_running.store(false);
}

struct PackedPointXYZI
{
  float x;
  float y;
  float z;
  float intensity;
};

class FoxglovePointCloudPublisher
{
public:
  FoxglovePointCloudPublisher() = default;

  bool init(const YAML::Node& foxglove_cfg, uint32_t lidar_index, const foxglove::Context& context)
  {
    lidar_index_ = lidar_index;

    yamlRead<std::string>(foxglove_cfg, "frame_id", frame_id_, "rslidar");
    std::string base_topic = "/rslidar_points";
    yamlRead<std::string>(foxglove_cfg, "topic", base_topic, "/rslidar_points");
    topic_ = (lidar_index == 0) ? base_topic : (base_topic + "_" + std::to_string(lidar_index));
    yamlRead<uint32_t>(foxglove_cfg, "prealloc_points", prealloc_points_, 120000);

    auto channel_result = foxglove::schemas::PointCloudChannel::create(topic_, context);
    if (!channel_result.has_value())
    {
      RS_ERROR << "Failed to create Foxglove PointCloud channel [" << topic_ << "]: "
               << foxglove::strerror(channel_result.error()) << RS_REND;
      return false;
    }
    channel_ = std::move(channel_result.value());

    reusable_cloud_.frame_id = frame_id_;
    reusable_cloud_.pose = defaultPose();
    reusable_cloud_.point_stride = sizeof(PackedPointXYZI);
    reusable_cloud_.fields = pointFields();
    if (prealloc_points_ > 0)
    {
      reusable_cloud_.data.reserve(static_cast<size_t>(prealloc_points_) * sizeof(PackedPointXYZI));
    }
    return true;
  }

  bool sendPointCloud(const LidarPointCloudMsg& msg)
  {
    if (!channel_.has_value())
    {
      return false;
    }

    reusable_cloud_.timestamp = toTimestamp(msg.timestamp);
    reusable_cloud_.data.resize(msg.points.size() * sizeof(PackedPointXYZI));

    for (size_t i = 0; i < msg.points.size(); ++i)
    {
      PackedPointXYZI p{
          msg.points[i].x,
          msg.points[i].y,
          msg.points[i].z,
          static_cast<float>(msg.points[i].intensity),
      };
      std::memcpy(reusable_cloud_.data.data() + i * sizeof(PackedPointXYZI), &p, sizeof(PackedPointXYZI));
    }

    const auto err = channel_->log(reusable_cloud_);
    if (err != foxglove::FoxgloveError::Ok)
    {
      RS_WARNING << "Failed to log Foxglove PointCloud on [" << topic_ << "]: "
                 << foxglove::strerror(err) << RS_REND;
      return false;
    }
    return true;
  }

private:
  static foxglove::schemas::Timestamp toTimestamp(double ts_sec)
  {
    const auto sec = static_cast<uint32_t>(ts_sec);
    const auto nsec = static_cast<uint32_t>((ts_sec - static_cast<double>(sec)) * 1e9);
    return foxglove::schemas::Timestamp{sec, nsec};
  }

  static foxglove::schemas::Pose defaultPose()
  {
    foxglove::schemas::Pose pose;
    pose.position = foxglove::schemas::Vector3{0.0, 0.0, 0.0};
    pose.orientation = foxglove::schemas::Quaternion{0.0, 0.0, 0.0, 1.0};
    return pose;
  }

  static std::vector<foxglove::schemas::PackedElementField> pointFields()
  {
    using NumericType = foxglove::schemas::PackedElementField::NumericType;
    return {
        {"x", 0, NumericType::FLOAT32},
        {"y", 4, NumericType::FLOAT32},
        {"z", 8, NumericType::FLOAT32},
        {"intensity", 12, NumericType::FLOAT32},
    };
  }

private:
  uint32_t lidar_index_ = 0;
  std::string frame_id_;
  std::string topic_;
  uint32_t prealloc_points_ = 120000;
  std::optional<foxglove::schemas::PointCloudChannel> channel_;
  foxglove::schemas::PointCloud reusable_cloud_;
};

class DriverPipeline
{
public:
  DriverPipeline() = default;

  bool init(
      const YAML::Node& lidar_node,
      InputType input_type,
      const YAML::Node& foxglove_cfg,
      const foxglove::Context& context,
      uint32_t lidar_index)
  {
    lidar_index_ = lidar_index;
    const YAML::Node driver_cfg = yamlSubNodeAbort(lidar_node, "driver");

    RSDriverParam param;
    param.input_type = input_type;

    std::string lidar_type = "RSM2";
    yamlRead<std::string>(driver_cfg, "lidar_type", lidar_type, "RSM2");
    param.lidar_type = strToLidarType(lidar_type);

    yamlRead<uint16_t>(driver_cfg, "msop_port", param.input_param.msop_port, 6699);
    yamlRead<uint16_t>(driver_cfg, "difop_port", param.input_param.difop_port, 7788);
    yamlRead<uint16_t>(driver_cfg, "imu_port", param.input_param.imu_port, 0);
    yamlRead<uint16_t>(driver_cfg, "user_layer_bytes", param.input_param.user_layer_bytes, 0);
    yamlRead<uint16_t>(driver_cfg, "tail_layer_bytes", param.input_param.tail_layer_bytes, 0);
    yamlRead<std::string>(driver_cfg, "host_address", param.input_param.host_address, "0.0.0.0");
    yamlRead<std::string>(driver_cfg, "group_address", param.input_param.group_address, "0.0.0.0");
    yamlRead<uint32_t>(driver_cfg, "socket_recv_buf", param.input_param.socket_recv_buf, 4 * 1024 * 1024);

    yamlRead<float>(driver_cfg, "min_distance", param.decoder_param.min_distance, 0.0f);
    yamlRead<float>(driver_cfg, "max_distance", param.decoder_param.max_distance, 0.0f);
    yamlRead<bool>(driver_cfg, "use_lidar_clock", param.decoder_param.use_lidar_clock, true);
    yamlRead<bool>(driver_cfg, "dense_points", param.decoder_param.dense_points, false);
    yamlRead<bool>(driver_cfg, "ts_first_point", param.decoder_param.ts_first_point, true);
    yamlRead<bool>(driver_cfg, "wait_for_difop", param.decoder_param.wait_for_difop, true);
    yamlRead<float>(driver_cfg, "start_angle", param.decoder_param.start_angle, 0.0f);
    yamlRead<float>(driver_cfg, "end_angle", param.decoder_param.end_angle, 360.0f);
    int split_frame_mode = static_cast<int>(param.decoder_param.split_frame_mode);
    yamlRead<int>(driver_cfg, "split_frame_mode", split_frame_mode, static_cast<int>(SplitFrameMode::SPLIT_BY_ANGLE));
    if (split_frame_mode < static_cast<int>(SplitFrameMode::SPLIT_BY_ANGLE) ||
        split_frame_mode > static_cast<int>(SplitFrameMode::SPLIT_BY_CUSTOM_BLKS))
    {
      RS_WARNING << "Invalid split_frame_mode=" << split_frame_mode
                 << ", fallback to SPLIT_BY_ANGLE(1)." << RS_REND;
      split_frame_mode = static_cast<int>(SplitFrameMode::SPLIT_BY_ANGLE);
    }
    param.decoder_param.split_frame_mode = static_cast<SplitFrameMode>(split_frame_mode);
    yamlRead<float>(driver_cfg, "split_angle", param.decoder_param.split_angle, 0.0f);
    yamlRead<uint16_t>(driver_cfg, "num_blks_split", param.decoder_param.num_blks_split, 1);

    yamlRead<bool>(foxglove_cfg, "low_latency_drop_old_frames", low_latency_drop_old_frames_, false);
    yamlRead<uint32_t>(foxglove_cfg, "drop_log_every", drop_log_every_, 100);
    yamlRead<uint32_t>(foxglove_cfg, "max_pending_frames", max_pending_frames_, 2);
    yamlRead<uint32_t>(foxglove_cfg, "prealloc_cloud_msgs", prealloc_cloud_msgs_, 8);
    yamlRead<uint32_t>(foxglove_cfg, "prealloc_points_per_msg", prealloc_points_per_msg_, 120000);

    for (uint32_t i = 0; i < prealloc_cloud_msgs_; ++i)
    {
      auto prealloc_msg = std::make_shared<LidarPointCloudMsg>();
      if (prealloc_points_per_msg_ > 0)
      {
        prealloc_msg->points.reserve(prealloc_points_per_msg_);
      }
      free_cloud_queue_.push(prealloc_msg);
    }

    if (input_type == InputType::PCAP_FILE)
    {
      yamlReadAbort<std::string>(driver_cfg, "pcap_path", param.input_param.pcap_path);
      yamlRead<bool>(driver_cfg, "pcap_repeat", param.input_param.pcap_repeat, true);
      yamlRead<float>(driver_cfg, "pcap_rate", param.input_param.pcap_rate, 1.0f);
      yamlRead<bool>(driver_cfg, "use_vlan", param.input_param.use_vlan, false);
    }

    if (!publisher_.init(foxglove_cfg, lidar_index_, context))
    {
      return false;
    }

    driver_.regPointCloudCallback(
        std::bind(&DriverPipeline::driverGetPointCloud, this),
        std::bind(&DriverPipeline::driverReturnPointCloud, this, std::placeholders::_1));
    driver_.regExceptionCallback(std::bind(&DriverPipeline::exceptionCallback, this, std::placeholders::_1));

    if (!driver_.init(param))
    {
      RS_ERROR << "Failed to init driver index " << lidar_index_ << RS_REND;
      return false;
    }

    return true;
  }

  bool start()
  {
    running_ = true;
    process_thread_ = std::thread(&DriverPipeline::processLoop, this);
    return driver_.start();
  }

  void stop()
  {
    driver_.stop();
    running_ = false;
    stuffed_cloud_queue_.push(nullptr);
    if (process_thread_.joinable())
    {
      process_thread_.join();
    }
  }

private:
  std::shared_ptr<LidarPointCloudMsg> driverGetPointCloud()
  {
    auto msg = free_cloud_queue_.pop();
    if (msg)
    {
      return msg;
    }
    return std::make_shared<LidarPointCloudMsg>();
  }

  void driverReturnPointCloud(std::shared_ptr<LidarPointCloudMsg> msg)
  {
    if (max_pending_frames_ > 0)
    {
      while (stuffed_cloud_queue_.size() >= max_pending_frames_)
      {
        auto stale = stuffed_cloud_queue_.pop();
        if (!stale)
        {
          break;
        }
        free_cloud_queue_.push(stale);
        dropped_frames_total_.fetch_add(1, std::memory_order_relaxed);
      }
    }
    stuffed_cloud_queue_.push(msg);
  }

  void exceptionCallback(const Error& code)
  {
    RS_WARNING << "Driver[" << lidar_index_ << "] " << code.toString() << RS_REND;
  }

  void processLoop()
  {
    while (running_)
    {
      auto msg = stuffed_cloud_queue_.popWait();
      if (!msg)
      {
        continue;
      }

      if (low_latency_drop_old_frames_)
      {
        uint32_t dropped = 0;
        while (true)
        {
          auto newer = stuffed_cloud_queue_.pop();
          if (!newer)
          {
            break;
          }
          free_cloud_queue_.push(msg);
          msg = newer;
          ++dropped;
        }
        if (dropped > 0)
        {
          const uint64_t total_dropped = dropped_frames_total_.fetch_add(dropped, std::memory_order_relaxed) + dropped;
          if (drop_log_every_ > 0 && (total_dropped / drop_log_every_) != (last_drop_log_bucket_))
          {
            last_drop_log_bucket_ = total_dropped / drop_log_every_;
            RS_WARNING << "Lidar[" << lidar_index_ << "] dropped old frames for low-latency mode, total dropped="
                       << total_dropped << RS_REND;
          }
        }
      }
      publisher_.sendPointCloud(*msg);
      free_cloud_queue_.push(msg);
    }
  }

private:
  uint32_t lidar_index_ = 0;
  std::atomic<bool> running_{false};
  std::thread process_thread_;
  LidarDriver<LidarPointCloudMsg> driver_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> stuffed_cloud_queue_;
  FoxglovePointCloudPublisher publisher_;
  bool low_latency_drop_old_frames_ = false;
  uint32_t max_pending_frames_ = 2;
  uint32_t drop_log_every_ = 100;
  uint32_t prealloc_cloud_msgs_ = 8;
  uint32_t prealloc_points_per_msg_ = 120000;
  std::atomic<uint64_t> dropped_frames_total_{0};
  uint64_t last_drop_log_bucket_ = 0;
};

std::string parseConfigPath(int argc, char** argv)
{
  if (argc > 1 && argv[1] != nullptr && std::strlen(argv[1]) > 0)
  {
    return argv[1];
  }
  return std::string(PROJECT_PATH) + "/config/config.yaml";
}

}  // namespace

int main(int argc, char** argv)
{
  signal(SIGINT, sigHandler);

  RS_TITLE << "********************************************************" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "**********    RSLidar_SDK Version: v" << RSLIDAR_VERSION_MAJOR << "." << RSLIDAR_VERSION_MINOR << "."
           << RSLIDAR_VERSION_PATCH << "     **********" << RS_REND;
  RS_TITLE << "**********                                    **********" << RS_REND;
  RS_TITLE << "********************************************************" << RS_REND;

  const std::string config_path = parseConfigPath(argc, argv);
  YAML::Node config;
  try
  {
    config = YAML::LoadFile(config_path);
    RS_INFO << "Config loaded from: " << config_path << RS_REND;
  }
  catch (const std::exception& e)
  {
    RS_ERROR << "Failed to load config: " << e.what() << RS_REND;
    return -1;
  }

  const YAML::Node common_cfg = yamlSubNodeAbort(config, "common");
  const YAML::Node lidar_list = yamlSubNodeAbort(config, "lidar");
  YAML::Node fox_cfg = config["foxglove"];
  if (!fox_cfg)
  {
    fox_cfg = YAML::Node(YAML::NodeType::Map);
  }

  int msg_source = 1;
  yamlRead<int>(common_cfg, "msg_source", msg_source, 1);
  if (msg_source != 1 && msg_source != 3)
  {
    RS_ERROR << "Unsupported msg_source after ROS removal. Use 1 (online lidar) or 3 (pcap)." << RS_REND;
    return -1;
  }
  const InputType input_type = (msg_source == 3) ? InputType::PCAP_FILE : InputType::ONLINE_LIDAR;

  foxglove::Context context;
  foxglove::WebSocketServerOptions ws_options;
  ws_options.context = context;
  ws_options.name = "rslidar_sdk_cpp";
  std::string ws_host = "127.0.0.1";
  uint16_t ws_port = 8765;
  yamlRead<std::string>(fox_cfg, "host", ws_host, "127.0.0.1");
  yamlRead<uint16_t>(fox_cfg, "port", ws_port, 8765);
  ws_options.host = ws_host;
  ws_options.port = ws_port;

  auto ws_result = foxglove::WebSocketServer::create(std::move(ws_options));
  if (!ws_result.has_value())
  {
    RS_ERROR << "Failed to create Foxglove WebSocket server: " << foxglove::strerror(ws_result.error()) << RS_REND;
    return -1;
  }
  auto ws_server = std::move(ws_result.value());
  RS_INFO << "Foxglove WebSocket listening at ws://" << ws_host << ":" << ws_server.port() << RS_REND;

  std::vector<std::shared_ptr<DriverPipeline>> pipelines;
  pipelines.reserve(lidar_list.size());
  for (size_t i = 0; i < lidar_list.size(); ++i)
  {
    auto pipeline = std::make_shared<DriverPipeline>();
    if (!pipeline->init(lidar_list[i], input_type, fox_cfg, context, static_cast<uint32_t>(i)))
    {
      RS_ERROR << "Failed to initialize lidar pipeline index " << i << RS_REND;
      return -1;
    }
    pipelines.emplace_back(std::move(pipeline));
  }

  for (auto& pipeline : pipelines)
  {
    if (!pipeline->start())
    {
      RS_ERROR << "Failed to start a lidar pipeline." << RS_REND;
      return -1;
    }
  }

  RS_MSG << "RoboSense-LiDAR-Driver is running with native C++ Foxglove WebSocket." << RS_REND;
  while (g_running.load())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  for (auto& pipeline : pipelines)
  {
    pipeline->stop();
  }
  ws_server.stop();
  RS_MSG << "RoboSense-LiDAR-Driver stopped." << RS_REND;
  return 0;
}
