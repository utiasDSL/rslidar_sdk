
#include "rslidar_component.hpp"
#include "source/destination_packet_ros.hpp"
#include "source/destination_pointcloud_ros.hpp"
#include "source/source_driver.hpp"
#include "source/source_packet_ros.hpp"

namespace robosense::lidar {

RSLiDARComponent::RSLiDARComponent(const rclcpp::NodeOptions &options)
    : rclcpp::Node("rslidar_component", options) {

  RCLCPP_INFO(get_logger(), "================================");
  RCLCPP_INFO(get_logger(), "        RSLIDAR ROS2 Node       ");
  RCLCPP_INFO(get_logger(), "================================");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), " * Intra process communication: %s",
              options.use_intra_process_comms() ? "Enabled" : "Disabled");
  if (!options.use_intra_process_comms())
    RCLCPP_WARN(get_logger(), " * Intra process communication disabled!");
  RCLCPP_INFO(get_logger(), "================================");

  const std::chrono::milliseconds init_msec(static_cast<int>(50.0));
  initTimer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(init_msec),
      std::bind(&RSLiDARComponent::init, this));

   start_thread_ = std::thread([this]() {
    this->start();
   });
}

RSLiDARComponent::~RSLiDARComponent() { stop(); 
if (start_thread_.joinable())
{
  start_thread_.join();
}}

void RSLiDARComponent::init() {

  initTimer_->cancel();

  RCLCPP_INFO(get_logger(), " * Initializing ...");

  auto config_path =
      static_cast<std::string>(PROJECT_PATH) + "/config/config.yaml";
  RCLCPP_WARN(get_logger(), " * Loading configurations from %s",
              config_path.c_str());
  const auto config_path_from_ros =
          this->declare_parameter<std::string>("config_path", "");

  if (!config_path_from_ros.empty()) {
    config_path = config_path_from_ros;
  }

  const auto config = YAML::LoadFile(config_path);

  const YAML::Node common_config = yamlSubNodeAbort(config, "common");

  int msg_source = 0;
  yamlRead<int>(common_config, "msg_source", msg_source, 0);

  bool send_packet_ros;
  yamlRead<bool>(common_config, "send_packet_ros", send_packet_ros, false);

  bool send_point_cloud_ros;
  yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros,
                 false);

  bool send_point_cloud_proto;
  yamlRead<bool>(common_config, "send_point_cloud_proto",
                 send_point_cloud_proto, false);

  bool send_packet_proto;
  yamlRead<bool>(common_config, "send_packet_proto", send_packet_proto, false);

  YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
  for (uint8_t i = 0; i < lidar_config.size(); ++i) {
    std::shared_ptr<Source> source;

    switch (msg_source) {
    case SourceType::MSG_FROM_LIDAR: // online lidar
    {
      RS_INFO << "------------------------------------------------------"
              << RS_REND;
      RS_INFO << "Receive Packets From : Online LiDAR" << RS_REND;
      RS_INFO << "Msop Port: "
              << lidar_config[i]["driver"]["msop_port"].as<uint16_t>()
              << RS_REND;
      RS_INFO << "Difop Port: "
              << lidar_config[i]["driver"]["difop_port"].as<uint16_t>()
              << RS_REND;
      RS_INFO << "------------------------------------------------------"
              << RS_REND;

      source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_LIDAR);
      source->init(lidar_config[i]);
      break;
    }
    case SourceType::MSG_FROM_ROS_PACKET: // pkt from ros
    {
      RS_INFO << "------------------------------------------------------"
              << RS_REND;
      RS_INFO << "Receive Packets From : ROS" << RS_REND;
      RS_INFO
          << "Msop Topic: "
          << lidar_config[i]["ros"]["ros_recv_packet_topic"].as<std::string>()
          << RS_REND;
      RS_INFO << "------------------------------------------------------"
              << RS_REND;
      auto node_ptr = std::make_unique<rclcpp::Node>(
          "rslidar_source_ros_package_" + std::to_string(i));
      source = std::make_shared<SourcePacketRos>(node_ptr.get());
      source->init(lidar_config[i]);

      break;
    }
    case SourceType::MSG_FROM_PCAP: // pcap
    {
      RS_INFO << "------------------------------------------------------"
              << RS_REND;
      RS_INFO << "Receive Packets From : Pcap" << RS_REND;
      RS_INFO << "Msop Port: "
              << lidar_config[i]["driver"]["msop_port"].as<uint16_t>()
              << RS_REND;
      RS_INFO << "Difop Port: "
              << lidar_config[i]["driver"]["difop_port"].as<uint16_t>()
              << RS_REND;
      RS_INFO << "------------------------------------------------------"
              << RS_REND;

      source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_PCAP);
      source->init(lidar_config[i]);
      break;
    }

    default: {
      RS_ERROR << "Unsupported LiDAR message source:" << msg_source << "."
               << RS_REND;
      exit(-1);
    }
    }

    if (send_packet_ros) {
      RS_DEBUG << "------------------------------------------------------"
               << RS_REND;
      RS_DEBUG << "Send Packets To : ROS" << RS_REND;
      RS_DEBUG
          << "Msop Topic: "
          << lidar_config[i]["ros"]["ros_send_packet_topic"].as<std::string>()
          << RS_REND;
      RS_DEBUG << "------------------------------------------------------"
               << RS_REND;
      std::shared_ptr<DestinationPacket> dst =
          std::make_shared<DestinationPacketRos>(this);
      dst->init(lidar_config[i]);
      source->regPacketCallback(dst);
    }

    if (send_point_cloud_ros) {
      RS_DEBUG << "------------------------------------------------------"
               << RS_REND;
      RS_DEBUG << "Send PointCloud To : ROS" << RS_REND;
      RS_DEBUG << "PointCloud Topic: "
               << lidar_config[i]["ros"]["ros_send_point_cloud_topic"]
                      .as<std::string>()
               << RS_REND;
      RS_DEBUG << "------------------------------------------------------"
               << RS_REND;

      std::shared_ptr<DestinationPointCloud> dst =
          std::make_shared<DestinationPointCloudRos>(this);
      dst->init(lidar_config[i]);
      source->regPointCloudCallback(dst);
    }

    sources_.emplace_back(source);
  }
  initialized_.store(true, std::memory_order_release);
 
}

void RSLiDARComponent::start() {
  while (!initialized_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  for (auto &iter : sources_) {
    if (iter != nullptr) {
      iter->start();
    }
  }
}
void RSLiDARComponent::stop() {
  for (auto &iter : sources_) {
    if (iter != nullptr) {
      iter->stop();
    }
  }
}

} // namespace robosense::lidar

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(robosense::lidar::RSLiDARComponent)