#ifndef RSLIDAR_SDK_DESTINATION_PACKET_ROS_HPP
#define RSLIDAR_SDK_DESTINATION_PACKET_ROS_HPP

#include "source/source_driver.hpp"
#include "rslidar_msg/msg/rslidar_packet.hpp"
#include <rclcpp/rclcpp.hpp>

namespace robosense::lidar {

class DestinationPacketRos : public DestinationPacket {
public:
  explicit DestinationPacketRos(rclcpp::Node *node_ptr)
      : DestinationPacket(), node_ptr_(node_ptr) {};

  void init(const YAML::Node &config) override;
  void sendPacket(const Packet &msg) override;
  ~DestinationPacketRos() override = default;

private:
  rclcpp::Publisher<rslidar_msg::msg::RslidarPacket>::SharedPtr pkt_pub_;
  std::string frame_id_;
  rclcpp::Node *node_ptr_;
};
}


#endif // RSLIDAR_SDK_DESTINATION_PACKET_ROS_HPP
