
#include "source/source_packet_ros.hpp"

namespace robosense::lidar {

Packet toRsMsg(const rslidar_msg::msg::RslidarPacket &ros_msg) {
  Packet rs_msg;
  rs_msg.timestamp =
      ros_msg.header.stamp.sec + double(ros_msg.header.stamp.nanosec) / 1e9;
  // rs_msg.seq = ros_msg.header.seq;
  rs_msg.is_difop = ros_msg.is_difop;
  rs_msg.is_frame_begin = ros_msg.is_frame_begin;

  for (const auto &i : ros_msg.data) {
    rs_msg.buf_.emplace_back(i);
  }

  return rs_msg;
}

void SourcePacketRos::init(const YAML::Node &config) {
  SourceDriver::init(config);

  std::string ros_recv_topic;
  yamlRead<std::string>(config["ros"], "ros_recv_packet_topic", ros_recv_topic,
                        "rslidar_packets");

  bool assign_callback_group;
  yamlRead<bool>(config["assign_callback_group"], "assign_callback_group",
                 assign_callback_group, false);

  rclcpp::SubscriptionOptions sub_options;
  if (assign_callback_group) {
    callback_group_ = node_ptr_->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options.callback_group = callback_group_;
  }

  pkt_sub_ = node_ptr_->create_subscription<rslidar_msg::msg::RslidarPacket>(
      ros_recv_topic, 100,
      std::bind(&SourcePacketRos::putPacket, this, std::placeholders::_1),
      sub_options);
}

void SourcePacketRos::putPacket(
    const rslidar_msg::msg::RslidarPacket::SharedPtr msg) const {
  driver_ptr_->decodePacket(toRsMsg(*msg));
}

} // namespace robosense::lidar
