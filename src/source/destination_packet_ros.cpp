//
// Created by haoming on 4/6/26.
//

#include "destination_packet_ros.hpp"

namespace robosense::lidar {
rslidar_msg::msg::RslidarPacket toRosMsg(const Packet &rs_msg,
                                         const std::string &frame_id) {
  rslidar_msg::msg::RslidarPacket ros_msg;
  ros_msg.header.stamp.sec = (uint32_t)floor(rs_msg.timestamp);
  ros_msg.header.stamp.nanosec =
      (uint32_t)round((rs_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  // ros_msg.header.seq = rs_msg.seq;
  ros_msg.header.frame_id = frame_id;
  ros_msg.is_difop = rs_msg.is_difop;
  ros_msg.is_frame_begin = rs_msg.is_frame_begin;

  for (const auto &i : rs_msg.buf_) {
    ros_msg.data.emplace_back(i);
  }

  return ros_msg;
}

void DestinationPacketRos::init(const YAML::Node &config) {
  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "rslidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_packet_topic", ros_send_topic,
                        "rslidar_packets");

  size_t ros_queue_length;
  yamlRead<size_t>(config["ros"], "ros_queue_length", ros_queue_length, 100);

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "rslidar_packets_destination_" << node_index++;

  pkt_pub_ = node_ptr_->create_publisher<rslidar_msg::msg::RslidarPacket>(
      ros_send_topic, ros_queue_length);
}

void DestinationPacketRos::sendPacket(const Packet &msg) {
  pkt_pub_->publish(toRosMsg(msg, frame_id_));
}

} // namespace robosense::lidar
