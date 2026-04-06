//
// Created by haoming on 4/5/26.
//

#include "source.hpp"

namespace robosense::lidar {
Source::Source(SourceType src_type) : src_type_(src_type) {}

void Source::regPacketCallback(DestinationPacket::Ptr dst) {
  pkt_cb_vec_.emplace_back(dst);
}

void Source::regPointCloudCallback(DestinationPointCloud::Ptr dst) {
  pc_cb_vec_.emplace_back(dst);
}

void Source::sendPacket(const Packet &msg) {
  for (const auto& iter : pkt_cb_vec_) {
    iter->sendPacket(msg);
  }
}

void Source::sendPointCloud(std::shared_ptr<LidarPointCloudMsg> msg) {
  for (const auto &iter : pc_cb_vec_) {
    iter->sendPointCloud(*msg);
  }
}

#ifdef ENABLE_IMU_DATA_PARSE
void Source::sendImuData(const std::shared_ptr<ImuData> &msg) {
  for (const auto& iter : pc_cb_vec_) {
    iter->sendImuData(msg);
  }
}
#endif

} // namespace robosense::lidar