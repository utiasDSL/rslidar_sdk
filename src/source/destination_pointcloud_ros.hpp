/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install, copy or
use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the
names of other contributors may be used to endorse or promote products derived
from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include "source/source.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#ifdef ENABLE_IMU_DATA_PARSE
#include <sensor_msgs/msg/imu.hpp>
#endif
#include <sstream>

namespace robosense::lidar {

class DestinationPointCloudRos : virtual public DestinationPointCloud {
public:
  explicit DestinationPointCloudRos(rclcpp::Node *node_ptr)
      : DestinationPointCloud(), send_by_rows_(false), node_ptr_(node_ptr) {}
  void init(const YAML::Node &config) override;
  void sendPointCloud(const LidarPointCloudMsg &msg) override;
#ifdef ENABLE_IMU_DATA_PARSE
  void sendImuData(const std::shared_ptr<ImuData> &data) override;
#endif
  ~DestinationPointCloudRos() override = default;

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
#ifdef ENABLE_IMU_DATA_PARSE
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
#endif
  std::string frame_id_;
  bool send_by_rows_;
  rclcpp::Node *node_ptr_;
};

} // namespace robosense::lidar
