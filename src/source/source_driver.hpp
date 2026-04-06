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
#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/utility/sync_queue.hpp>

namespace robosense::lidar {

class SourceDriver : public Source {
public:
  void init(const YAML::Node &config) override;
  void start() override;
  void stop() override;
  void regPacketCallback(DestinationPacket::Ptr dst) override;
  ~SourceDriver() override;

  explicit SourceDriver(SourceType src_type);

protected:
  std::shared_ptr<LidarPointCloudMsg> getPointCloud(void);
  void putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);
  void putPacket(const Packet &msg);
  void putException(const lidar::Error &msg);
  void processPointCloud();

  std::shared_ptr<lidar::LidarDriver<LidarPointCloudMsg>> driver_ptr_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_;
  SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;
#ifdef ENABLE_IMU_DATA_PARSE
  std::shared_ptr<ImuData> getImuData(void);
  void putImuData(const std::shared_ptr<ImuData> &msg);
  void processImuData();
  SyncQueue<std::shared_ptr<ImuData>> free_imu_data_queue_;
  SyncQueue<std::shared_ptr<ImuData>> imu_data_queue_;
  std::thread imu_data_process_thread_;
#endif
  std::thread point_cloud_process_thread_;
  bool to_exit_process_;
};

} // namespace robosense::lidar