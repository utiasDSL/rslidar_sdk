
#include "source_driver.hpp"

#include <memory>

namespace robosense::lidar {

SourceDriver::SourceDriver(SourceType src_type)
    : Source(src_type), to_exit_process_(false) {}

void SourceDriver::init(const YAML::Node &config) {
  YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
  lidar::RSDriverParam driver_param;

  // input related
  yamlRead<uint16_t>(driver_config, "msop_port",
                     driver_param.input_param.msop_port, 6699);
  yamlRead<uint16_t>(driver_config, "difop_port",
                     driver_param.input_param.difop_port, 7788);
#ifdef ENABLE_IMU_DATA_PARSE
  yamlRead<uint16_t>(driver_config, "imu_port",
                     driver_param.input_param.imu_port, 6688);
#endif
  yamlRead<std::string>(driver_config, "host_address",
                        driver_param.input_param.host_address, "0.0.0.0");
  yamlRead<std::string>(driver_config, "group_address",
                        driver_param.input_param.group_address, "0.0.0.0");
  yamlRead<bool>(driver_config, "use_vlan", driver_param.input_param.use_vlan,
                 false);
  yamlRead<std::string>(driver_config, "pcap_path",
                        driver_param.input_param.pcap_path, "");
  yamlRead<float>(driver_config, "pcap_rate",
                  driver_param.input_param.pcap_rate, 1);
  yamlRead<bool>(driver_config, "pcap_repeat",
                 driver_param.input_param.pcap_repeat, true);
  yamlRead<uint16_t>(driver_config, "user_layer_bytes",
                     driver_param.input_param.user_layer_bytes, 0);
  yamlRead<uint16_t>(driver_config, "tail_layer_bytes",
                     driver_param.input_param.tail_layer_bytes, 0);
  yamlRead<uint32_t>(driver_config, "socket_recv_buf",
                     driver_param.input_param.socket_recv_buf, 106496);
  // decoder related
  std::string lidar_type;
  yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
  driver_param.lidar_type = strToLidarType(lidar_type);

  // decoder
  yamlRead<bool>(driver_config, "wait_for_difop",
                 driver_param.decoder_param.wait_for_difop, true);
  yamlRead<bool>(driver_config, "use_lidar_clock",
                 driver_param.decoder_param.use_lidar_clock, false);
  yamlRead<float>(driver_config, "min_distance",
                  driver_param.decoder_param.min_distance, 0.2);
  yamlRead<float>(driver_config, "max_distance",
                  driver_param.decoder_param.max_distance, 200);
  yamlRead<float>(driver_config, "start_angle",
                  driver_param.decoder_param.start_angle, 0);
  yamlRead<float>(driver_config, "end_angle",
                  driver_param.decoder_param.end_angle, 360);
  yamlRead<bool>(driver_config, "dense_points",
                 driver_param.decoder_param.dense_points, false);
  yamlRead<bool>(driver_config, "ts_first_point",
                 driver_param.decoder_param.ts_first_point, false);

  // mechanical decoder
  yamlRead<bool>(driver_config, "config_from_file",
                 driver_param.decoder_param.config_from_file, false);
  yamlRead<std::string>(driver_config, "angle_path",
                        driver_param.decoder_param.angle_path, "");

  uint16_t split_frame_mode;
  yamlRead<uint16_t>(driver_config, "split_frame_mode", split_frame_mode, 1);
  driver_param.decoder_param.split_frame_mode =
      SplitFrameMode(split_frame_mode);

  yamlRead<float>(driver_config, "split_angle",
                  driver_param.decoder_param.split_angle, 0);
  yamlRead<uint16_t>(driver_config, "num_blks_split",
                     driver_param.decoder_param.num_blks_split, 0);

  // transform
  yamlRead<float>(driver_config, "x",
                  driver_param.decoder_param.transform_param.x, 0);
  yamlRead<float>(driver_config, "y",
                  driver_param.decoder_param.transform_param.y, 0);
  yamlRead<float>(driver_config, "z",
                  driver_param.decoder_param.transform_param.z, 0);
  yamlRead<float>(driver_config, "roll",
                  driver_param.decoder_param.transform_param.roll, 0);
  yamlRead<float>(driver_config, "pitch",
                  driver_param.decoder_param.transform_param.pitch, 0);
  yamlRead<float>(driver_config, "yaw",
                  driver_param.decoder_param.transform_param.yaw, 0);

  switch (src_type_) {
  case SourceType::MSG_FROM_LIDAR:
    driver_param.input_type = InputType::ONLINE_LIDAR;
    break;
  case SourceType::MSG_FROM_PCAP:
    driver_param.input_type = InputType::PCAP_FILE;
    break;
  default:
    driver_param.input_type = InputType::RAW_PACKET;
    break;
  }

  driver_param.print();

  driver_ptr_ = std::make_shared<lidar::LidarDriver<LidarPointCloudMsg>>();
  driver_ptr_->regPointCloudCallback(
      std::bind(&SourceDriver::getPointCloud, this),
      std::bind(&SourceDriver::putPointCloud, this, std::placeholders::_1));
  driver_ptr_->regExceptionCallback(
      std::bind(&SourceDriver::putException, this, std::placeholders::_1));
  point_cloud_process_thread_ =
      std::thread(std::bind(&SourceDriver::processPointCloud, this));

#ifdef ENABLE_IMU_DATA_PARSE
  driver_ptr_->regImuDataCallback(
      std::bind(&SourceDriver::getImuData, this),
      std::bind(&SourceDriver::putImuData, this, std::placeholders::_1));
  imu_data_process_thread_ =
      std::thread(std::bind(&SourceDriver::processImuData, this));
#endif

  if (!driver_ptr_->init(driver_param)) {
    RS_ERROR << "Driver Initialize Error...." << RS_REND;
    exit(-1);
  }
}

void SourceDriver::start() { driver_ptr_->start(); }

SourceDriver::~SourceDriver() { stop(); }

void SourceDriver::stop() {
  driver_ptr_->stop();

  to_exit_process_ = true;
  point_cloud_process_thread_.join();
}

std::shared_ptr<LidarPointCloudMsg> SourceDriver::getPointCloud(void) {
  std::shared_ptr<LidarPointCloudMsg> point_cloud =
      free_point_cloud_queue_.pop();
  if (point_cloud.get() != NULL) {
    return point_cloud;
  }

  return std::make_shared<LidarPointCloudMsg>();
}

void SourceDriver::regPacketCallback(DestinationPacket::Ptr dst) {
  Source::regPacketCallback(dst);
  if (pkt_cb_vec_.size() == 1) {
    driver_ptr_->regPacketCallback(
        std::bind(&SourceDriver::putPacket, this, std::placeholders::_1));
  }
}

void SourceDriver::putPacket(const Packet &msg) { sendPacket(msg); }

void SourceDriver::putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg) {
  point_cloud_queue_.push(msg);
}
#ifdef ENABLE_IMU_DATA_PARSE
std::shared_ptr<ImuData> SourceDriver::getImuData(void) {
  std::shared_ptr<ImuData> imuDataPtr = free_imu_data_queue_.pop();
  if (imuDataPtr.get() != NULL) {
    return imuDataPtr;
  }
  return std::make_shared<ImuData>();
}
void SourceDriver::putImuData(const std::shared_ptr<ImuData> &msg) {
  imu_data_queue_.push(msg);
}

void SourceDriver::processImuData() {
  while (!to_exit_process_) {
    std::shared_ptr<ImuData> msg = imu_data_queue_.popWait(100);
    if (msg.get() == NULL) {
      continue;
    }
    sendImuData(msg);

    free_imu_data_queue_.push(msg);
  }
}
#endif
void SourceDriver::processPointCloud() {
  while (!to_exit_process_) {
    std::shared_ptr<LidarPointCloudMsg> msg = point_cloud_queue_.popWait(1000);
    if (msg.get() == NULL) {
      continue;
    }
    sendPointCloud(msg);

    free_point_cloud_queue_.push(msg);
  }
}

void SourceDriver::putException(const lidar::Error &msg) {
  switch (msg.error_code_type) {
  case lidar::ErrCodeType::INFO_CODE:
    RS_INFO << msg.toString() << RS_REND;
    break;
  case lidar::ErrCodeType::WARNING_CODE:
    RS_WARNING << msg.toString() << RS_REND;
    break;
  case lidar::ErrCodeType::ERROR_CODE:
    RS_ERROR << msg.toString() << RS_REND;
    break;
  }
}

}