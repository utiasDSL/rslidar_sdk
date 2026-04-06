#include "destination_pointcloud_ros.hpp"
#include "msg/rs_msg/lidar_point_cloud_msg.hpp"
namespace robosense::lidar {
sensor_msgs::msg::PointCloud2 toRosMsg(const LidarPointCloudMsg &rs_msg,
                                       const std::string &frame_id,
                                       bool send_by_rows) {
  sensor_msgs::msg::PointCloud2 ros_msg;

  int fields = 4;
#ifdef POINT_TYPE_XYZIF
  fields = 5;
#elif defined(POINT_TYPE_XYZIRT)
  fields = 6;
#elif defined(POINT_TYPE_XYZIRTF)
  fields = 7;
#endif
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

  if (send_by_rows) {
    ros_msg.width = rs_msg.width;
    ros_msg.height = rs_msg.height;
  } else {
    ros_msg.width = rs_msg.height; // exchange width and height to be compatible
                                   // with pcl::PointCloud<>
    ros_msg.height = rs_msg.width;
  }

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                         offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                         offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                         offset);
  offset = addPointField(ros_msg, "intensity", 1,
                         sensor_msgs::msg::PointField::FLOAT32, offset);

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
  offset = addPointField(ros_msg, "ring", 1,
                         sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1,
                         sensor_msgs::msg::PointField::FLOAT64, offset);
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF)
  offset = addPointField(ros_msg, "feature", 1,
                         sensor_msgs::msg::PointField::UINT8, offset);
#endif

#if 0
  std::cout << "off:" << offset << std::endl;
#endif

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = rs_msg.is_dense;
  ros_msg.data.resize(ros_msg.point_step * ros_msg.width * ros_msg.height);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg,
                                                           "timestamp");
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF)
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_feature_(ros_msg, "feature");
#endif

  if (send_by_rows) {
    for (size_t i = 0; i < rs_msg.height; i++) {
      for (size_t j = 0; j < rs_msg.width; j++) {
        const LidarPointCloudMsg::PointT &point =
            rs_msg.points[i + j * rs_msg.height];

        *iter_x_ = point.x;
        *iter_y_ = point.y;
        *iter_z_ = point.z;
        *iter_intensity_ = point.intensity;

        ++iter_x_;
        ++iter_y_;
        ++iter_z_;
        ++iter_intensity_;

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
        *iter_ring_ = point.ring;
        *iter_timestamp_ = point.timestamp;

        ++iter_ring_;
        ++iter_timestamp_;
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF)
        *iter_feature_ = point.feature;
        ++iter_feature_;
#endif
      }
    }
  } else {
    for (size_t i = 0; i < rs_msg.points.size(); i++) {
      const LidarPointCloudMsg::PointT &point = rs_msg.points[i];

      *iter_x_ = point.x;
      *iter_y_ = point.y;
      *iter_z_ = point.z;
      *iter_intensity_ = point.intensity;

      ++iter_x_;
      ++iter_y_;
      ++iter_z_;
      ++iter_intensity_;

#if defined(POINT_TYPE_XYZIRT) || defined(POINT_TYPE_XYZIRTF)
      *iter_ring_ = point.ring;
      *iter_timestamp_ = point.timestamp;

      ++iter_ring_;
      ++iter_timestamp_;
#endif

#if defined(POINT_TYPE_XYZIF) || defined(POINT_TYPE_XYZIRTF)
      *iter_feature_ = point.feature;
      ++iter_feature_;
#endif
    }
  }

  ros_msg.header.stamp.sec = (uint32_t)floor(rs_msg.timestamp);
  ros_msg.header.stamp.nanosec =
      (uint32_t)round((rs_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
  ros_msg.header.frame_id = frame_id;

  return ros_msg;
}
#ifdef ENABLE_IMU_DATA_PARSE
sensor_msgs::msg::Imu toRosMsg(const std::shared_ptr<ImuData> &data,
                               const std::string &frame_id) {
  sensor_msgs::msg::Imu imu_msg;

  imu_msg.header.stamp =
      rclcpp::Time(static_cast<uint64_t>(data->timestamp * 1e9));
  imu_msg.header.frame_id = frame_id;
  // Set IMU data
  imu_msg.angular_velocity.x = data->angular_velocity_x;
  imu_msg.angular_velocity.y = data->angular_velocity_y;
  imu_msg.angular_velocity.z = data->angular_velocity_z;

  imu_msg.linear_acceleration.x = data->linear_acceleration_x;
  imu_msg.linear_acceleration.y = data->linear_acceleration_y;
  imu_msg.linear_acceleration.z = data->linear_acceleration_z;
  return imu_msg;
}
#endif

void DestinationPointCloudRos::init(const YAML::Node &config) {
  yamlRead<bool>(config["ros"], "ros_send_by_rows", send_by_rows_, false);

  bool dense_points;
  yamlRead<bool>(config["driver"], "dense_points", dense_points, false);
  if (dense_points)
    send_by_rows_ = false;

  yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "rslidar");

  std::string ros_send_topic;
  yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic",
                        ros_send_topic, "rslidar_points");

  size_t ros_queue_length;
  yamlRead<size_t>(config["ros"], "ros_queue_length", ros_queue_length, 100);

  static int node_index = 0;
  std::stringstream node_name;
  node_name << "rslidar_points_destination_" << node_index++;

  pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(
      ros_send_topic, ros_queue_length);

#ifdef ENABLE_IMU_DATA_PARSE
  std::string ros_send_imu_data_topic;
  yamlRead<std::string>(config["ros"], "ros_send_imu_data_topic",
                        ros_send_imu_data_topic, "rslidar_imu_data");
  imu_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(
      ros_send_imu_data_topic, 1000);
#endif
}

void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg &msg) {
  pub_->publish(toRosMsg(msg, frame_id_, send_by_rows_));
}
#ifdef ENABLE_IMU_DATA_PARSE
void DestinationPointCloudRos::sendImuData(
    const std::shared_ptr<ImuData> &data) {
  imu_pub_->publish(toRosMsg(data, frame_id_));
}
#endif

} // namespace robosense::lidar
