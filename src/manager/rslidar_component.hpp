#ifndef RSLIDAR_SDK_RSLIDAR_COMPONENT_HPP
#define RSLIDAR_SDK_RSLIDAR_COMPONENT_HPP

#include "source/source.hpp"
#include "utility/yaml_reader.hpp"
#include <rclcpp/node.hpp>

namespace robosense::lidar {

class RSLiDARComponent : public rclcpp::Node {
public:
  explicit RSLiDARComponent(const rclcpp::NodeOptions &options);
  ~RSLiDARComponent() override;

  void init();
  void start() const;
  void stop() const;
private:
  std::vector<Source::Ptr> sources_;
  rclcpp::TimerBase::SharedPtr initTimer_;
};
} // namespace robosense::lidar

#endif // RSLIDAR_SDK_RSLIDAR_COMPONENT_HPP
