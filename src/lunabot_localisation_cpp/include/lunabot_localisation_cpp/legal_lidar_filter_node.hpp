#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "lunabot_localisation_cpp/legal_lidar_filter_core.hpp"

namespace lunabot_localisation_cpp
{

class LegalLidarFilterNode : public rclcpp::Node
{
public:
  explicit LegalLidarFilterNode(const rclcpp::NodeOptions & options);

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  bool lookup_cloud_transform(
    const sensor_msgs::msg::PointCloud2 & msg,
    RigidTransform & transform);
  void publish_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);
  ArenaBounds load_bounds() const;

  std::string input_topic_;
  std::string output_topic_;
  std::string source_name_;
  std::string mask_frame_;
  double stale_timeout_s_{2.5};
  double tf_lookup_timeout_s_{0.05};
  bool allow_latest_tf_on_future_extrapolation_{false};
  ArenaBounds bounds_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  diagnostic_updater::Updater diagnostics_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;

  bool has_result_{false};
  bool has_cloud_time_{false};
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};
  std::string last_error_;
  LegalLidarFilterResult last_result_;
  std::size_t input_cloud_count_{0};
  std::size_t output_cloud_count_{0};
  std::size_t tf_failure_count_{0};
  double last_processing_ms_{0.0};
};

}  // namespace lunabot_localisation_cpp
