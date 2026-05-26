#include "lunabot_localisation_cpp/legal_lidar_filter_node.hpp"

#include <chrono>
#include <exception>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2/exceptions.h"

namespace lunabot_localisation_cpp
{
namespace
{

using Clock = std::chrono::steady_clock;

RigidTransform transform_from_msg(const geometry_msgs::msg::TransformStamped & transform)
{
  const auto & translation = transform.transform.translation;
  const auto & rotation = transform.transform.rotation;
  return RigidTransform{
    {translation.x, translation.y, translation.z},
    {rotation.x, rotation.y, rotation.z, rotation.w},
  };
}

}  // namespace

LegalLidarFilterNode::LegalLidarFilterNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("legal_lidar_filter", options),
  diagnostics_(this)
{
  declare_parameter<std::string>("input_topic", "/ouster/points");
  declare_parameter<std::string>("output_topic", "/localisation/lidar/points_legal");
  declare_parameter<std::string>("source_name", "ouster");
  declare_parameter<std::string>("mask_frame", "odom");
  declare_parameter<double>("arena_min_x", -1.0);
  declare_parameter<double>("arena_max_x", 6.9);
  declare_parameter<double>("arena_min_y", -3.3);
  declare_parameter<double>("arena_max_y", 1.1);
  declare_parameter<double>("wall_exclusion_margin_m", 0.35);
  declare_parameter<double>("diagnostic_publish_hz", 1.0);
  declare_parameter<double>("stale_timeout_s", 2.5);
  declare_parameter<double>("tf_lookup_timeout_s", 0.05);
  declare_parameter<bool>("allow_latest_tf_on_future_extrapolation", false);

  input_topic_ = get_parameter("input_topic").as_string();
  output_topic_ = get_parameter("output_topic").as_string();
  source_name_ = get_parameter("source_name").as_string();
  mask_frame_ = get_parameter("mask_frame").as_string();
  stale_timeout_s_ = get_parameter("stale_timeout_s").as_double();
  tf_lookup_timeout_s_ = get_parameter("tf_lookup_timeout_s").as_double();
  allow_latest_tf_on_future_extrapolation_ =
    get_parameter("allow_latest_tf_on_future_extrapolation").as_bool();
  const auto diagnostic_publish_hz = get_parameter("diagnostic_publish_hz").as_double();
  bounds_ = load_bounds();

  if (diagnostic_publish_hz <= 0.0) {
    throw std::invalid_argument("diagnostic_publish_hz must be > 0");
  }
  if (stale_timeout_s_ <= 0.0) {
    throw std::invalid_argument("stale_timeout_s must be > 0");
  }
  if (tf_lookup_timeout_s_ < 0.0) {
    throw std::invalid_argument("tf_lookup_timeout_s must be >= 0");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    output_topic_,
    rclcpp::SensorDataQoS());
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_,
    rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
      cloud_callback(msg);
    });

  diagnostics_.setHardwareID(source_name_);
  diagnostics_.add(
    "legal_lidar_filter",
    this,
    &LegalLidarFilterNode::publish_diagnostics);
  diagnostic_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / diagnostic_publish_hz),
    [this]() {
      diagnostics_.force_update();
    });

  RCLCPP_INFO(
    get_logger(),
    "Legal LiDAR C++ filter %s: %s -> %s; mask in %s; output frame stays sensor-native",
    source_name_.c_str(),
    input_topic_.c_str(),
    output_topic_.c_str(),
    mask_frame_.c_str());
}

ArenaBounds LegalLidarFilterNode::load_bounds() const
{
  ArenaBounds bounds{
    get_parameter("arena_min_x").as_double(),
    get_parameter("arena_max_x").as_double(),
    get_parameter("arena_min_y").as_double(),
    get_parameter("arena_max_y").as_double(),
    get_parameter("wall_exclusion_margin_m").as_double(),
  };
  bounds.validate();
  return bounds;
}

void LegalLidarFilterNode::cloud_callback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  ++input_cloud_count_;

  RigidTransform mask_from_cloud;
  if (!lookup_cloud_transform(*msg, mask_from_cloud)) {
    return;
  }

  const auto started_at = Clock::now();
  try {
    auto result = filter_cloud_to_legal_bounds(*msg, bounds_, mask_from_cloud);
    last_processing_ms_ =
      std::chrono::duration<double, std::milli>(Clock::now() - started_at).count();
    cloud_pub_->publish(result.cloud);
    last_result_ = std::move(result);
    has_result_ = true;
    has_cloud_time_ = true;
    last_cloud_time_ = now();
    last_error_.clear();
    ++output_cloud_count_;
  } catch (const std::exception & error) {
    last_error_ = error.what();
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Failed to filter legal LiDAR cloud: %s",
      error.what());
  }
}

bool LegalLidarFilterNode::lookup_cloud_transform(
  const sensor_msgs::msg::PointCloud2 & msg,
  RigidTransform & transform)
{
  try {
    transform = transform_from_msg(
      tf_buffer_->lookupTransform(
        mask_frame_,
        msg.header.frame_id,
        msg.header.stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_s_)));
    return true;
  } catch (const tf2::ExtrapolationException & error) {
    if (!allow_latest_tf_on_future_extrapolation_) {
      last_error_ = error.what();
      ++tf_failure_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "TF lookup %s -> %s failed; dropping cloud: %s",
        msg.header.frame_id.c_str(),
        mask_frame_.c_str(),
        error.what());
      return false;
    }
    try {
      transform = transform_from_msg(
        tf_buffer_->lookupTransform(
          mask_frame_,
          msg.header.frame_id,
          rclcpp::Time(0, 0, get_clock()->get_clock_type()),
          rclcpp::Duration::from_seconds(tf_lookup_timeout_s_)));
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "TF lookup %s -> %s needed extrapolation; using latest available transform: %s",
        msg.header.frame_id.c_str(),
        mask_frame_.c_str(),
        error.what());
      return true;
    } catch (const tf2::TransformException & latest_error) {
      last_error_ = latest_error.what();
      ++tf_failure_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Latest TF lookup %s -> %s failed; dropping cloud: %s",
        msg.header.frame_id.c_str(),
        mask_frame_.c_str(),
        latest_error.what());
      return false;
    }
  } catch (const tf2::TransformException & error) {
    last_error_ = error.what();
    ++tf_failure_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "TF lookup %s -> %s failed; dropping cloud: %s",
      msg.header.frame_id.c_str(),
      mask_frame_.c_str(),
      error.what());
    return false;
  }
}

void LegalLidarFilterNode::publish_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  double age_s = -1.0;
  if (!has_result_ || !has_cloud_time_) {
    status.summary(
      diagnostic_msgs::msg::DiagnosticStatus::STALE,
      "No legal LiDAR cloud published yet");
  } else {
    age_s = (now() - last_cloud_time_).seconds();
    if (age_s > stale_timeout_s_) {
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::STALE,
        "No fresh legal LiDAR cloud");
    } else if (!last_error_.empty()) {
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Legal LiDAR filtering warning");
    } else {
      status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "Filtering LiDAR wall/out-of-field points");
    }
  }

  status.add("input_topic", input_topic_);
  status.add("output_topic", output_topic_);
  status.add("mask_frame", mask_frame_);
  status.add("source_name", source_name_);
  status.add("raw_count", static_cast<int>(last_result_.raw_count));
  status.add("finite_count", static_cast<int>(last_result_.finite_count));
  status.add("kept_count", static_cast<int>(last_result_.kept_count));
  status.add("rejected_count", static_cast<int>(last_result_.rejected_count));
  status.add("reject_ratio", last_result_.reject_ratio());
  status.add("last_cloud_age_s", age_s);
  status.add("last_processing_ms", last_processing_ms_);
  status.add("input_cloud_count", static_cast<int>(input_cloud_count_));
  status.add("output_cloud_count", static_cast<int>(output_cloud_count_));
  status.add("tf_failure_count", static_cast<int>(tf_failure_count_));
  status.add("legal_min_x", bounds_.legal_min_x());
  status.add("legal_max_x", bounds_.legal_max_x());
  status.add("legal_min_y", bounds_.legal_min_y());
  status.add("legal_max_y", bounds_.legal_max_y());
  status.add("wall_exclusion_margin_m", bounds_.wall_exclusion_margin_m);
  status.add("last_error", last_error_);
}

}  // namespace lunabot_localisation_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(lunabot_localisation_cpp::LegalLidarFilterNode)
