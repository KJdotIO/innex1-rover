#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace lunabot_localisation_cpp
{

struct ArenaBounds
{
  double min_x{-1.0};
  double max_x{6.9};
  double min_y{-3.3};
  double max_y{1.1};
  double wall_exclusion_margin_m{0.35};

  void validate() const;
  double legal_min_x() const;
  double legal_max_x() const;
  double legal_min_y() const;
  double legal_max_y() const;
};

struct RigidTransform
{
  std::array<double, 3> translation_xyz{0.0, 0.0, 0.0};
  std::array<double, 4> quaternion_xyzw{0.0, 0.0, 0.0, 1.0};
};

struct LegalLidarFilterResult
{
  sensor_msgs::msg::PointCloud2 cloud;
  std::size_t raw_count{0};
  std::size_t finite_count{0};
  std::size_t kept_count{0};
  std::size_t rejected_count{0};

  double reject_ratio() const;
};

LegalLidarFilterResult filter_cloud_to_legal_bounds(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const ArenaBounds & bounds,
  const RigidTransform & mask_from_cloud);

}  // namespace lunabot_localisation_cpp
