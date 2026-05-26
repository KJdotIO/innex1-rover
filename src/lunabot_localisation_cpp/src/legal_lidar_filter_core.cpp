#include "lunabot_localisation_cpp/legal_lidar_filter_core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

namespace lunabot_localisation_cpp
{
namespace
{

struct FieldView
{
  std::uint32_t offset{0};
  std::uint8_t datatype{0};
};

struct PreparedTransform
{
  std::array<double, 3> translation{};
  std::array<double, 9> rotation{};
};

bool host_is_bigendian()
{
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01;
}

std::size_t datatype_size(const std::uint8_t datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return 1;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return 2;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return 4;
    case sensor_msgs::msg::PointField::FLOAT64:
      return 8;
    default:
      throw std::invalid_argument("Unsupported PointCloud2 datatype " + std::to_string(datatype));
  }
}

FieldView find_xyz_field(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const std::string & name)
{
  const auto field_it = std::find_if(
    cloud.fields.begin(),
    cloud.fields.end(),
    [&name](const sensor_msgs::msg::PointField & field) {
      return field.name == name;
    });

  if (field_it == cloud.fields.end()) {
    throw std::invalid_argument("PointCloud2 is missing required field '" + name + "'");
  }
  if (
    field_it->datatype != sensor_msgs::msg::PointField::FLOAT32 &&
    field_it->datatype != sensor_msgs::msg::PointField::FLOAT64)
  {
    throw std::invalid_argument("PointCloud2 field '" + name + "' must be FLOAT32 or FLOAT64");
  }
  if (field_it->count != 1) {
    throw std::invalid_argument("PointCloud2 field '" + name + "' must have count 1");
  }

  const auto field_size = datatype_size(field_it->datatype);
  if (static_cast<std::size_t>(field_it->offset) + field_size > cloud.point_step) {
    throw std::invalid_argument("PointCloud2 field '" + name + "' exceeds point_step");
  }

  return FieldView{field_it->offset, field_it->datatype};
}

template<typename T>
T read_scalar(const std::uint8_t * bytes, const bool data_is_bigendian)
{
  std::array<std::uint8_t, sizeof(T)> value_bytes{};
  std::memcpy(value_bytes.data(), bytes, sizeof(T));
  if (data_is_bigendian != host_is_bigendian()) {
    std::reverse(value_bytes.begin(), value_bytes.end());
  }

  T value{};
  std::memcpy(&value, value_bytes.data(), sizeof(T));
  return value;
}

double read_coordinate(
  const std::uint8_t * point_bytes,
  const FieldView & field,
  const bool data_is_bigendian)
{
  const auto * field_bytes = point_bytes + field.offset;
  if (field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
    return static_cast<double>(read_scalar<float>(field_bytes, data_is_bigendian));
  }
  return read_scalar<double>(field_bytes, data_is_bigendian);
}

PreparedTransform prepare_transform(const RigidTransform & transform)
{
  auto x = transform.quaternion_xyzw[0];
  auto y = transform.quaternion_xyzw[1];
  auto z = transform.quaternion_xyzw[2];
  auto w = transform.quaternion_xyzw[3];
  const auto norm = std::sqrt((x * x) + (y * y) + (z * z) + (w * w));
  if (norm == 0.0) {
    throw std::invalid_argument("quaternion must be non-zero");
  }

  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  return PreparedTransform{
    transform.translation_xyz,
    {
      1.0 - 2.0 * ((y * y) + (z * z)),
      2.0 * ((x * y) - (z * w)),
      2.0 * ((x * z) + (y * w)),
      2.0 * ((x * y) + (z * w)),
      1.0 - 2.0 * ((x * x) + (z * z)),
      2.0 * ((y * z) - (x * w)),
      2.0 * ((x * z) - (y * w)),
      2.0 * ((y * z) + (x * w)),
      1.0 - 2.0 * ((x * x) + (y * y)),
    },
  };
}

std::array<double, 3> transform_point(
  const std::array<double, 3> & point,
  const PreparedTransform & transform)
{
  const auto & r = transform.rotation;

  return {
    (r[0] * point[0]) + (r[1] * point[1]) + (r[2] * point[2]) + transform.translation[0],
    (r[3] * point[0]) + (r[4] * point[1]) + (r[5] * point[2]) + transform.translation[1],
    (r[6] * point[0]) + (r[7] * point[1]) + (r[8] * point[2]) + transform.translation[2],
  };
}

bool inside_legal_bounds(const std::array<double, 3> & point, const ArenaBounds & bounds)
{
  return std::isfinite(point[0]) && std::isfinite(point[1]) && std::isfinite(point[2]) &&
         point[0] >= bounds.legal_min_x() && point[0] <= bounds.legal_max_x() &&
         point[1] >= bounds.legal_min_y() && point[1] <= bounds.legal_max_y();
}

}  // namespace

void ArenaBounds::validate() const
{
  if (min_x >= max_x) {
    throw std::invalid_argument("arena_min_x must be less than arena_max_x");
  }
  if (min_y >= max_y) {
    throw std::invalid_argument("arena_min_y must be less than arena_max_y");
  }
  if (wall_exclusion_margin_m < 0.0) {
    throw std::invalid_argument("wall_exclusion_margin_m must be >= 0");
  }
  if (2.0 * wall_exclusion_margin_m >= (max_x - min_x)) {
    throw std::invalid_argument("wall_exclusion_margin_m removes all legal arena width");
  }
  if (2.0 * wall_exclusion_margin_m >= (max_y - min_y)) {
    throw std::invalid_argument("wall_exclusion_margin_m removes all legal arena height");
  }
}

double ArenaBounds::legal_min_x() const {return min_x + wall_exclusion_margin_m;}
double ArenaBounds::legal_max_x() const {return max_x - wall_exclusion_margin_m;}
double ArenaBounds::legal_min_y() const {return min_y + wall_exclusion_margin_m;}
double ArenaBounds::legal_max_y() const {return max_y - wall_exclusion_margin_m;}

double LegalLidarFilterResult::reject_ratio() const
{
  if (finite_count == 0) {
    return 0.0;
  }
  return static_cast<double>(rejected_count) / static_cast<double>(finite_count);
}

LegalLidarFilterResult filter_cloud_to_legal_bounds(
  const sensor_msgs::msg::PointCloud2 & cloud,
  const ArenaBounds & bounds,
  const RigidTransform & mask_from_cloud)
{
  bounds.validate();
  if (cloud.point_step == 0) {
    throw std::invalid_argument("PointCloud2 point_step must be > 0");
  }

  const auto row_point_bytes = static_cast<std::size_t>(cloud.width) * cloud.point_step;
  if (cloud.row_step < row_point_bytes) {
    throw std::invalid_argument("PointCloud2 row_step is smaller than width * point_step");
  }

  const auto required_bytes = static_cast<std::size_t>(cloud.row_step) * cloud.height;
  if (cloud.data.size() < required_bytes) {
    throw std::invalid_argument("PointCloud2 data is smaller than row_step * height");
  }

  const auto x_field = find_xyz_field(cloud, "x");
  const auto y_field = find_xyz_field(cloud, "y");
  const auto z_field = find_xyz_field(cloud, "z");
  const auto transform = prepare_transform(mask_from_cloud);

  LegalLidarFilterResult result;
  result.raw_count = static_cast<std::size_t>(cloud.width) * cloud.height;
  result.cloud.header = cloud.header;
  result.cloud.fields = cloud.fields;
  result.cloud.is_bigendian = cloud.is_bigendian;
  result.cloud.point_step = cloud.point_step;
  result.cloud.is_dense = true;
  result.cloud.height = 1;
  result.cloud.width = 0;
  result.cloud.row_step = 0;
  result.cloud.data.resize(result.raw_count * cloud.point_step);

  std::size_t output_offset = 0;
  const auto * input_data = cloud.data.data();
  for (std::uint32_t row = 0; row < cloud.height; ++row) {
    const auto row_offset = static_cast<std::size_t>(row) * cloud.row_step;
    for (std::uint32_t col = 0; col < cloud.width; ++col) {
      const auto point_offset = row_offset + (static_cast<std::size_t>(col) * cloud.point_step);
      const auto * point_bytes = input_data + point_offset;
      const std::array<double, 3> point = {
        read_coordinate(point_bytes, x_field, cloud.is_bigendian),
        read_coordinate(point_bytes, y_field, cloud.is_bigendian),
        read_coordinate(point_bytes, z_field, cloud.is_bigendian),
      };

      if (
        std::isfinite(point[0]) && std::isfinite(point[1]) &&
        std::isfinite(point[2]))
      {
        ++result.finite_count;
      }

      const auto mask_point = transform_point(point, transform);
      if (!inside_legal_bounds(mask_point, bounds)) {
        continue;
      }

      std::memcpy(result.cloud.data.data() + output_offset, point_bytes, cloud.point_step);
      output_offset += cloud.point_step;
      ++result.kept_count;
    }
  }

  result.rejected_count = result.finite_count - result.kept_count;
  result.cloud.width = static_cast<std::uint32_t>(result.kept_count);
  result.cloud.row_step = result.cloud.point_step * result.cloud.width;
  result.cloud.data.resize(output_offset);
  return result;
}

}  // namespace lunabot_localisation_cpp
