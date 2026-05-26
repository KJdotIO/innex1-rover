#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "lunabot_localisation_cpp/legal_lidar_filter_core.hpp"

namespace
{

using lunabot_localisation_cpp::ArenaBounds;
using lunabot_localisation_cpp::RigidTransform;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

PointField field(
  const std::string & name,
  const std::uint32_t offset,
  const std::uint8_t datatype,
  const std::uint32_t count = 1)
{
  PointField point_field;
  point_field.name = name;
  point_field.offset = offset;
  point_field.datatype = datatype;
  point_field.count = count;
  return point_field;
}

bool host_is_bigendian()
{
  const std::uint16_t value = 0x0102;
  return *reinterpret_cast<const std::uint8_t *>(&value) == 0x01;
}

template<typename T>
void write_scalar(std::vector<std::uint8_t> & data, std::size_t offset, T value, bool bigendian)
{
  std::array<std::uint8_t, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  if (bigendian != host_is_bigendian()) {
    std::reverse(bytes.begin(), bytes.end());
  }
  std::memcpy(data.data() + offset, bytes.data(), sizeof(T));
}

PointCloud2 make_cloud(
  const std::uint32_t width,
  const std::uint32_t height,
  const std::uint32_t point_step,
  const std::uint32_t row_padding,
  const bool bigendian = false)
{
  PointCloud2 cloud;
  cloud.header.frame_id = "os_lidar";
  cloud.height = height;
  cloud.width = width;
  cloud.fields = {
    field("y", 0, PointField::FLOAT32),
    field("x", 4, PointField::FLOAT32),
    field("z", 8, PointField::FLOAT32),
    field("intensity", 12, PointField::FLOAT32),
    field("t", 16, PointField::UINT32),
    field("ring", 20, PointField::UINT16),
  };
  cloud.is_bigendian = bigendian;
  cloud.point_step = point_step;
  cloud.row_step = (width * point_step) + row_padding;
  cloud.is_dense = false;
  cloud.data.assign(static_cast<std::size_t>(cloud.row_step) * height, 0xA5);
  return cloud;
}

void write_point(
  PointCloud2 & cloud,
  const std::uint32_t row,
  const std::uint32_t col,
  const float x,
  const float y,
  const float z,
  const float intensity,
  const std::uint32_t t,
  const std::uint16_t ring)
{
  const auto offset = static_cast<std::size_t>(row) * cloud.row_step +
    static_cast<std::size_t>(col) * cloud.point_step;
  write_scalar(cloud.data, offset + 0, y, cloud.is_bigendian);
  write_scalar(cloud.data, offset + 4, x, cloud.is_bigendian);
  write_scalar(cloud.data, offset + 8, z, cloud.is_bigendian);
  write_scalar(cloud.data, offset + 12, intensity, cloud.is_bigendian);
  write_scalar(cloud.data, offset + 16, t, cloud.is_bigendian);
  write_scalar(cloud.data, offset + 20, ring, cloud.is_bigendian);
}

std::vector<std::uint8_t> point_record(
  const PointCloud2 & cloud,
  const std::uint32_t row,
  const std::uint32_t col)
{
  const auto offset = static_cast<std::size_t>(row) * cloud.row_step +
    static_cast<std::size_t>(col) * cloud.point_step;
  return {
    cloud.data.begin() + static_cast<std::ptrdiff_t>(offset),
    cloud.data.begin() + static_cast<std::ptrdiff_t>(offset + cloud.point_step),
  };
}

}  // namespace

TEST(LegalLidarFilterCore, PreservesKeptPointBytesAndHandlesRowPadding)
{
  auto cloud = make_cloud(3, 2, 24, 8);
  write_point(cloud, 0, 0, -0.9F, 0.0F, 0.2F, 10.0F, 1000, 1);
  write_point(cloud, 0, 1, 8.0F, 0.0F, 0.2F, 11.0F, 1001, 2);
  write_point(cloud, 0, 2, NAN, 0.0F, 0.2F, 12.0F, 1002, 3);
  write_point(cloud, 1, 0, 0.0F, 0.0F, 0.2F, 13.0F, 1003, 4);
  write_point(cloud, 1, 1, 0.5F, -3.2F, 0.2F, 14.0F, 1004, 5);
  write_point(cloud, 1, 2, 6.0F, 0.0F, 0.2F, 15.0F, 1005, 6);
  const auto expected_first = point_record(cloud, 1, 0);
  const auto expected_second = point_record(cloud, 1, 2);

  const auto result = lunabot_localisation_cpp::filter_cloud_to_legal_bounds(
    cloud,
    ArenaBounds{},
    RigidTransform{});

  EXPECT_EQ(result.raw_count, 6U);
  EXPECT_EQ(result.finite_count, 5U);
  EXPECT_EQ(result.kept_count, 2U);
  EXPECT_EQ(result.rejected_count, 3U);
  EXPECT_EQ(result.cloud.header.frame_id, "os_lidar");
  EXPECT_TRUE(result.cloud.is_dense);
  EXPECT_EQ(result.cloud.height, 1U);
  EXPECT_EQ(result.cloud.width, 2U);
  EXPECT_EQ(result.cloud.row_step, result.cloud.point_step * result.cloud.width);
  ASSERT_EQ(result.cloud.data.size(), 2U * cloud.point_step);
  EXPECT_TRUE(std::equal(expected_first.begin(), expected_first.end(), result.cloud.data.begin()));
  EXPECT_TRUE(
    std::equal(
      expected_second.begin(),
      expected_second.end(),
      result.cloud.data.begin() + cloud.point_step));
}

TEST(LegalLidarFilterCore, UsesTransformForMembershipWithoutRewritingPointBytes)
{
  auto cloud = make_cloud(1, 1, 24, 0);
  write_point(cloud, 0, 0, 0.0F, 0.0F, 0.2F, 20.0F, 2000, 7);
  const auto expected = point_record(cloud, 0, 0);

  const auto result = lunabot_localisation_cpp::filter_cloud_to_legal_bounds(
    cloud,
    ArenaBounds{0.0, 2.0, 0.0, 2.0, 0.35},
    RigidTransform{{0.5, 0.5, 0.0}, {0.0, 0.0, 0.0, 1.0}});

  EXPECT_EQ(result.kept_count, 1U);
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), result.cloud.data.begin()));
}

TEST(LegalLidarFilterCore, ReadsBigEndianFloatFields)
{
  auto cloud = make_cloud(1, 1, 24, 0, true);
  write_point(cloud, 0, 0, 0.0F, 0.0F, 0.2F, 30.0F, 3000, 8);

  const auto result = lunabot_localisation_cpp::filter_cloud_to_legal_bounds(
    cloud,
    ArenaBounds{},
    RigidTransform{});

  EXPECT_EQ(result.kept_count, 1U);
  EXPECT_EQ(result.cloud.is_bigendian, true);
}

TEST(LegalLidarFilterCore, RejectsMissingOrInvalidXyzFieldsLoudly)
{
  auto cloud = make_cloud(1, 1, 24, 0);
  cloud.fields[1].datatype = PointField::UINT32;

  EXPECT_THROW(
    lunabot_localisation_cpp::filter_cloud_to_legal_bounds(
      cloud,
      ArenaBounds{},
      RigidTransform{}),
    std::invalid_argument);
}

TEST(LegalLidarFilterCore, RejectsMalformedRowStep)
{
  auto cloud = make_cloud(2, 1, 24, 0);
  cloud.row_step = 12;

  EXPECT_THROW(
    lunabot_localisation_cpp::filter_cloud_to_legal_bounds(
      cloud,
      ArenaBounds{},
      RigidTransform{}),
    std::invalid_argument);
}
