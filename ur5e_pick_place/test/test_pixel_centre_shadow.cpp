// Offline/unit coverage for the default-off pixel-centre shadow estimator.
// No RGB-D subscription, segmentation, D10 implementation change, TF listener,
// Gazebo, MoveIt, or manipulation node is involved.

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include "ur5e_pick_place/d10_trimmed_mean.hpp"
#include "ur5e_pick_place/pixel_centre_shadow.hpp"

namespace
{

using ur5e_pick_place::CameraPoint;
using ur5e_pick_place::PinholeIntrinsics;
using ur5e_pick_place::PixelCentroid;
using ur5e_pick_place::backproject_centroid;
using ur5e_pick_place::kPixelCentreCorrectionPx;
using ur5e_pick_place::kPixelCentreShadowDefaultEnabled;
using ur5e_pick_place::pixel_centre_corrected;

constexpr double kFx960 = 831.574069234;
constexpr double kFy960 = 831.574069234;
constexpr double kCx960 = 480.0;
constexpr double kCy960 = 360.0;
constexpr double kCurrentDepthM = 1.605;

geometry_msgs::msg::TransformStamped world_from_optical(
  const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = "world";
  transform.child_frame_id = "camera_optical_frame";
  transform.transform.translation.x = 0.450;
  transform.transform.translation.y = 0.025;
  transform.transform.translation.z = 2.400;
  // +X_opt=-Y_world, +Y_opt=-X_world, +Z_opt=-Z_world.
  tf2::Quaternion q;
  tf2::Matrix3x3(
    0.0, -1.0, 0.0,
    -1.0, 0.0, 0.0,
    0.0, 0.0, -1.0).getRotation(q);
  transform.transform.rotation = tf2::toMsg(q);
  return transform;
}

geometry_msgs::msg::PointStamped stamped_camera_point(
  const CameraPoint & point, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::PointStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = "camera_optical_frame";
  message.point.x = point.x;
  message.point.y = point.y;
  message.point.z = point.z;
  return message;
}

TEST(PixelCentreShadow, DefaultProductionCentroidAndBackprojectionAreUnchanged)
{
  EXPECT_FALSE(kPixelCentreShadowDefaultEnabled);

  const PixelCentroid raw{570.25, 359.75};
  const PinholeIntrinsics intrinsics{kFx960, kFy960, kCx960, kCy960};
  const CameraPoint production = backproject_centroid(raw, kCurrentDepthM, intrinsics);

  // Exact established production equation: no +0.5 is applied here.
  EXPECT_DOUBLE_EQ(production.x, (raw.u - kCx960) * kCurrentDepthM / kFx960);
  EXPECT_DOUBLE_EQ(production.y, (raw.v - kCy960) * kCurrentDepthM / kFy960);
  EXPECT_DOUBLE_EQ(production.z, kCurrentDepthM);
}

TEST(PixelCentreShadow, CorrectedPathAddsExactlyHalfPixelAndSharesD10Depth)
{
  const PixelCentroid raw{570.25, 359.75};
  const PixelCentroid corrected = pixel_centre_corrected(raw);
  EXPECT_DOUBLE_EQ(corrected.u, raw.u + 0.5);
  EXPECT_DOUBLE_EQ(corrected.v, raw.v + 0.5);
  EXPECT_DOUBLE_EQ(corrected.u - raw.u, kPixelCentreCorrectionPx);
  EXPECT_DOUBLE_EQ(corrected.v - raw.v, kPixelCentreCorrectionPx);

  // D10 is calculated once from depth samples, then supplied unchanged to
  // both projections; the shadow helper neither reads nor changes samples.
  const auto d10 = ur5e_pick_place::d10_trimmed_mean(
    {1.590, 1.600, 1.601, 1.602, 1.603, 1.604, 1.605, 1.606, 1.607, 1.608,
      1.609, 1.610});
  ASSERT_TRUE(d10.has_value());
  const PinholeIntrinsics intrinsics{kFx960, kFy960, kCx960, kCy960};
  const CameraPoint production = backproject_centroid(raw, *d10, intrinsics);
  const CameraPoint shadow = backproject_centroid(corrected, *d10, intrinsics);

  EXPECT_DOUBLE_EQ(production.z, *d10);
  EXPECT_DOUBLE_EQ(shadow.z, *d10);
  EXPECT_NEAR(shadow.x - production.x, 0.5 * *d10 / kFx960, 1e-15);
  EXPECT_NEAR(shadow.y - production.y, 0.5 * *d10 / kFy960, 1e-15);
}

TEST(PixelCentreShadow, Current960ScaleIsApproximatelyPoint965MmPerHalfPixelAxis)
{
  const PixelCentroid raw{kCx960, kCy960};
  const PinholeIntrinsics intrinsics{kFx960, kFy960, kCx960, kCy960};
  const CameraPoint production = backproject_centroid(raw, kCurrentDepthM, intrinsics);
  const CameraPoint shadow = backproject_centroid(
    pixel_centre_corrected(raw), kCurrentDepthM, intrinsics);

  const double expected_axis_mm = 0.5 * kCurrentDepthM / kFx960 * 1000.0;
  EXPECT_NEAR(expected_axis_mm, 0.965, 0.001);
  EXPECT_NEAR((shadow.x - production.x) * 1000.0, expected_axis_mm, 1e-12);
  EXPECT_NEAR((shadow.y - production.y) * 1000.0, expected_axis_mm, 1e-12);
}

TEST(PixelCentreShadow, TFMappingIsIdenticalApartFromCorrectedCameraXY)
{
  auto buffer = std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 100;
  buffer->setTransform(world_from_optical(stamp), "test_authority", true);

  const PixelCentroid raw{570.25, 359.75};
  const PinholeIntrinsics intrinsics{kFx960, kFy960, kCx960, kCy960};
  const CameraPoint production_camera = backproject_centroid(raw, kCurrentDepthM, intrinsics);
  const CameraPoint corrected_camera = backproject_centroid(
    pixel_centre_corrected(raw), kCurrentDepthM, intrinsics);

  geometry_msgs::msg::PointStamped production_world;
  geometry_msgs::msg::PointStamped corrected_world;
  ASSERT_NO_THROW(
    buffer->transform(stamped_camera_point(production_camera, stamp), production_world, "world"));
  ASSERT_NO_THROW(
    buffer->transform(stamped_camera_point(corrected_camera, stamp), corrected_world, "world"));

  // The same TF mapping rotates +camera Y into -world X and +camera X into
  // -world Y; Z is unchanged because both projections share D10 depth.
  EXPECT_DOUBLE_EQ(corrected_world.point.x - production_world.point.x,
                   -(corrected_camera.y - production_camera.y));
  EXPECT_DOUBLE_EQ(corrected_world.point.y - production_world.point.y,
                   -(corrected_camera.x - production_camera.x));
  EXPECT_DOUBLE_EQ(corrected_world.point.z - production_world.point.z, 0.0);
}

}  // namespace
