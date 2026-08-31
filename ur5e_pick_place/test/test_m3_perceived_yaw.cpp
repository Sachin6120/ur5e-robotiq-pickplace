// Focused Stage-2C unit tests for the m3_grasp perceived-yaw acceptance and
// composition seam.  Include the implementation in test mode so every check
// calls the same helpers used by the node, without a live ROS graph, MoveIt,
// or Gazebo process.

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>

#define UR5E_PICK_PLACE_M3_GRASP_UNIT_TEST
#include "../src/m3_grasp.cpp"  // NOLINT(build/include)

namespace
{

constexpr double kPi = 3.14159265358979323846;

double rad(double degrees)
{
  return degrees * kPi / 180.0;
}

tf2::Matrix3x3 downward_grasp_basis(double yaw)
{
  return R_from_rpy(0.0, 0.0, yaw) * R_from_rpy(kPi, 0.0, 0.0);
}

tf2::Transform transform_with_basis(const tf2::Matrix3x3 & basis)
{
  tf2::Quaternion q;
  basis.getRotation(q);
  return tf2::Transform(q, tf2::Vector3(0.0, 0.0, 0.0));
}

void expect_same_basis(const tf2::Matrix3x3 & actual, const tf2::Matrix3x3 & expected)
{
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      EXPECT_NEAR(actual[row][col], expected[row][col], 1e-12);
    }
  }
}

geometry_msgs::msg::PoseStamped pose_world(double yaw_deg, int sec)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.stamp.sec = sec;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, rad(yaw_deg));
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}

TEST(M3PerceivedYaw, DefaultOffLeavesBasisByteForByteUnchanged)
{
  const auto existing = downward_grasp_basis(rad(-37.0));
  const auto result = grasp_basis_with_perceived_yaw(
    existing, false, rad(45.0), rad(-20.0));
  expect_same_basis(result, existing);
}

TEST(M3PerceivedYaw, DefaultOffReportsConfiguredYawWithoutConsumingPerceivedYaw)
{
  const auto object = transform_with_basis(R_from_rpy(0.0, 0.0, 0.0));
  const auto configured_yaw = configured_object_planar_yaw(object);
  ASSERT_TRUE(configured_yaw);
  EXPECT_TRUE(std::isfinite(*configured_yaw));
  EXPECT_NEAR(*configured_yaw, 0.0, 1e-12);

  // Default telemetry semantics remain configured source with no perceived
  // sample or delta, and the command basis remains the configured basis.
  const std::string yaw_source = "configured";
  const double perceived_object_yaw = std::numeric_limits<double>::quiet_NaN();
  const double yaw_delta = std::numeric_limits<double>::quiet_NaN();
  const auto existing = downward_grasp_basis(rad(-37.0));
  const auto result = grasp_basis_with_perceived_yaw(
    existing, false, rad(45.0), *configured_yaw);
  EXPECT_EQ(yaw_source, "configured");
  EXPECT_TRUE(std::isnan(perceived_object_yaw));
  EXPECT_TRUE(std::isnan(yaw_delta));
  expect_same_basis(result, existing);
}

TEST(M3PerceivedYaw, ConfiguredYawTelemetryRejectsNonLevelObjectFrame)
{
  const auto tilted_object = transform_with_basis(R_from_rpy(rad(1.0), 0.0, 0.0));
  EXPECT_FALSE(configured_object_planar_yaw(tilted_object));
}

TEST(M3PerceivedYaw, PerceivedYawSweepPreMultipliesAxialDelta)
{
  const auto existing = downward_grasp_basis(rad(12.0));
  for (const double perceived_deg : {0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 85.0}) {
    const double delta = ur5e_pick_place::axial_difference(rad(perceived_deg), 0.0);
    const auto result = grasp_basis_with_perceived_yaw(existing, true, rad(perceived_deg), 0.0);
    const auto expected = R_from_rpy(0.0, 0.0, delta) * existing;
    expect_same_basis(result, expected);
  }
}

TEST(M3PerceivedYaw, AxiallyEquivalentSamplesProduceTheSameBasis)
{
  const auto existing = downward_grasp_basis(rad(8.0));
  const auto a = grasp_basis_with_perceived_yaw(existing, true, rad(30.0), rad(-15.0));
  const auto b = grasp_basis_with_perceived_yaw(existing, true, rad(210.0), rad(-15.0));
  expect_same_basis(a, b);
}

TEST(M3PerceivedYaw, AxialDifferenceTreatsConfiguredMinus95AndPerceivedPlus85AsEqual)
{
  const double delta = ur5e_pick_place::axial_difference(rad(85.0), rad(-95.0));
  EXPECT_NEAR(delta, 0.0, 1e-12);

  const auto existing = downward_grasp_basis(rad(-23.0));
  const auto result = grasp_basis_with_perceived_yaw(existing, true, rad(85.0), rad(-95.0));
  expect_same_basis(result, existing);
}

TEST(M3PerceivedYaw, PreMultiplicationKeepsLocalGraspZWorldDown)
{
  const auto result = grasp_basis_with_perceived_yaw(
    downward_grasp_basis(rad(18.0)), true, rad(85.0), rad(-15.0));
  const tf2::Vector3 local_z = result * tf2::Vector3(0.0, 0.0, 1.0);
  EXPECT_NEAR(local_z.x(), 0.0, 1e-12);
  EXPECT_NEAR(local_z.y(), 0.0, 1e-12);
  EXPECT_NEAR(local_z.z(), -1.0, 1e-12);
}

TEST(M3PerceivedYaw, NoFreshPoseMeansNoAcceptedYawAndWouldTimeout)
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr no_pose;
  EXPECT_FALSE(no_pose);
  EXPECT_EQ(perceived_yaw_sample_result(static_cast<bool>(no_pose)), Result::PERCEPTION_TIMEOUT);
}

TEST(M3PerceivedYaw, StalePoseIsRejected)
{
  const auto pose = pose_world(30.0, 100);
  const rclcpp::Time boundary(100, 0, RCL_ROS_TIME);
  EXPECT_FALSE(is_fresh_world_pose(pose, "world", boundary));
  EXPECT_EQ(perceived_yaw_sample_result(false), Result::PERCEPTION_TIMEOUT);
}

TEST(M3PerceivedYaw, InvalidQuaternionIsRejectedAndWouldTimeout)
{
  auto pose = pose_world(30.0, 101);
  pose.pose.orientation.w = 0.0;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  EXPECT_FALSE(planar_yaw_from_valid_quaternion(pose));
  EXPECT_EQ(perceived_yaw_sample_result(false), Result::PERCEPTION_TIMEOUT);
}

TEST(M3PerceivedYaw, FreshWorldPoseWithUnitQuaternionIsAccepted)
{
  const auto pose = pose_world(30.0, 101);
  const rclcpp::Time boundary(100, 0, RCL_ROS_TIME);
  ASSERT_TRUE(is_fresh_world_pose(pose, "world", boundary));
  const auto yaw = planar_yaw_from_valid_quaternion(pose);
  ASSERT_TRUE(yaw);
  EXPECT_NEAR(*yaw, rad(30.0), 1e-12);
}

TEST(M3PerceivedYaw, PerceivedYawRequiresPerceivedPosition)
{
  EXPECT_FALSE(perceived_yaw_configuration_valid(true, false));
  EXPECT_TRUE(perceived_yaw_configuration_valid(true, true));
  EXPECT_TRUE(perceived_yaw_configuration_valid(false, false));
}

TEST(M3PerceivedYaw, TiltedApproachGeometryIsRejected)
{
  double configured_yaw = std::numeric_limits<double>::quiet_NaN();
  std::string error;
  const auto object = transform_with_basis(R_from_rpy(0.0, 0.0, rad(20.0)));
  const auto tilted_grasp = transform_with_basis(R_from_rpy(0.0, rad(10.0), 0.0));
  EXPECT_FALSE(configured_yaw_reference_supported(object, tilted_grasp, configured_yaw, error));
  EXPECT_NE(error.find("world -Z"), std::string::npos);
}

TEST(M3PerceivedYaw, NonLevelConfiguredObjectFrameIsRejected)
{
  double configured_yaw = std::numeric_limits<double>::quiet_NaN();
  std::string error;
  const auto tilted_object = transform_with_basis(R_from_rpy(rad(1.0), 0.0, 0.0));
  const auto grasp = transform_with_basis(downward_grasp_basis(0.0));
  EXPECT_FALSE(configured_yaw_reference_supported(tilted_object, grasp, configured_yaw, error));
  EXPECT_NE(error.find("not level"), std::string::npos);
}

}  // namespace
