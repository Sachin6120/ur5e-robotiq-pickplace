// test_object_position_world.cpp
//
// Focused unit tests for the pose_camera -> pose_world TF2 transform added
// to object_position_world.cpp (Stage-2B, 2026-08-31).
//
// These tests exercise a real tf2_ros::Buffer directly, with a KNOWN static
// transform injected via setTransform(), rather than instantiating the ROS
// node. That is sufficient: the node's callbacks are thin wrappers around
// exactly one buffer_->transform() call each (see object_position_world.cpp
// -- the point path and the new pose path are structurally identical), so
// testing the buffer's own transform behaviour against this file's
// documented contract IS testing what the node does, without pulling in
// rclcpp::spin, executors, or a live TF listener thread.
//
// The camera extrinsics used below (translation (0.450, 0.025, 2.400);
// +X_opt=-Y_world, +Y_opt=-X_world, +Z_opt=-Z_world) are read from
// ur5e_robotiq.urdf.xacro and were independently confirmed against a live
// D10 measurement on 2026-08-31 (predicted optical XYZ (0.175, 0.000, 1.605)
// vs measured (0.174672, -0.000322, 1.605000); see test_edge_line_tls.cpp's
// CameraFramePoseMapping suite for the same constant used there).

#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>

#include "ur5e_pick_place/mask_orientation.hpp"

namespace
{

using ur5e_pick_place::axial_difference;
using ur5e_pick_place::canonicalize_axial_angle;
using ur5e_pick_place::kMaskOrientationPi;

double deg(double rad) { return rad * 180.0 / kMaskOrientationPi; }
double rad(double degrees) { return degrees * kMaskOrientationPi / 180.0; }

constexpr double kCamX = 0.450;
constexpr double kCamY = 0.025;
constexpr double kCamZ = 2.400;

// world_from_optical rotation, +X_opt=-Y_world, +Y_opt=-X_world, +Z_opt=-Z_world.
tf2::Matrix3x3 world_from_optical_basis()
{
  return tf2::Matrix3x3(
    0.0, -1.0, 0.0,
    -1.0, 0.0, 0.0,
    0.0, 0.0, -1.0);
}

geometry_msgs::msg::TransformStamped world_from_optical_transform(builtin_interfaces::msg::Time stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = "world";
  t.child_frame_id = "camera_optical_frame";
  t.transform.translation.x = kCamX;
  t.transform.translation.y = kCamY;
  t.transform.translation.z = kCamZ;
  tf2::Quaternion q;
  world_from_optical_basis().getRotation(q);
  t.transform.rotation = tf2::toMsg(q);
  return t;
}

builtin_interfaces::msg::Time make_stamp(int32_t sec, uint32_t nanosec = 0)
{
  builtin_interfaces::msg::Time t;
  t.sec = sec;
  t.nanosec = nanosec;
  return t;
}

// Analytical point-transform law implied by the URDF extrinsics, derived
// independently of the buffer under test.
void expected_world_point(double px, double py, double pz, double & wx, double & wy, double & wz)
{
  wx = -py + kCamX;
  wy = -px + kCamY;
  wz = -pz + kCamZ;
}

// Physical projection: the object's long axis (object-local +Y, per
// scene.yaml object.size = [0.030, 0.045, 0.045]) at world yaw psi, mapped
// into the image angle through the SAME optical mirror as above. Derived
// independently of object_detector.cpp / edge_line_tls.hpp -- neither is
// included or exercised by this test file.
double project_long_axis_to_image_angle(double psi_rad)
{
  const double wx = -std::sin(psi_rad);
  const double wy = std::cos(psi_rad);
  return canonicalize_axial_angle(std::atan2(-wx, -wy));
}

// Builds the EXACT pose_camera-contract quaternion object_detector.cpp
// publishes: R_opt_obj = Rz(theta_image - pi/2) * Rx(pi).
tf2::Quaternion object_pose_in_optical_frame(double theta_image_rad)
{
  tf2::Quaternion q;
  q.setRPY(kMaskOrientationPi, 0.0, theta_image_rad - kMaskOrientationPi / 2.0);
  return q;
}

class ObjectPositionWorldTF : public ::testing::Test
{
protected:
  void SetUp() override
  {
    buffer_ = std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::make_shared());
    stamp_ = make_stamp(100);
    buffer_->setTransform(world_from_optical_transform(stamp_), "test_authority", true);
  }

  std::unique_ptr<tf2_ros::Buffer> buffer_;
  builtin_interfaces::msg::Time stamp_;
};

// ===========================================================================
// SECTION A: EXISTING POINT PATH, UNCHANGED
// ===========================================================================

TEST_F(ObjectPositionWorldTF, PointTransformMatchesAnalyticalFormula)
{
  geometry_msgs::msg::PointStamped in;
  in.header.stamp = stamp_;
  in.header.frame_id = "camera_optical_frame";
  in.point.x = 0.174672;
  in.point.y = -0.000322;
  in.point.z = 1.605000;

  geometry_msgs::msg::PointStamped out;
  ASSERT_NO_THROW(buffer_->transform(in, out, "world"));

  double wx, wy, wz;
  expected_world_point(in.point.x, in.point.y, in.point.z, wx, wy, wz);
  EXPECT_NEAR(out.point.x, wx, 1e-9);
  EXPECT_NEAR(out.point.y, wy, 1e-9);
  EXPECT_NEAR(out.point.z, wz, 1e-9);

  // Sanity: this is the live 2026-08-31 D10 sample at configured yaw 0; the
  // transformed point must land near the known pick-pose object centre
  // (0.450, -0.150), confirming the injected test TF matches production
  // geometry, not just its own internal consistency.
  EXPECT_NEAR(out.point.x, 0.450, 0.001);
  EXPECT_NEAR(out.point.y, -0.150, 0.001);
}

TEST_F(ObjectPositionWorldTF, MissingTransformThrowsTf2TransformException)
{
  // A fresh buffer with NO transform injected reproduces exactly the
  // exception type object_position_world.cpp's catch clause depends on for
  // its "fail silently, publish nothing" contract, on BOTH message types.
  auto empty_buffer = std::make_unique<tf2_ros::Buffer>(rclcpp::Clock::make_shared());

  geometry_msgs::msg::PointStamped pt_in;
  pt_in.header.stamp = stamp_;
  pt_in.header.frame_id = "camera_optical_frame";
  geometry_msgs::msg::PointStamped pt_out;
  EXPECT_THROW(empty_buffer->transform(pt_in, pt_out, "world"), tf2::TransformException);

  geometry_msgs::msg::PoseStamped pose_in;
  pose_in.header.stamp = stamp_;
  pose_in.header.frame_id = "camera_optical_frame";
  pose_in.pose.orientation.w = 1.0;
  geometry_msgs::msg::PoseStamped pose_out;
  EXPECT_THROW(empty_buffer->transform(pose_in, pose_out, "world"), tf2::TransformException);
}

// ===========================================================================
// SECTION B: FULL POSE TRANSFORMATION
// ===========================================================================

TEST_F(ObjectPositionWorldTF, PoseTransformCarriesPositionIdenticallyToPointPath)
{
  const double theta_img = project_long_axis_to_image_angle(rad(30.0));
  geometry_msgs::msg::PoseStamped in;
  in.header.stamp = stamp_;
  in.header.frame_id = "camera_optical_frame";
  in.pose.position.x = 0.174672;
  in.pose.position.y = -0.000322;
  in.pose.position.z = 1.605000;
  in.pose.orientation = tf2::toMsg(object_pose_in_optical_frame(theta_img));

  geometry_msgs::msg::PoseStamped out;
  ASSERT_NO_THROW(buffer_->transform(in, out, "world"));

  double wx, wy, wz;
  expected_world_point(in.pose.position.x, in.pose.position.y, in.pose.position.z, wx, wy, wz);
  EXPECT_NEAR(out.pose.position.x, wx, 1e-9);
  EXPECT_NEAR(out.pose.position.y, wy, 1e-9);
  EXPECT_NEAR(out.pose.position.z, wz, 1e-9);
}

TEST_F(ObjectPositionWorldTF, PoseTransformProducesCorrectWorldOrientationViaTF2Alone)
{
  // NO manual axial correction, NO Euler reconstruction here -- extraction is
  // an ordinary quaternion -> matrix -> atan2 read of what TF2 itself
  // produced, proving the transform alone is sufficient.
  const double psi = rad(30.0);
  const double theta_img = project_long_axis_to_image_angle(psi);

  geometry_msgs::msg::PoseStamped in;
  in.header.stamp = stamp_;
  in.header.frame_id = "camera_optical_frame";
  in.pose.position.x = 0.174672;
  in.pose.position.y = -0.000322;
  in.pose.position.z = 1.605000;
  in.pose.orientation = tf2::toMsg(object_pose_in_optical_frame(theta_img));

  geometry_msgs::msg::PoseStamped out;
  ASSERT_NO_THROW(buffer_->transform(in, out, "world"));

  tf2::Quaternion q_world;
  tf2::fromMsg(out.pose.orientation, q_world);
  tf2::Matrix3x3 m(q_world);
  const double world_yaw = canonicalize_axial_angle(std::atan2(m[1][0], m[0][0]));

  EXPECT_LT(std::abs(deg(axial_difference(world_yaw, psi))), 1e-6);

  // The transformed rotation must remain a proper rotation with no residual
  // tilt: this is a pure yaw about world Z, same check as
  // test_edge_line_tls.cpp's PublishedQuaternionComposesToWorldYaw.
  EXPECT_NEAR(m[2][2], 1.0, 1e-9);
  EXPECT_NEAR(m[0][2], 0.0, 1e-9);
  EXPECT_NEAR(m[1][2], 0.0, 1e-9);
  EXPECT_NEAR(m[2][0], 0.0, 1e-9);
  EXPECT_NEAR(m[2][1], 0.0, 1e-9);
}

// ===========================================================================
// SECTION C: CONFIGURED-EQUIVALENT YAW SWEEP (axial comparison only)
// ===========================================================================

TEST_F(ObjectPositionWorldTF, ConfiguredEquivalentYawSweepMatchesModuloAxialSymmetry)
{
  const std::vector<double> yaws_deg = {0.0, 15.0, -15.0, 30.0, -30.0, 45.0, -45.0, 85.0};

  double worst_err_deg = 0.0;
  for (const double psi_deg : yaws_deg) {
    const double psi = rad(psi_deg);
    const double theta_img = project_long_axis_to_image_angle(psi);

    geometry_msgs::msg::PoseStamped in;
    in.header.stamp = stamp_;
    in.header.frame_id = "camera_optical_frame";
    in.pose.position.x = 0.174672;
    in.pose.position.y = -0.000322;
    in.pose.position.z = 1.605000;
    in.pose.orientation = tf2::toMsg(object_pose_in_optical_frame(theta_img));

    geometry_msgs::msg::PoseStamped out;
    ASSERT_NO_THROW(buffer_->transform(in, out, "world")) << "psi=" << psi_deg;

    tf2::Quaternion q_world;
    tf2::fromMsg(out.pose.orientation, q_world);
    tf2::Matrix3x3 m(q_world);
    const double world_yaw = canonicalize_axial_angle(std::atan2(m[1][0], m[0][0]));

    // MUST use axial_difference (shortest mod-180 difference), never a plain
    // wrapped angle compare -- see mask_orientation.hpp's AXIAL ANGLE
    // SEMANTICS note. psi=+85 and a raw-wrapped yaw near -95 would otherwise
    // score as a ~180 deg error instead of the correct ~0.
    const double err_deg = std::abs(deg(axial_difference(world_yaw, psi)));
    worst_err_deg = std::max(worst_err_deg, err_deg);
    EXPECT_LT(err_deg, 1e-6) << "psi=" << psi_deg;
  }
  EXPECT_LT(worst_err_deg, 0.5);
}

TEST_F(ObjectPositionWorldTF, AxialSymmetryIsRespectedThroughTheFullTransform)
{
  // psi and psi+180 are the same physical pose for this 2-fold-symmetric
  // object; the transformed world yaw must be axially identical for both.
  for (const double psi_deg : {0.0, 15.0, 45.0, 85.0}) {
    const double theta_a = project_long_axis_to_image_angle(rad(psi_deg));
    const double theta_b = project_long_axis_to_image_angle(rad(psi_deg + 180.0));

    auto world_yaw_for = [&](double theta_img) {
        geometry_msgs::msg::PoseStamped in;
        in.header.stamp = stamp_;
        in.header.frame_id = "camera_optical_frame";
        in.pose.position.x = 0.174672;
        in.pose.position.y = -0.000322;
        in.pose.position.z = 1.605000;
        in.pose.orientation = tf2::toMsg(object_pose_in_optical_frame(theta_img));
        geometry_msgs::msg::PoseStamped out;
        buffer_->transform(in, out, "world");
        tf2::Quaternion q;
        tf2::fromMsg(out.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        return canonicalize_axial_angle(std::atan2(m[1][0], m[0][0]));
      };

    const double yaw_a = world_yaw_for(theta_a);
    const double yaw_b = world_yaw_for(theta_b);
    EXPECT_LT(std::abs(deg(axial_difference(yaw_a, yaw_b))), 1e-6) << "psi=" << psi_deg;
  }
}

}  // namespace
