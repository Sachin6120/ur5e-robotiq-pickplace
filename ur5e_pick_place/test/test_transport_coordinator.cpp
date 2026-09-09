// test_transport_coordinator.cpp — Stage-3C C3 unit tests for pure logic
// and predicates: State E validation, obstacle freshness predicate,
// quaternion shortest angle (sign invariance), pose coherence, and failure codes.

#include "ur5e_pick_place/transport_coordinator.hpp"

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <optional>
#include <new>
#include <stdexcept>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <future>
#include <thread>
#include <moveit/robot_model/robot_model.hpp>
#include <srdfdom/srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{

const std::vector<std::string> kArmJoints = {
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint"
};

std::shared_ptr<moveit::core::RobotModel> createMockRobotModel()
{
  std::string urdf =
    "<?xml version=\"1.0\"?>"
    "<robot name=\"test_ur5e\">"
    "  <link name=\"base_link\"/>";

  std::string prev_link = "base_link";
  for (std::size_t i = 0; i < kArmJoints.size(); ++i) {
    const std::string child_link = "link_" + std::to_string(i + 1);
    urdf += "  <link name=\"" + child_link + "\"/>"
            "  <joint name=\"" + kArmJoints[i] + "\" type=\"revolute\">"
            "    <parent link=\"" + prev_link + "\"/>"
            "    <child link=\"" + child_link + "\"/>"
            "    <axis xyz=\"0 0 1\"/>"
            "    <limit lower=\"-6.28\" upper=\"6.28\" effort=\"150.0\" velocity=\"3.14\"/>"
            "  </joint>";
    prev_link = child_link;
  }
  urdf += "</robot>";

  auto urdf_model = urdf::parseURDF(urdf);
  if (!urdf_model) {
    return nullptr;
  }
  auto srdf_model = std::make_shared<srdf::Model>();
  srdf_model->initString(*urdf_model, "<robot name=\"test_ur5e\"/>");
  return std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
}

// --- validate_state_e tests ------------------------------------------------

TEST(TransportCoordinatorLogic, StateEValid)
{
  auto robot_model = createMockRobotModel();
  ASSERT_NE(robot_model, nullptr);

  ur5e_pick_place::SettledStateE state_e;
  state_e.captured = true;
  state_e.joint_names = kArmJoints;
  state_e.positions = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};

  std::string error;
  EXPECT_TRUE(ur5e_pick_place::validate_state_e(state_e, kArmJoints, *robot_model, error));
  EXPECT_TRUE(error.empty());
}

TEST(TransportCoordinatorLogic, StateENotCaptured)
{
  auto robot_model = createMockRobotModel();
  ASSERT_NE(robot_model, nullptr);

  ur5e_pick_place::SettledStateE state_e;
  state_e.captured = false;
  state_e.joint_names = kArmJoints;
  state_e.positions = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};

  std::string error;
  EXPECT_FALSE(ur5e_pick_place::validate_state_e(state_e, kArmJoints, *robot_model, error));
  EXPECT_NE(error.find("STATE_E_NOT_CAPTURED"), std::string::npos);
}

TEST(TransportCoordinatorLogic, StateEInvalidSize)
{
  auto robot_model = createMockRobotModel();
  ASSERT_NE(robot_model, nullptr);

  ur5e_pick_place::SettledStateE state_e;
  state_e.captured = true;
  state_e.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint"};
  state_e.positions = {0.1, -0.2};

  std::string error;
  EXPECT_FALSE(ur5e_pick_place::validate_state_e(state_e, kArmJoints, *robot_model, error));
  EXPECT_NE(error.find("STATE_E_INVALID_SIZE"), std::string::npos);
}

TEST(TransportCoordinatorLogic, StateENonFinite)
{
  auto robot_model = createMockRobotModel();
  ASSERT_NE(robot_model, nullptr);

  ur5e_pick_place::SettledStateE state_e;
  state_e.captured = true;
  state_e.joint_names = kArmJoints;
  state_e.positions = {0.1, std::numeric_limits<double>::quiet_NaN(), 0.3, -0.4, 0.5, -0.6};

  std::string error;
  EXPECT_FALSE(ur5e_pick_place::validate_state_e(state_e, kArmJoints, *robot_model, error));
  EXPECT_NE(error.find("STATE_E_NON_FINITE"), std::string::npos);
}

TEST(TransportCoordinatorLogic, StateEDuplicateJoints)
{
  auto robot_model = createMockRobotModel();
  ASSERT_NE(robot_model, nullptr);

  ur5e_pick_place::SettledStateE state_e;
  state_e.captured = true;
  state_e.joint_names = {
    "shoulder_pan_joint", "shoulder_pan_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
  };
  state_e.positions = {0.1, 0.1, 0.3, -0.4, 0.5, -0.6};

  std::string error;
  EXPECT_FALSE(ur5e_pick_place::validate_state_e(state_e, kArmJoints, *robot_model, error));
  EXPECT_NE(error.find("STATE_E_DUPLICATE_NAMES"), std::string::npos);
}

TEST(TransportCoordinatorLogic, StateESetMismatch)
{
  auto robot_model = createMockRobotModel();
  ASSERT_NE(robot_model, nullptr);

  ur5e_pick_place::SettledStateE state_e;
  state_e.captured = true;
  state_e.joint_names = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "other_joint"
  };
  state_e.positions = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};

  std::string error;
  EXPECT_FALSE(ur5e_pick_place::validate_state_e(state_e, kArmJoints, *robot_model, error));
  EXPECT_NE(error.find("STATE_E_SET_MISMATCH"), std::string::npos);
}

// --- is_obstacle_fresh_post_time tests -------------------------------------

TEST(TransportCoordinatorLogic, FreshnessPostTimePredicate)
{
  const double baseline = 10.0;
  const double stale_thresh = 0.250;

  // 1. Obstacle older than baseline -> rejected
  EXPECT_FALSE(ur5e_pick_place::is_obstacle_fresh_post_time(9.999, baseline, 10.050, stale_thresh));

  // 2. Obstacle equal to baseline and fresh (age = 50ms) -> accepted
  EXPECT_TRUE(ur5e_pick_place::is_obstacle_fresh_post_time(10.000, baseline, 10.050, stale_thresh));

  // 3. Obstacle newer than query time (negative age) -> rejected
  EXPECT_FALSE(ur5e_pick_place::is_obstacle_fresh_post_time(10.100, baseline, 10.050, stale_thresh));

  // 4. Obstacle age exceeds 250ms (age = 251ms) -> rejected
  EXPECT_FALSE(ur5e_pick_place::is_obstacle_fresh_post_time(10.000, baseline, 10.251, stale_thresh));

  // 5. Obstacle age at exact 250ms boundary -> accepted
  EXPECT_TRUE(ur5e_pick_place::is_obstacle_fresh_post_time(10.000, baseline, 10.250, stale_thresh));

  // 6. Typical fresh post-plan sample: baseline=10.0, obstacle=10.050, query=10.070 (age=20ms) -> accepted
  EXPECT_TRUE(ur5e_pick_place::is_obstacle_fresh_post_time(10.050, baseline, 10.070, stale_thresh));

  // 7. Non-finite values -> rejected
  EXPECT_FALSE(ur5e_pick_place::is_obstacle_fresh_post_time(
    std::numeric_limits<double>::quiet_NaN(), baseline, 10.050, stale_thresh));
}

// --- quaternion_shortest_angle_rad tests ------------------------------------

TEST(TransportCoordinatorLogic, QuaternionShortestAngleSignInvariant)
{
  geometry_msgs::msg::Quaternion q_id;
  q_id.x = 0.0; q_id.y = 0.0; q_id.z = 0.0; q_id.w = 1.0;

  // Identity against identity -> 0.0
  EXPECT_NEAR(ur5e_pick_place::quaternion_shortest_angle_rad(q_id, q_id), 0.0, 1e-9);

  // Invariant C: q vs -q must yield 0.0 rad
  geometry_msgs::msg::Quaternion q_neg_id;
  q_neg_id.x = 0.0; q_neg_id.y = 0.0; q_neg_id.z = 0.0; q_neg_id.w = -1.0;
  EXPECT_NEAR(ur5e_pick_place::quaternion_shortest_angle_rad(q_id, q_neg_id), 0.0, 1e-9);

  // 90 deg rotation about Z: q = [0, 0, sin(pi/4), cos(pi/4)]
  geometry_msgs::msg::Quaternion q_z90;
  q_z90.x = 0.0; q_z90.y = 0.0;
  q_z90.z = std::sin(M_PI / 4.0);
  q_z90.w = std::cos(M_PI / 4.0);
  EXPECT_NEAR(
    ur5e_pick_place::quaternion_shortest_angle_rad(q_id, q_z90),
    M_PI / 2.0, 1e-6);

  // -q_z90 against q_id must also be pi/2
  geometry_msgs::msg::Quaternion q_z90_neg;
  q_z90_neg.x = -q_z90.x; q_z90_neg.y = -q_z90.y;
  q_z90_neg.z = -q_z90.z; q_z90_neg.w = -q_z90.w;
  EXPECT_NEAR(
    ur5e_pick_place::quaternion_shortest_angle_rad(q_id, q_z90_neg),
    M_PI / 2.0, 1e-6);

  // q_z90 against -q_z90 must be 0.0
  EXPECT_NEAR(
    ur5e_pick_place::quaternion_shortest_angle_rad(q_z90, q_z90_neg),
    0.0, 1e-9);
}

// --- is_pose_coherent tests ------------------------------------------------

TEST(TransportCoordinatorLogic, PoseCoherencePositionAndAngle)
{
  geometry_msgs::msg::Pose p_scene;
  p_scene.position.x = 0.7000;
  p_scene.position.y = 0.3500;
  p_scene.position.z = 0.8500;
  p_scene.orientation.w = 1.0;

  geometry_msgs::msg::Pose p_exp = p_scene;

  // Exact match -> true
  EXPECT_TRUE(ur5e_pick_place::is_pose_coherent(p_scene, p_exp));

  // Position error = 0.08 mm (within 0.1 mm) -> true
  p_exp.position.x += 8.0e-5;
  EXPECT_TRUE(ur5e_pick_place::is_pose_coherent(p_scene, p_exp));

  // Position error = 0.12 mm (exceeds 0.1 mm) -> false
  p_exp.position.x += 4.0e-5;  // total 1.2e-4 m
  EXPECT_FALSE(ur5e_pick_place::is_pose_coherent(p_scene, p_exp));

  // Reset position, test inverted quaternion sign
  p_exp = p_scene;
  p_exp.orientation.w = -1.0;  // -q representation
  EXPECT_TRUE(ur5e_pick_place::is_pose_coherent(p_scene, p_exp));

  // Small rotation angle: 0.5 mrad (within 1.0 mrad threshold) -> true
  const double half_angle = 0.5e-3 / 2.0;
  p_exp.orientation.z = std::sin(half_angle);
  p_exp.orientation.w = std::cos(half_angle);
  EXPECT_TRUE(ur5e_pick_place::is_pose_coherent(p_scene, p_exp));

  // Rotation angle: 1.5 mrad (exceeds 1.0 mrad threshold) -> false
  const double half_angle_large = 1.5e-3 / 2.0;
  p_exp.orientation.z = std::sin(half_angle_large);
  p_exp.orientation.w = std::cos(half_angle_large);
  EXPECT_FALSE(ur5e_pick_place::is_pose_coherent(p_scene, p_exp));
}

// --- Result::TRANSPORT_REPLAN_LIMIT_REACHED test ---------------------------

TEST(TransportCoordinatorLogic, ResultReplanLimitReached)
{
  const auto r = ur5e_pick_place::Result::TRANSPORT_REPLAN_LIMIT_REACHED;
  EXPECT_FALSE(ur5e_pick_place::ok(r));
  EXPECT_STREQ(ur5e_pick_place::to_string(r), "TRANSPORT_REPLAN_LIMIT_REACHED");
}

// --- Orientation upright tilt tests ---------------------------------------

TEST(TransportCoordinatorLogic, QuaternionUprightTiltDeg)
{
  // 1. Identity quaternion -> 0.0 deg
  EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(0.0, 0.0, 0.0, 1.0), 0.0, 1e-9);

  // 2. Negated identity -> 0.0 deg (strict sign-invariance)
  EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(0.0, 0.0, 0.0, -1.0), 0.0, 1e-9);

  // 3. Pure yaw rotations about Z (any angle) -> 0.0 deg (yaw-invariance)
  for (double yaw_deg : {0.0, 30.0, 45.0, 90.0, 120.0, 180.0, 270.0, 359.0}) {
    const double yaw_rad = yaw_deg * (M_PI / 180.0);
    const double qz = std::sin(yaw_rad / 2.0);
    const double qw = std::cos(yaw_rad / 2.0);
    EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(0.0, 0.0, qz, qw), 0.0, 1e-7);
    EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(0.0, 0.0, -qz, -qw), 0.0, 1e-7);
  }

  // 4. Pure pitch by 5 deg
  {
    const double pitch_rad = 5.0 * (M_PI / 180.0);
    const double qy = std::sin(pitch_rad / 2.0);
    const double qw = std::cos(pitch_rad / 2.0);
    EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(0.0, qy, 0.0, qw), 5.0, 1e-7);
  }

  // 5. Pure roll by 1.8 deg (orientation path constraint boundary)
  {
    const double roll_rad = 1.8 * (M_PI / 180.0);
    const double qx = std::sin(roll_rad / 2.0);
    const double qw = std::cos(roll_rad / 2.0);
    EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(qx, 0.0, 0.0, qw), 1.8, 1e-7);
  }

  // 5b. Pure roll by 2.0 deg (manipulation acceptance gate boundary)
  {
    const double roll_rad = 2.0 * (M_PI / 180.0);
    const double qx = std::sin(roll_rad / 2.0);
    const double qw = std::cos(roll_rad / 2.0);
    EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(qx, 0.0, 0.0, qw), 2.0, 1e-7);
  }

  // 5c. Pure roll by 14 deg
  {
    const double roll_rad = 14.0 * (M_PI / 180.0);
    const double qx = std::sin(roll_rad / 2.0);
    const double qw = std::cos(roll_rad / 2.0);
    EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(qx, 0.0, 0.0, qw), 14.0, 1e-7);
  }

  // 6. Complete inversion (180 deg flip)
  EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(1.0, 0.0, 0.0, 0.0), 180.0, 1e-7);
  EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(0.0, 1.0, 0.0, 0.0), 180.0, 1e-7);

  // 7. Replacement trajectory wrist flip angle (145.6258 deg from audit)
  {
    const double flip_rad = 145.6258 * (M_PI / 180.0);
    const double qx = std::sin(flip_rad / 2.0);
    const double qw = std::cos(flip_rad / 2.0);
    EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(qx, 0.0, 0.0, qw), 145.6258, 1e-4);
  }

  // 8. 2-argument overload (qx, qy) with normalized assumption
  EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(0.0, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(1.0, 0.0), 180.0, 1e-9);

  // 9. Zero norm -> 180.0 deg
  EXPECT_NEAR(ur5e_pick_place::quaternion_upright_tilt_deg(0.0, 0.0, 0.0, 0.0), 180.0, 1e-9);
}

TEST(TransportCoordinatorLogic, ComputePayloadUprightTiltDeg)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0; q.y = 0.0; q.z = 0.0; q.w = 1.0;
  EXPECT_NEAR(ur5e_pick_place::compute_payload_upright_tilt_deg(q), 0.0, 1e-9);

  // 10 deg tilt
  const double tilt_rad = 10.0 * (M_PI / 180.0);
  q.x = std::sin(tilt_rad / 2.0);
  q.w = std::cos(tilt_rad / 2.0);
  EXPECT_NEAR(ur5e_pick_place::compute_payload_upright_tilt_deg(q), 10.0, 1e-7);
}

// --- Tool tilt tests ------------------------------------------------------

TEST(TransportCoordinatorLogic, ComputeToolTiltDeg)
{
  // Nominal tool0 orientation: [sqrt(0.5), sqrt(0.5), 0, 0] pointing along world -Z
  const double s = std::sqrt(0.5);
  EXPECT_NEAR(ur5e_pick_place::compute_tool_tilt_deg(s, s, 0.0, 0.0), 0.0, 1e-7);

  // Sign-invariance: [-s, -s, 0, 0]
  EXPECT_NEAR(ur5e_pick_place::compute_tool_tilt_deg(-s, -s, 0.0, 0.0), 0.0, 1e-7);

  // Yaw-invariance: tool0 rotated around world Z
  for (double yaw_deg : {0.0, 30.0, 60.0, 90.0, 180.0, 270.0}) {
    const double yaw_rad = yaw_deg * (M_PI / 180.0);
    // Multiply nominal [s, s, 0, 0] by yaw quaternion [0, 0, sin(yaw/2), cos(yaw/2)]
    const double cz = std::cos(yaw_rad / 2.0);
    const double sz = std::sin(yaw_rad / 2.0);
    const double qx = cz * s - sz * s;
    const double qy = cz * s + sz * s;
    EXPECT_NEAR(ur5e_pick_place::compute_tool_tilt_deg(qx, qy, 0.0, 0.0), 0.0, 1e-7);
  }

  // Flipped upside down (pointing along world +Z): [0, 0, 0, 1]
  EXPECT_NEAR(ur5e_pick_place::compute_tool_tilt_deg(0.0, 0.0, 0.0, 1.0), 180.0, 1e-7);

  // geometry_msgs::msg::Quaternion overload
  geometry_msgs::msg::Quaternion q;
  q.x = s; q.y = s; q.z = 0.0; q.w = 0.0;
  EXPECT_NEAR(ur5e_pick_place::compute_tool_tilt_deg(q), 0.0, 1e-7);
}

// --- tilt_deg_from_up_dot_checked (nonfinite orientation guard) tests -----

TEST(TransportCoordinatorLogic, TiltFromUpDotRejectsNaN)
{
  double tilt_deg = -1.0;
  const bool ok = ur5e_pick_place::tilt_deg_from_up_dot_checked(
    std::numeric_limits<double>::quiet_NaN(), tilt_deg);
  EXPECT_FALSE(ok);
}

TEST(TransportCoordinatorLogic, TiltFromUpDotRejectsPositiveInfinity)
{
  double tilt_deg = -1.0;
  const bool ok = ur5e_pick_place::tilt_deg_from_up_dot_checked(
    std::numeric_limits<double>::infinity(), tilt_deg);
  EXPECT_FALSE(ok);
}

TEST(TransportCoordinatorLogic, TiltFromUpDotRejectsNegativeInfinity)
{
  double tilt_deg = -1.0;
  const bool ok = ur5e_pick_place::tilt_deg_from_up_dot_checked(
    -std::numeric_limits<double>::infinity(), tilt_deg);
  EXPECT_FALSE(ok);
}

TEST(TransportCoordinatorLogic, TiltFromUpDotAcceptsFiniteUpright)
{
  // dot = 1.0 -> local +Z exactly aligned with world +Z -> 0 deg tilt.
  double tilt_deg = -1.0;
  ASSERT_TRUE(ur5e_pick_place::tilt_deg_from_up_dot_checked(1.0, tilt_deg));
  EXPECT_NEAR(tilt_deg, 0.0, 1e-9);
}

TEST(TransportCoordinatorLogic, TiltFromUpDotAcceptsFiniteInverted)
{
  // dot = -1.0 -> complete inversion -> 180 deg tilt. Still finite, still
  // accepted by this helper -- the separate 2.0 deg threshold check in
  // validateCandidateTrajectory() is what rejects it, not this guard.
  double tilt_deg = -1.0;
  ASSERT_TRUE(ur5e_pick_place::tilt_deg_from_up_dot_checked(-1.0, tilt_deg));
  EXPECT_NEAR(tilt_deg, 180.0, 1e-9);
}

TEST(TransportCoordinatorLogic, TiltFromUpDotAcceptsFinite145Deg)
{
  // dot = cos(145 deg) -> the exact "large replacement tilt" audit value.
  // This must still convert correctly and finitely: the 2.0 deg candidate
  // gate remains a separate, unchanged comparison at the call site.
  const double dot_145 = std::cos(145.0 * (M_PI / 180.0));
  double tilt_deg = -1.0;
  ASSERT_TRUE(ur5e_pick_place::tilt_deg_from_up_dot_checked(dot_145, tilt_deg));
  EXPECT_NEAR(tilt_deg, 145.0, 1e-7);
  EXPECT_GT(tilt_deg, ur5e_pick_place::kMaxAllowedPayloadTiltDeg);
}

TEST(TransportCoordinatorLogic, TiltFromUpDotClampsOutOfRangeFiniteNoise)
{
  // Ordinary floating-point slop (e.g. 1.0 + 1e-10) must still clamp and
  // convert finitely -- only actual NaN/Inf are rejected by this guard.
  double tilt_deg = -1.0;
  ASSERT_TRUE(ur5e_pick_place::tilt_deg_from_up_dot_checked(1.0 + 1e-10, tilt_deg));
  EXPECT_NEAR(tilt_deg, 0.0, 1e-4);

  ASSERT_TRUE(ur5e_pick_place::tilt_deg_from_up_dot_checked(-1.0 - 1e-10, tilt_deg));
  EXPECT_NEAR(tilt_deg, 180.0, 1e-4);
}

// --- Orientation constraint construction tests ----------------------------

TEST(TransportCoordinatorLogic, CreateTransportOrientationConstraint)
{
  geometry_msgs::msg::Quaternion target_q;
  target_q.x = 0.7071; target_q.y = 0.7071; target_q.z = 0.0; target_q.w = 0.0;

  const auto c = ur5e_pick_place::create_transport_orientation_constraint(
    "tool0", target_q, 0.2443, 3.14159, "world");

  EXPECT_EQ(c.name, "transport_orientation_constraint");
  ASSERT_EQ(c.orientation_constraints.size(), 1u);

  const auto & oc = c.orientation_constraints[0];
  EXPECT_EQ(oc.link_name, "tool0");
  EXPECT_EQ(oc.header.frame_id, "world");
  EXPECT_NEAR(oc.orientation.x, 0.7071, 1e-4);
  EXPECT_NEAR(oc.orientation.y, 0.7071, 1e-4);
  EXPECT_NEAR(oc.absolute_x_axis_tolerance, 0.2443, 1e-4);
  EXPECT_NEAR(oc.absolute_y_axis_tolerance, 0.2443, 1e-4);
  EXPECT_NEAR(oc.absolute_z_axis_tolerance, 3.14159, 1e-4);
  EXPECT_DOUBLE_EQ(oc.weight, 1.0);
}

// --- ScopedPathConstraint RAII tests --------------------------------------

namespace
{
struct MockMoveGroup
{
  moveit_msgs::msg::Constraints current_constraints;
  int set_calls{0};
  int clear_calls{0};

  void setPathConstraints(const moveit_msgs::msg::Constraints & constraints)
  {
    current_constraints = constraints;
    ++set_calls;
  }

  void clearPathConstraints()
  {
    current_constraints = moveit_msgs::msg::Constraints{};
    ++clear_calls;
  }
};
}  // namespace

TEST(TransportCoordinatorLogic, ScopedPathConstraintRAII)
{
  MockMoveGroup mock_arm;
  EXPECT_EQ(mock_arm.set_calls, 0);
  EXPECT_EQ(mock_arm.clear_calls, 0);

  geometry_msgs::msg::Quaternion q;
  q.w = 1.0;
  const auto constraints = ur5e_pick_place::create_transport_orientation_constraint("tool0", q);

  {
    ur5e_pick_place::ScopedPathConstraintImpl<MockMoveGroup> scope(mock_arm, constraints);
    EXPECT_EQ(mock_arm.set_calls, 1);
    EXPECT_EQ(mock_arm.clear_calls, 0);
    EXPECT_EQ(mock_arm.current_constraints.name, "transport_orientation_constraint");
  }

  // After exiting scope, clearPathConstraints must have been called
  EXPECT_EQ(mock_arm.set_calls, 1);
  EXPECT_EQ(mock_arm.clear_calls, 1);
  EXPECT_TRUE(mock_arm.current_constraints.name.empty());
}

TEST(TransportCoordinatorLogic, ScopedPathConstraintExceptionSafety)
{
  MockMoveGroup mock_arm;
  geometry_msgs::msg::Quaternion q;
  q.w = 1.0;
  const auto constraints = ur5e_pick_place::create_transport_orientation_constraint("tool0", q);

  try {
    ur5e_pick_place::ScopedPathConstraintImpl<MockMoveGroup> scope(mock_arm, constraints);
    EXPECT_EQ(mock_arm.set_calls, 1);
    EXPECT_EQ(mock_arm.clear_calls, 0);
    throw std::runtime_error("simulated planning exception");
  } catch (const std::runtime_error &) {
    // Caught
  }

  // Destructor must have executed during stack unwinding
  EXPECT_EQ(mock_arm.set_calls, 1);
  EXPECT_EQ(mock_arm.clear_calls, 1);
  EXPECT_TRUE(mock_arm.current_constraints.name.empty());
}

// --- Stage-3C C3C attempt-aware telemetry predicate tests ------------------

TEST(TransportCoordinatorLogic, IsValidAttemptIndexAcceptsOnlyZeroAndOne)
{
  EXPECT_TRUE(ur5e_pick_place::is_valid_attempt_index(0));
  EXPECT_TRUE(ur5e_pick_place::is_valid_attempt_index(1));
  EXPECT_FALSE(ur5e_pick_place::is_valid_attempt_index(2));
  EXPECT_FALSE(ur5e_pick_place::is_valid_attempt_index(3));
  EXPECT_FALSE(ur5e_pick_place::is_valid_attempt_index(-1));
}

TEST(TransportCoordinatorLogic, AttemptStateLabelDistinguishesEFromE2)
{
  EXPECT_STREQ(ur5e_pick_place::attempt_state_label(0), "State E");
  EXPECT_STREQ(ur5e_pick_place::attempt_state_label(1), "State E2");
  // Never expected in production (is_valid_attempt_index() gates every
  // attempt the coordinator itself produces) but must not be silently
  // mislabeled as E or E2 if ever called with something else.
  EXPECT_STREQ(ur5e_pick_place::attempt_state_label(2), "INVALID_ATTEMPT");
  EXPECT_STREQ(ur5e_pick_place::attempt_state_label(-1), "INVALID_ATTEMPT");
}

TEST(TransportCoordinatorLogic, GoalAcceptanceCountOkRequiresExactlyOneEach)
{
  // The C3C target shape: exactly one attempt-0 goal, exactly one
  // attempt-1 (replacement) goal, no third. This predicate is scoped to
  // the two-goal C3C acceptance contract specifically (per its own header
  // doc) -- it is not a general "any valid run" check, so a C3B-shaped
  // run (attempt-0 only, no replacement ever sent) is correctly false
  // here, not true; see GoalAcceptanceCountOkRejectsMissingOrDuplicateGoals.
  EXPECT_TRUE(ur5e_pick_place::goal_acceptance_count_ok(1, 1, 0));
}

TEST(TransportCoordinatorLogic, GoalAcceptanceCountOkRejectsThirdGoal)
{
  // A third accepted goal (attempt index >= 2) must fail qualification
  // immediately, regardless of how attempt 0/1 look.
  EXPECT_FALSE(ur5e_pick_place::goal_acceptance_count_ok(1, 1, 1));
  EXPECT_FALSE(ur5e_pick_place::goal_acceptance_count_ok(1, 1, 2));
}

TEST(TransportCoordinatorLogic, GoalAcceptanceCountOkRejectsMissingOrDuplicateGoals)
{
  EXPECT_FALSE(ur5e_pick_place::goal_acceptance_count_ok(0, 1, 0));  // attempt 0 missing
  EXPECT_FALSE(ur5e_pick_place::goal_acceptance_count_ok(1, 2, 0));  // attempt 1 duplicated
  EXPECT_FALSE(ur5e_pick_place::goal_acceptance_count_ok(2, 1, 0));  // attempt 0 duplicated
  EXPECT_FALSE(ur5e_pick_place::goal_acceptance_count_ok(0, 0, 0));  // nothing accepted
}

TEST(TransportCoordinatorLogic, BudgetExhaustedTelemetryValidAcceptsOnlyTheExactContract)
{
  EXPECT_TRUE(
    ur5e_pick_place::budget_exhausted_telemetry_valid(
      1, 1, 1, ur5e_pick_place::Result::TRANSPORT_REPLAN_LIMIT_REACHED));
}

TEST(TransportCoordinatorLogic, BudgetExhaustedTelemetryValidRejectsWrongValues)
{
  using ur5e_pick_place::Result;
  // Wrong replan_count.
  EXPECT_FALSE(
    ur5e_pick_place::budget_exhausted_telemetry_valid(
      2, 1, 1, Result::TRANSPORT_REPLAN_LIMIT_REACHED));
  EXPECT_FALSE(
    ur5e_pick_place::budget_exhausted_telemetry_valid(
      0, 1, 1, Result::TRANSPORT_REPLAN_LIMIT_REACHED));
  // Wrong max_replans (this project's policy is fixed at 1).
  EXPECT_FALSE(
    ur5e_pick_place::budget_exhausted_telemetry_valid(
      1, 2, 1, Result::TRANSPORT_REPLAN_LIMIT_REACHED));
  // Wrong attempt -- only the replacement attempt (1) can exhaust budget.
  EXPECT_FALSE(
    ur5e_pick_place::budget_exhausted_telemetry_valid(
      1, 1, 0, Result::TRANSPORT_REPLAN_LIMIT_REACHED));
  EXPECT_FALSE(
    ur5e_pick_place::budget_exhausted_telemetry_valid(
      1, 1, 2, Result::TRANSPORT_REPLAN_LIMIT_REACHED));
  // Wrong result -- must be exactly TRANSPORT_REPLAN_LIMIT_REACHED.
  EXPECT_FALSE(
    ur5e_pick_place::budget_exhausted_telemetry_valid(1, 1, 1, Result::SUCCESS));
  EXPECT_FALSE(
    ur5e_pick_place::budget_exhausted_telemetry_valid(
      1, 1, 1, Result::TRANSPORT_COLLISION_STOPPED));
}

// --- Stage-3C C3 OPTIONAL pre-replan scene gate ----------------------------
//
// The handshake tests below run against a REAL rclcpp node and a REAL
// std_srvs/srv/Trigger server, spun by a SingleThreadedExecutor on its own
// dedicated thread -- deliberately the same executor model m3_grasp.cpp uses
// (executor on a std::thread, transport logic on the caller thread). That is
// what makes them evidence for the design's executor-safety claim: the
// caller-thread future wait must not starve the callback completing it.

namespace
{

class PreReplanGateFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("pre_replan_gate_test_node");
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spinner_ = std::thread([this]() {executor_->spin();});

    // Block until the executor is provably INSIDE spin() before any test body
    // runs. Without this, a test that finishes in microseconds can reach
    // TearDown's cancel() before spin() has started, in which case the cancel
    // is a no-op and spin() then blocks forever on join(). (Observed exactly
    // that: the one test doing no service work hung while the four slower ones
    // passed.) A one-shot timer firing is direct proof spin() is servicing work.
    auto spinning = std::make_shared<std::promise<void>>();
    auto fired = std::make_shared<std::atomic<bool>>(false);
    auto spinning_future = spinning->get_future();
    auto probe = node_->create_wall_timer(
      std::chrono::milliseconds(1),
      [spinning, fired]() {
        if (!fired->exchange(true)) {
          spinning->set_value();
        }
      });
    ASSERT_EQ(spinning_future.wait_for(std::chrono::seconds(5)), std::future_status::ready)
      << "executor never began spinning";
    probe->cancel();
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {
      spinner_.join();
    }
    server_.reset();
    client_.reset();
    node_.reset();
    executor_.reset();
  }

  // Advertises the Trigger service. `success` is the reply value; when
  // `hang` is true the callback never returns, forcing the caller's bounded
  // wait to expire.
  void advertise(const std::string & name, bool success, bool hang = false)
  {
    server_ = node_->create_service<std_srvs::srv::Trigger>(
      name,
      [this, success, hang](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        ++request_count_;
        if (hang) {
          // Block well past the caller's timeout without ever replying.
          std::this_thread::sleep_for(std::chrono::seconds(3));
        }
        res->success = success;
        res->message = success ? "transition complete" : "transition failed";
      });
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spinner_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr server_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
  std::atomic<int> request_count_{0};
};

}  // namespace

// 1. Gate disabled -> immediate no-op, and NO client is ever created.
TEST_F(PreReplanGateFixture, DisabledIsNoOpAndCreatesNoClient)
{
  double latency_ms = -1.0;
  const auto outcome = ur5e_pick_place::run_pre_replan_gate_handshake(
    node_, client_, /*service_name=*/"", /*timeout_s=*/5.0, latency_ms);

  EXPECT_EQ(outcome, ur5e_pick_place::PreReplanGateOutcome::DISABLED);
  EXPECT_TRUE(ur5e_pick_place::pre_replan_gate_ok(outcome));
  EXPECT_EQ(client_, nullptr) << "disabled gate must not construct a client";
  EXPECT_DOUBLE_EQ(latency_ms, 0.0);
  EXPECT_EQ(request_count_.load(), 0);
}

// 2. Enabled + server replies success=true -> SUCCESS.
// 7. Exactly one request is sent.
TEST_F(PreReplanGateFixture, EnabledSuccessSendsExactlyOneRequest)
{
  advertise("/gate_success", true);
  double latency_ms = -1.0;
  const auto outcome = ur5e_pick_place::run_pre_replan_gate_handshake(
    node_, client_, "/gate_success", 5.0, latency_ms);

  EXPECT_EQ(outcome, ur5e_pick_place::PreReplanGateOutcome::SUCCESS);
  EXPECT_TRUE(ur5e_pick_place::pre_replan_gate_ok(outcome));
  EXPECT_EQ(request_count_.load(), 1) << "the gate must send exactly one request";
  EXPECT_GE(latency_ms, 0.0);
}

// 3. Service never advertised -> SERVICE_UNAVAILABLE, bounded by the timeout.
TEST_F(PreReplanGateFixture, ServiceUnavailable)
{
  double latency_ms = -1.0;
  const auto outcome = ur5e_pick_place::run_pre_replan_gate_handshake(
    node_, client_, "/gate_never_advertised", /*timeout_s=*/0.4, latency_ms);

  EXPECT_EQ(outcome, ur5e_pick_place::PreReplanGateOutcome::SERVICE_UNAVAILABLE);
  EXPECT_FALSE(ur5e_pick_place::pre_replan_gate_ok(outcome));
  EXPECT_GE(latency_ms, 350.0) << "must actually wait out its bounded timeout";
  EXPECT_LT(latency_ms, 4000.0) << "must not exceed its bound materially";
}

// 4. Server accepts but never replies -> TIMEOUT (not a hang).
TEST_F(PreReplanGateFixture, RequestTimeout)
{
  advertise("/gate_hang", true, /*hang=*/true);
  double latency_ms = -1.0;
  const auto outcome = ur5e_pick_place::run_pre_replan_gate_handshake(
    node_, client_, "/gate_hang", /*timeout_s=*/0.5, latency_ms);

  EXPECT_EQ(outcome, ur5e_pick_place::PreReplanGateOutcome::TIMEOUT);
  EXPECT_FALSE(ur5e_pick_place::pre_replan_gate_ok(outcome));
  EXPECT_LT(latency_ms, 2500.0) << "must return on its own deadline, not the server's";
}

// 5. Server replies success=false -> SUCCESS_FALSE.
TEST_F(PreReplanGateFixture, ResponseSuccessFalse)
{
  advertise("/gate_refuses", false);
  double latency_ms = -1.0;
  const auto outcome = ur5e_pick_place::run_pre_replan_gate_handshake(
    node_, client_, "/gate_refuses", 5.0, latency_ms);

  EXPECT_EQ(outcome, ur5e_pick_place::PreReplanGateOutcome::SUCCESS_FALSE);
  EXPECT_FALSE(ur5e_pick_place::pre_replan_gate_ok(outcome));
  EXPECT_EQ(request_count_.load(), 1);
}

// 6. Every failure outcome is distinctly named, including SHUTDOWN, so gate
//    failures never surface as an undifferentiated error.
TEST(TransportCoordinatorLogic, PreReplanGateOutcomeNames)
{
  using ur5e_pick_place::PreReplanGateOutcome;
  EXPECT_STREQ(ur5e_pick_place::to_string(PreReplanGateOutcome::DISABLED), "DISABLED");
  EXPECT_STREQ(ur5e_pick_place::to_string(PreReplanGateOutcome::SUCCESS), "SUCCESS");
  EXPECT_STREQ(
    ur5e_pick_place::to_string(PreReplanGateOutcome::SERVICE_UNAVAILABLE),
    "SERVICE_UNAVAILABLE");
  EXPECT_STREQ(ur5e_pick_place::to_string(PreReplanGateOutcome::SEND_FAILED), "SEND_FAILED");
  EXPECT_STREQ(ur5e_pick_place::to_string(PreReplanGateOutcome::TIMEOUT), "TIMEOUT");
  EXPECT_STREQ(ur5e_pick_place::to_string(PreReplanGateOutcome::SUCCESS_FALSE), "SUCCESS_FALSE");
  EXPECT_STREQ(ur5e_pick_place::to_string(PreReplanGateOutcome::SHUTDOWN), "SHUTDOWN");

  // Only DISABLED and SUCCESS may let the replan proceed.
  EXPECT_TRUE(ur5e_pick_place::pre_replan_gate_ok(PreReplanGateOutcome::DISABLED));
  EXPECT_TRUE(ur5e_pick_place::pre_replan_gate_ok(PreReplanGateOutcome::SUCCESS));
  for (auto bad : {PreReplanGateOutcome::SERVICE_UNAVAILABLE,
      PreReplanGateOutcome::SEND_FAILED, PreReplanGateOutcome::TIMEOUT,
      PreReplanGateOutcome::SUCCESS_FALSE, PreReplanGateOutcome::SHUTDOWN})
  {
    EXPECT_FALSE(ur5e_pick_place::pre_replan_gate_ok(bad));
  }
}

// 14. Default (no gate) SCENE_A behavior is preserved exactly: with no gate,
//     t_gate_done is 0.0 and the baseline is t_settle unchanged.
TEST(TransportCoordinatorLogic, SceneABaselineDefaultIsSettleOnly)
{
  EXPECT_DOUBLE_EQ(ur5e_pick_place::scene_a_freshness_baseline_s(35.384, 0.0), 35.384);
  EXPECT_DOUBLE_EQ(ur5e_pick_place::scene_a_freshness_baseline_s(0.0, 0.0), 0.0);
}

// 13. baseline == max(t_settle, t_gate_done).
TEST(TransportCoordinatorLogic, SceneABaselineIsMaxOfSettleAndGate)
{
  EXPECT_DOUBLE_EQ(ur5e_pick_place::scene_a_freshness_baseline_s(35.384, 36.500), 36.500);
  // A gate that completed BEFORE the settle must never lower the bar.
  EXPECT_DOUBLE_EQ(ur5e_pick_place::scene_a_freshness_baseline_s(35.384, 35.000), 35.384);
  // Equal values are stable.
  EXPECT_DOUBLE_EQ(ur5e_pick_place::scene_a_freshness_baseline_s(35.384, 35.384), 35.384);
}

// 11. An obstacle update that postdates the settle but PREDATES the gate is
//     rejected -- the defect the invariant exists to prevent.
// 12. A post-gate update is accepted.
TEST(TransportCoordinatorLogic, PostGateFreshnessRejectsPreGateUpdate)
{
  const double t_settle = 35.384;
  const double t_gate_done = 36.500;
  const double baseline = ur5e_pick_place::scene_a_freshness_baseline_s(t_settle, t_gate_done);
  const double stale_thresh = 0.250;

  // Post-settle but pre-gate (36.100): would have satisfied the OLD baseline,
  // must be rejected under the gated one.
  EXPECT_TRUE(
    ur5e_pick_place::is_obstacle_fresh_post_time(36.100, t_settle, 36.200, stale_thresh));
  EXPECT_FALSE(
    ur5e_pick_place::is_obstacle_fresh_post_time(36.100, baseline, 36.200, stale_thresh));

  // Post-gate and fresh (36.520, age 20 ms) -> accepted.
  EXPECT_TRUE(
    ur5e_pick_place::is_obstacle_fresh_post_time(36.520, baseline, 36.540, stale_thresh));

  // Post-gate but stale (age 300 ms) -> still rejected; the gate does not
  // relax the 250 ms staleness cap.
  EXPECT_FALSE(
    ur5e_pick_place::is_obstacle_fresh_post_time(36.520, baseline, 36.820, stale_thresh));

  // The gate response itself never establishes scene authority: an update
  // exactly AT the gate timestamp still has to pass the staleness cap.
  EXPECT_TRUE(
    ur5e_pick_place::is_obstacle_fresh_post_time(t_gate_done, baseline, 36.600, stale_thresh));
}

// 18. The gate-failure Result is its own typed cause and is NOT success, so
//     lift_transport_place()'s existing `!ok(...)` early return suppresses
//     PLACE/release/detach/retreat without any duplicated stage logic.
TEST(TransportCoordinatorLogic, ResultPreReplanGateFailedIsTypedFailure)
{
  EXPECT_STREQ(
    ur5e_pick_place::to_string(ur5e_pick_place::Result::TRANSPORT_PRE_REPLAN_GATE_FAILED),
    "TRANSPORT_PRE_REPLAN_GATE_FAILED");
  EXPECT_FALSE(ur5e_pick_place::ok(ur5e_pick_place::Result::TRANSPORT_PRE_REPLAN_GATE_FAILED));
  // Deliberately distinct from a scene-integrity failure.
  EXPECT_NE(ur5e_pick_place::Result::TRANSPORT_PRE_REPLAN_GATE_FAILED, ur5e_pick_place::Result::SCENE_STALE_OR_CORRUPT);
  EXPECT_NE(ur5e_pick_place::Result::TRANSPORT_PRE_REPLAN_GATE_FAILED, ur5e_pick_place::Result::SUCCESS);
}


// ===========================================================================
// Stage-3C C3 STATIC-CLOSEOUT CORRECTION A — gate send exception
// ===========================================================================
//
// Root cause these tests pin down: rclcpp reports a request-send failure by
// THROWING out of Client<T>::async_send_request() (rcl_send_request() ->
// rclcpp::exceptions::throw_from_rcl_error(), client.hpp:636-645 / :841-855).
// The future it returns is built from a promise created immediately before the
// send, so it is always valid on a normal return -- the old post-call
// future.valid() check could never observe a send failure, and the throw would
// have escaped executeTransport() instead of becoming a typed gate result.
//
// These tests exercise the ACTUAL THROW PATH, not future.valid()==false: they
// inject a send that raises, through the production handshake, and require the
// typed chain SEND_FAILED -> "SEND_FAILED" reason ->
// Result::TRANSPORT_PRE_REPLAN_GATE_FAILED with zero requests reaching the
// server.

namespace
{
using TriggerFuture = rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId;

// One throwing send per std base class throw_from_rcl_error() can produce:
//   RCLError          : std::runtime_error
//   RCLBadAlloc       : std::bad_alloc
//   RCLInvalidArgument: std::invalid_argument
// (exceptions.hpp:152-183). Plus a non-std throw, which no documented rclcpp
// path produces but which still must not escape.
enum class ThrowKind { RUNTIME_ERROR, BAD_ALLOC, INVALID_ARGUMENT, NON_STD };

ur5e_pick_place::GateRequestSendFn makeThrowingSender(
  ThrowKind kind, std::shared_ptr<std::atomic<int>> call_count)
{
  return [kind, call_count](
    rclcpp::Client<std_srvs::srv::Trigger> &,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> &) -> TriggerFuture {
      ++(*call_count);
      switch (kind) {
        case ThrowKind::RUNTIME_ERROR:
          throw std::runtime_error("failed to send request");
        case ThrowKind::BAD_ALLOC:
          throw std::bad_alloc();
        case ThrowKind::INVALID_ARGUMENT:
          throw std::invalid_argument("failed to send request, invalid argument");
        case ThrowKind::NON_STD:
        default:
          throw 42;
      }
    };
}
}  // namespace

// A1. Every documented rclcpp send-failure exception type becomes SEND_FAILED,
//     through the REAL handshake, and nothing escapes executeTransport()'s
//     call boundary.
TEST_F(PreReplanGateFixture, SendThrowBecomesSendFailedForEveryExceptionType)
{
  advertise("/gate_send_throws", true);

  for (auto kind : {ThrowKind::RUNTIME_ERROR, ThrowKind::BAD_ALLOC,
      ThrowKind::INVALID_ARGUMENT, ThrowKind::NON_STD})
  {
    auto sends = std::make_shared<std::atomic<int>>(0);
    double latency_ms = -1.0;
    ur5e_pick_place::PreReplanGateOutcome outcome =
      ur5e_pick_place::PreReplanGateOutcome::SUCCESS;

    ASSERT_NO_THROW({
      outcome = ur5e_pick_place::run_pre_replan_gate_handshake(
        node_, client_, "/gate_send_throws", /*timeout_s=*/2.0, latency_ms,
        makeThrowingSender(kind, sends));
    }) << "a send failure must never escape the gate call boundary";

    EXPECT_EQ(outcome, ur5e_pick_place::PreReplanGateOutcome::SEND_FAILED);
    EXPECT_FALSE(ur5e_pick_place::pre_replan_gate_ok(outcome));
    EXPECT_EQ(sends->load(), 1) << "the send is attempted exactly once";
    EXPECT_GE(latency_ms, 0.0) << "latency accounting still runs on the throw path";
  }

  // The service exists and was reachable, yet no request ever reached it:
  // the failure happened AT the send, and no retry was issued.
  EXPECT_EQ(request_count_.load(), 0);
}

// A2. The full typed chain, each link asserted on the production function that
//     implements it:
//       throw -> SEND_FAILED -> reason text "SEND_FAILED"
//             -> Result::TRANSPORT_PRE_REPLAN_GATE_FAILED (not ok())
TEST_F(PreReplanGateFixture, SendThrowYieldsTypedGateFailedResultAndReason)
{
  advertise("/gate_send_throws_chain", true);
  auto sends = std::make_shared<std::atomic<int>>(0);
  double latency_ms = -1.0;

  const auto outcome = ur5e_pick_place::run_pre_replan_gate_handshake(
    node_, client_, "/gate_send_throws_chain", 2.0, latency_ms,
    makeThrowingSender(ThrowKind::RUNTIME_ERROR, sends));

  ASSERT_EQ(outcome, ur5e_pick_place::PreReplanGateOutcome::SEND_FAILED);
  // runPreReplanSceneGate() writes exactly to_string(outcome) into
  // telemetry_.pre_replan_gate_failure_reason and into the
  // "PRE_REPLAN_SCENE_GATE_FAILED ... reason=%s" line.
  EXPECT_STREQ(ur5e_pick_place::to_string(outcome), "SEND_FAILED");
  // executeTransport()'s own mapping, not a restatement of it.
  EXPECT_EQ(
    ur5e_pick_place::pre_replan_gate_result(outcome),
    ur5e_pick_place::Result::TRANSPORT_PRE_REPLAN_GATE_FAILED);
  EXPECT_FALSE(ur5e_pick_place::ok(ur5e_pick_place::pre_replan_gate_result(outcome)));
  // Not remapped onto a scene-integrity cause, and never SUCCESS.
  EXPECT_NE(
    ur5e_pick_place::pre_replan_gate_result(outcome),
    ur5e_pick_place::Result::SCENE_STALE_OR_CORRUPT);
  EXPECT_EQ(request_count_.load(), 0) << "zero replacement sends: the request never left";
}

// A3. The guard itself: a throwing send yields THREW, captures the exception
//     text, and leaves no future behind. A succeeding send yields OK.
TEST(TransportCoordinatorLogic, SendGuardConvertsThrowAndPassesThroughSuccess)
{
  // Minimal stand-in for rclcpp's FutureAndRequestId: move-only, not
  // default-constructible, exposing valid() -- the only surface the guard uses.
  struct FakeFuture
  {
    bool is_valid;
    explicit FakeFuture(bool v) : is_valid(v) {}
    FakeFuture(FakeFuture &&) = default;
    FakeFuture & operator=(FakeFuture &&) = default;
    FakeFuture(const FakeFuture &) = delete;
    FakeFuture & operator=(const FakeFuture &) = delete;
    bool valid() const noexcept { return is_valid; }
  };

  std::optional<FakeFuture> fake_future;
  std::string err;

  auto threw = ur5e_pick_place::send_gate_request_guarded(
    []() -> FakeFuture { throw std::runtime_error("failed to send request"); },
    fake_future, err);
  EXPECT_EQ(threw, ur5e_pick_place::GateSendStatus::THREW);
  EXPECT_EQ(err, "failed to send request") << "the exception text must be reported";
  EXPECT_FALSE(fake_future.has_value());

  auto non_std = ur5e_pick_place::send_gate_request_guarded(
    []() -> FakeFuture { throw 7; }, fake_future, err);
  EXPECT_EQ(non_std, ur5e_pick_place::GateSendStatus::THREW);
  EXPECT_FALSE(err.empty());

  // A future that is somehow not valid is still not accepted as a good send.
  auto invalid = ur5e_pick_place::send_gate_request_guarded(
    []() -> FakeFuture { return FakeFuture(false); }, fake_future, err);
  EXPECT_EQ(invalid, ur5e_pick_place::GateSendStatus::INVALID_FUTURE);
  EXPECT_FALSE(err.empty());

  auto ok_status = ur5e_pick_place::send_gate_request_guarded(
    []() -> FakeFuture { return FakeFuture(true); }, fake_future, err);
  EXPECT_EQ(ok_status, ur5e_pick_place::GateSendStatus::OK);
  ASSERT_TRUE(fake_future.has_value());
  EXPECT_TRUE(fake_future->valid());
  EXPECT_TRUE(err.empty());

  EXPECT_STREQ(ur5e_pick_place::to_string(ur5e_pick_place::GateSendStatus::OK), "OK");
  EXPECT_STREQ(ur5e_pick_place::to_string(ur5e_pick_place::GateSendStatus::THREW), "THREW");
  EXPECT_STREQ(
    ur5e_pick_place::to_string(ur5e_pick_place::GateSendStatus::INVALID_FUTURE),
    "INVALID_FUTURE");
}

// ===========================================================================
// Stage-3C C3 STATIC-CLOSEOUT CORRECTION B — nonfinite pose coherence
// ===========================================================================
//
// Root cause: with a NaN component, pos_err is NaN, `pos_err > pos_tol_m` is
// FALSE, std::clamp() leaves NaN untouched (both its comparisons are false for
// NaN), acos(NaN) is NaN, and `angle_err <= max_angle_error_rad` is also FALSE
// -- so the function fell through to `return true`. Every case below was
// reported COHERENT before the fix.

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kPosInf = std::numeric_limits<double>::infinity();
constexpr double kNegInf = -std::numeric_limits<double>::infinity();

geometry_msgs::msg::Pose finitePose(double x = 1.0, double y = 2.0, double z = 3.0)
{
  geometry_msgs::msg::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = z;
  p.orientation.w = 1.0;
  return p;
}
}  // namespace

TEST(TransportCoordinatorLogic, PoseCoherenceRejectsNonFinitePosition)
{
  const auto expected = finitePose();
  for (double bad : {kNaN, kPosInf, kNegInf}) {
    auto px = finitePose(); px.position.x = bad;
    auto py = finitePose(); py.position.y = bad;
    auto pz = finitePose(); pz.position.z = bad;
    for (const auto & scene : {px, py, pz}) {
      EXPECT_FALSE(ur5e_pick_place::is_pose_finite(scene));
      EXPECT_FALSE(ur5e_pick_place::is_pose_coherent(scene, expected))
        << "nonfinite position must be incoherent";
      // Symmetric: nonfinite on the EXPECTED side is rejected too.
      EXPECT_FALSE(ur5e_pick_place::is_pose_coherent(expected, scene));
    }
  }
}

TEST(TransportCoordinatorLogic, PoseCoherenceRejectsNonFiniteQuaternion)
{
  const auto expected = finitePose();
  for (double bad : {kNaN, kPosInf, kNegInf}) {
    auto qx = finitePose(); qx.orientation.x = bad;
    auto qy = finitePose(); qy.orientation.y = bad;
    auto qz = finitePose(); qz.orientation.z = bad;
    auto qw = finitePose(); qw.orientation.w = bad;
    for (const auto & scene : {qx, qy, qz, qw}) {
      EXPECT_FALSE(ur5e_pick_place::is_pose_finite(scene));
      EXPECT_FALSE(ur5e_pick_place::is_pose_coherent(scene, expected))
        << "nonfinite quaternion must be incoherent";
      EXPECT_FALSE(ur5e_pick_place::is_pose_coherent(expected, scene));
    }
  }
}

// Finite controls: the valid-data tolerances are UNCHANGED (0.1 mm position,
// 1e-3 rad orientation) and the quaternion comparison is still sign-invariant.
TEST(TransportCoordinatorLogic, PoseCoherenceFiniteControlsUnchanged)
{
  const auto expected = finitePose();
  EXPECT_TRUE(ur5e_pick_place::is_pose_finite(expected));
  EXPECT_TRUE(ur5e_pick_place::is_pose_coherent(expected, expected));

  auto within = finitePose();
  within.position.x += 0.00009;  // 0.09 mm < 0.1 mm
  EXPECT_TRUE(ur5e_pick_place::is_pose_coherent(within, expected));

  auto beyond = finitePose();
  beyond.position.x += 0.0002;  // 0.2 mm > 0.1 mm
  EXPECT_FALSE(ur5e_pick_place::is_pose_coherent(beyond, expected));

  auto negated = finitePose();
  negated.orientation.w = -1.0;  // q == -q
  EXPECT_TRUE(ur5e_pick_place::is_pose_coherent(negated, expected));

  auto rotated = finitePose();
  rotated.orientation.w = std::cos(0.01 / 2.0);   // 0.01 rad > 1e-3 rad
  rotated.orientation.z = std::sin(0.01 / 2.0);
  EXPECT_FALSE(ur5e_pick_place::is_pose_coherent(rotated, expected));
}

// ===========================================================================
// Stage-3C C3 STATIC-CLOSEOUT CORRECTION C — response-time freshness
// ===========================================================================
//
// The invariant is evaluated at SCENE ACCEPTANCE, not at sample selection.
// decide_scene_acceptance() below is the exact function
// acquireFreshCoherentScene() calls; the 250 ms threshold is unchanged.

using ur5e_pick_place::SceneAcceptanceDecision;

// C-A. Fresh when the request went out, stale (>250 ms) when the response is
//      accepted -> the scene is REJECTED.
TEST(TransportCoordinatorLogic, SceneRejectedWhenSampleExpiresBeforeAcceptance)
{
  const double baseline = 36.124;
  const double sample = 36.200;         // 76 ms after the baseline
  const double request_time = 36.250;   // age 50 ms  -> fresh at selection
  const double accept_time = 36.500;    // age 300 ms -> stale at acceptance

  EXPECT_TRUE(
    ur5e_pick_place::is_obstacle_fresh_post_time(sample, baseline, request_time, 0.250));
  EXPECT_FALSE(
    ur5e_pick_place::is_sample_fresh_at_scene_acceptance(sample, baseline, accept_time, 0.250));
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(sample, baseline, accept_time, /*budget=*/0.5),
    SceneAcceptanceDecision::RETRY_NEWER_SAMPLE);
}

// C-B. Still <= 250 ms at acceptance -> ACCEPTED. Boundary is inclusive, and a
//      negative age (response stamped before the sample) is never "fresh".
TEST(TransportCoordinatorLogic, SceneAcceptedWhenSampleStillFreshAtAcceptance)
{
  const double baseline = 36.124;
  const double sample = 36.200;

  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(sample, baseline, 36.400, 0.5),
    SceneAcceptanceDecision::ACCEPT);                       // age 200 ms
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(sample, baseline, 36.450, 0.5),
    SceneAcceptanceDecision::ACCEPT);                       // age exactly 250 ms
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(sample, baseline, 36.451, 0.5),
    SceneAcceptanceDecision::RETRY_NEWER_SAMPLE);           // age 251 ms
  EXPECT_FALSE(
    ur5e_pick_place::is_sample_fresh_at_scene_acceptance(sample, baseline, 36.100, 0.250));

  // The baseline half of the invariant still binds at acceptance time: a
  // sample that predates the causal baseline is never accepted, however fresh.
  EXPECT_FALSE(
    ur5e_pick_place::is_sample_fresh_at_scene_acceptance(36.000, baseline, 36.010, 0.250));
}

// C-C. First sample expires but budget remains -> retry with a NEWER sample,
//      which may then succeed.
TEST(TransportCoordinatorLogic, ExpiredSampleRetriesWithNewerSampleInsideBudget)
{
  const double baseline = 36.124;

  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(
      /*sample=*/36.200, baseline, /*accept=*/36.600, /*budget_remaining=*/0.9),
    SceneAcceptanceDecision::RETRY_NEWER_SAMPLE);

  // The newer sample, accepted at the same instant, is inside the authority.
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(
      /*sample=*/36.550, baseline, /*accept=*/36.600, /*budget_remaining=*/0.4),
    SceneAcceptanceDecision::ACCEPT);
}

// C-D. Samples keep expiring until the budget runs out -> terminal, and the
//      retry is bounded: budget_remaining <= 0 can only be BUDGET_EXHAUSTED.
TEST(TransportCoordinatorLogic, ExpiredSampleAtExhaustedBudgetIsTerminal)
{
  const double baseline = 36.124;
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(36.200, baseline, 36.600, /*budget=*/0.0),
    SceneAcceptanceDecision::BUDGET_EXHAUSTED);
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(36.200, baseline, 36.600, /*budget=*/-0.2),
    SceneAcceptanceDecision::BUDGET_EXHAUSTED);
  // A still-fresh sample is accepted even with no budget left -- the budget
  // only bounds RETRIES, it never rejects a valid snapshot.
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(36.550, baseline, 36.600, /*budget=*/0.0),
    SceneAcceptanceDecision::ACCEPT);
}

// C-E. The post-gate SCENE_A baseline is still max(t_settle, t_gate_done), and
// C-F. SCENE_B's baseline is still t_plan_done -- neither causal timestamp
//      requirement is relaxed by the acceptance-time check.
TEST(TransportCoordinatorLogic, AcceptanceCheckDoesNotRelaxCausalBaselines)
{
  const double t_settle = 34.394;
  const double t_gate_done = 36.124;
  const double baseline_a =
    ur5e_pick_place::scene_a_freshness_baseline_s(t_settle, t_gate_done);
  EXPECT_DOUBLE_EQ(baseline_a, 36.124);

  // Post-settle but pre-gate, and perfectly fresh at acceptance: still rejected.
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(35.500, baseline_a, 35.510, 1.0),
    SceneAcceptanceDecision::RETRY_NEWER_SAMPLE);
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(36.130, baseline_a, 36.140, 1.0),
    SceneAcceptanceDecision::ACCEPT);

  // SCENE_B: baseline is t_plan_done; a pre-plan sample cannot authorize it.
  const double t_plan_done = 47.946;
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(47.900, t_plan_done, 47.910, 1.0),
    SceneAcceptanceDecision::RETRY_NEWER_SAMPLE);
  EXPECT_EQ(
    ur5e_pick_place::decide_scene_acceptance(47.960, t_plan_done, 47.996, 1.0),
    SceneAcceptanceDecision::ACCEPT);
}

// ===========================================================================
// Stage-3C C3 STATIC-CLOSEOUT CORRECTION D — same-snapshot scene integrity
// ===========================================================================
//
// verify_retained_snapshot_obstacle_and_attachment() reads ONLY the snapshot
// passed to it. The tests below build TWO snapshots -- a defective "A" that the
// coordinator retains and a correct "B" that a service call would have
// returned -- and require that validating A fails, i.e. that the authority
// cannot be satisfied by B.

namespace
{
geometry_msgs::msg::Pose obstaclePose()
{
  geometry_msgs::msg::Pose p;
  p.position.x = 0.450;
  p.position.y = -1.2081;
  p.position.z = 0.860;
  p.orientation.w = 1.0;
  return p;
}

moveit_msgs::msg::PlanningScene makeCorrectSnapshot(
  const geometry_msgs::msg::Pose & obstacle_pose)
{
  moveit_msgs::msg::PlanningScene scene;

  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.id = "dynamic_obstacle_0";
  obstacle.header.frame_id = "world";
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.05, 0.05, 0.10};
  obstacle.primitives.push_back(box);
  obstacle.primitive_poses.push_back(obstacle_pose);
  obstacle.pose.orientation.w = 1.0;
  scene.world.collision_objects.push_back(obstacle);

  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "gripper_base_link";
  attached.object.id = "pick_target";
  attached.object.header.frame_id = "world";
  attached.touch_links = ur5e_pick_place::PlanningSceneManager::padTouchLinks();
  scene.robot_state.attached_collision_objects.push_back(attached);

  return scene;
}
}  // namespace

// D-control. A correct snapshot passes.
TEST(TransportCoordinatorLogic, RetainedSnapshotCorrectControlPasses)
{
  const auto pose = obstaclePose();
  const auto snapshot = makeCorrectSnapshot(pose);
  std::string error;
  EXPECT_TRUE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      snapshot, pose, error)) << error;
  EXPECT_TRUE(error.empty());
}

// D1. Snapshot A has the correct obstacle but BAD touch links; snapshot B (the
//     one a service call would return) is correct. Validating A must FAIL --
//     it must not pass by validating B.
TEST(TransportCoordinatorLogic, RetainedSnapshotWithBadTouchLinksFailsDespiteCorrectServiceScene)
{
  const auto pose = obstaclePose();
  auto snapshot_a = makeCorrectSnapshot(pose);
  snapshot_a.robot_state.attached_collision_objects[0].touch_links =
    {"pad_fixed_link"};  // truncated, not the expected pad set
  const auto snapshot_b = makeCorrectSnapshot(pose);  // what the service would return

  std::string error_a;
  EXPECT_FALSE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      snapshot_a, pose, error_a));
  EXPECT_NE(error_a.find("SNAPSHOT_TOUCH_LINKS_MISMATCH"), std::string::npos) << error_a;

  std::string error_b;
  EXPECT_TRUE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      snapshot_b, pose, error_b)) << error_b;
}

// D2. MISSING touch links, missing attachment, and a wrong parent link are all
//     rejected on the retained snapshot.
TEST(TransportCoordinatorLogic, RetainedSnapshotAttachmentDefectsRejected)
{
  const auto pose = obstaclePose();
  std::string error;

  auto no_touch_links = makeCorrectSnapshot(pose);
  no_touch_links.robot_state.attached_collision_objects[0].touch_links.clear();
  EXPECT_FALSE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      no_touch_links, pose, error));
  EXPECT_NE(error.find("SNAPSHOT_TOUCH_LINKS_MISMATCH"), std::string::npos) << error;

  auto wrong_parent = makeCorrectSnapshot(pose);
  wrong_parent.robot_state.attached_collision_objects[0].link_name = "tool0";
  EXPECT_FALSE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      wrong_parent, pose, error));
  EXPECT_NE(error.find("SNAPSHOT_ATTACH_LINK_MISMATCH"), std::string::npos) << error;

  auto detached = makeCorrectSnapshot(pose);
  detached.robot_state.attached_collision_objects.clear();
  EXPECT_FALSE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      detached, pose, error));
  EXPECT_NE(error.find("SNAPSHOT_TARGET_NOT_ATTACHED"), std::string::npos) << error;
}

// D3. The obstacle must be present in, and coherent with, the RETAINED
//     snapshot -- including the CORRECTION B nonfinite case.
TEST(TransportCoordinatorLogic, RetainedSnapshotObstacleDefectsRejected)
{
  const auto pose = obstaclePose();
  std::string error;

  auto absent = makeCorrectSnapshot(pose);
  absent.world.collision_objects.clear();
  EXPECT_FALSE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      absent, pose, error));
  EXPECT_NE(error.find("SNAPSHOT_OBSTACLE_ABSENT"), std::string::npos) << error;

  auto moved = makeCorrectSnapshot(pose);
  moved.world.collision_objects[0].primitive_poses[0].position.y += 0.05;
  EXPECT_FALSE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      moved, pose, error));
  EXPECT_NE(error.find("SNAPSHOT_OBSTACLE_INCOHERENT"), std::string::npos) << error;

  auto nonfinite = makeCorrectSnapshot(pose);
  nonfinite.world.collision_objects[0].primitive_poses[0].position.y = kNaN;
  EXPECT_FALSE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      nonfinite, pose, error));
  EXPECT_NE(error.find("SNAPSHOT_OBSTACLE_INCOHERENT"), std::string::npos) << error;
}

// D4. Validation does not mutate the snapshot it inspects.
TEST(TransportCoordinatorLogic, RetainedSnapshotValidationDoesNotMutate)
{
  const auto pose = obstaclePose();
  const auto snapshot = makeCorrectSnapshot(pose);
  auto before = snapshot;
  std::string error;
  EXPECT_TRUE(ur5e_pick_place::verify_retained_snapshot_obstacle_and_attachment(
      snapshot, pose, error));
  EXPECT_EQ(snapshot.world.collision_objects.size(), before.world.collision_objects.size());
  EXPECT_EQ(
    snapshot.robot_state.attached_collision_objects[0].touch_links,
    before.robot_state.attached_collision_objects[0].touch_links);
  EXPECT_DOUBLE_EQ(
    snapshot.world.collision_objects[0].primitive_poses[0].position.y,
    before.world.collision_objects[0].primitive_poses[0].position.y);
}

}  // namespace
