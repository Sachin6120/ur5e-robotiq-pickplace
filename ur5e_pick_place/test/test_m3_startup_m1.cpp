// test_m3_startup_m1.cpp — focused unit tests for m3_grasp startup state acquisition and M1 verification.
//
// Verifies:
// A. No / incomplete JointState sample does NOT evaluate default zero state as valid robot state:
//    - nullptr sample (timeout) returns Result::STARTUP_STATE_UNAVAILABLE
//    - empty or incomplete arm joints message returns Result::STARTUP_STATE_UNAVAILABLE
//    - unpopulated all-zero default state fails closed with Result::STARTUP_NOT_AT_M1
// B. Genuine M1 state (including out-of-order joint names) evaluates to Result::SUCCESS (STARTUP_M1_VERIFIED)
// C. Non-M1 physical state evaluates to Result::STARTUP_NOT_AT_M1
// D. Dimension/configuration mismatch fails closed with Result::CONFIG_ERROR

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#define UR5E_PICK_PLACE_M3_GRASP_UNIT_TEST
#include "../src/m3_grasp.cpp"  // NOLINT(build/include)

namespace
{

const std::vector<std::string> kArmJointNames = {
  "shoulder_pan_joint",
  "shoulder_lift_joint",
  "elbow_joint",
  "wrist_1_joint",
  "wrist_2_joint",
  "wrist_3_joint"
};

const std::vector<double> kAuthoritativeM1 = {
  0.5, -1.2, 1.0, -1.4, -1.5708, 0.0
};

constexpr double kStartupToleranceRad = 0.01;

sensor_msgs::msg::JointState::SharedPtr make_joint_state(
  const std::vector<std::string> & names,
  const std::vector<double> & positions)
{
  auto js = std::make_shared<sensor_msgs::msg::JointState>();
  js->name = names;
  js->position = positions;
  return js;
}

// Test 1: Timeout with nullptr sample returns STARTUP_STATE_UNAVAILABLE
TEST(M3StartupM1, TimeoutNullSampleReturnsStartupStateUnavailable)
{
  const sensor_msgs::msg::JointState::ConstSharedPtr null_sample = nullptr;
  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    null_sample, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::STARTUP_STATE_UNAVAILABLE);
  EXPECT_TRUE(std::isinf(max_error));
}

// Test 2: Empty JointState message returns STARTUP_STATE_UNAVAILABLE
TEST(M3StartupM1, EmptyJointStateReturnsStartupStateUnavailable)
{
  auto empty_js = std::make_shared<sensor_msgs::msg::JointState>();
  EXPECT_FALSE(joint_state_has_joints(*empty_js, kArmJointNames));
  EXPECT_FALSE(extract_joint_positions(*empty_js, kArmJointNames).has_value());

  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    empty_js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::STARTUP_STATE_UNAVAILABLE);
  EXPECT_TRUE(std::isinf(max_error));
}

// Test 3: Incomplete arm joints message returns STARTUP_STATE_UNAVAILABLE
TEST(M3StartupM1, IncompleteArmJointsSampleReturnsStartupStateUnavailable)
{
  // Partial message with only gripper joint
  auto gripper_js = make_joint_state({"robotiq_85_left_knuckle_joint"}, {0.0});
  EXPECT_FALSE(joint_state_has_joints(*gripper_js, kArmJointNames));
  EXPECT_FALSE(extract_joint_positions(*gripper_js, kArmJointNames).has_value());

  double max_error = -1.0;
  Result res = evaluate_startup_sample_and_m1(
    gripper_js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::STARTUP_STATE_UNAVAILABLE);
  EXPECT_TRUE(std::isinf(max_error));

  // Partial message with 5 of 6 arm joints
  std::vector<std::string> partial_names(kArmJointNames.begin(), kArmJointNames.begin() + 5);
  std::vector<double> partial_pos = {0.5, -1.2, 1.0, -1.4, -1.5708};
  auto partial_js = make_joint_state(partial_names, partial_pos);
  EXPECT_FALSE(joint_state_has_joints(*partial_js, kArmJointNames));
  EXPECT_FALSE(extract_joint_positions(*partial_js, kArmJointNames).has_value());

  res = evaluate_startup_sample_and_m1(
    partial_js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::STARTUP_STATE_UNAVAILABLE);
  EXPECT_TRUE(std::isinf(max_error));
}

// Test 4: Unpopulated default all-zero state MUST NOT evaluate as M1 verified
TEST(M3StartupM1, UnpopulatedAllZeroStateRejectsAsNotAtM1)
{
  const std::vector<double> all_zeros = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  auto zero_js = make_joint_state(kArmJointNames, all_zeros);
  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    zero_js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::STARTUP_NOT_AT_M1);
  EXPECT_GT(max_error, kStartupToleranceRad);
  // Max error is at wrist_2_joint (|0.0 - (-1.5708)| = 1.5708 rad)
  EXPECT_NEAR(max_error, 1.5708, 1e-4);
}

// Test 5: Exact authoritative M1 state verifies successfully
TEST(M3StartupM1, ExactM1StateVerifies)
{
  auto js = make_joint_state(kArmJointNames, kAuthoritativeM1);
  EXPECT_TRUE(joint_state_has_joints(*js, kArmJointNames));

  auto pos = extract_joint_positions(*js, kArmJointNames);
  ASSERT_TRUE(pos.has_value());
  EXPECT_EQ(*pos, kAuthoritativeM1);

  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::SUCCESS);
  EXPECT_NEAR(max_error, 0.0, 1e-12);
}

// Test 6: M1 state with joint error within tolerance (<= 0.01 rad) verifies
TEST(M3StartupM1, M1WithinToleranceVerifies)
{
  std::vector<double> perturbed = kAuthoritativeM1;
  perturbed[0] += 0.005;  // +0.005 rad within 0.01 rad tolerance
  perturbed[3] -= 0.008;  // -0.008 rad within 0.01 rad tolerance

  auto js = make_joint_state(kArmJointNames, perturbed);
  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::SUCCESS);
  EXPECT_NEAR(max_error, 0.008, 1e-6);
  EXPECT_LE(max_error, kStartupToleranceRad);
}

// Test 7: Out-of-order joint names in JointState are properly ordered and verified
TEST(M3StartupM1, OutOfOrderJointStateReorderedAndVerified)
{
  // Reverse joint order and include additional non-arm joint
  std::vector<std::string> scrambled_names = {
    "robotiq_85_left_knuckle_joint",
    "wrist_3_joint",
    "wrist_2_joint",
    "wrist_1_joint",
    "elbow_joint",
    "shoulder_lift_joint",
    "shoulder_pan_joint"
  };
  std::vector<double> scrambled_pos = {
    0.04,
    kAuthoritativeM1[5],
    kAuthoritativeM1[4],
    kAuthoritativeM1[3],
    kAuthoritativeM1[2],
    kAuthoritativeM1[1],
    kAuthoritativeM1[0]
  };
  auto js = make_joint_state(scrambled_names, scrambled_pos);
  EXPECT_TRUE(joint_state_has_joints(*js, kArmJointNames));

  auto pos = extract_joint_positions(*js, kArmJointNames);
  ASSERT_TRUE(pos.has_value());
  EXPECT_EQ(*pos, kAuthoritativeM1);

  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::SUCCESS);
  EXPECT_NEAR(max_error, 0.0, 1e-12);
}

// Test 8: Home configuration fails M1 check
TEST(M3StartupM1, HomeConfigurationFailsClosed)
{
  const std::vector<double> home_positions = {
    0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0
  };
  auto js = make_joint_state(kArmJointNames, home_positions);
  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::STARTUP_NOT_AT_M1);
  EXPECT_GT(max_error, kStartupToleranceRad);
  // Max error between home and M1 is 0.5708 rad (elbow: |1.5708 - 1.0| = 0.5708 rad)
  EXPECT_NEAR(max_error, 0.5708, 1e-4);
}

// Test 9: M1 with one joint error exceeding tolerance (> 0.01 rad) fails closed
TEST(M3StartupM1, SingleJointPerturbationExceedingToleranceFailsClosed)
{
  std::vector<double> perturbed = kAuthoritativeM1;
  perturbed[1] += 0.015;  // +0.015 rad exceeds 0.01 rad tolerance

  auto js = make_joint_state(kArmJointNames, perturbed);
  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    js, kArmJointNames, kAuthoritativeM1, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::STARTUP_NOT_AT_M1);
  EXPECT_NEAR(max_error, 0.015, 1e-6);
  EXPECT_GT(max_error, kStartupToleranceRad);
}

// Test 10: Dimension mismatch returns CONFIG_ERROR
TEST(M3StartupM1, DimensionMismatchFailsWithConfigError)
{
  const std::vector<double> truncated_goal = {0.5, -1.2, 1.0};
  auto js = make_joint_state(kArmJointNames, kAuthoritativeM1);
  double max_error = -1.0;
  const Result res = evaluate_startup_sample_and_m1(
    js, kArmJointNames, truncated_goal, kStartupToleranceRad, max_error);
  EXPECT_EQ(res, Result::CONFIG_ERROR);
  EXPECT_TRUE(std::isinf(max_error));
}

}  // namespace
