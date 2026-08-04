// m2_cartesian_approach.cpp — Milestone M2.
//
// EVIDENCE THIS PRODUCES
//   A Cartesian approach move to the pre-grasp pose (grasp_frame, backed off
//   by grasp.standoff along -approach_axis), planned via
//   computeCartesianPath() with the 0.95 fraction abort, executed, then
//   TCP-versus-commanded evidence sourced from GAZEBO GROUND TRUTH — not TF.
//
// WHY GAZEBO GROUND TRUTH AND NOT TF FOR THE EVIDENCE
//   Same distinction as M0-C. TF (via robot_state_publisher) reports where
//   MoveIt BELIEVES the TCP is, computed from /joint_states and the URDF's
//   own kinematic model. That is the thing under test — using it to grade
//   itself proves nothing. Ground truth here is Gazebo's own pose topic.
//
//   TRIED FIRST, DROPPED: bridging gz.msgs.Pose_V -> tf2_msgs/msg/TFMessage
//   via ros_gz_bridge (a real, existing conversion — see
//   ros_gz_bridge/convert/tf2_msgs.hpp) to avoid a gz-transport C++
//   dependency. Verified live: the bridge runs and publishes, but every
//   TransformStamped it produces has frame_id AND child_frame_id empty —
//   the per-entity name is lost in that specific conversion, at least for
//   this topic, on this ros_gz_bridge version. Confirmed by echoing
//   /gz_pose_tf directly, not inferred. There is no way to tell which
//   transform belongs to which link through that path, so it cannot be used.
//   Fell back to the SAME method scripts/00_recon.sh, 04_mimic_contact_probe.sh
//   and 05_measure_gripper_geometry.sh already use successfully: shell out to
//   `gz topic -e -t <pose topic> -n 1` and parse its native text dump, which
//   does carry the name field. Same proven method, just from C++ instead of
//   bash, and only for one link instead of a whole sweep.
//
//   tool0 itself never appears as a separate entity in Gazebo's pose output —
//   it, flange, robotiq_85_base_link and ur_to_robotiq_link are fixed-joint-
//   lumped into wrist_3_link at the physics level (confirmed when tcp_offset
//   was measured: docs/M-1_reference_report.md §6 item 5,
//   scripts/05_measure_gripper_geometry.sh). Since every joint in that chain
//   has zero translation, tool0's world POSITION equals wrist_3_link's;
//   only orientation differs, by the two known fixed rotations. This node
//   reconstructs tool0's ground-truth pose the same way that script does,
//   then adds tcp_offset along its local Z to get grasp_tcp's ground truth.
//
// WHAT THIS DELIBERATELY DOES NOT DO
//   Does not close the gripper, does not lift, does not touch the object.
//   Approach only. Keeping M2 to "reach the pre-grasp pose" means a failure
//   here is the Cartesian path / TF composition / clock or frame setup, not a
//   grasp problem wearing an approach costume — same reasoning M1 applied to
//   keeping the arm joint-space only.
//
// NOTHING IS HARDCODED
//   standoff, the fraction threshold, the TF lookup timeout and tcp_offset
//   all arrive as parameters, read from config/scene.yaml by the launch file.
//   The only compile-time constants are the wrist_3_link -> flange -> tool0
//   fixed rotations, which are properties of the URDF itself (recon-confirmed,
//   zero translation, known rpy), not tunable scene parameters — the same
//   treatment scripts/05_measure_gripper_geometry.sh already gives them.

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ur5e_pick_place/failure.hpp"
#include "ur5e_pick_place/moveit_compat.hpp"

using ur5e_pick_place::Result;
using ur5e_pick_place::to_string;
using namespace std::chrono_literals;

namespace
{
constexpr char kPlanningGroup[] = "arm";

// wrist_3_link -> flange -> tool0, confirmed by recon: fixed joints, zero
// translation both. Same constants as scripts/05_measure_gripper_geometry.sh.
tf2::Matrix3x3 R_from_rpy(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  return tf2::Matrix3x3(q);
}

const tf2::Matrix3x3 kR_wrist3_to_flange = R_from_rpy(0.0, -M_PI_2, -M_PI_2);
const tf2::Matrix3x3 kR_flange_to_tool0 = R_from_rpy(M_PI_2, 0.0, M_PI_2);

std::string run_command(const std::string & cmd)
{
  std::array<char, 4096> buf;
  std::string out;
  std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) {
    return out;
  }
  while (fgets(buf.data(), buf.size(), pipe.get()) != nullptr) {
    out += buf.data();
  }
  return out;
}

// Extract the position+orientation of one named link from a `gz topic -e`
// text dump of a gz.msgs.Pose_V message. Same text format
// scripts/05_measure_gripper_geometry.sh already parses successfully with a
// Python regex; this is the same parse, done with plain string search
// instead of std::regex (simpler than getting std::regex to do dot-all
// matching across the multi-line block correctly).
std::optional<tf2::Transform> parse_link_pose(const std::string & dump, const std::string & link_name)
{
  const std::string name_needle = "name: \"" + link_name + "\"";
  auto name_pos = dump.find(name_needle);
  if (name_pos == std::string::npos) {
    return std::nullopt;
  }

  auto extract_field = [&](const std::string & block, char field) -> double {
      const std::string needle = std::string(1, field) + ":";
      auto p = block.find(needle);
      if (p == std::string::npos) {
        return 0.0;
      }
      return std::strtod(block.c_str() + p + needle.size(), nullptr);
    };

  auto pos_start = dump.find("position {", name_pos);
  auto pos_end = dump.find('}', pos_start);
  auto ori_start = dump.find("orientation {", name_pos);
  auto ori_end = dump.find('}', ori_start);
  if (pos_start == std::string::npos || pos_end == std::string::npos ||
    ori_start == std::string::npos || ori_end == std::string::npos)
  {
    return std::nullopt;
  }

  const std::string pos_block = dump.substr(pos_start, pos_end - pos_start);
  const std::string ori_block = dump.substr(ori_start, ori_end - ori_start);

  tf2::Vector3 origin(
    extract_field(pos_block, 'x'), extract_field(pos_block, 'y'), extract_field(pos_block, 'z'));
  // Default w=1 (identity) matches gz's own convention of omitting a field
  // at its default value in the text dump.
  auto oz = ori_block.find("z:");
  auto ow = ori_block.find("w:");
  tf2::Quaternion q(
    extract_field(ori_block, 'x'), extract_field(ori_block, 'y'),
    oz == std::string::npos ? 0.0 : extract_field(ori_block, 'z'),
    ow == std::string::npos ? 1.0 : extract_field(ori_block, 'w'));
  return tf2::Transform(q, origin);
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "m2_cartesian_approach",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  Result result = Result::SUCCESS;

  // ---------------------------------------------------------------------
  // Parameters. All originate in config/scene.yaml via the launch file.
  // ---------------------------------------------------------------------
  std::string world_frame = "world";
  std::string grasp_frame_name = "grasp_frame";
  std::string tool0_frame = "tool0";
  double standoff = 0.0;
  double tcp_offset = 0.0;
  double tf_lookup_timeout_s = 2.0;
  double cartesian_fraction_min = 0.95;
  double planning_time_s = 5.0;
  int plan_attempts = 10;
  double vel_scale = 0.1;
  double acc_scale = 0.1;
  double eef_step = 0.01;
  std::string csv_path = "m2_approach.csv";
  std::string grasp_mode = "friction";
  std::string gt_wrist3_link_name = "wrist_3_link";
  std::string gz_world = "empty";
  std::string gt_base_link_name = "base_link";
  std::vector<double> expected_base_xyz;
  std::vector<double> expected_base_rpy;
  double base_pose_tol_m = 0.005;
  double base_pose_tol_rad = 0.01;

  node->get_parameter_or("world_frame", world_frame, world_frame);
  node->get_parameter_or("grasp_frame_name", grasp_frame_name, grasp_frame_name);
  node->get_parameter_or("tool0_frame", tool0_frame, tool0_frame);
  node->get_parameter_or("standoff", standoff, standoff);
  node->get_parameter_or("tcp_offset", tcp_offset, tcp_offset);
  node->get_parameter_or("tf_lookup_timeout_s", tf_lookup_timeout_s, tf_lookup_timeout_s);
  node->get_parameter_or("cartesian_fraction_min", cartesian_fraction_min, cartesian_fraction_min);
  node->get_parameter_or("planning_time_s", planning_time_s, planning_time_s);
  node->get_parameter_or("plan_attempts", plan_attempts, plan_attempts);
  node->get_parameter_or("velocity_scaling", vel_scale, vel_scale);
  node->get_parameter_or("acceleration_scaling", acc_scale, acc_scale);
  node->get_parameter_or("eef_step", eef_step, eef_step);
  node->get_parameter_or("csv_path", csv_path, csv_path);
  node->get_parameter_or("grasp_mode", grasp_mode, grasp_mode);
  node->get_parameter_or("gt_wrist3_link_name", gt_wrist3_link_name, gt_wrist3_link_name);
  node->get_parameter_or("gz_world", gz_world, gz_world);
  node->get_parameter_or("gt_base_link_name", gt_base_link_name, gt_base_link_name);
  node->get_parameter_or("expected_base_xyz", expected_base_xyz, expected_base_xyz);
  node->get_parameter_or("expected_base_rpy", expected_base_rpy, expected_base_rpy);
  node->get_parameter_or("base_pose_tol_m", base_pose_tol_m, base_pose_tol_m);
  node->get_parameter_or("base_pose_tol_rad", base_pose_tol_rad, base_pose_tol_rad);

  RCLCPP_INFO(
    logger, "GRASP MODE: %s",
    grasp_mode == "friction" ? "friction (physics)"
                             : "contact-triggered attach (fallback)");

  if (standoff <= 0.0) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: grasp.standoff must be > 0, got %.4f. Approach with zero "
      "standoff is not an approach.",
      standoff);
    result = Result::CONFIG_ERROR;
  }

  // Ground-truth query: shell out to `gz topic -e`, exactly as
  // scripts/05_measure_gripper_geometry.sh does, and reconstruct tool0's pose
  // from wrist_3_link's via the fixed chain. See file header for why this
  // replaced the ros_gz_bridge TFMessage approach.
  const std::string gz_pose_topic = "/world/" + gz_world + "/pose/info";
  auto ground_truth_tool0 = [gz_pose_topic, gt_wrist3_link_name]() -> std::optional<tf2::Transform> {
      const std::string dump = run_command(
        "gz topic -e -t " + gz_pose_topic + " -n 1 2>/dev/null");
      auto w = parse_link_pose(dump, gt_wrist3_link_name);
      if (!w) {
        return std::nullopt;
      }
      tf2::Matrix3x3 R_world_tool0 = w->getBasis() * kR_wrist3_to_flange * kR_flange_to_tool0;
      tf2::Quaternion q;
      R_world_tool0.getRotation(q);
      // Zero translation through the whole fixed chain: tool0's position
      // equals wrist_3_link's.
      return tf2::Transform(q, w->getOrigin());
    };

  // ---------------------------------------------------------------------
  // Base-pose guard: three launch files (Gazebo bringup, move_group,
  // this node) each derive a robot_description from scene.yaml's
  // robot.base_pose independently — now through one shared module
  // (config/scene_xacro_args.py), but a stale cache, a manual override, or
  // a future fourth caller could still let them drift apart. Found this
  // exact class of bug during M2: MoveIt silently planning against a
  // ground-mounted robot while Gazebo simulated one elevated to table
  // height, discovered only because compute_ik results didn't match manual
  // reasoning about reach. Checking here, against Gazebo GROUND TRUTH (not
  // TF — TF would just report whatever the node's OWN possibly-stale model
  // believes), turns that class of silent divergence into a named,
  // immediate CONFIG_ERROR instead of a confusing downstream planning
  // failure. Skipped if expected_base_xyz isn't provided (e.g. manual
  // standalone testing without the full launch file).
  if (ur5e_pick_place::ok(result) && expected_base_xyz.size() == 3 && expected_base_rpy.size() == 3) {
    const std::string dump = run_command("gz topic -e -t " + gz_pose_topic + " -n 1 2>/dev/null");
    auto base_gt = parse_link_pose(dump, gt_base_link_name);
    if (!base_gt) {
      RCLCPP_ERROR(
        logger,
        "CONFIG_ERROR: could not read '%s' pose via `gz topic -e -t %s` to "
        "verify it matches scene.yaml's robot.base_pose. Is the sim running?",
        gt_base_link_name.c_str(), gz_pose_topic.c_str());
      result = Result::CONFIG_ERROR;
    } else {
      tf2::Vector3 expected_origin(expected_base_xyz[0], expected_base_xyz[1], expected_base_xyz[2]);
      tf2::Quaternion expected_q;
      expected_q.setRPY(expected_base_rpy[0], expected_base_rpy[1], expected_base_rpy[2]);

      const double pos_err = base_gt->getOrigin().distance(expected_origin);
      tf2::Quaternion actual_q;
      base_gt->getBasis().getRotation(actual_q);
      const double dot = std::abs(actual_q.dot(expected_q));
      const double angle_err = 2.0 * std::acos(std::min(1.0, dot));

      if (pos_err > base_pose_tol_m || angle_err > base_pose_tol_rad) {
        RCLCPP_ERROR(
          logger,
          "CONFIG_ERROR: Gazebo's actual '%s' pose does not match "
          "scene.yaml's robot.base_pose (pos error %.4f m, angle error "
          "%.4f rad; tolerances %.4f m / %.4f rad). MoveIt and Gazebo are "
          "planning/simulating against different robots — check that every "
          "launch file deriving base_xyz/base_rpy actually used "
          "config/scene_xacro_args.py.",
          gt_base_link_name.c_str(), pos_err, angle_err, base_pose_tol_m, base_pose_tol_rad);
        result = Result::CONFIG_ERROR;
      } else {
        RCLCPP_INFO(
          logger, "base-pose guard OK: '%s' matches scene.yaml within tolerance "
          "(pos error %.5f m, angle error %.5f rad)",
          gt_base_link_name.c_str(), pos_err, angle_err);
      }
    }
  }

  bool got_grasp_tf = false;
  tf2::Transform T_world_grasp;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  if (ur5e_pick_place::ok(result)) {
    tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    tf_buffer->setUsingDedicatedThread(true);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // NOT using lookupTransform's own timeout overload here. Measured
    // directly: against a frame that has never been seen even once (the
    // startup race against static_scene_tf, a separate process, still
    // completing discovery), that overload failed after ~51-212ms across
    // repeated runs despite a 2.0s and even a 10.0s budget — nowhere near
    // the requested timeout, and not consistently reproducible even at the
    // same configured value. Polling canTransform() with the zero-timeout
    // (non-blocking) overload in an explicit loop, timed against a steady
    // clock, sidesteps whatever internal wait-registration subtlety that
    // is and gives a directly verifiable timeout contract instead.
    auto t0 = std::chrono::steady_clock::now();
    const auto deadline = t0 + std::chrono::duration<double>(tf_lookup_timeout_s);
    std::string tf_error;
    while (std::chrono::steady_clock::now() < deadline) {
      if (tf_buffer->canTransform(world_frame, grasp_frame_name, tf2::TimePointZero, &tf_error)) {
        auto stamped = tf_buffer->lookupTransform(world_frame, grasp_frame_name, tf2::TimePointZero);
        tf2::fromMsg(stamped.transform, T_world_grasp);
        got_grasp_tf = true;
        break;
      }
      rclcpp::sleep_for(20ms);
    }
    if (!got_grasp_tf) {
      RCLCPP_ERROR(
        logger,
        "TF_LOOKUP_TIMEOUT: could not resolve %s -> %s within %.1fs: %s",
        world_frame.c_str(), grasp_frame_name.c_str(), tf_lookup_timeout_s,
        tf_error.c_str());
      result = Result::TF_LOOKUP_TIMEOUT;
    }
    auto t1 = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      logger, "TF lookup %s -> %s took %.1f ms",
      world_frame.c_str(), grasp_frame_name.c_str(),
      std::chrono::duration<double, std::milli>(t1 - t0).count());
  }

  double achieved_fraction = 0.0;
  bool executed = false;
  double commanded_tcp[3] = {0, 0, 0};
  double achieved_tcp[3] = {0, 0, 0};
  double tcp_error_m = -1.0;
  bool have_ground_truth = false;

  if (ur5e_pick_place::ok(result) && got_grasp_tf) {
    moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);
    move_group.setPlanningTime(planning_time_s);
    move_group.setNumPlanningAttempts(plan_attempts);
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(acc_scale);

    // Two targets, both expressed for tool0 (what MoveIt actually plans for):
    //   pre-grasp: grasp_frame backed off by `standoff` along local -Z.
    //   grasp target: grasp_frame itself (no standoff) — the point the TCP
    //     is meant to occupy. Reaching it does not touch, close, or lift
    //     anything; it is still "approach," not a grasp action.
    // tool0 -> grasp_tcp is Translation(0,0,tcp_offset); its inverse
    // (grasp_tcp target -> tool0 target) is Translation(0,0,-tcp_offset).
    tf2::Transform T_tcp_tool0(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, -tcp_offset));

    tf2::Transform T_grasp_pregrasp(tf2::Quaternion(0, 0, 0, 1), tf2::Vector3(0, 0, -standoff));
    tf2::Transform T_world_pregrasp = T_world_grasp * T_grasp_pregrasp;
    tf2::Transform T_world_pregrasp_tool0 = T_world_pregrasp * T_tcp_tool0;

    tf2::Transform T_world_grasp_tool0 = T_world_grasp * T_tcp_tool0;

    geometry_msgs::msg::Pose pregrasp_pose;
    tf2::toMsg(T_world_pregrasp_tool0, pregrasp_pose);
    geometry_msgs::msg::Pose grasp_pose;
    tf2::toMsg(T_world_grasp_tool0, grasp_pose);

    RCLCPP_INFO(
      logger,
      "pre-grasp tool0 target (world): [%.4f %.4f %.4f]  grasp tool0 target: "
      "[%.4f %.4f %.4f]",
      pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z,
      grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z);

    // Stage 1: joint-space plan+execute to pre-grasp. This is potentially a
    // large traversal from wherever the arm currently is (home, in the
    // common case) with a large reorientation — there is no reason to force
    // that to be a straight line, and measured directly, computeCartesianPath
    // cannot do it reliably (58.9% fraction from home in testing, well under
    // threshold). Same PLAN_FAILURE/EXECUTE_FAILURE mechanism M1 already
    // proved reliable.
    move_group.setPoseTarget(pregrasp_pose, tool0_frame);
    moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
    const bool pregrasp_planned =
      (move_group.plan(pregrasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!pregrasp_planned) {
      RCLCPP_ERROR(logger, "PLAN_FAILURE: could not plan to the pre-grasp pose.");
      result = Result::PLAN_FAILURE;
    } else {
      const auto pregrasp_exec = move_group.execute(pregrasp_plan);
      if (pregrasp_exec != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(
          logger, "EXECUTE_FAILURE: pre-grasp plan found but execution returned code %d.",
          pregrasp_exec.val);
        result = Result::EXECUTE_FAILURE;
      }
    }

    // Stage 2: the actual "approach" — a short, purely-vertical Cartesian
    // descent from pre-grasp to the grasp target, standoff metres, with the
    // orientation unchanged from stage 1. This is the segment where
    // straight-line motion actually matters (not sweeping sideways into the
    // object on the way down), and the 0.95 fraction abort applies to it.
    if (ur5e_pick_place::ok(result)) {
      commanded_tcp[0] = T_world_grasp.getOrigin().x();
      commanded_tcp[1] = T_world_grasp.getOrigin().y();
      commanded_tcp[2] = T_world_grasp.getOrigin().z();

      moveit_msgs::msg::RobotTrajectory trajectory;
      achieved_fraction = move_group.computeCartesianPath(
        {grasp_pose}, eef_step, trajectory, true, nullptr);

      RCLCPP_INFO(
        logger, "computeCartesianPath fraction achieved: %.4f (need > %.4f)",
        achieved_fraction, cartesian_fraction_min);

      if (achieved_fraction < cartesian_fraction_min) {
        RCLCPP_ERROR(
          logger,
          "CARTESIAN_FRACTION_LOW: achieved %.4f, threshold is %.4f. Not "
          "executing a partial Cartesian path.",
          achieved_fraction, cartesian_fraction_min);
        result = Result::CARTESIAN_FRACTION_LOW;
      } else {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ur5e_pick_place::set_trajectory(plan, trajectory);
        RCLCPP_INFO(logger, "executing Cartesian approach...");
        const auto exec_code = move_group.execute(plan);
        executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);
        if (executed) {
          RCLCPP_INFO(logger, "execution reported SUCCESS");
        } else {
          RCLCPP_ERROR(
            logger,
            "EXECUTE_FAILURE: MoveIt returned error code %d after a valid "
            "Cartesian path was found.",
            exec_code.val);
          result = Result::EXECUTE_FAILURE;
        }
      }
    }
  }

  // ---------------------------------------------------------------------
  // Ground-truth evidence: query `gz topic -e` once physics has had a moment
  // to settle after execution, then compare achieved grasp_tcp (Gazebo
  // ground truth) against commanded (the target this node itself computed).
  // ---------------------------------------------------------------------
  if (executed) {
    rclcpp::sleep_for(500ms);
    if (auto tool0_gt = ground_truth_tool0()) {
      tf2::Vector3 tcp_gt = tool0_gt->getOrigin() +
        tool0_gt->getBasis() * tf2::Vector3(0, 0, tcp_offset);
      achieved_tcp[0] = tcp_gt.x();
      achieved_tcp[1] = tcp_gt.y();
      achieved_tcp[2] = tcp_gt.z();
      tcp_error_m = std::sqrt(
        std::pow(achieved_tcp[0] - commanded_tcp[0], 2) +
        std::pow(achieved_tcp[1] - commanded_tcp[1], 2) +
        std::pow(achieved_tcp[2] - commanded_tcp[2], 2));
      have_ground_truth = true;
      RCLCPP_INFO(
        logger,
        "ground-truth grasp_tcp (Gazebo, not TF): [%.4f %.4f %.4f]  "
        "error vs commanded: %.4f m",
        achieved_tcp[0], achieved_tcp[1], achieved_tcp[2], tcp_error_m);
    } else {
      RCLCPP_ERROR(
        logger,
        "no ground-truth pose found for '%s' via `gz topic -e -t %s` — is "
        "the sim running? Evidence is commanded-only, not verified against "
        "physical truth.",
        gt_wrist3_link_name.c_str(), gz_pose_topic.c_str());
    }
  }

  // ---------------------------------------------------------------------
  // CSV evidence, written regardless of outcome.
  // ---------------------------------------------------------------------
  {
    std::ofstream csv(csv_path);
    if (csv) {
      csv << "result,cartesian_fraction,executed,"
             "commanded_x,commanded_y,commanded_z,"
             "achieved_x,achieved_y,achieved_z,tcp_error_m,have_ground_truth\n";
      csv << to_string(result) << ',' << achieved_fraction << ',' << (executed ? 1 : 0)
          << ',' << commanded_tcp[0] << ',' << commanded_tcp[1] << ',' << commanded_tcp[2]
          << ',' << achieved_tcp[0] << ',' << achieved_tcp[1] << ',' << achieved_tcp[2]
          << ',' << tcp_error_m << ',' << (have_ground_truth ? 1 : 0) << '\n';
      RCLCPP_INFO(logger, "wrote evidence to %s", csv_path.c_str());
    } else {
      RCLCPP_ERROR(logger, "could not open %s for writing", csv_path.c_str());
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("m2_cartesian_approach"),
    "RUN SUMMARY: milestone=M2 result=%s cartesian_fraction=%.4f executed=%s "
    "tcp_error_m=%.4f ground_truth=%s",
    to_string(result), achieved_fraction, executed ? "yes" : "no", tcp_error_m,
    have_ground_truth ? "yes" : "no");

  executor.cancel();
  if (spinner.joinable()) { spinner.join(); }
  rclcpp::shutdown();

  return ur5e_pick_place::ok(result) ? 0 : 1;
}
