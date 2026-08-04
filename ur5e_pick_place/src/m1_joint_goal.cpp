// m1_joint_goal.cpp — Milestone M1.
//
// EVIDENCE THIS PRODUCES
//   "20/20 planning success to a fixed goal, logged."
//   One CSV row per trial plus a one-line typed run summary.
//
// WHAT IT DOES
//   Plans to a fixed joint-space goal N times (default 20), recording success
//   and planning time for each. Then executes one plan, because the milestone
//   is "MoveIt executes a joint goal", not merely "MoveIt plans one".
//
// WHAT IT DELIBERATELY DOES NOT DO
//   No Cartesian paths, no TF composition, no gripper, no object. Those are M2
//   and later. Keeping M1 to joint-space means a failure here can only be the
//   arm planning group, the controller, or the config — not a grasp problem
//   wearing a planning costume.
//
// NOTHING IS HARDCODED
//   Joint names, the goal, trial count and all thresholds arrive as ROS
//   parameters, which the launch file reads from config/scene.yaml. If you find
//   yourself typing a joint value into this file, it belongs in scene.yaml.

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "ur5e_pick_place/failure.hpp"
#include "ur5e_pick_place/moveit_compat.hpp"

using ur5e_pick_place::Result;
using ur5e_pick_place::to_string;

namespace
{
constexpr char kPlanningGroup[] = "arm";

struct Trial
{
  int index = 0;
  bool planned = false;
  double plan_ms = 0.0;
  std::size_t traj_points = 0;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "m1_joint_goal",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  auto logger = node->get_logger();

  // MoveGroupInterface blocks on services, so the node must be spinning on
  // another thread or every call deadlocks.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  Result result = Result::SUCCESS;

  // ---------------------------------------------------------------------
  // Parameters. Every one of these originates in config/scene.yaml.
  // ---------------------------------------------------------------------
  std::vector<std::string> joint_names;
  std::vector<double> goal_positions;
  int trials = 20;
  double planning_time_s = 5.0;
  int plan_attempts = 10;
  double vel_scale = 0.1;
  double acc_scale = 0.1;
  std::string csv_path = "m1_planning.csv";
  std::string grasp_mode = "friction";

  node->get_parameter_or("arm_joints", joint_names, joint_names);
  node->get_parameter_or("goal_positions", goal_positions, goal_positions);
  node->get_parameter_or("trials", trials, trials);
  node->get_parameter_or("planning_time_s", planning_time_s, planning_time_s);
  node->get_parameter_or("plan_attempts", plan_attempts, plan_attempts);
  node->get_parameter_or("velocity_scaling", vel_scale, vel_scale);
  node->get_parameter_or("acceleration_scaling", acc_scale, acc_scale);
  node->get_parameter_or("csv_path", csv_path, csv_path);
  node->get_parameter_or("grasp_mode", grasp_mode, grasp_mode);

  // Spec: every run logs its mode at startup, one line, unambiguous later.
  // M1 does no grasping, but the line is emitted anyway so that every log in
  // the project has it in the same place.
  RCLCPP_INFO(
    logger, "GRASP MODE: %s (M1 performs no grasp; line emitted for log uniformity)",
    grasp_mode == "friction" ? "friction (physics)"
                             : "contact-triggered attach (fallback)");

  // Fail loudly on bad config rather than planning to a silently wrong goal.
  if (joint_names.empty() || goal_positions.empty()) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: arm_joints (%zu) and goal_positions (%zu) must both be "
      "non-empty. They come from config/scene.yaml via the launch file.",
      joint_names.size(), goal_positions.size());
    result = Result::CONFIG_ERROR;
  } else if (joint_names.size() != goal_positions.size()) {
    RCLCPP_ERROR(
      logger,
      "CONFIG_ERROR: arm_joints has %zu entries but goal_positions has %zu. "
      "A mismatch here would plan to a goal that is not the one written down.",
      joint_names.size(), goal_positions.size());
    result = Result::CONFIG_ERROR;
  }

  std::vector<Trial> results;
  int planned_ok = 0;
  bool executed = false;

  if (ur5e_pick_place::ok(result)) {
    moveit::planning_interface::MoveGroupInterface move_group(node, kPlanningGroup);
    move_group.setPlanningTime(planning_time_s);
    move_group.setNumPlanningAttempts(plan_attempts);
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(acc_scale);

    RCLCPP_INFO(
      logger, "planning frame: %s | end effector link: %s",
      move_group.getPlanningFrame().c_str(), move_group.getEndEffectorLink().c_str());

    {
      std::string goal_str;
      for (std::size_t i = 0; i < joint_names.size(); ++i) {
        goal_str += joint_names[i] + "=" + std::to_string(goal_positions[i]);
        if (i + 1 < joint_names.size()) { goal_str += ", "; }
      }
      RCLCPP_INFO(logger, "fixed joint goal: %s", goal_str.c_str());
    }

    // -------------------------------------------------------------------
    // Planning trials.
    // -------------------------------------------------------------------
    moveit::planning_interface::MoveGroupInterface::Plan last_good_plan;
    bool have_good_plan = false;

    for (int i = 1; i <= trials && rclcpp::ok(); ++i) {
      Trial t;
      t.index = i;

      // Re-set the target each iteration. Planning is stochastic, so repeated
      // identical requests are the point — this measures planner reliability,
      // not whether one lucky plan exists.
      std::map<std::string, double> target;
      for (std::size_t j = 0; j < joint_names.size(); ++j) {
        target[joint_names[j]] = goal_positions[j];
      }
      move_group.setJointValueTarget(target);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      auto t0 = std::chrono::steady_clock::now();
      const bool success = (move_group.plan(plan) ==
                            moveit::core::MoveItErrorCode::SUCCESS);
      auto t1 = std::chrono::steady_clock::now();

      t.plan_ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
      t.planned = success;
      t.traj_points = success ? ur5e_pick_place::points_in(plan) : 0;

      if (success) {
        ++planned_ok;
        last_good_plan = plan;
        have_good_plan = true;
        RCLCPP_INFO(
          logger, "trial %2d/%d: PLAN OK   %7.1f ms  %zu points",
          i, trials, t.plan_ms, t.traj_points);
      } else {
        RCLCPP_ERROR(
          logger, "trial %2d/%d: PLAN FAILURE after %.1f ms — planner returned "
          "no solution for the fixed joint goal", i, trials, t.plan_ms);
      }
      results.push_back(t);
    }

    if (planned_ok != trials) {
      result = Result::PLAN_FAILURE;
    }

    // -------------------------------------------------------------------
    // Execute one plan. "MoveIt executes a joint goal" is part of M1.
    // Attempted even if some trials failed — a partial planning rate with
    // working execution is different information from neither working.
    // -------------------------------------------------------------------
    if (have_good_plan && rclcpp::ok()) {
      RCLCPP_INFO(logger, "executing the last successful plan...");
      const auto exec_code = move_group.execute(last_good_plan);
      executed = (exec_code == moveit::core::MoveItErrorCode::SUCCESS);
      if (executed) {
        RCLCPP_INFO(logger, "execution reported SUCCESS");
      } else {
        RCLCPP_ERROR(
          logger,
          "EXECUTE_FAILURE: MoveIt returned error code %d. A plan existed, so "
          "this is the controller path, not the planner. Check that "
          "arm_controller is active and that moveit_controllers.yaml names it "
          "exactly as the bringup spawns it.",
          exec_code.val);
        if (ur5e_pick_place::ok(result)) { result = Result::EXECUTE_FAILURE; }
      }
    } else if (!have_good_plan) {
      RCLCPP_ERROR(logger, "no successful plan to execute — skipping execution");
    }
  }

  // ---------------------------------------------------------------------
  // CSV. Written even on failure; a failed run's data is the useful data.
  // ---------------------------------------------------------------------
  if (!results.empty()) {
    std::ofstream csv(csv_path);
    if (csv) {
      csv << "trial,planned,plan_ms,trajectory_points\n";
      for (const auto & t : results) {
        csv << t.index << ',' << (t.planned ? 1 : 0) << ','
            << t.plan_ms << ',' << t.traj_points << '\n';
      }
      RCLCPP_INFO(logger, "wrote %zu trials to %s", results.size(), csv_path.c_str());
    } else {
      RCLCPP_ERROR(logger, "could not open %s for writing", csv_path.c_str());
    }
  }

  // ---------------------------------------------------------------------
  // One-line run summary. Required by the spec; grep-able across runs.
  // ---------------------------------------------------------------------
  double rate = trials > 0 ? (100.0 * planned_ok / trials) : 0.0;
  double mean_ms = 0.0;
  if (!results.empty()) {
    mean_ms = std::accumulate(
      results.begin(), results.end(), 0.0,
      [](double acc, const Trial & t) { return acc + t.plan_ms; }) /
      static_cast<double>(results.size());
  }

  RCLCPP_INFO(
    rclcpp::get_logger("m1_joint_goal"),
    "RUN SUMMARY: milestone=M1 result=%s trials=%d planned=%d rate=%.1f%% "
    "mean_plan_ms=%.1f executed=%s",
    to_string(result), trials, planned_ok, rate, mean_ms,
    executed ? "yes" : "no");

  executor.cancel();
  if (spinner.joinable()) { spinner.join(); }
  rclcpp::shutdown();

  // Non-zero exit on failure so a wrapper script cannot mistake a bad run for
  // a good one.
  return ur5e_pick_place::ok(result) ? 0 : 1;
}
