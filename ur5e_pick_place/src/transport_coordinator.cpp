// transport_coordinator.cpp — Stage-3C C3: One-replan reactive recovery coordinator.

#include "ur5e_pick_place/transport_coordinator.hpp"

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <thread>

namespace ur5e_pick_place
{

namespace
{
constexpr char kDynamicObstacleId[] = "dynamic_obstacle_0";
constexpr char kAttachedTargetId[] = "pick_target";
constexpr char kExpectedAttachLink[] = "gripper_base_link";
constexpr auto kSceneServiceTimeout = std::chrono::seconds(2);
constexpr double kObstacleStaleThresholdS = 0.250;

// Stage-3C C3C: observational-only telemetry helpers. Neither function
// influences control flow -- they format and log already-computed values.

void logMonitorSummary(
  const rclcpp::Logger & logger, int attempt,
  const TransportMonitorSummary & summary, bool trigger_occurred)
{
  RCLCPP_INFO(logger,
    "M3 C3 REPLACEMENT_MONITOR_SUMMARY attempt=%d tick_count=%d invalid_tick_count=%d "
    "has_invalidity=%d first_invalidity_monitor_elapsed_s=%.6f "
    "max_consecutive_invalid_ticks=%d stale_tick_count=%d trigger_count=%d",
    attempt, summary.tick_count, summary.invalid_tick_count, summary.has_invalidity,
    summary.first_invalidity_monitor_elapsed_s, summary.max_consecutive_invalid_ticks,
    summary.stale_tick_count, trigger_occurred ? 1 : 0);
  // NOTE: first_invalid_sample / first_invalid_time_s / progress_time_at_trigger /
  // temporal_lead_s / collision_pair are NOT duplicated onto this line --
  // they are already emitted per-trigger by TransportPathMonitor's own
  // (unmodified, C1/C2-validated) "M3 C2 COLLISION_TRIGGER" log line. A
  // harness correlates this summary with that line by attempt/timestamp
  // rather than this coordinator re-deriving or re-logging monitor-internal
  // per-trigger fields it does not own.
}

void logSettledStateTelemetry(
  const rclcpp::Logger & logger, int attempt, const SettledStateE & state)
{
  std::ostringstream joints;
  for (std::size_t i = 0; i < state.joint_names.size(); ++i) {
    joints << " " << state.joint_names[i] << "=" << std::setprecision(12) << state.positions[i];
  }
  RCLCPP_INFO(logger,
    "M3 C3 SETTLED_STATE attempt=%d label=\"%s\" captured=%d timestamp=%.9f%s",
    attempt, attempt_state_label(attempt), state.captured,
    state.captured ? state.stamp.seconds() : -1.0, joints.str().c_str());
}
}  // namespace

bool validate_state_e(
  const SettledStateE & state_e,
  const std::vector<std::string> & expected_group_joints,
  const moveit::core::RobotModel & robot_model,
  std::string & error)
{
  if (!state_e.captured) {
    error = "STATE_E_NOT_CAPTURED: physical settle state E flag is false";
    return false;
  }
  if (state_e.positions.size() != 6 || state_e.joint_names.size() != 6) {
    error = "STATE_E_INVALID_SIZE: expected 6 joints, got names=" +
      std::to_string(state_e.joint_names.size()) + " positions=" +
      std::to_string(state_e.positions.size());
    return false;
  }
  for (std::size_t i = 0; i < state_e.positions.size(); ++i) {
    if (!std::isfinite(state_e.positions[i])) {
      error = "STATE_E_NON_FINITE: joint '" + state_e.joint_names[i] + "' has non-finite position";
      return false;
    }
  }
  std::set<std::string> unique_names(state_e.joint_names.begin(), state_e.joint_names.end());
  if (unique_names.size() != 6) {
    error = "STATE_E_DUPLICATE_NAMES: state E contains duplicate joint names";
    return false;
  }
  std::set<std::string> expected_set(expected_group_joints.begin(), expected_group_joints.end());
  if (unique_names != expected_set) {
    error = "STATE_E_SET_MISMATCH: state E joint names do not match expected planning group joints";
    return false;
  }
  for (const auto & name : state_e.joint_names) {
    if (!robot_model.hasJointModel(name)) {
      error = "STATE_E_UNKNOWN_JOINT: joint '" + name + "' does not exist in RobotModel";
      return false;
    }
  }
  return true;
}

bool is_obstacle_fresh_post_time(
  double obstacle_stamp_s, double baseline_time_s, double query_time_s,
  double stale_threshold_s)
{
  if (obstacle_stamp_s < baseline_time_s) {
    return false;
  }
  const double age = query_time_s - obstacle_stamp_s;
  return std::isfinite(age) && age >= 0.0 && age <= stale_threshold_s;
}

double quaternion_shortest_angle_rad(
  const geometry_msgs::msg::Quaternion & q1,
  const geometry_msgs::msg::Quaternion & q2)
{
  const double norm1 = std::sqrt(q1.x * q1.x + q1.y * q1.y + q1.z * q1.z + q1.w * q1.w);
  const double norm2 = std::sqrt(q2.x * q2.x + q2.y * q2.y + q2.z * q2.z + q2.w * q2.w);
  if (norm1 < 1e-9 || norm2 < 1e-9) {
    return std::numeric_limits<double>::infinity();
  }
  const double dot = (q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w) / (norm1 * norm2);
  const double abs_dot = std::clamp(std::abs(dot), 0.0, 1.0);
  return 2.0 * std::acos(abs_dot);
}

bool is_pose_finite(const geometry_msgs::msg::Pose & p)
{
  return std::isfinite(p.position.x) && std::isfinite(p.position.y) &&
    std::isfinite(p.position.z) &&
    std::isfinite(p.orientation.x) && std::isfinite(p.orientation.y) &&
    std::isfinite(p.orientation.z) && std::isfinite(p.orientation.w);
}

bool is_pose_coherent(
  const geometry_msgs::msg::Pose & p_scene,
  const geometry_msgs::msg::Pose & p_expected,
  double pos_tol_m,
  double max_angle_error_rad)
{
  // CORRECTION B: reject NaN/+Inf/-Inf in EITHER pose before any comparison.
  // Without this, a nonfinite component produces a NaN error term, and every
  // comparison below ("pos_err > pos_tol_m", "angle_err <= max_angle_error_rad")
  // resolves in the ACCEPTING direction for NaN -- the incoherent scene would
  // have been reported coherent. Nonfinite input is rejected, never sanitized
  // or clamped into range.
  if (!is_pose_finite(p_scene) || !is_pose_finite(p_expected)) {
    return false;
  }
  const double dx = p_scene.position.x - p_expected.position.x;
  const double dy = p_scene.position.y - p_expected.position.y;
  const double dz = p_scene.position.z - p_expected.position.z;
  const double pos_err = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (pos_err > pos_tol_m) {
    return false;
  }
  const double angle_err = quaternion_shortest_angle_rad(p_scene.orientation, p_expected.orientation);
  return angle_err <= max_angle_error_rad;
}

bool verify_retained_snapshot_obstacle_and_attachment(
  const moveit_msgs::msg::PlanningScene & scene,
  const geometry_msgs::msg::Pose & expected_obstacle_pose,
  std::string & error)
{
  // CORRECTION D: every check below reads ONLY `scene`. No PlanningScene
  // service call is made, so the properties proven here are properties of THIS
  // retained snapshot -- the one the coordinator hands to
  // usePlanningSceneMsg() -- and cannot be satisfied by a different, later
  // snapshot that happens to be correct.
  const auto obstacle_it = std::find_if(
    scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
    [](const auto & co) { return co.id == kDynamicObstacleId; });
  if (obstacle_it == scene.world.collision_objects.end() ||
    obstacle_it->primitive_poses.empty())
  {
    error = std::string("SNAPSHOT_OBSTACLE_ABSENT: '") + kDynamicObstacleId +
      "' missing (or has no primitive pose) in the retained snapshot";
    return false;
  }
  const auto snapshot_pose = PlanningSceneManager::effectivePrimitivePose(*obstacle_it, 0);
  if (!is_pose_coherent(snapshot_pose, expected_obstacle_pose, 1.0e-4, 1.0e-3)) {
    error = std::string("SNAPSHOT_OBSTACLE_INCOHERENT: '") + kDynamicObstacleId +
      "' pose in the retained snapshot does not match the authorizing sample";
    return false;
  }

  const auto attached_it = std::find_if(
    scene.robot_state.attached_collision_objects.begin(),
    scene.robot_state.attached_collision_objects.end(),
    [](const auto & aco) { return aco.object.id == kAttachedTargetId; });
  if (attached_it == scene.robot_state.attached_collision_objects.end()) {
    error = std::string("SNAPSHOT_TARGET_NOT_ATTACHED: '") + kAttachedTargetId +
      "' is not an attached collision object in the retained snapshot";
    return false;
  }
  if (attached_it->link_name != kExpectedAttachLink) {
    error = std::string("SNAPSHOT_ATTACH_LINK_MISMATCH: '") + kAttachedTargetId +
      "' attached to '" + attached_it->link_name + "', expected '" +
      kExpectedAttachLink + "'";
    return false;
  }
  if (attached_it->touch_links != PlanningSceneManager::padTouchLinks()) {
    error = std::string("SNAPSHOT_TOUCH_LINKS_MISMATCH: '") + kAttachedTargetId +
      "' touch links in the retained snapshot are not the expected pad set";
    return false;
  }
  return true;
}

double quaternion_upright_tilt_deg(double qx, double qy)
{
  const double dot = 1.0 - 2.0 * (qx * qx + qy * qy);
  return std::acos(std::clamp(dot, -1.0, 1.0)) * (180.0 / M_PI);
}

double quaternion_upright_tilt_deg(double qx, double qy, double qz, double qw)
{
  const double norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;
  if (norm_sq < 1e-12) {
    return 180.0;
  }
  const double dot = 1.0 - 2.0 * (qx * qx + qy * qy) / norm_sq;
  return std::acos(std::clamp(dot, -1.0, 1.0)) * (180.0 / M_PI);
}

double compute_payload_upright_tilt_deg(const geometry_msgs::msg::Quaternion & q)
{
  return quaternion_upright_tilt_deg(q.x, q.y, q.z, q.w);
}

double compute_tool_tilt_deg(double qx, double qy, double qz, double qw)
{
  const double norm_sq = qx * qx + qy * qy + qz * qz + qw * qw;
  if (norm_sq < 1e-12) {
    return 180.0;
  }
  const double dot = (qx * qx + qy * qy - qz * qz - qw * qw) / norm_sq;
  return std::acos(std::clamp(dot, -1.0, 1.0)) * (180.0 / M_PI);
}

double compute_tool_tilt_deg(const geometry_msgs::msg::Quaternion & q)
{
  return compute_tool_tilt_deg(q.x, q.y, q.z, q.w);
}

bool tilt_deg_from_up_dot_checked(double up_dot, double & tilt_deg_out)
{
  if (!std::isfinite(up_dot)) {
    return false;
  }
  const double tilt_deg = std::acos(std::clamp(up_dot, -1.0, 1.0)) * (180.0 / M_PI);
  if (!std::isfinite(tilt_deg)) {
    return false;
  }
  tilt_deg_out = tilt_deg;
  return true;
}

bool is_valid_attempt_index(int attempt)
{
  return attempt == 0 || attempt == 1;
}

const char * attempt_state_label(int attempt)
{
  switch (attempt) {
    case 0: return "State E";
    case 1: return "State E2";
    default: return "INVALID_ATTEMPT";
  }
}

bool goal_acceptance_count_ok(
  int attempt0_accepted_count, int attempt1_accepted_count, int attempt2_plus_accepted_count)
{
  return attempt0_accepted_count == 1 &&
    attempt1_accepted_count == 1 &&
    attempt2_plus_accepted_count == 0;
}

bool budget_exhausted_telemetry_valid(
  int replan_count, int max_replans, int attempt, Result result)
{
  return replan_count == 1 &&
    max_replans == 1 &&
    attempt == 1 &&
    result == Result::TRANSPORT_REPLAN_LIMIT_REACHED;
}

moveit_msgs::msg::Constraints create_transport_orientation_constraint(
  const std::string & link_name,
  const geometry_msgs::msg::Quaternion & target_orientation,
  double tilt_tol_rad,
  double yaw_tol_rad,
  const std::string & frame_id)
{
  moveit_msgs::msg::Constraints constraints;
  constraints.name = "transport_orientation_constraint";

  moveit_msgs::msg::OrientationConstraint oc;
  oc.header.frame_id = frame_id.empty() ? "world" : frame_id;
  oc.link_name = link_name;
  oc.orientation = target_orientation;
  oc.absolute_x_axis_tolerance = tilt_tol_rad;
  oc.absolute_y_axis_tolerance = tilt_tol_rad;
  oc.absolute_z_axis_tolerance = yaw_tol_rad;
  oc.weight = 1.0;

  constraints.orientation_constraints.push_back(oc);
  return constraints;
}

TransportCoordinator::TransportCoordinator(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface & arm,
  const TransportParams & params,
  const geometry_msgs::msg::Pose & above_place,
  std::shared_ptr<PlanningSceneManager> scene_manager)
: node_(std::move(node)),
  arm_(arm),
  params_(params),
  above_place_(above_place),
  scene_manager_(std::move(scene_manager))
{
  scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
    params_.transport_monitor_scene_service_name);

  obstacle_tracker_state_ = std::make_shared<ObstacleTrackerState>();
  obstacle_sub_ = node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/collision_object", rclcpp::QoS(10).reliable(),
    [state = obstacle_tracker_state_](moveit_msgs::msg::CollisionObject::ConstSharedPtr msg) {
      if (msg->id != kDynamicObstacleId) {
        return;
      }
      std::lock_guard<std::mutex> lock(state->mutex);
      state->latest.valid = true;
      state->latest.stamp_s = rclcpp::Time(msg->header.stamp).seconds();
      state->latest.pose = msg->pose;
      state->cv.notify_all();
    });
}

bool TransportCoordinator::waitForFreshObstacleUpdate(
  double baseline_time_s, double timeout_s, ObstacleSample & sample_out)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));

  std::unique_lock<std::mutex> lock(obstacle_tracker_state_->mutex);
  while (std::chrono::steady_clock::now() < deadline) {
    const double now_s = node_->now().seconds();
    if (obstacle_tracker_state_->latest.valid &&
      is_obstacle_fresh_post_time(
        obstacle_tracker_state_->latest.stamp_s, baseline_time_s, now_s, kObstacleStaleThresholdS))
    {
      sample_out = obstacle_tracker_state_->latest;
      return true;
    }
    obstacle_tracker_state_->cv.wait_until(lock, deadline);
  }
  const double now_s = node_->now().seconds();
  if (obstacle_tracker_state_->latest.valid &&
    is_obstacle_fresh_post_time(
      obstacle_tracker_state_->latest.stamp_s, baseline_time_s, now_s, kObstacleStaleThresholdS))
  {
    sample_out = obstacle_tracker_state_->latest;
    return true;
  }
  return false;
}

const char * to_string(GateSendStatus status)
{
  switch (status) {
    case GateSendStatus::OK:             return "OK";
    case GateSendStatus::THREW:          return "THREW";
    case GateSendStatus::INVALID_FUTURE: return "INVALID_FUTURE";
  }
  return "UNKNOWN";
}

const char * to_string(PreReplanGateOutcome outcome)
{
  switch (outcome) {
    case PreReplanGateOutcome::DISABLED:            return "DISABLED";
    case PreReplanGateOutcome::SUCCESS:             return "SUCCESS";
    case PreReplanGateOutcome::SERVICE_UNAVAILABLE: return "SERVICE_UNAVAILABLE";
    case PreReplanGateOutcome::SEND_FAILED:         return "SEND_FAILED";
    case PreReplanGateOutcome::TIMEOUT:             return "TIMEOUT";
    case PreReplanGateOutcome::SUCCESS_FALSE:       return "SUCCESS_FALSE";
    case PreReplanGateOutcome::SHUTDOWN:            return "SHUTDOWN";
  }
  return "UNKNOWN";
}

PreReplanGateOutcome run_pre_replan_gate_handshake(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const std::string & service_name, double timeout_s, double & latency_ms_out,
  const GateRequestSendFn & send_fn)
{
  latency_ms_out = 0.0;

  // Disabled path: no client, no discovery, no wait. Single branch.
  if (service_name.empty()) {
    return PreReplanGateOutcome::DISABLED;
  }

  // STEADY clock for every deadline here: the handshake must not hang if
  // simulation time stops advancing.
  const auto t_begin = std::chrono::steady_clock::now();
  const auto deadline = t_begin +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));
  auto elapsed_ms = [&t_begin]() {
      return std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_begin).count();
    };
  auto finish = [&](PreReplanGateOutcome o) {
      latency_ms_out = elapsed_ms();
      return o;
    };

  if (!client) {
    client = node->create_client<std_srvs::srv::Trigger>(service_name);
  }

  // Bounded availability wait, re-checking shutdown so Ctrl-C during the
  // handshake reports SHUTDOWN instead of spinning out the full timeout.
  while (!client->wait_for_service(std::chrono::milliseconds(50))) {
    if (!rclcpp::ok()) {
      return finish(PreReplanGateOutcome::SHUTDOWN);
    }
    if (std::chrono::steady_clock::now() >= deadline) {
      return finish(PreReplanGateOutcome::SERVICE_UNAVAILABLE);
    }
  }

  // CORRECTION A: the send is an external-system boundary that reports failure
  // by THROWING (rcl_send_request() -> throw_from_rcl_error(); see the header's
  // GateSendStatus comment for the exact installed implementation lines). The
  // previous `future.valid()` test after the call was unreachable as a
  // send-failure detector -- rclcpp always returns a valid future when the call
  // returns normally -- and an exception would have escaped executeTransport()
  // entirely instead of becoming the typed gate failure. Every send failure now
  // becomes PreReplanGateOutcome::SEND_FAILED, which
  // TransportCoordinator::runPreReplanSceneGate() reports as
  // "PRE_REPLAN_SCENE_GATE_FAILED reason=SEND_FAILED" and
  // executeTransport() converts into Result::TRANSPORT_PRE_REPLAN_GATE_FAILED.
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  std::optional<rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId> future_opt;
  std::string send_error;
  const GateSendStatus send_status = send_gate_request_guarded(
    [&client, &request, &send_fn]() {
      return send_fn ? send_fn(*client, request) : client->async_send_request(request);
    },
    future_opt, send_error);
  if (send_status != GateSendStatus::OK) {
    RCLCPP_ERROR(
      node->get_logger(),
      "M3 C3 PRE_REPLAN_GATE_SEND_FAILED: service=%s status=%s exception=\"%s\"",
      service_name.c_str(), to_string(send_status), send_error.c_str());
    return finish(PreReplanGateOutcome::SEND_FAILED);
  }
  auto & future = *future_opt;

  // Raw future wait, never rclcpp::spin_until_future_complete(): the node is
  // already owned by a SingleThreadedExecutor spinning on its own dedicated
  // thread (m3_grasp.cpp), so this caller-thread wait cannot starve the
  // callback that completes it -- the same idiom acquireCoherentScene() uses.
  const auto remaining = deadline - std::chrono::steady_clock::now();
  if (remaining <= std::chrono::steady_clock::duration::zero() ||
    future.wait_for(remaining) != std::future_status::ready)
  {
    return finish(
      rclcpp::ok() ? PreReplanGateOutcome::TIMEOUT : PreReplanGateOutcome::SHUTDOWN);
  }

  const auto response = future.get();
  if (!response || !response->success) {
    return finish(PreReplanGateOutcome::SUCCESS_FALSE);
  }
  return finish(PreReplanGateOutcome::SUCCESS);
}

bool TransportCoordinator::runPreReplanSceneGate(
  int attempt, std::string & failure_reason_out, PreReplanGateOutcome & outcome_out)
{
  failure_reason_out.clear();
  outcome_out = PreReplanGateOutcome::DISABLED;
  const std::string & service_name = params_.transport_pre_replan_gate_service_name;
  telemetry_.pre_replan_gate_enabled = !service_name.empty();
  if (!telemetry_.pre_replan_gate_enabled) {
    return true;
  }

  const auto & logger = node_->get_logger();
  const double timeout_s = params_.transport_pre_replan_gate_timeout_s;
  RCLCPP_INFO(
    logger,
    "M3 C3 PRE_REPLAN_SCENE_GATE_BEGIN attempt=%d service=%s timeout_s=%.3f",
    attempt, service_name.c_str(), timeout_s);

  double latency_ms = 0.0;
  const PreReplanGateOutcome outcome = run_pre_replan_gate_handshake(
    node_, pre_replan_gate_client_, service_name, timeout_s, latency_ms);
  outcome_out = outcome;
  telemetry_.pre_replan_gate_latency_ms = latency_ms;

  if (!pre_replan_gate_ok(outcome)) {
    failure_reason_out = to_string(outcome);
    telemetry_.pre_replan_gate_completed = false;
    telemetry_.pre_replan_gate_failure_reason = failure_reason_out;
    RCLCPP_ERROR(
      logger,
      "M3 C3 PRE_REPLAN_SCENE_GATE_FAILED attempt=%d reason=%s latency_ms=%.3f",
      attempt, failure_reason_out.c_str(), latency_ms);
    return false;
  }

  telemetry_.pre_replan_gate_completed = true;
  // ROS/node time on purpose -- this becomes a SCENE_A freshness baseline
  // compared against /collision_object header stamps (also ROS/sim time).
  telemetry_.pre_replan_gate_done_stamp_s = node_->now().seconds();
  RCLCPP_INFO(
    logger,
    "M3 C3 PRE_REPLAN_SCENE_GATE_DONE attempt=%d latency_ms=%.3f t_gate_done=%.9f",
    attempt, latency_ms, telemetry_.pre_replan_gate_done_stamp_s);
  return true;
}

bool TransportCoordinator::acquireCoherentScene(
  const ObstacleSample & expected_sample, double timeout_s,
  moveit_msgs::msg::PlanningScene & scene_msg_out, double & pose_err_out,
  double & latency_ms_out)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));

  while (std::chrono::steady_clock::now() < deadline) {
    if (!scene_client_->wait_for_service(kSceneServiceTimeout)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components =
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
      moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
      moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
      moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
      moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS;

    const auto t_req = std::chrono::steady_clock::now();
    auto future = scene_client_->async_send_request(req);
    if (future.wait_for(kSceneServiceTimeout) != std::future_status::ready) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }
    const auto t_resp = std::chrono::steady_clock::now();
    latency_ms_out = std::chrono::duration<double, std::milli>(t_resp - t_req).count();

    const auto scene = future.get()->scene;
    const auto obstacle_it = std::find_if(
      scene.world.collision_objects.begin(), scene.world.collision_objects.end(),
      [](const auto & co) { return co.id == kDynamicObstacleId; });

    if (obstacle_it != scene.world.collision_objects.end() && !obstacle_it->primitive_poses.empty()) {
      const auto scene_pose = PlanningSceneManager::effectivePrimitivePose(*obstacle_it, 0);
      const double dx = scene_pose.position.x - expected_sample.pose.position.x;
      const double dy = scene_pose.position.y - expected_sample.pose.position.y;
      const double dz = scene_pose.position.z - expected_sample.pose.position.z;
      pose_err_out = std::sqrt(dx * dx + dy * dy + dz * dz);

      if (is_pose_coherent(scene_pose, expected_sample.pose, 1.0e-4, 1.0e-3)) {
        scene_msg_out = scene;
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return false;
}

bool TransportCoordinator::acquireFreshCoherentScene(
  double baseline_time_s, double sample_timeout_s, double scene_timeout_s,
  SceneAcquisition & out)
{
  out = SceneAcquisition{};

  // CORRECTION C. ONE deadline for the whole acquisition, equal to the sum of
  // the two bounded steps this replaces (fresh-sample wait + coherent-scene
  // wait). Every retry below is spent INSIDE that same budget -- the budget is
  // never extended, the 250 ms staleness threshold is never relaxed, and the
  // loop cannot run past the deadline.
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(sample_timeout_s + scene_timeout_s));
  auto remaining_s = [&deadline]() {
      return std::chrono::duration<double>(
        deadline - std::chrono::steady_clock::now()).count();
    };

  out.failure = SceneAcquisition::Failure::NO_FRESH_SAMPLE;
  while (remaining_s() > 0.0) {
    // 1. Obtain a qualifying fresh obstacle sample (stamp >= baseline, age
    //    within the authority at selection time).
    ObstacleSample sample;
    if (!waitForFreshObstacleUpdate(
        baseline_time_s, std::min(sample_timeout_s, remaining_s()), sample))
    {
      out.failure = SceneAcquisition::Failure::NO_FRESH_SAMPLE;
      break;
    }
    ++out.sample_attempts;
    out.sample = sample;
    out.sample_age_at_request_ms = (node_->now().seconds() - sample.stamp_s) * 1000.0;

    if (remaining_s() <= 0.0) {
      break;
    }

    // 2. Fetch a scene that coheres with THAT sample.
    moveit_msgs::msg::PlanningScene scene_msg;
    double pose_err = 0.0;
    double latency_ms = 0.0;
    const bool cohered = acquireCoherentScene(
      sample, std::min(scene_timeout_s, remaining_s()), scene_msg, pose_err, latency_ms);
    out.pose_err_m = pose_err;
    out.latency_ms = latency_ms;
    if (!cohered) {
      out.failure = SceneAcquisition::Failure::NO_COHERENT_SCENE;
      continue;  // a newer sample may still cohere inside the remaining budget
    }

    // 3. Recompute the sample's age at RESPONSE/ACCEPTANCE time, and
    // 4. refuse to accept this scene if the sample that authorizes it has
    //    aged out of the authority while the response was in flight.
    const double accept_time_s = node_->now().seconds();
    out.sample_age_at_accept_ms = (accept_time_s - sample.stamp_s) * 1000.0;
    const SceneAcceptanceDecision decision = decide_scene_acceptance(
      sample.stamp_s, baseline_time_s, accept_time_s, remaining_s(),
      kObstacleStaleThresholdS);
    if (decision != SceneAcceptanceDecision::ACCEPT) {
      ++out.expired_at_accept_count;
      out.failure = SceneAcquisition::Failure::SAMPLE_EXPIRED_AT_ACCEPT;
      if (decision == SceneAcceptanceDecision::BUDGET_EXHAUSTED) {
        break;
      }
      continue;  // 5. obtain a newer sample and retry, still inside the budget
    }

    out.scene_msg = std::move(scene_msg);
    out.failure = SceneAcquisition::Failure::NONE;
    return true;
  }
  return false;
}

bool TransportCoordinator::validateCandidateTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::shared_ptr<planning_scene::PlanningScene> & scene_b,
  std::string & collision_info_out)
{
  const std::size_t n_points = trajectory.points.size();
  const std::size_t n_joints = trajectory.joint_names.size();
  if (n_points < 2 || n_joints != 6) {
    collision_info_out = "INVALID_TRAJECTORY_SHAPE";
    return false;
  }

  std::vector<std::vector<double>> waypoints(n_points);
  std::vector<double> times(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    waypoints[i] = trajectory.points[i].positions;
    times[i] = rclcpp::Duration(trajectory.points[i].time_from_start).seconds();
  }
  const double total_duration_s = times.back();

  // Dense sampling schedule: t=0, stepped by future_sample_dt_s, and exact total_duration_s
  const double dt_s = params_.transport_monitor_future_sample_dt_s;
  const auto sample_times = generate_future_sample_times(0.0, dt_s, total_duration_s);

  telemetry_.candidate_max_payload_tilt_deg = 0.0;
  telemetry_.candidate_max_tool_tilt_deg = 0.0;

  for (std::size_t k = 0; k < sample_times.size(); ++k) {
    const TimeSegmentBlend blend = find_time_segment(times, sample_times[k]);
    const std::vector<double> interp_positions = interpolate_joint_positions(waypoints, blend);

    // INVARIANT B: Construct sample_state strictly from scene_b to retain attached payload
    moveit::core::RobotState sample_state = scene_b->getCurrentState();
    for (std::size_t j = 0; j < n_joints; ++j) {
      sample_state.setVariablePosition(trajectory.joint_names[j], interp_positions[j]);
    }
    sample_state.update(true);

    const auto * attached_body = sample_state.getAttachedBody(kAttachedTargetId);
    if (!attached_body || attached_body->getAttachedLinkName() != kExpectedAttachLink) {
      collision_info_out = "ATTACHED_BODY_CORRUPT_DURING_SAMPLING";
      return false;
    }

    // Orientation acceptance check: verify payload and tool tilt stay within threshold
    Eigen::Isometry3d body_tf;
    if (!attached_body->getGlobalCollisionBodyTransforms().empty()) {
      body_tf = attached_body->getGlobalCollisionBodyTransforms()[0];
    } else if (!attached_body->getShapePosesInLinkFrame().empty()) {
      body_tf = sample_state.getGlobalLinkTransform(attached_body->getAttachedLinkName()) *
        attached_body->getShapePosesInLinkFrame()[0];
    } else {
      body_tf = sample_state.getGlobalLinkTransform(attached_body->getAttachedLinkName());
    }
    const double payload_up_dot = -body_tf.rotation().col(2).z();

    const Eigen::Isometry3d & tool_tf = sample_state.getGlobalLinkTransform("tool0");
    const double tool_up_dot = -tool_tf.rotation().col(2).z();

    // Nonfinite guard: a NaN/Inf dot value would otherwise survive
    // std::clamp() unchanged (clamp's comparisons are both false for NaN)
    // and propagate through acos() into payload_tilt_deg/tool_tilt_deg, at
    // which point the "> kMaxAllowedPayloadTiltDeg" comparison below is
    // ALSO false for NaN -- an invalid orientation would pass undetected.
    // tilt_deg_from_up_dot_checked() rejects explicitly, before either
    // value can reach that comparison; see its own unit tests for the
    // NaN/+Inf/-Inf and finite-passthrough cases.
    double payload_tilt_deg = 0.0;
    double tool_tilt_deg = 0.0;
    if (!tilt_deg_from_up_dot_checked(payload_up_dot, payload_tilt_deg) ||
      !tilt_deg_from_up_dot_checked(tool_up_dot, tool_tilt_deg))
    {
      std::ostringstream ss;
      ss << "ORIENTATION_NONFINITE: sample_index=" << k
         << " time_s=" << sample_times[k]
         << " payload_up_dot=" << payload_up_dot
         << " tool_up_dot=" << tool_up_dot;
      collision_info_out = ss.str();
      return false;
    }

    telemetry_.candidate_max_payload_tilt_deg = std::max(
      telemetry_.candidate_max_payload_tilt_deg, payload_tilt_deg);
    telemetry_.candidate_max_tool_tilt_deg = std::max(
      telemetry_.candidate_max_tool_tilt_deg, tool_tilt_deg);

    if (payload_tilt_deg > kMaxAllowedPayloadTiltDeg || tool_tilt_deg > kMaxAllowedPayloadTiltDeg) {
      std::ostringstream ss;
      ss << "ORIENTATION_CONSTRAINT_VIOLATED: sample_index=" << k
         << " time_s=" << sample_times[k]
         << " payload_tilt_deg=" << payload_tilt_deg
         << " tool_tilt_deg=" << tool_tilt_deg
         << " max_allowed_deg=" << kMaxAllowedPayloadTiltDeg;
      collision_info_out = ss.str();
      return false;
    }

    collision_detection::CollisionRequest req;
    req.contacts = true;
    req.max_contacts = 10;
    collision_detection::CollisionResult res;
    scene_b->checkCollision(req, res, sample_state);

    if (res.collision) {
      std::ostringstream ss;
      ss << "sample_index=" << k << " time_s=" << sample_times[k] << " pairs=";
      for (const auto & pair : res.contacts) {
        ss << pair.first.first << "<->" << pair.first.second << "; ";
      }
      collision_info_out = ss.str();
      return false;
    }
  }
  return true;
}

Result TransportCoordinator::executeTransport(
  const moveit::planning_interface::MoveGroupInterface::Plan & initial_plan)
{
  const auto logger = node_->get_logger();

  TransportExecutionParams exec_params;
  exec_params.fjt_action_name = params_.transport_fjt_action_name;
  exec_params.controller_name = params_.transport_controller_name;
  exec_params.controller_wait_timeout_s = params_.transport_controller_wait_timeout_s;
  exec_params.allowed_start_tolerance_rad = params_.transport_allowed_start_tolerance_rad;
  exec_params.execution_duration_scaling = params_.transport_execution_duration_scaling;
  exec_params.goal_duration_margin_s = params_.transport_goal_duration_margin_s;
  exec_params.joint_states_topic = params_.joint_states_topic;
  exec_params.stationary_velocity_eps_rad_s = params_.stationary_velocity_eps_rad_s;
  exec_params.stationary_consecutive_samples = params_.stationary_consecutive_samples;
  exec_params.stationary_timeout_s = params_.stationary_timeout_s;

  TransportMonitorParams monitor_params;
  monitor_params.enabled = params_.transport_monitor_enabled;
  monitor_params.rate_hz = params_.transport_monitor_rate_hz;
  monitor_params.future_sample_dt_s = params_.transport_monitor_future_sample_dt_s;
  monitor_params.scene_service_name = params_.transport_monitor_scene_service_name;

  // Attempt 0: Initial trajectory execution
  TransportExecutor executor_0(node_, exec_params);
  double max_start_error_rad_0 = 0.0;
  const Result validate_0 = executor_0.preSendValidate(
    initial_plan.trajectory.joint_trajectory, arm_, max_start_error_rad_0);
  if (!ok(validate_0)) {
    telemetry_.final_result = validate_0;
    return validate_0;
  }

  auto stop_signal_0 = std::make_shared<TransportReactiveStopSignal>(
    params_.transport_reactive_stop_enabled);
  TransportPathMonitor monitor_0(node_, arm_.getRobotModel(), monitor_params, stop_signal_0);
  monitor_0.start(initial_plan.trajectory.joint_trajectory);

  const Result exec_r0 = executor_0.executeAndWait(
    initial_plan.trajectory.joint_trajectory, stop_signal_0, /*attempt=*/0);
  monitor_0.stop();  // Unconditionally joined before moving forward
  logMonitorSummary(logger, 0, monitor_0.summary(), exec_r0 == Result::TRANSPORT_COLLISION_STOPPED);

  if (ok(exec_r0)) {
    telemetry_.final_result = Result::SUCCESS;
    RCLCPP_INFO(logger, "M3 C3 TRANSPORT_SUCCESS attempt=0 replan_count=0");
    return Result::SUCCESS;
  }

  if (exec_r0 != Result::TRANSPORT_COLLISION_STOPPED) {
    telemetry_.final_result = exec_r0;
    RCLCPP_ERROR(
      logger, "M3 C3 ATTEMPT_0_FAILED result=%s fjt_error_code=%d",
      to_string(exec_r0), executor_0.lastFjtErrorCode());
    return exec_r0;
  }

  // Reactive Replan Recovery
  telemetry_.replan_attempted = true;
  telemetry_.replan_count = 1;
  telemetry_.state_e = executor_0.lastSettledStateE();
  logSettledStateTelemetry(logger, /*attempt=*/0, telemetry_.state_e);

  RCLCPP_INFO(
    logger,
    "M3 C3 REPLAN_TRIGGERED replan_count=1 max_replans=1 "
    "state_e_captured=%d t_settle_sample=%.6f",
    telemetry_.state_e.captured,
    telemetry_.state_e.stamp.nanoseconds() > 0 ? telemetry_.state_e.stamp.seconds() : -1.0);

  // 1. Validate State E
  std::string state_e_err;
  if (!validate_state_e(
      telemetry_.state_e, arm_.getJointNames(), *arm_.getRobotModel(), state_e_err))
  {
    RCLCPP_ERROR(logger, "M3 C3 STATE_E_INVALID: %s", state_e_err.c_str());
    telemetry_.final_result = Result::CONFIG_ERROR;
    return Result::CONFIG_ERROR;
  }

  const double t_settle_s = telemetry_.state_e.stamp.nanoseconds() > 0 ?
    telemetry_.state_e.stamp.seconds() : node_->now().seconds();

  // 1b. OPTIONAL pre-replan scene gate.
  //
  // Strictly AFTER the attempt-0 exact cancel, terminal CANCELED, physical
  // settle and State E validation above, and strictly BEFORE the SCENE_A
  // acquisition below. This is the ONLY place the gate is ever invoked:
  // executeTransport() is straight-line with no loop or recursion over
  // attempts, and this statement sits inside the single one-shot replan block,
  // so a second gate after an attempt-1 collision is structurally impossible.
  // Inert (one branch, no client, no wait) unless a service name is configured.
  {
    std::string gate_failure_reason;
    PreReplanGateOutcome gate_outcome = PreReplanGateOutcome::DISABLED;
    if (!runPreReplanSceneGate(/*attempt=*/0, gate_failure_reason, gate_outcome)) {
      // No SCENE_A, no replacement plan, no attempt-1 goal -- every one of
      // those statements is BELOW this return. lift_transport_place() returns
      // on any !ok() result before Stage 5, so PLACE/release/detach/retreat
      // stay suppressed and the payload stays attached. The mapping itself is
      // pre_replan_gate_result(), the single pure function the unit tests
      // assert on, so a SEND_FAILED outcome (including the exception path)
      // provably lands on exactly this typed result.
      const Result gate_result = pre_replan_gate_result(gate_outcome);
      telemetry_.final_result = gate_result;
      return gate_result;
    }
  }

  // 2. SCENE_A Acquisition (Post-Settle Coherence Gate)
  //
  // Freshness baseline is the LATER of the physical settle and, when the gate
  // ran, the moment it confirmed the external transition complete. Without the
  // gate, pre_replan_gate_done_stamp_s stays 0.0 and this is exactly t_settle_s
  // -- the C3A/C3B-qualified behavior. With it, an obstacle update that
  // predates the transition can no longer satisfy SCENE_A merely by postdating
  // the settle. Both quantities are ROS/sim-time seconds, matching the
  // /collision_object header stamps this is compared against.
  const double scene_a_baseline_s = scene_a_freshness_baseline_s(
    t_settle_s, telemetry_.pre_replan_gate_done_stamp_s);

  telemetry_.scene_a_request_stamp_s = node_->now().seconds();
  SceneAcquisition acq_a;
  const bool scene_a_ok = acquireFreshCoherentScene(
    scene_a_baseline_s, /*sample_timeout_s=*/1.0, /*scene_timeout_s=*/1.0, acq_a);
  telemetry_.scene_a_obstacle_stamp_s = acq_a.sample.stamp_s;
  telemetry_.scene_a_obstacle_age_ms = acq_a.sample_age_at_request_ms;
  telemetry_.scene_a_obstacle_age_at_accept_ms = acq_a.sample_age_at_accept_ms;
  telemetry_.scene_a_sample_attempts = acq_a.sample_attempts;
  telemetry_.scene_a_expired_at_accept_count = acq_a.expired_at_accept_count;
  telemetry_.scene_a_pose_match_error_m = acq_a.pose_err_m;
  telemetry_.scene_a_latency_ms = acq_a.latency_ms;

  if (!scene_a_ok) {
    // The pre-existing distinct diagnostics are preserved verbatim; the third
    // is CORRECTION C's new cause (every cohering scene arrived authorized by
    // a sample that had already aged past the 250 ms authority).
    switch (acq_a.failure) {
      case SceneAcquisition::Failure::NO_COHERENT_SCENE:
        RCLCPP_ERROR(
          logger,
          "M3 C3 SCENE_A_COHERENCE_TIMEOUT: PlanningScene did not cohere with obstacle update within 1.0s (err=%.6f m)",
          telemetry_.scene_a_pose_match_error_m);
        break;
      case SceneAcquisition::Failure::SAMPLE_EXPIRED_AT_ACCEPT:
        RCLCPP_ERROR(
          logger,
          "M3 C3 SCENE_A_SAMPLE_EXPIRED_AT_ACCEPT: obstacle sample aged %.3f ms "
          "(> %.0f ms) by the time the PlanningScene response was accepted; "
          "expired_at_accept_count=%d sample_attempts=%d",
          telemetry_.scene_a_obstacle_age_at_accept_ms, kObstacleStaleThresholdS * 1000.0,
          telemetry_.scene_a_expired_at_accept_count, telemetry_.scene_a_sample_attempts);
        break;
      case SceneAcquisition::Failure::NO_FRESH_SAMPLE:
      case SceneAcquisition::Failure::NONE:
      default:
        RCLCPP_ERROR(
          logger,
          "M3 C3 SCENE_A_UPDATE_TIMEOUT: no dynamic_obstacle_0 update with t >= %.6f within 1.0s",
          scene_a_baseline_s);
        break;
    }
    telemetry_.final_result = Result::SCENE_STALE_OR_CORRUPT;
    return Result::SCENE_STALE_OR_CORRUPT;
  }
  const moveit_msgs::msg::PlanningScene & scene_a_msg = acq_a.scene_msg;
  telemetry_.scene_a_response_stamp_s = node_->now().seconds();

  // CORRECTION D: SCENE_A integrity is asserted against the RETAINED snapshot
  // -- `scene_a_msg`, the exact message loaded into scene_a below -- and not
  // against a second PlanningScene fetched from the service. Both calls take
  // that one snapshot by const reference and neither mutates it.
  {
    std::string scene_err;
    if (!verify_retained_snapshot_obstacle_and_attachment(
        scene_a_msg, acq_a.sample.pose, scene_err))
    {
      RCLCPP_ERROR(logger, "M3 C3 SCENE_A_VERIFY_FAILED: %s", scene_err.c_str());
      telemetry_.final_result = Result::SCENE_STALE_OR_CORRUPT;
      return Result::SCENE_STALE_OR_CORRUPT;
    }
    if (scene_manager_) {
      auto spec = scene_manager_->expectedSceneSpec();
      // SCENE_A deliberately does not request LINK_PADDING_AND_SCALING (see
      // acquireCoherentScene()'s component set), so this snapshot carries no
      // padding/scale section and must not be asked to prove one. Stated
      // explicitly rather than passing vacuously.
      spec.check_link_padding_and_scaling = false;
      if (!PlanningSceneManager::verifyExpectedSceneSnapshot(scene_a_msg, spec, scene_err)) {
        RCLCPP_ERROR(logger, "M3 C3 SCENE_A_VERIFY_FAILED: %s", scene_err.c_str());
        telemetry_.final_result = Result::SCENE_STALE_OR_CORRUPT;
        return Result::SCENE_STALE_OR_CORRUPT;
      }
    }
  }

  auto scene_a = std::make_shared<planning_scene::PlanningScene>(arm_.getRobotModel());
  if (!scene_a->usePlanningSceneMsg(scene_a_msg)) {
    RCLCPP_ERROR(logger, "M3 C3 SCENE_A_LOAD_FAILED: could not load PlanningSceneMsg into scene_a");
    telemetry_.final_result = Result::SCENE_STALE_OR_CORRUPT;
    return Result::SCENE_STALE_OR_CORRUPT;
  }

  RCLCPP_INFO(
    logger,
    "M3 C3 SCENE_A_ACQUIRED t_request=%.6f t_response=%.6f latency_ms=%.3f "
    "obstacle_age_ms=%.3f obstacle_age_at_accept_ms=%.3f sample_attempts=%d "
    "expired_at_accept_count=%d pose_match_error_m=%.6f",
    telemetry_.scene_a_request_stamp_s, telemetry_.scene_a_response_stamp_s,
    telemetry_.scene_a_latency_ms, telemetry_.scene_a_obstacle_age_ms,
    telemetry_.scene_a_obstacle_age_at_accept_ms, telemetry_.scene_a_sample_attempts,
    telemetry_.scene_a_expired_at_accept_count,
    telemetry_.scene_a_pose_match_error_m);

  // 3. Construct Attachment-Preserving Start State from SCENE_A + State E
  moveit::core::RobotState start_state = scene_a->getCurrentState();
  for (std::size_t i = 0; i < telemetry_.state_e.joint_names.size(); ++i) {
    start_state.setVariablePosition(
      telemetry_.state_e.joint_names[i], telemetry_.state_e.positions[i]);
  }
  start_state.update(true);

  const auto * attached_target = start_state.getAttachedBody(kAttachedTargetId);
  if (!attached_target || attached_target->getAttachedLinkName() != kExpectedAttachLink) {
    RCLCPP_ERROR(
      logger, "M3 C3 START_STATE_ATTACHMENT_LOST: target not attached to %s in start_state",
      kExpectedAttachLink);
    telemetry_.final_result = Result::CONFIG_ERROR;
    return Result::CONFIG_ERROR;
  }
  arm_.setStartState(start_state);

  // 4. Replacement Planning to the EXACT ORIGINAL above_place target
  arm_.setPoseTarget(above_place_);
  telemetry_.plan_start_stamp_s = node_->now().seconds();
  moveit::planning_interface::MoveGroupInterface::Plan replacement_plan;

  // Enforce orientation path constraint with RAII cleanup
  const std::string ee_link = arm_.getEndEffectorLink().empty() ? "tool0" : arm_.getEndEffectorLink();
  const std::string planning_frame = arm_.getPlanningFrame().empty() ? "world" : arm_.getPlanningFrame();
  const auto orientation_constraints = create_transport_orientation_constraint(
    ee_link,
    above_place_.orientation,
    kOrientationPathConstraintTiltTolRad,
    kOrientationPathConstraintYawTolRad,
    planning_frame);

  const auto original_planning_time = arm_.getPlanningTime();
  arm_.setPlanningTime(std::max(original_planning_time, 10.0));

  moveit::core::MoveItErrorCode plan_code;
  {
    ScopedPathConstraint constraint_scope(arm_, orientation_constraints);
    plan_code = arm_.plan(replacement_plan);
  }
  arm_.setPlanningTime(original_planning_time);
  telemetry_.plan_done_stamp_s = node_->now().seconds();
  telemetry_.plan_latency_ms =
    (telemetry_.plan_done_stamp_s - telemetry_.plan_start_stamp_s) * 1000.0;

  arm_.setStartStateToCurrentState();

  if (plan_code != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      logger, "M3 C3 REPLAN_FAILED: MoveGroup plan() returned error code %d (latency=%.3f ms)",
      plan_code.val, telemetry_.plan_latency_ms);
    telemetry_.final_result = Result::PLAN_FAILURE;
    return Result::PLAN_FAILURE;
  }

  const auto & rep_traj = replacement_plan.trajectory.joint_trajectory;
  telemetry_.replacement_waypoint_count = rep_traj.points.size();
  telemetry_.replacement_planned_duration_s =
    rep_traj.points.empty() ? 0.0 :
    rclcpp::Duration(rep_traj.points.back().time_from_start).seconds();

  RCLCPP_INFO(
    logger,
    "M3 C3 REPLAN_PLAN_OK latency_ms=%.3f waypoints=%zu planned_duration_s=%.4f",
    telemetry_.plan_latency_ms, telemetry_.replacement_waypoint_count,
    telemetry_.replacement_planned_duration_s);

  // 5. SCENE_B Acquisition (Strict Post-Plan Gate — INVARIANT A)
  // INVARIANT A is unchanged: SCENE_B's causal baseline stays t_plan_done, so
  // only an obstacle update that postdates the completed replacement plan can
  // authorize it. CORRECTION C adds the acceptance-time half of the same
  // authority, exactly as for SCENE_A.
  telemetry_.scene_b_request_stamp_s = node_->now().seconds();
  SceneAcquisition acq_b;
  const bool scene_b_ok = acquireFreshCoherentScene(
    telemetry_.plan_done_stamp_s, /*sample_timeout_s=*/1.0, /*scene_timeout_s=*/1.0, acq_b);
  telemetry_.scene_b_obstacle_stamp_s = acq_b.sample.stamp_s;
  telemetry_.scene_b_obstacle_age_ms = acq_b.sample_age_at_request_ms;
  telemetry_.scene_b_obstacle_age_at_accept_ms = acq_b.sample_age_at_accept_ms;
  telemetry_.scene_b_sample_attempts = acq_b.sample_attempts;
  telemetry_.scene_b_expired_at_accept_count = acq_b.expired_at_accept_count;
  telemetry_.scene_b_pose_match_error_m = acq_b.pose_err_m;
  telemetry_.scene_b_latency_ms = acq_b.latency_ms;

  if (!scene_b_ok) {
    switch (acq_b.failure) {
      case SceneAcquisition::Failure::NO_COHERENT_SCENE:
        RCLCPP_ERROR(
          logger,
          "M3 C3 SCENE_B_COHERENCE_TIMEOUT: PlanningScene did not cohere with post-plan update within 1.0s (err=%.6f m)",
          telemetry_.scene_b_pose_match_error_m);
        break;
      case SceneAcquisition::Failure::SAMPLE_EXPIRED_AT_ACCEPT:
        RCLCPP_ERROR(
          logger,
          "M3 C3 SCENE_B_SAMPLE_EXPIRED_AT_ACCEPT: obstacle sample aged %.3f ms "
          "(> %.0f ms) by the time the PlanningScene response was accepted; "
          "expired_at_accept_count=%d sample_attempts=%d",
          telemetry_.scene_b_obstacle_age_at_accept_ms, kObstacleStaleThresholdS * 1000.0,
          telemetry_.scene_b_expired_at_accept_count, telemetry_.scene_b_sample_attempts);
        break;
      case SceneAcquisition::Failure::NO_FRESH_SAMPLE:
      case SceneAcquisition::Failure::NONE:
      default:
        RCLCPP_ERROR(
          logger,
          "M3 C3 SCENE_B_UPDATE_TIMEOUT: no dynamic_obstacle_0 update with t >= plan_done (%.6f) within 1.0s",
          telemetry_.plan_done_stamp_s);
        break;
    }
    telemetry_.final_result = Result::SCENE_STALE_OR_CORRUPT;
    return Result::SCENE_STALE_OR_CORRUPT;
  }
  const moveit_msgs::msg::PlanningScene & scene_b_msg = acq_b.scene_msg;
  telemetry_.scene_b_response_stamp_s = node_->now().seconds();

  // CORRECTION D, bound to SCENE_B: the snapshot the replacement candidate is
  // validated against is the snapshot whose integrity is asserted. No second
  // PlanningScene is fetched and the retained snapshot is not mutated.
  {
    std::string scene_err;
    if (!verify_retained_snapshot_obstacle_and_attachment(
        scene_b_msg, acq_b.sample.pose, scene_err))
    {
      RCLCPP_ERROR(logger, "M3 C3 SCENE_B_VERIFY_FAILED: %s", scene_err.c_str());
      telemetry_.final_result = Result::SCENE_STALE_OR_CORRUPT;
      return Result::SCENE_STALE_OR_CORRUPT;
    }
    if (scene_manager_) {
      auto spec = scene_manager_->expectedSceneSpec();
      spec.check_link_padding_and_scaling = false;  // component not requested; see SCENE_A
      if (!PlanningSceneManager::verifyExpectedSceneSnapshot(scene_b_msg, spec, scene_err)) {
        RCLCPP_ERROR(logger, "M3 C3 SCENE_B_VERIFY_FAILED: %s", scene_err.c_str());
        telemetry_.final_result = Result::SCENE_STALE_OR_CORRUPT;
        return Result::SCENE_STALE_OR_CORRUPT;
      }
    }
  }

  auto scene_b = std::make_shared<planning_scene::PlanningScene>(arm_.getRobotModel());
  if (!scene_b->usePlanningSceneMsg(scene_b_msg)) {
    RCLCPP_ERROR(logger, "M3 C3 SCENE_B_LOAD_FAILED: could not load PlanningSceneMsg into scene_b");
    telemetry_.final_result = Result::SCENE_STALE_OR_CORRUPT;
    return Result::SCENE_STALE_OR_CORRUPT;
  }

  RCLCPP_INFO(
    logger,
    "M3 C3 SCENE_B_ACQUIRED t_request=%.6f t_response=%.6f latency_ms=%.3f "
    "obstacle_age_ms=%.3f obstacle_age_at_accept_ms=%.3f sample_attempts=%d "
    "expired_at_accept_count=%d pose_match_error_m=%.6f",
    telemetry_.scene_b_request_stamp_s, telemetry_.scene_b_response_stamp_s,
    telemetry_.scene_b_latency_ms, telemetry_.scene_b_obstacle_age_ms,
    telemetry_.scene_b_obstacle_age_at_accept_ms, telemetry_.scene_b_sample_attempts,
    telemetry_.scene_b_expired_at_accept_count,
    telemetry_.scene_b_pose_match_error_m);

  // 6. Replacement Candidate Validation against SCENE_B (INVARIANT B)
  std::string candidate_col_info;
  if (!validateCandidateTrajectory(rep_traj, scene_b, candidate_col_info)) {
    RCLCPP_ERROR(
      logger, "M3 C3 CANDIDATE_VALIDATION_FAILED: replacement path in collision against SCENE_B (%s)",
      candidate_col_info.c_str());
    telemetry_.candidate_validation_passed = false;
    telemetry_.final_result = Result::PAYLOAD_COLLISION;
    return Result::PAYLOAD_COLLISION;
  }
  telemetry_.candidate_validation_passed = true;
  telemetry_.candidate_validation_done_stamp_s = node_->now().seconds();

  RCLCPP_INFO(
    logger,
    "M3 C3 CANDIDATE_VALIDATION_OK max_payload_tilt_deg=%.4f max_tool_tilt_deg=%.4f",
    telemetry_.candidate_max_payload_tilt_deg, telemetry_.candidate_max_tool_tilt_deg);

  // 7. Pre-Send Live Start Check
  TransportExecutor executor_1(node_, exec_params);
  const Result validate_1 = executor_1.preSendValidate(
    rep_traj, arm_, telemetry_.pre_send_start_error_rad);
  telemetry_.pre_send_validation_done_stamp_s = node_->now().seconds();

  if (!ok(validate_1)) {
    RCLCPP_ERROR(
      logger, "M3 C3 PRE_SEND_VALIDATE_1_FAILED result=%s start_error_rad=%.6f",
      to_string(validate_1), telemetry_.pre_send_start_error_rad);
    telemetry_.final_result = validate_1;
    return validate_1;
  }

  telemetry_.fjt_send_stamp_s = node_->now().seconds();
  telemetry_.validation_to_send_latency_ms =
    (telemetry_.fjt_send_stamp_s - telemetry_.candidate_validation_done_stamp_s) * 1000.0;

  RCLCPP_INFO(
    logger,
    "M3 C3 REPLACEMENT_SEND_READY start_error_rad=%.6f validation_to_send_latency_ms=%.3f",
    telemetry_.pre_send_start_error_rad, telemetry_.validation_to_send_latency_ms);

  // 8. Execute Replacement with Fresh Objects (signal_1, monitor_1, executor_1)
  auto stop_signal_1 = std::make_shared<TransportReactiveStopSignal>(
    params_.transport_reactive_stop_enabled);
  TransportPathMonitor monitor_1(node_, arm_.getRobotModel(), monitor_params, stop_signal_1);
  monitor_1.start(rep_traj);

  const Result exec_r1 = executor_1.executeAndWait(rep_traj, stop_signal_1, /*attempt=*/1);
  monitor_1.stop();
  logMonitorSummary(
    logger, 1, monitor_1.summary(), exec_r1 == Result::TRANSPORT_COLLISION_STOPPED);

  telemetry_.replacement_monitor_ticks = monitor_1.summary().tick_count;
  telemetry_.replacement_monitor_invalid_ticks = monitor_1.summary().invalid_tick_count;

  if (exec_r1 == Result::TRANSPORT_COLLISION_STOPPED) {
    // Second collision trigger occurred! Budget (1) exhausted.
    telemetry_.second_trigger_occurred = true;
    telemetry_.state_e2 = executor_1.lastSettledStateE();
    telemetry_.final_result = Result::TRANSPORT_REPLAN_LIMIT_REACHED;
    logSettledStateTelemetry(logger, /*attempt=*/1, telemetry_.state_e2);

    RCLCPP_ERROR(
      logger,
      "M3 C3 SECOND_TRIGGER_REPLAN_LIMIT_REACHED: replacement collision stopped; "
      "replan budget (1) exhausted; state_e2_captured=%d",
      telemetry_.state_e2.captured);
    // Stage-3C C3C: explicit budget-exhaustion telemetry, emitted before
    // returning -- observational only, the return value/control flow below
    // is unchanged from before this line existed. budget_exhausted_telemetry_valid()
    // is the pure predicate backing this contract; asserted here as a
    // defensive, always-true-by-construction check, not a new decision point.
    const bool budget_ok = budget_exhausted_telemetry_valid(
      telemetry_.replan_count, /*max_replans=*/1, /*attempt=*/1,
      Result::TRANSPORT_REPLAN_LIMIT_REACHED);
    RCLCPP_INFO(
      logger,
      "M3 C3 REPLAN_BUDGET_EXHAUSTED replan_count=%d max_replans=1 attempt=1 "
      "result=%s telemetry_valid=%d",
      telemetry_.replan_count, to_string(Result::TRANSPORT_REPLAN_LIMIT_REACHED), budget_ok);
    return Result::TRANSPORT_REPLAN_LIMIT_REACHED;
  }

  if (ok(exec_r1)) {
    telemetry_.final_result = Result::SUCCESS;
    RCLCPP_INFO(
      logger,
      "M3 C3 REPLACEMENT_SUCCESS attempt=1 replan_count=1 full_cycle_continues=1");
    return Result::SUCCESS;
  }

  telemetry_.final_result = exec_r1;
  RCLCPP_ERROR(
    logger, "M3 C3 REPLACEMENT_EXECUTION_FAILED result=%s fjt_error_code=%d",
    to_string(exec_r1), executor_1.lastFjtErrorCode());
  return exec_r1;
}

}  // namespace ur5e_pick_place
