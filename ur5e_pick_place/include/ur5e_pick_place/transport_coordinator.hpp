// transport_coordinator.hpp — Stage-3C C3: One-replan reactive recovery coordinator.
//
// INVARIANTS
//   - Sits strictly ABOVE TransportExecutor and TransportPathMonitor.
//   - TransportExecutor remains the sole owner of direct-FJT goals, cancellation,
//     UUID confirmation, physical settle, and State E capture.
//   - TransportCoordinator owns the 1-replan budget, SCENE_A/B acquisition,
//     State E validation/handoff, start-state construction, replacement planning,
//     candidate validation, and second execution orchestration.
//   - Max permitted reactive replans = 1. No recursion. No prediction.

#ifndef UR5E_PICK_PLACE__TRANSPORT_COORDINATOR_HPP_
#define UR5E_PICK_PLACE__TRANSPORT_COORDINATOR_HPP_

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "ur5e_pick_place/failure.hpp"
#include "ur5e_pick_place/planning_scene_manager.hpp"
#include "ur5e_pick_place/transport.hpp"
#include "ur5e_pick_place/transport_executor.hpp"
#include "ur5e_pick_place/transport_path_monitor.hpp"
#include "ur5e_pick_place/transport_reactive_stop.hpp"

namespace ur5e_pick_place
{

// Pure testable logic helpers for C3

// Validates that State E has exactly 6 expected arm joints, finite positions,
// no duplicates, and exact name set match with the planning group.
bool validate_state_e(
  const SettledStateE & state_e,
  const std::vector<std::string> & expected_group_joints,
  const moveit::core::RobotModel & robot_model,
  std::string & error);

// Checks if an obstacle update is strictly at or after a baseline timestamp
// and within the authoritative staleness bound (0 <= age <= stale_threshold_s).
// --- Stage-3C C3 OPTIONAL pre-replan scene gate ----------------------------
//
// Outcome of one pre-replan gate handshake. DISABLED is the default-path
// result (no service configured) and is a SUCCESS-equivalent no-op.
enum class PreReplanGateOutcome
{
  DISABLED,
  SUCCESS,
  SERVICE_UNAVAILABLE,
  SEND_FAILED,
  TIMEOUT,
  SUCCESS_FALSE,
  SHUTDOWN
};

const char * to_string(PreReplanGateOutcome outcome);

// True only for outcomes that permit the replan to proceed.
inline bool pre_replan_gate_ok(PreReplanGateOutcome o)
{
  return o == PreReplanGateOutcome::DISABLED || o == PreReplanGateOutcome::SUCCESS;
}

// --- Stage-3C C3 static-closeout CORRECTION A: service send-boundary guard --
//
// rclcpp does NOT report a request-send failure by returning an invalid
// future. Client<T>::async_send_request(SharedRequest) creates a promise,
// takes its future, and then calls async_send_request_impl(), which calls
// rcl_send_request() and, on ANY non-OK return, calls
// rclcpp::exceptions::throw_from_rcl_error(ret, "failed to send request")
// -- see the installed implementation,
// /opt/ros/jazzy/include/rclcpp/rclcpp/client.hpp:636-645 (async_send_request)
// and :841-855 (async_send_request_impl). Consequences, both load-bearing:
//   1. the returned future is derived from a promise created immediately
//      beforehand, so it is ALWAYS valid when the call returns normally --
//      a post-call future.valid() test can never observe a send failure;
//   2. a send failure leaves the function by THROWING, which would escape
//      run_pre_replan_gate_handshake() and executeTransport() entirely
//      instead of becoming the typed TRANSPORT_PRE_REPLAN_GATE_FAILED
//      result with reason=SEND_FAILED.
//
// throw_from_rcl_error() raises one of RCLError (: std::runtime_error),
// RCLBadAlloc (: std::bad_alloc) or RCLInvalidArgument (: std::invalid_argument)
// -- three unrelated std branches whose only common std base is
// std::exception (/opt/ros/jazzy/include/rclcpp/rclcpp/exceptions/exceptions.hpp:152-183).
// std::exception is therefore the NARROWEST single type covering the whole
// documented set at this external-system boundary, and its what() text is
// logged. The trailing catch-all exists only so a non-std throw still cannot
// escape as an untyped failure; no documented rclcpp path produces one.
enum class GateSendStatus
{
  OK,
  THREW,           // async_send_request() raised -- the real rclcpp failure mode
  INVALID_FUTURE   // defensive: a future that is somehow not valid
};

const char * to_string(GateSendStatus status);

// Performs exactly one request send THROUGH send_fn and converts every failure
// of that send into a status code -- no exception leaves this function.
// send_fn is a callable returning the future; production passes a lambda that
// calls client->async_send_request(request). Taking a callable (rather than a
// client) is what makes the try/catch below the SAME production code a test
// exercises: the test injects a throwing send_fn, and the handler that
// converts the throw into SEND_FAILED is this one, not a copy of it.
// future_out is a std::optional because rclcpp's FutureAndRequestId is
// move-only and not default-constructible.
template <typename SendFnT, typename FutureOptionalT>
GateSendStatus send_gate_request_guarded(
  SendFnT && send_fn, FutureOptionalT & future_out, std::string & exception_text_out)
{
  exception_text_out.clear();
  try {
    future_out.emplace(send_fn());
  } catch (const std::exception & e) {
    exception_text_out = e.what();
    return GateSendStatus::THREW;
  } catch (...) {
    exception_text_out = "non-std exception raised at the service send boundary";
    return GateSendStatus::THREW;
  }
  if (!future_out.has_value() || !future_out->valid()) {
    exception_text_out = "async_send_request() returned an invalid future";
    return GateSendStatus::INVALID_FUTURE;
  }
  return GateSendStatus::OK;
}

// The ONE mapping from a gate outcome to the coordinator's typed result.
// executeTransport() uses this, so a test asserting on it is asserting on the
// production mapping: any non-OK outcome (SEND_FAILED included) becomes
// Result::TRANSPORT_PRE_REPLAN_GATE_FAILED, and ok() is false for it, which is
// what suppresses SCENE_A, the replacement plan, the attempt-1 FJT goal and
// every Stage 5-7 step.
inline Result pre_replan_gate_result(PreReplanGateOutcome o)
{
  return pre_replan_gate_ok(o) ? Result::SUCCESS : Result::TRANSPORT_PRE_REPLAN_GATE_FAILED;
}

// Test seam for the send boundary ONLY. Empty (the production default) means
// "call client.async_send_request(request)". A test supplies a callable that
// throws, so the real handshake -- its availability wait, its guarded send,
// its outcome mapping, its latency accounting -- runs unmodified against a
// send that fails the way rclcpp actually fails one.
using GateRequestSendFn = std::function<
  rclcpp::Client<std_srvs::srv::Trigger>::FutureAndRequestId(
    rclcpp::Client<std_srvs::srv::Trigger> &,
    const std_srvs::srv::Trigger::Request::SharedPtr &)>;

// Performs at most ONE std_srvs/srv/Trigger handshake, free of any
// TransportCoordinator state so it is directly unit-testable against a real
// node and a real service server.
//
// An empty service_name returns DISABLED without creating a client, touching
// the ROS graph, or waiting. Otherwise it lazily creates the client (reusing
// an already-created one), waits boundedly for availability, sends exactly one
// request, and waits for the response with a raw future wait -- never
// rclcpp::spin_until_future_complete(), because the node is expected to be
// owned by an executor already spinning on another thread.
//
// All deadlines use steady_clock so a paused simulation clock cannot hang it.
PreReplanGateOutcome run_pre_replan_gate_handshake(
  const rclcpp::Node::SharedPtr & node,
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
  const std::string & service_name, double timeout_s, double & latency_ms_out,
  const GateRequestSendFn & send_fn = GateRequestSendFn());

// SCENE_A's fresh-obstacle baseline. Without a gate, t_gate_done_s is 0.0 and
// this is exactly t_settle_s -- the behavior C3A/C3B qualified. With a
// completed gate it is the later of the two, so an obstacle update that
// predates the external transition cannot satisfy SCENE_A merely by
// postdating the physical settle. Both are ROS/sim-time seconds.
inline double scene_a_freshness_baseline_s(double t_settle_s, double t_gate_done_s)
{
  return t_gate_done_s > t_settle_s ? t_gate_done_s : t_settle_s;
}

bool is_obstacle_fresh_post_time(
  double obstacle_stamp_s, double baseline_time_s, double query_time_s,
  double stale_threshold_s = 0.250);

// --- Stage-3C C3 static-closeout CORRECTION C: acceptance-time freshness ----
//
// The qualification invariant is NOT "the sample was fresh when it was
// selected"; it is "the obstacle sample that authorizes the retained scene is
// still fresh AT THE MOMENT THE SCENE IS ACCEPTED". Selecting a fresh sample
// and then spending up to a full PlanningScene round trip before accepting the
// response can age that sample past the authority without any check noticing.
// Same baseline rule, same 250 ms threshold (deliberately unchanged) -- the
// only difference is WHEN the age is evaluated: at scene acceptance, not at
// sample selection.
inline bool is_sample_fresh_at_scene_acceptance(
  double sample_stamp_s, double baseline_time_s, double scene_accept_time_s,
  double stale_threshold_s = 0.250)
{
  return is_obstacle_fresh_post_time(
    sample_stamp_s, baseline_time_s, scene_accept_time_s, stale_threshold_s);
}

// What acquireFreshCoherentScene() does when a PlanningScene response finally
// arrives: accept it only if the sample that authorizes it is STILL within the
// authority; otherwise reject THAT scene and, if the existing acquisition
// budget has time left, go get a newer sample. BUDGET_EXHAUSTED is the only
// terminal outcome -- there is no unbounded retry.
enum class SceneAcceptanceDecision
{
  ACCEPT,
  RETRY_NEWER_SAMPLE,
  BUDGET_EXHAUSTED
};

inline SceneAcceptanceDecision decide_scene_acceptance(
  double sample_stamp_s, double baseline_time_s, double scene_accept_time_s,
  double budget_remaining_s, double stale_threshold_s = 0.250)
{
  if (is_sample_fresh_at_scene_acceptance(
      sample_stamp_s, baseline_time_s, scene_accept_time_s, stale_threshold_s))
  {
    return SceneAcceptanceDecision::ACCEPT;
  }
  return budget_remaining_s > 0.0 ?
         SceneAcceptanceDecision::RETRY_NEWER_SAMPLE :
         SceneAcceptanceDecision::BUDGET_EXHAUSTED;
}

// --- Stage-3C C3 static-closeout CORRECTION B: nonfinite pose rejection -----
//
// True only when every one of the seven pose components (position x/y/z and
// orientation x/y/z/w) is finite. NaN and +/-Inf must be rejected BEFORE any
// distance or angular comparison: a NaN distance makes `pos_err > pos_tol_m`
// false, and a NaN quaternion makes std::clamp() a no-op (both of its
// comparisons are false for NaN) so the resulting NaN angle also fails
// `angle_err <= tol` in the accepting direction -- i.e. a fully nonfinite pose
// would have been reported COHERENT. Rejection is explicit; the input is never
// sanitized or clamped into range.
bool is_pose_finite(const geometry_msgs::msg::Pose & p);

// Checks if a scene pose matches an expected obstacle update within
// position tolerance (pos_tol_m) and orientation tolerance (shortest angle in radians),
// sign-invariant with respect to quaternion negation (q == -q).
//
// Returns false immediately unless BOTH poses are entirely finite
// (is_pose_finite()); tolerances for valid data are unchanged (0.1 mm
// position, 1e-3 rad orientation) and the quaternion comparison remains
// sign-invariant.
bool is_pose_coherent(
  const geometry_msgs::msg::Pose & p_scene,
  const geometry_msgs::msg::Pose & p_expected,
  double pos_tol_m = 1.0e-4,
  double max_angle_error_rad = 1.0e-3);

// Computes shortest angular distance in radians between two unit quaternions,
// strictly sign-invariant (treats q and -q as identical).
double quaternion_shortest_angle_rad(
  const geometry_msgs::msg::Quaternion & q1,
  const geometry_msgs::msg::Quaternion & q2);

// --- Stage-3C C3 static-closeout CORRECTION D: same-snapshot integrity -----
//
// Verifies, against ONE already-retained PlanningScene snapshot and nothing
// else, the obstacle/attachment properties that authorize a replacement plan.
// It performs NO service call: the authority it establishes belongs to the
// exact snapshot passed in, which is the snapshot the coordinator retains and
// hands to planning_scene::PlanningScene::usePlanningSceneMsg(). Checks, in
// order, with a distinct error string each:
//   - dynamic_obstacle_0 present with at least one primitive pose
//   - its effective primitive pose coherent with expected_obstacle_pose
//     (same tolerances/sign-invariance as is_pose_coherent())
//   - pick_target present as an ATTACHED collision object
//   - attached to gripper_base_link
//   - carrying exactly PlanningSceneManager::padTouchLinks()
// The snapshot is taken by const reference and is never mutated.
bool verify_retained_snapshot_obstacle_and_attachment(
  const moveit_msgs::msg::PlanningScene & scene,
  const geometry_msgs::msg::Pose & expected_obstacle_pose,
  std::string & error);

// Stage-3C C3B: Orientation thresholds and tolerances
constexpr double kMaxAllowedPayloadTiltDeg = 2.0;
constexpr double kOrientationPathConstraintTiltTolRad = 0.031416;  // ~1.8 deg
constexpr double kOrientationPathConstraintYawTolRad = 3.14159;    // ~180.0 deg

// Computes upright tilt in degrees of a quaternion (local +Z relative to world +Z),
// sign-invariant with respect to quaternion negation (q == -q) and yaw-invariant.
double quaternion_upright_tilt_deg(double qx, double qy);
double quaternion_upright_tilt_deg(double qx, double qy, double qz, double qw);
double compute_payload_upright_tilt_deg(const geometry_msgs::msg::Quaternion & q);

// Computes tool upright tilt in degrees (nominal tool0 pointing along world -Z),
// sign-invariant and yaw-invariant.
double compute_tool_tilt_deg(double qx, double qy, double qz, double qw);
double compute_tool_tilt_deg(const geometry_msgs::msg::Quaternion & q);

// Converts a "local +Z into world +Z" dot product (as used by the candidate
// orientation validator, from an FK-derived rotation matrix column) into a
// tilt angle in degrees. Returns false (leaving tilt_deg_out unspecified)
// if up_dot is not finite, INSTEAD of letting NaN/Inf silently survive
// std::clamp()+std::acos() and then fail an "> threshold" comparison as
// though it were a small, in-tolerance angle. Returns true and writes the
// tilt in [0, 180] degrees for any finite input (out-of-[-1,1] values are
// clamped first, matching ordinary floating-point tolerance slop).
bool tilt_deg_from_up_dot_checked(double up_dot, double & tilt_deg_out);

// Stage-3C C3C: pure, directly-unit-tested predicates backing the
// attempt-aware observational telemetry. None of these influence control
// flow -- the coordinator's own attempt indices are always the literals 0
// (initial) and 1 (replacement); these exist so that contract is itself
// checkable and so a qualification harness has a documented, testable
// definition of "valid attempt" / "State E vs E2" / "exactly one goal per
// attempt, no third" / "budget-exhaustion telemetry is well-formed".

// Only 0 (initial attempt) and 1 (the single permitted replacement
// attempt) are valid coordinator attempt indices -- this project's
// one-replan budget means no attempt >= 2 is ever architecturally
// reachable; this predicate is what makes that claim checkable rather
// than merely asserted.
bool is_valid_attempt_index(int attempt);

// "State E" for attempt 0, "State E2" for attempt 1, an explicit
// diagnostic label for anything else (never expected in production, since
// is_valid_attempt_index() gates every attempt index the coordinator
// itself ever produces).
const char * attempt_state_label(int attempt);

// True iff the accepted-FJT-goal counts for a completed (or in-progress)
// C3 run match the one-replan budget exactly: exactly one attempt-0 goal
// accepted, exactly one attempt-1 (replacement) goal accepted, and zero
// goals accepted at attempt index 2 or higher. A qualification harness
// evaluates this from its own FJT_GOAL_ACCEPTED attempt=<n> tally; it is
// exposed here, pure and unit-tested, as the documented acceptance
// definition rather than leaving "no third goal" as prose only.
bool goal_acceptance_count_ok(
  int attempt0_accepted_count, int attempt1_accepted_count, int attempt2_plus_accepted_count);

// True iff a REPLAN_BUDGET_EXHAUSTED telemetry line's own values are
// internally consistent with this project's fixed one-replan policy:
// replan_count == 1, max_replans == 1, attempt == 1 (only the replacement
// attempt can exhaust the budget), and result ==
// Result::TRANSPORT_REPLAN_LIMIT_REACHED.
bool budget_exhausted_telemetry_valid(
  int replan_count, int max_replans, int attempt, Result result);

// Creates an orientation path constraint for the transport phase
moveit_msgs::msg::Constraints create_transport_orientation_constraint(
  const std::string & link_name,
  const geometry_msgs::msg::Quaternion & target_orientation,
  double tilt_tol_rad = kOrientationPathConstraintTiltTolRad,
  double yaw_tol_rad = kOrientationPathConstraintYawTolRad,
  const std::string & frame_id = "world");

// RAII guard for path constraints on MoveGroupInterface.
// Applies constraints upon construction and unconditionally clears them on destruction.
template <typename MoveGroupT = moveit::planning_interface::MoveGroupInterface>
class ScopedPathConstraintImpl
{
public:
  ScopedPathConstraintImpl(
    MoveGroupT & arm,
    const moveit_msgs::msg::Constraints & constraints)
  : arm_(arm)
  {
    arm_.setPathConstraints(constraints);
  }

  ~ScopedPathConstraintImpl()
  {
    arm_.clearPathConstraints();
  }

  ScopedPathConstraintImpl(const ScopedPathConstraintImpl &) = delete;
  ScopedPathConstraintImpl & operator=(const ScopedPathConstraintImpl &) = delete;
  ScopedPathConstraintImpl(ScopedPathConstraintImpl &&) = delete;
  ScopedPathConstraintImpl & operator=(ScopedPathConstraintImpl &&) = delete;

private:
  MoveGroupT & arm_;
};

using ScopedPathConstraint = ScopedPathConstraintImpl<moveit::planning_interface::MoveGroupInterface>;

// Telemetry structure for Stage-3C C3
struct TransportCoordinatorTelemetry
{
  bool replan_attempted{false};
  int replan_count{0};
  SettledStateE state_e{};

  // --- OPTIONAL pre-replan scene gate (inert unless configured) ---
  bool pre_replan_gate_enabled{false};
  bool pre_replan_gate_completed{false};
  double pre_replan_gate_latency_ms{0.0};
  // ROS/node time, NOT steady time: this is compared against /collision_object
  // header stamps, which are ROS (sim) time. 0.0 means "no gate ran", which is
  // why the SCENE_A baseline uses std::max() against it.
  double pre_replan_gate_done_stamp_s{0.0};
  std::string pre_replan_gate_failure_reason{};

  double scene_a_request_stamp_s{0.0};
  double scene_a_response_stamp_s{0.0};
  double scene_a_latency_ms{0.0};
  double scene_a_obstacle_stamp_s{0.0};
  // Age of the authorizing sample when it was SELECTED (unchanged meaning --
  // this is the value C3A/C3B/C3C evidence recorded as obstacle_age_ms).
  double scene_a_obstacle_age_ms{0.0};
  // CORRECTION C: age of that SAME sample at the moment the scene snapshot was
  // ACCEPTED. This, not the value above, is what the 250 ms authority gates.
  double scene_a_obstacle_age_at_accept_ms{0.0};
  int scene_a_sample_attempts{0};
  int scene_a_expired_at_accept_count{0};
  double scene_a_pose_match_error_m{0.0};

  double plan_start_stamp_s{0.0};
  double plan_done_stamp_s{0.0};
  double plan_latency_ms{0.0};
  double replacement_planned_duration_s{0.0};
  std::size_t replacement_waypoint_count{0};

  double scene_b_request_stamp_s{0.0};
  double scene_b_response_stamp_s{0.0};
  double scene_b_latency_ms{0.0};
  double scene_b_obstacle_stamp_s{0.0};
  double scene_b_obstacle_age_ms{0.0};
  // CORRECTION C: the same acceptance-time freshness invariant applies to
  // SCENE_B, with its own causal baseline (t_plan_done).
  double scene_b_obstacle_age_at_accept_ms{0.0};
  int scene_b_sample_attempts{0};
  int scene_b_expired_at_accept_count{0};
  double scene_b_pose_match_error_m{0.0};

  bool candidate_validation_passed{false};
  double candidate_max_payload_tilt_deg{0.0};
  double candidate_max_tool_tilt_deg{0.0};
  double candidate_validation_done_stamp_s{0.0};
  double pre_send_validation_done_stamp_s{0.0};
  double fjt_send_stamp_s{0.0};
  double validation_to_send_latency_ms{0.0};
  double pre_send_start_error_rad{0.0};

  int replacement_monitor_ticks{0};
  int replacement_monitor_invalid_ticks{0};
  Result final_result{Result::SUCCESS};

  // If second trigger occurred:
  bool second_trigger_occurred{false};
  SettledStateE state_e2{};
};

class TransportCoordinator
{
public:
  TransportCoordinator(
    rclcpp::Node::SharedPtr node,
    moveit::planning_interface::MoveGroupInterface & arm,
    const TransportParams & params,
    const geometry_msgs::msg::Pose & above_place,
    std::shared_ptr<PlanningSceneManager> scene_manager = nullptr);

  ~TransportCoordinator() = default;

  // Runs attempt 0 (initial trajectory). If collision stop occurs, performs
  // exactly one reactive recovery replan and replacement execution.
  Result executeTransport(
    const moveit::planning_interface::MoveGroupInterface::Plan & initial_plan);

  const TransportCoordinatorTelemetry & telemetry() const { return telemetry_; }

private:
  struct ObstacleSample
  {
    bool valid{false};
    double stamp_s{0.0};
    geometry_msgs::msg::Pose pose;
  };

  struct ObstacleTrackerState
  {
    std::mutex mutex;
    std::condition_variable cv;
    ObstacleSample latest;
  };

  bool waitForFreshObstacleUpdate(
    double baseline_time_s, double timeout_s, ObstacleSample & sample_out);

  bool acquireCoherentScene(
    const ObstacleSample & expected_sample, double timeout_s,
    moveit_msgs::msg::PlanningScene & scene_msg_out, double & pose_err_out,
    double & latency_ms_out);

  // --- CORRECTION C: one bounded, retrying acquisition ----------------------
  //
  // Result of acquireFreshCoherentScene(). `failure` names WHICH bounded step
  // exhausted the budget so the caller can keep emitting the pre-existing
  // distinct typed diagnostics rather than one undifferentiated timeout.
  struct SceneAcquisition
  {
    enum class Failure
    {
      NONE,
      NO_FRESH_SAMPLE,           // no qualifying obstacle update inside the budget
      NO_COHERENT_SCENE,         // scene never cohered with any qualifying sample
      SAMPLE_EXPIRED_AT_ACCEPT   // every cohering scene arrived with a stale sample
    };

    ObstacleSample sample;
    moveit_msgs::msg::PlanningScene scene_msg;
    double pose_err_m{0.0};
    double latency_ms{0.0};
    double sample_age_at_request_ms{0.0};
    double sample_age_at_accept_ms{0.0};
    int sample_attempts{0};
    int expired_at_accept_count{0};
    Failure failure{Failure::NONE};
  };

  // Acquires a PlanningScene snapshot that is BOTH coherent with a qualifying
  // obstacle sample AND authorized by a sample that is still within the 250 ms
  // freshness authority at the moment the scene is accepted.
  //
  // Deadline: exactly sample_timeout_s + scene_timeout_s from entry -- i.e.
  // precisely the sum of the two bounded steps this replaces, so the retry
  // re-spends the existing acquisition budget and never extends it. The loop
  // is bounded by that single steady_clock deadline; there is no unbounded
  // retry and the 250 ms threshold is unchanged.
  bool acquireFreshCoherentScene(
    double baseline_time_s, double sample_timeout_s, double scene_timeout_s,
    SceneAcquisition & out);

  // Runs the OPTIONAL pre-replan scene gate. Returns true immediately (a no-op)
  // when no service name is configured. When configured, performs exactly one
  // Trigger handshake and, on success, sets telemetry_.pre_replan_gate_done_stamp_s
  // to the ROS time at which the transition was confirmed complete.
  bool runPreReplanSceneGate(
    int attempt, std::string & failure_reason_out, PreReplanGateOutcome & outcome_out);

  bool validateCandidateTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::shared_ptr<planning_scene::PlanningScene> & scene_b,
    std::string & collision_info_out);

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface & arm_;
  TransportParams params_;
  geometry_msgs::msg::Pose above_place_;
  std::shared_ptr<PlanningSceneManager> scene_manager_;

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr obstacle_sub_;
  // Created lazily, and ONLY when the gate is configured -- an unconfigured run
  // never constructs a client and never touches the ROS graph for it.
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr pre_replan_gate_client_;
  std::shared_ptr<ObstacleTrackerState> obstacle_tracker_state_;

  TransportCoordinatorTelemetry telemetry_;
};

}  // namespace ur5e_pick_place

#endif  // UR5E_PICK_PLACE__TRANSPORT_COORDINATOR_HPP_
