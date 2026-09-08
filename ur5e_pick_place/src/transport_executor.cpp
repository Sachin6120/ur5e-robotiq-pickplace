#include "ur5e_pick_place/transport_executor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <thread>

using namespace std::chrono_literals;

namespace ur5e_pick_place
{
namespace
{
std::string result_code_to_string(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::SUCCEEDED: return "SUCCEEDED";
    case rclcpp_action::ResultCode::CANCELED: return "CANCELED";
    case rclcpp_action::ResultCode::ABORTED: return "ABORTED";
    case rclcpp_action::ResultCode::UNKNOWN: return "UNKNOWN";
    default: return "UNRECOGNIZED";
  }
}

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using FjtGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

// C0.3: heap-allocated, shared_ptr-refcounted state for every async
// callback executeAndWait() registers. Captured BY VALUE (a copy of the
// shared_ptr, not the pointee) in every callback, so a callback that
// fires after executeAndWait() -- or even the owning TransportExecutor,
// which is itself a local object in transport.cpp's Stage 4 block -- has
// already returned/been destroyed still writes into memory that is
// provably still alive (kept alive by the callback itself, which
// rclcpp_action's own goal-handle bookkeeping retains until the callback
// runs or is explicitly cleared). This is the fix for the dangling-
// reference hazard the C0.1 implementation had: all three bounded waits
// in executeAndWait() can time out before their corresponding callback
// fires, and a sufficiently late-arriving controller/action-server
// response must never be assumed impossible.
//
// Scalar single-value flags are plain atomics. The three terminal-result
// fields that must be read together (ResultCode, error_code,
// error_string) share one mutex so a reader can never observe a
// partially-updated group -- result_done is set only AFTER that group is
// fully written, giving a correct happens-before via the mutex/atomic
// combination rather than relying on timing.
struct GoalCallbackState
{
  rclcpp::Node::SharedPtr node;

  std::atomic<bool> goal_response_done{false};
  std::atomic<bool> goal_accepted{false};

  std::atomic<bool> result_done{false};
  std::atomic<double> terminal_stamp_s{0.0};
  std::shared_ptr<TransportReactiveStopSignal> stop_signal;
  std::mutex result_mutex;
  rclcpp_action::ResultCode terminal_code{rclcpp_action::ResultCode::UNKNOWN};
  int32_t terminal_error_code{0};
  std::string terminal_error_string;

  std::atomic<bool> cancel_response_done{false};
  std::atomic<int32_t> cancel_return_code{-1};
  // True only if THIS EXACT goal's UUID appeared in the CancelResponse's
  // goals_canceling list with return_code ERROR_NONE (0) -- see
  // action_msgs/srv/CancelGoal.srv. Distinct from cancel_response_done:
  // a response can arrive and still not confirm cancellation of this
  // goal (rejected, wrong/no goal in goals_canceling, already terminal).
  std::atomic<bool> cancel_goal_confirmed{false};
};

struct TerminalResultSnapshot
{
  rclcpp_action::ResultCode code;
  int32_t error_code;
  std::string error_string;
};

TerminalResultSnapshot snapshot_terminal_result(GoalCallbackState & state)
{
  std::lock_guard<std::mutex> lock(state.result_mutex);
  return {state.terminal_code, state.terminal_error_code, state.terminal_error_string};
}

}  // namespace

TransportExecutor::TransportExecutor(rclcpp::Node::SharedPtr node, TransportExecutionParams params)
: node_(std::move(node)), params_(std::move(params))
{
  fjt_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    node_, params_.fjt_action_name);
  list_controllers_client_ = node_->create_client<controller_manager_msgs::srv::ListControllers>(
    "/controller_manager/list_controllers");
  // Watchdog-cleanup-path physical-settle confirmation only (see the class
  // header's C0.1/C0.3 notes) -- reuses the SAME node m3_grasp already owns
  // and whichever executor is already spinning it; no second node/executor.
  // Harmless and idle during the normal success path. Every callback
  // invocation advances joint_state_seq_ exactly once and notifies
  // joint_state_cv_, so waitForPhysicalSettle() can prove it never
  // evaluates the same message twice.
  // C0.4: the subscription callback captures a COPY of joint_state_state_
  // by value -- never `this`, never a TransportExecutor member reference.
  // If TransportExecutor is destroyed while a callback invocation is
  // queued or in flight, that callback's own copy of the shared_ptr keeps
  // JointStateCallbackState alive regardless (see the class header's
  // C0.4 note).
  joint_state_state_ = std::make_shared<JointStateCallbackState>();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    params_.joint_states_topic, 10,
    [state = joint_state_state_](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->latest = std::move(msg);
      ++state->sequence;
      state->cv.notify_all();
    });
}

bool TransportExecutor::waitForPhysicalSettle(
  const std::vector<std::string> & joint_names,
  double & max_velocity_observed_out,
  double & last_velocity_observed_out,
  int & consecutive_achieved_out,
  double & settle_elapsed_s_out,
  sensor_msgs::msg::JointState::ConstSharedPtr & settled_sample_out)
{
  // Dereferencing joint_state_state_ here is safe: this method runs
  // synchronously on the calling thread while TransportExecutor (and
  // therefore this shared_ptr member) is still alive -- unlike the
  // subscription callback above, which is asynchronous and must never
  // depend on TransportExecutor's own lifetime (see C0.4 class header
  // note).
  JointStateCallbackState & jstate = *joint_state_state_;
  const auto settle_begin = std::chrono::steady_clock::now();
  const auto deadline = settle_begin + std::chrono::duration<double>(params_.stationary_timeout_s);
  SettleTracker tracker(joint_names, params_.stationary_velocity_eps_rad_s,
    params_.stationary_consecutive_samples);

  std::unique_lock<std::mutex> lock(jstate.mutex);
  // Baseline of 0: jstate.sequence is never decremented and starts at 0
  // before any message has ever arrived, so if a message is already
  // cached when this call begins, it is evaluated once as the first
  // sample (a genuine, not-yet-consumed live reading) rather than being
  // discarded -- but no message is ever evaluated a second time, because
  // last_seen_seq is advanced to match jstate.sequence every time a
  // sample is consumed below (is_new_joint_state_sample() is the single,
  // directly-tested predicate that enforces this -- see
  // test/test_transport_executor_settle.cpp).
  uint64_t last_seen_seq = 0;
  while (!tracker.settled()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      break;
    }
    // Wait until a NEW message (jstate.sequence advances past
    // last_seen_seq) arrives or the deadline expires. A stalled stream
    // (seq never advancing) falls through to the deadline check above on
    // every spurious wakeup and ultimately times out -- it can never
    // fabricate consecutive samples.
    jstate.cv.wait_until(
      lock, deadline,
      [&jstate, last_seen_seq] { return is_new_joint_state_sample(jstate.sequence, last_seen_seq); });
    if (!is_new_joint_state_sample(jstate.sequence, last_seen_seq)) {
      continue;  // woke on deadline with nothing new; loop exits via the time check above
    }
    last_seen_seq = jstate.sequence;
    auto snap = jstate.latest;
    if (!snap) {
      continue;
    }
    tracker.addSample(*snap);
  }
  max_velocity_observed_out = tracker.maxVelocityRadS();
  last_velocity_observed_out = tracker.lastVelocityRadS();
  consecutive_achieved_out = tracker.consecutiveAchieved();
  settle_elapsed_s_out =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - settle_begin).count();
  if (tracker.settled()) {
    settled_sample_out = jstate.latest;  // same locked sample that completed the C0 settle proof
  }
  return tracker.settled();
}

bool TransportExecutor::checkControllerActive(std::string & error) const
{
  if (!list_controllers_client_->wait_for_service(
      std::chrono::duration<double>(params_.controller_wait_timeout_s)))
  {
    error = "CONTROLLER_MANAGER_SERVICE_UNAVAILABLE: /controller_manager/list_controllers "
      "did not appear within " + std::to_string(params_.controller_wait_timeout_s) + "s";
    return false;
  }
  auto request = std::make_shared<controller_manager_msgs::srv::ListControllers::Request>();
  auto future = list_controllers_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(params_.controller_wait_timeout_s)) !=
    std::future_status::ready)
  {
    error = "LIST_CONTROLLERS_TIMEOUT: no response within " +
      std::to_string(params_.controller_wait_timeout_s) + "s";
    return false;
  }
  const auto response = future.get();
  const auto it = std::find_if(
    response->controller.begin(), response->controller.end(),
    [this](const auto & c) { return c.name == params_.controller_name; });
  if (it == response->controller.end()) {
    error = "CONTROLLER_NOT_FOUND: '" + params_.controller_name +
      "' is not listed by controller_manager";
    return false;
  }
  if (it->state != "active") {
    error = "CONTROLLER_NOT_ACTIVE: '" + params_.controller_name + "' state is '" +
      it->state + "', not 'active'";
    return false;
  }
  return true;
}

Result TransportExecutor::preSendValidate(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  moveit::planning_interface::MoveGroupInterface & arm,
  double & max_start_error_rad_out)
{
  max_start_error_rad_out = std::numeric_limits<double>::quiet_NaN();
  const auto logger = node_->get_logger();
  const double t_validation_start_s = node_->now().seconds();
  RCLCPP_INFO(
    logger, "M3 C0 TRANSPORT_EXECUTOR t_pre_send_validation_start=%.6f", t_validation_start_s);

  // --- structural validation ---
  if (trajectory.joint_names.empty() || trajectory.points.empty()) {
    RCLCPP_ERROR(
      logger, "CONFIG_ERROR: transport trajectory has empty joint_names or points "
      "(joint_names=%zu points=%zu).", trajectory.joint_names.size(), trajectory.points.size());
    state_ = TransportExecutionState::FAILED;
    return Result::CONFIG_ERROR;
  }
  for (const auto & pt : trajectory.points) {
    if (pt.positions.size() != trajectory.joint_names.size()) {
      RCLCPP_ERROR(
        logger, "CONFIG_ERROR: transport trajectory point has %zu positions, expected %zu "
        "(joint_names.size()).", pt.positions.size(), trajectory.joint_names.size());
      state_ = TransportExecutionState::FAILED;
      return Result::CONFIG_ERROR;
    }
  }
  for (size_t i = 0; i + 1 < trajectory.points.size(); ++i) {
    const double t0 = rclcpp::Duration(trajectory.points[i].time_from_start).seconds();
    const double t1 = rclcpp::Duration(trajectory.points[i + 1].time_from_start).seconds();
    if (t1 <= t0) {
      RCLCPP_ERROR(
        logger, "CONFIG_ERROR: transport trajectory time_from_start is not strictly "
        "increasing at point %zu->%zu (%.6f -> %.6f).", i, i + 1, t0, t1);
      state_ = TransportExecutionState::FAILED;
      return Result::CONFIG_ERROR;
    }
  }
  const double planned_duration_s =
    rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
  if (planned_duration_s <= 0.0) {
    RCLCPP_ERROR(
      logger, "CONFIG_ERROR: transport trajectory final time_from_start is %.6f (must be > 0).",
      planned_duration_s);
    state_ = TransportExecutionState::FAILED;
    return Result::CONFIG_ERROR;
  }
  last_planned_duration_s_ = planned_duration_s;

  // Expected arm-joint set comes from the planning group itself (`arm`),
  // not a re-typed list -- this is exactly the joint list
  // arm_controller's own moveit_controllers_parallel_jaw.yaml
  // configuration was built from, so no second list can drift out of
  // sync with it here.
  const auto expected_joints = arm.getJointNames();
  std::set<std::string> expected_set(expected_joints.begin(), expected_joints.end());
  std::set<std::string> traj_set(trajectory.joint_names.begin(), trajectory.joint_names.end());
  if (traj_set != expected_set) {
    RCLCPP_ERROR(
      logger, "CONFIG_ERROR: transport trajectory joint_names do not match the 'arm' planning "
      "group's own joint set.");
    state_ = TransportExecutionState::FAILED;
    return Result::CONFIG_ERROR;
  }

  // --- start-tolerance validation (reproduces MoveIt TEM's
  // allowed_start_tolerance, since direct FJT execution bypasses TEM
  // entirely) ---
  const auto current_state = arm.getCurrentState(2.0);
  if (!current_state) {
    RCLCPP_ERROR(
      logger, "CONFIG_ERROR: no current RobotState available for the transport "
      "start-tolerance check.");
    state_ = TransportExecutionState::FAILED;
    return Result::CONFIG_ERROR;
  }
  double max_start_error_rad = 0.0;
  for (size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    const double * actual = current_state->getJointPositions(trajectory.joint_names[i]);
    if (!actual) {
      RCLCPP_ERROR(
        logger, "CONFIG_ERROR: current RobotState has no position for joint '%s'.",
        trajectory.joint_names[i].c_str());
      state_ = TransportExecutionState::FAILED;
      return Result::CONFIG_ERROR;
    }
    const double err = std::abs(*actual - trajectory.points.front().positions[i]);
    max_start_error_rad = std::max(max_start_error_rad, err);
  }
  max_start_error_rad_out = max_start_error_rad;
  RCLCPP_INFO(
    logger, "M3 C0 TRANSPORT_EXECUTOR max_start_error_rad=%.6f allowed=%.6f",
    max_start_error_rad, params_.allowed_start_tolerance_rad);
  if (max_start_error_rad > params_.allowed_start_tolerance_rad) {
    RCLCPP_ERROR(
      logger, "TRANSPORT_START_TOLERANCE_VIOLATED: max joint error %.6f rad exceeds "
      "allowed_start_tolerance %.6f rad.", max_start_error_rad, params_.allowed_start_tolerance_rad);
    state_ = TransportExecutionState::FAILED;
    return Result::TRANSPORT_START_TOLERANCE_VIOLATED;
  }

  // --- controller-liveness validation ---
  std::string controller_err;
  if (!checkControllerActive(controller_err)) {
    RCLCPP_ERROR(logger, "CONTROLLER_UNAVAILABLE: %s", controller_err.c_str());
    state_ = TransportExecutionState::FAILED;
    return Result::CONTROLLER_UNAVAILABLE;
  }
  if (!fjt_client_->wait_for_action_server(
      std::chrono::duration<double>(params_.controller_wait_timeout_s)))
  {
    RCLCPP_ERROR(
      logger, "CONTROLLER_UNAVAILABLE: FollowJointTrajectory action server '%s' did not "
      "become available within %.1fs.", params_.fjt_action_name.c_str(),
      params_.controller_wait_timeout_s);
    state_ = TransportExecutionState::FAILED;
    return Result::CONTROLLER_UNAVAILABLE;
  }

  RCLCPP_INFO(
    logger, "M3 C0 TRANSPORT_EXECUTOR t_pre_send_validation_done=%.6f planned_duration_s=%.6f",
    node_->now().seconds(), planned_duration_s);
  state_ = TransportExecutionState::PRE_SEND_VALIDATED;
  return Result::SUCCESS;
}

Result TransportExecutor::executeAndWait(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::shared_ptr<TransportReactiveStopSignal> stop_signal)
{
  const auto logger = node_->get_logger();
  if (state_ != TransportExecutionState::PRE_SEND_VALIDATED) {
    RCLCPP_ERROR(
      logger, "CONFIG_ERROR: executeAndWait() called without a preceding successful "
      "preSendValidate().");
    state_ = TransportExecutionState::FAILED;
    return Result::CONFIG_ERROR;
  }

  FollowJointTrajectory::Goal goal;
  goal.trajectory = trajectory;  // sent exactly as MoveIt planned it; no retiming, no
                                  // tolerance fields set here -- arm_controller's own
                                  // configured defaults (controllers.yaml) apply.

  // See the C0.3 class header note and the GoalCallbackState comment
  // above: heap-owned, captured by value everywhere below.
  auto state = std::make_shared<GoalCallbackState>();
  state->node = node_;
  if (!stop_signal) {stop_signal = std::make_shared<TransportReactiveStopSignal>(false);}
  state->stop_signal = stop_signal;

  rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions send_opts;
  send_opts.goal_response_callback =
    [state](FjtGoalHandle::SharedPtr gh) {
      state->goal_accepted = (gh != nullptr);
      RCLCPP_INFO(
        state->node->get_logger(), "M3 C0 TRANSPORT_EXECUTOR t_fjt_goal_accept=%.6f accepted=%d",
        state->node->now().seconds(), state->goal_accepted.load());
      state->goal_response_done = true;
    };
  send_opts.result_callback =
    [state](const FjtGoalHandle::WrappedResult & wr) {
      const auto observed_at = TransportClock::now();
      // Synchronous publication under the arbiter lock; the wr reference
      // never escapes this callback. Lock order is arbiter -> result_mutex.
      state->stop_signal->publishTerminal(observed_at, [state, &wr, observed_at] {
        state->terminal_stamp_s = transport_stamp(observed_at);
        {
          std::lock_guard<std::mutex> lock(state->result_mutex);
          state->terminal_code = wr.code;
          if (wr.result) {
            state->terminal_error_code = wr.result->error_code;
            state->terminal_error_string = wr.result->error_string;
          }
        }
        state->result_done = true;
      });
      RCLCPP_INFO(
        state->node->get_logger(), "M3 C0 TRANSPORT_EXECUTOR t_fjt_result=%.6f "
        "terminal_action_status=%s fjt_error_code=%d fjt_error_string=\"%s\"",
        state->node->now().seconds(), result_code_to_string(wr.code).c_str(),
        wr.result ? wr.result->error_code : 0,
        wr.result ? wr.result->error_string.c_str() : "");

    };

  const auto t_execution_begin = TransportClock::now();
  last_watchdog_limit_s_ =
    last_planned_duration_s_ * params_.execution_duration_scaling + params_.goal_duration_margin_s;
  stop_signal->arm(t_execution_begin +
    std::chrono::duration_cast<TransportClock::duration>(
      std::chrono::duration<double>(last_watchdog_limit_s_)));
  RCLCPP_INFO(logger, "M3 C0 TRANSPORT_EXECUTOR t_fjt_goal_send=%.6f", node_->now().seconds());
  auto send_goal_future = fjt_client_->async_send_goal(goal, send_opts);
  state_ = TransportExecutionState::EXECUTING;

  // C0.3: reuse transport_controller_wait_timeout_s for this bounded
  // wait rather than an independent hidden literal -- same class of
  // "how long do we tolerate the controller/action-server stack being
  // slow" as the controller-liveness check in preSendValidate().
  const auto accept_deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(params_.controller_wait_timeout_s);
  while (!state->goal_response_done && std::chrono::steady_clock::now() < accept_deadline) {
    std::this_thread::sleep_for(2ms);
  }
  if (!state->goal_response_done || !state->goal_accepted) {
    RCLCPP_ERROR(
      logger, "TRANSPORT_FJT_GOAL_REJECTED: direct FollowJointTrajectory goal was not "
      "accepted within the bounded wait.");
    state_ = TransportExecutionState::FAILED;
    return Result::TRANSPORT_FJT_GOAL_REJECTED;
    // NOTE: state's callbacks may still fire later (e.g. a very late
    // goal_response after this bound expired) -- safe by construction,
    // since `state` is heap-owned and only referenced by value from here
    // on; no dangling reference to this stack frame or to `this` exists.
  }
  auto goal_handle = send_goal_future.get();
  const auto goal_uuid = goal_handle->get_goal_id();

  stop_signal->goalAccepted();
  RCLCPP_INFO(
    logger, "M3 C0 TRANSPORT_EXECUTOR watchdog_limit_s=%.6f (planned_duration_s=%.6f "
    "scaling=%.3f margin_s=%.3f)", last_watchdog_limit_s_, last_planned_duration_s_,
    params_.execution_duration_scaling, params_.goal_duration_margin_s);

  const auto cause = stop_signal->wait();
  const bool collision = cause == TransportTerminalCause::COLLISION;
  const bool watchdog_fired = cause == TransportTerminalCause::WATCHDOG;
  const auto trigger = stop_signal->evidence();
  const double t_observed = transport_stamp(TransportClock::now());
  if (collision) {
    RCLCPP_INFO(logger,
      "M3 C2 EXECUTOR_REACTION t_executor_observed_signal=%.9f signal_to_executor_ms=%.6f",
      t_observed, (t_observed - transport_stamp(trigger->latched)) * 1000.0);
  }
  if (watchdog_fired || collision) {
    // Single cancellation owner and a single cleanup primitive for both causes.
    // Every callback owns its state by value, including late timeout responses.
    const std::string label = collision ? "COLLISION_STOP" : "WATCHDOG_CLEANUP";
    if (watchdog_fired) {
      RCLCPP_ERROR(logger,
        "TRANSPORT_EXECUTION_WATCHDOG_TIMEOUT: t_watchdog_trigger=%.6f "
        "watchdog_limit_s=%.6f; WATCHDOG_CLEANUP cancellation (not a collision trigger).",
        node_->now().seconds(), last_watchdog_limit_s_);
    }
    std::ostringstream uuid;
    uuid << std::hex << std::setfill('0');
    for (auto byte : goal_uuid) {uuid << std::setw(2) << static_cast<unsigned>(byte);}
    const std::string uuid_string = uuid.str();
    sensor_msgs::msg::JointState::ConstSharedPtr immediate_sample;
    {
      std::lock_guard<std::mutex> lock(joint_state_state_->mutex);
      immediate_sample = joint_state_state_->latest;
    }
    const double t_cancel = transport_stamp(TransportClock::now());
    RCLCPP_INFO(logger,
      "M3 C2 CANCEL_REQUEST cause=%s t_cancel_request=%.9f goal_uuid=%s "
      "cancel_request_count_for_collision=%d invalidity_to_cancel_ms=%.6f",
      label.c_str(), t_cancel, uuid_string.c_str(), collision ? 1 : 0,
      trigger ? (t_cancel - transport_stamp(trigger->detected)) * 1000.0 : -1.0);
    if (watchdog_fired) {
      RCLCPP_INFO(logger,
        "M3 C0 TRANSPORT_EXECUTOR WATCHDOG_CLEANUP t_watchdog_cancel_request=%.6f",
        node_->now().seconds());
    }
    auto cancel_future = fjt_client_->async_cancel_goal(
      goal_handle,
      [state, goal_uuid, uuid_string, label, t_cancel](
        rclcpp_action::Client<FollowJointTrajectory>::CancelResponse::SharedPtr resp) {
        const double t_response = transport_stamp(TransportClock::now());
        const bool this_goal_present = std::any_of(
          resp->goals_canceling.begin(), resp->goals_canceling.end(),
          [&goal_uuid](const auto & gi) {return gi.goal_id.uuid == goal_uuid;});
        const bool confirmed = (resp->return_code == 0) && this_goal_present;
        state->cancel_return_code = static_cast<int32_t>(resp->return_code);
        state->cancel_goal_confirmed = confirmed;
        RCLCPP_INFO(state->node->get_logger(),
          "M3 C2 CANCEL_RESPONSE cause=%s t_cancel_response=%.9f goal_uuid=%s "
          "cancel_response_latency_ms=%.6f return_code=%d n_goals_canceling=%zu this_goal_confirmed=%d",
          label.c_str(), t_response, uuid_string.c_str(), (t_response - t_cancel) * 1000.0,
          static_cast<int>(resp->return_code), resp->goals_canceling.size(), confirmed);
        state->cancel_response_done = true;
      });
    const auto cancel_deadline = TransportClock::now() +
      std::chrono::duration<double>(params_.controller_wait_timeout_s);
    while (!state->cancel_response_done && TransportClock::now() < cancel_deadline) {
      std::this_thread::sleep_for(5ms);
    }
    last_cancel_confirmed_ = state->cancel_goal_confirmed.load();
    if (!state->cancel_response_done) {
      RCLCPP_WARN(logger, "M3 C2 CANCEL_RESPONSE_TIMEOUT cause=%s; observing terminal/settle",
        label.c_str());
    }
    const auto result_deadline = TransportClock::now() +
      std::chrono::duration<double>(params_.controller_wait_timeout_s);
    while (!state->result_done && TransportClock::now() < result_deadline) {
      std::this_thread::sleep_for(5ms);
    }
    const bool terminal_observed = state->result_done.load();
    const auto terminal = snapshot_terminal_result(*state);
    last_fjt_error_code_ = terminal.error_code;
    last_fjt_error_string_ = terminal.error_string;
    const double t_terminal = state->terminal_stamp_s.load();
    RCLCPP_INFO(logger,
      "M3 C2 TERMINAL cause=%s observed=%d t_fjt_terminal=%.9f "
      "terminal_action_status=%s fjt_error_code=%d fjt_error_string=\"%s\" "
      "cancel_to_terminal_ms=%.6f",
      label.c_str(), terminal_observed, t_terminal, result_code_to_string(terminal.code).c_str(),
      terminal.error_code, terminal.error_string.c_str(),
      terminal_observed ? (t_terminal - t_cancel) * 1000.0 : -1.0);

    // C0 authority is unchanged: six DISTINCT messages, true running maximum,
    // velocity epsilon 1e-3. Observe even on unconfirmed cancel/terminal timeout,
    // but those paths can NEVER report a successful C2 stop.
    const double t_settle_begin = transport_stamp(TransportClock::now());
    RCLCPP_INFO(logger, "M3 C2 SETTLE_BEGIN cause=%s t_settle_start=%.9f velocity_eps=%.9f",
      label.c_str(), t_settle_begin, params_.stationary_velocity_eps_rad_s);
    sensor_msgs::msg::JointState::ConstSharedPtr settled_sample;
    last_settle_confirmed_ = waitForPhysicalSettle(
      trajectory.joint_names, last_settle_max_velocity_rad_s_, last_settle_last_velocity_rad_s_,
      last_settle_consecutive_achieved_, last_settle_elapsed_s_, settled_sample);
    const double t_settle = transport_stamp(TransportClock::now());
    last_execution_elapsed_s_ = t_settle - transport_stamp(t_execution_begin);
    RCLCPP_INFO(logger,
      "M3 C2 %s cause=%s t_physical_settle=%.9f distinct_samples=%d "
      "max_velocity_observed_rad_s=%.9e final_velocity_rad_s=%.9e "
      "cancel_to_settle_ms=%.6f terminal_to_settle_ms=%.6f invalidity_to_settle_ms=%.6f",
      last_settle_confirmed_ ? "PHYSICAL_SETTLE_CONFIRMED" : "PHYSICAL_SETTLE_UNCONFIRMED",
      label.c_str(), t_settle, last_settle_consecutive_achieved_,
      last_settle_max_velocity_rad_s_, last_settle_last_velocity_rad_s_,
      (t_settle - t_cancel) * 1000.0,
      terminal_observed ? (t_settle - t_terminal) * 1000.0 : -1.0,
      trigger ? (t_settle - transport_stamp(trigger->detected)) * 1000.0 : -1.0);

    // Capture positions ONLY after the physical-settle predicate completed,
    // from that exact locked final message, rather than a trajectory estimate
    // or a possibly older MoveGroup current-state cache.
    bool captured = last_settle_confirmed_ && settled_sample &&
      trajectory.joint_names.size() == 6;
    double max_delta = 0.0;
    double target_error = 0.0;
    for (std::size_t i = 0; captured && i < trajectory.joint_names.size(); ++i) {
      const auto & name = trajectory.joint_names[i];
      const auto it = std::find(settled_sample->name.begin(), settled_sample->name.end(), name);
      const auto idx = static_cast<std::size_t>(std::distance(settled_sample->name.begin(), it));
      if (it == settled_sample->name.end() || idx >= settled_sample->position.size() ||
        !std::isfinite(settled_sample->position[idx]))
      {
        captured = false;
        break;
      }
      const double e = settled_sample->position[idx];
      double immediate = std::numeric_limits<double>::quiet_NaN();
      if (immediate_sample) {
        const auto before = std::find(immediate_sample->name.begin(), immediate_sample->name.end(), name);
        const auto j = static_cast<std::size_t>(std::distance(immediate_sample->name.begin(), before));
        if (before != immediate_sample->name.end() && j < immediate_sample->position.size()) {
          immediate = immediate_sample->position[j];
        }
      }
      max_delta = std::max(max_delta, std::abs(e - immediate));
      target_error = std::max(target_error, std::abs(e - trajectory.points.back().positions[i]));
      RCLCPP_INFO(logger,
        "M3 C2 STATE_E_JOINT joint=%s state_e_rad=%.12f immediate_cancel_rad=%.12f delta_rad=%.12f",
        name.c_str(), e, immediate, e - immediate);
    }
    RCLCPP_INFO(logger,
      "M3 C2 STATE_E captured=%d immediate_to_e_max_delta_rad=%.12f "
      "original_target_max_error_rad=%.12f state_e_sample_stamp=%.9f",
      captured, max_delta, target_error,
      settled_sample ? rclcpp::Time(settled_sample->header.stamp).seconds() : -1.0);
    state_ = TransportExecutionState::FAILED;  // deliberate stop is not manipulation SUCCESS
    if (collision) {
      const auto result = collision_stop_result(last_cancel_confirmed_,
        terminal_observed && terminal.code == rclcpp_action::ResultCode::CANCELED,
        last_settle_confirmed_, captured);
      RCLCPP_INFO(logger,
        "M3 C2 STOP_RESULT result=%s collision_triggered=1 collision_trigger_count=1 "
        "cancel_request_count_for_collision=1 stop_count=%d replan_count=0 watchdog_count=0",
        to_string(result), result == Result::TRANSPORT_COLLISION_STOPPED ? 1 : 0);
      return result;
    }
    if (!last_settle_confirmed_) {return Result::TRANSPORT_PHYSICAL_SETTLE_TIMEOUT;}
    if (!last_cancel_confirmed_) {return Result::TRANSPORT_WATCHDOG_CANCEL_UNCONFIRMED;}
    return Result::TRANSPORT_EXECUTION_WATCHDOG_TIMEOUT;
  }
  RCLCPP_INFO(logger,
    "M3 C2 NATURAL_RESULT collision_triggered=0 collision_trigger_count=0 "
    "cancel_request_count_for_collision=0 stop_count=0 replan_count=0 watchdog_count=0");

  last_execution_elapsed_s_ =
    std::chrono::duration<double>(std::chrono::steady_clock::now() - t_execution_begin).count();
  const auto terminal = snapshot_terminal_result(*state);
  last_fjt_error_code_ = terminal.error_code;
  last_fjt_error_string_ = terminal.error_string;
  RCLCPP_INFO(
    logger, "M3 C0 TRANSPORT_EXECUTOR t_transport_execution_done=%.6f execution_elapsed_s=%.6f",
    node_->now().seconds(), last_execution_elapsed_s_);

  if (terminal.code != rclcpp_action::ResultCode::SUCCEEDED ||
    terminal.error_code != control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL)
  {
    // Terminal status is preserved above (terminal_action_status,
    // fjt_error_code, fjt_error_string) and never reinterpreted -- a
    // CANCELED or ABORTED result is always a C0 failure, never SUCCESS.
    RCLCPP_ERROR(
      logger, "TRANSPORT_FJT_EXECUTION_FAILED: terminal_action_status=%s fjt_error_code=%d "
      "fjt_error_string=\"%s\"", result_code_to_string(terminal.code).c_str(),
      terminal.error_code, terminal.error_string.c_str());
    state_ = TransportExecutionState::FAILED;
    return Result::TRANSPORT_FJT_EXECUTION_FAILED;
  }

  state_ = TransportExecutionState::SUCCEEDED;
  return Result::SUCCESS;
}

}  // namespace ur5e_pick_place
