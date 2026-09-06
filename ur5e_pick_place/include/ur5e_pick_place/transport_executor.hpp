// transport_executor.hpp — Stage-3C C0: direct FollowJointTrajectory
// execution for the TRANSPORT leg only.
//
// WHY THIS EXISTS, AND WHY IT IS NOT MoveGroupInterface::execute()
//   Stage-3C's empirical proof phases (Phase 0.1/0.2/0.3, see
//   docs/ — repository current-authority sections at the time this was
//   written) established, on this exact installed Jazzy/MoveIt 2.12.4
//   runtime:
//     - MoveGroupInterface::asyncExecute() + MoveGroupInterface::stop()
//       from a second thread reproducibly (2/2) segfaults inside
//       libmoveit_move_group_interface.so while it handles the
//       ExecuteTrajectory result callback.
//     - a dedicated client to moveit_msgs::action::ExecuteTrajectory does
//       not crash, but its cancel request is serialized behind
//       ExecuteTrajectory's own blocking execute callback inside
//       move_group and was never honored before natural completion (2/2).
//     - a dedicated client directly to arm_controller's own
//       control_msgs::action::FollowJointTrajectory action cancels
//       cleanly and quickly (2/2): cancel accepted in under 1 ms,
//       terminal CANCELED in ~45 ms, physical settle ~219 ms after the
//       cancel request, robot demonstrably short of the original target.
//   This class is the accepted primitive from that investigation, used
//   for the TRANSPORT leg only. Every other leg (pregrasp, descent,
//   pickup-clearance, lift, place, retreat) keeps using
//   MoveGroupInterface::execute() unchanged.
//
// C0 SCOPE — READ THIS BEFORE EXTENDING
//   This class implements ONLY: trajectory structural validation,
//   start-tolerance validation (reproducing MoveIt TrajectoryExecution-
//   Manager's allowed_start_tolerance, since direct FJT execution
//   bypasses TEM entirely), controller-liveness validation, a direct FJT
//   send, and an execution-duration watchdog (reproducing TEM's
//   allowed_execution_duration_scaling / allowed_goal_duration_margin).
//   There is no collision monitor, no future-trajectory validity check,
//   no on-demand cancellation API, and no replanning here. The ONLY
//   cancellation this class ever issues is the watchdog's own cleanup
//   path when execution overruns its computed limit — that is a
//   failure-cleanup mechanism (do not leave a runaway controller goal
//   active forever), not Stage-3C collision-triggered behavior. Adding a
//   public cancel-on-demand method, a scene-validity check, or a replan
//   path is Stage-3C C1/C2/C3 and does not belong in this file yet.
//
// C0.1 CORRECTION — WATCHDOG CLEANUP MUST WAIT FOR PHYSICAL SETTLE
//   Stage-3C Phase 0.3's own proof evidence measured, for the direct-FJT
//   cancellation primitive this class uses: the FJT terminal result
//   turns CANCELED almost immediately after a cancel is accepted, but
//   physical arm motion continued for ~175 ms AFTER that terminal result
//   before joint velocity actually settled. FJT CANCELED is therefore
//   NOT physical stop. The watchdog cleanup path (see executeAndWait())
//   must not return control to the caller while the arm may still be
//   moving -- it waits for live joint-velocity evidence of settle,
//   exactly the same stationary_velocity_eps /
//   stationary_consecutive_samples / stationary_timeout_s authority this
//   project's own m3_grasp.cpp startup stationarity check already uses,
//   before returning any watchdog-path Result.
//
// C0.3 CORRECTION — DISTINCT SAMPLES, ASYNC CALLBACK LIFETIME, CANCEL
//   CONFIRMATION
//   Three defects found in the C0.1 implementation, corrected here:
//   (1) waitForPhysicalSettle() polled `latest_joint_state_` on a fixed
//       cadence without proving each poll observed a NEW message -- the
//       same cached low-velocity sample could be counted multiple times
//       toward stationary_consecutive_samples. Fixed with a monotonic
//       receive-sequence counter (incremented once per subscription
//       callback) plus a condition variable: waitForPhysicalSettle() only
//       ever evaluates a sample whose sequence number it has not already
//       consumed, so N "consecutive" samples are provably N distinct
//       live messages. A stalled /joint_states stream (sequence never
//       advancing) times out rather than producing a false settle.
//   (2) the telemetry field previously named max_velocity_observed
//       actually held the LAST evaluated sample's velocity, not the
//       maximum across the settle interval -- corrected to track a true
//       running maximum separately from the last sample (both retained).
//   (3) all async action callbacks (goal response, terminal result,
//       watchdog cancel response) used to capture executeAndWait()'s own
//       local variables by reference. Every bounded wait in that function
//       can time out before its corresponding callback fires; a
//       sufficiently late callback would then write through a dangling
//       reference to an already-destroyed stack frame (and, since
//       TransportExecutor itself is constructed as a local object in
//       transport.cpp's Stage 4 block, `this` is not safe to capture
//       either once executeAndWait() has returned). Fixed by moving all
//       state the callbacks touch into a heap-allocated, shared_ptr-owned
//       struct captured BY VALUE in every callback -- its lifetime is
//       then governed by the callback's own lifetime (kept alive by
//       rclcpp_action's internal goal-handle bookkeeping), never by
//       executeAndWait()'s or TransportExecutor's. Scalar flags use
//       atomics; the grouped terminal-result fields (ResultCode,
//       error_code, error_string) that must be read together share one
//       mutex so a reader never observes a partially-updated group.
//   Watchdog cancellation is now also explicitly evaluated (not merely
//   logged): the specific goal's UUID must appear in the CancelResponse's
//   goals_canceling list with return_code ERROR_NONE, or the watchdog
//   path returns the new, distinct TRANSPORT_WATCHDOG_CANCEL_UNCONFIRMED
//   (settle handling still runs regardless -- fail-safe observation of
//   the arm never depends on cancel having been confirmed).
//
// C0.4 CORRECTION — /joint_states SUBSCRIPTION CALLBACK LIFETIME
//   C0.3 fixed the three FJT action callbacks' lifetime but left the
//   /joint_states subscription callback capturing `[this]` and writing
//   directly into TransportExecutor's own members. TransportExecutor is
//   constructed as a LOCAL object inside transport.cpp's Stage 4 block;
//   its subscription is serviced by the background executor concurrently
//   with the orchestration thread, so a joint-state message can arrive
//   (or be queued) while TransportExecutor is being destroyed --
//   dereferencing `this` inside that callback would then touch an
//   object whose members are mid-destruction or already destroyed. Fixed
//   the same way as C0.3: all state the callback touches now lives in a
//   heap-allocated JointStateCallbackState, owned by a shared_ptr the
//   callback captures BY VALUE (never `this`, never a reference to a
//   TransportExecutor member or a stack variable). waitForPhysicalSettle()
//   dereferences the SAME shared_ptr through TransportExecutor's own
//   member -- safe because that method runs synchronously on the calling
//   thread while TransportExecutor is still alive, unlike the
//   asynchronous subscription callback.
//
// THREAD MODEL
//   No new ROS node and no new executor are created here. The action
//   client and the controller-liveness service client are both
//   constructed on the SAME rclcpp::Node m3_grasp already owns; their
//   callbacks (goal response, result, cancel response) are serviced by
//   whichever executor is already spinning that node (m3_grasp's own
//   background SingleThreadedExecutor spin thread, unchanged). This
//   class's own public methods block the CALLING thread with plain
//   std::future waits / bounded sleep-polls on atomics, or a condition
//   variable for the settle wait -- exactly the pattern already used by
//   PlanningSceneManager::fetch() and proven crash-free across
//   Stage-3C's own proof harnesses -- never a second spin() call on the
//   same node.

#ifndef UR5E_PICK_PLACE__TRANSPORT_EXECUTOR_HPP_
#define UR5E_PICK_PLACE__TRANSPORT_EXECUTOR_HPP_

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ur5e_pick_place/failure.hpp"

namespace ur5e_pick_place
{

// Every field here reproduces an existing, validated MoveIt/controller
// authority value (ur5e_robotiq_moveit_config/config/
// moveit_controllers_parallel_jaw.yaml's trajectory_execution block, and
// that same file's moveit_simple_controller_manager.arm_controller
// entry). None of these are new thresholds -- see transport_executor.cpp
// and m3_grasp.launch.py for where each default is sourced from.
// stationary_* fields reproduce this project's existing, already-validated
// m3_grasp.cpp startup-stationarity authority values (same parameter
// names/semantics as stationary_velocity_eps / stationary_consecutive_samples
// / stationary_timeout_s and joint_states_topic) -- not new thresholds. They
// are used ONLY by the watchdog cleanup path's physical-settle confirmation
// (see the C0.1 class header note); normal successful execution never
// touches them.
struct TransportExecutionParams
{
  std::string fjt_action_name = "/arm_controller/follow_joint_trajectory";
  std::string controller_name = "arm_controller";
  double controller_wait_timeout_s = 5.0;
  double allowed_start_tolerance_rad = 0.01;
  double execution_duration_scaling = 1.2;
  double goal_duration_margin_s = 1.5;
  std::string joint_states_topic = "/joint_states";
  double stationary_velocity_eps_rad_s = 1.0e-3;
  int stationary_consecutive_samples = 6;
  double stationary_timeout_s = 25.0;
};

// True iff a sample carrying `current_seq` has not already been
// evaluated by a caller that has processed everything through
// `last_seen_seq`. This is the ENTIRE mechanism (C0.3) that prevents a
// cached /joint_states message from being counted more than once toward
// a physical-settle decision -- pulled out as a standalone, pure,
// directly-testable function rather than left inline in a wait loop.
inline bool is_new_joint_state_sample(uint64_t current_seq, uint64_t last_seen_seq)
{
  return current_seq != last_seen_seq;
}

// C0.4: heap-owned state for the /joint_states subscription callback,
// exactly mirroring GoalCallbackState's role for the FJT action
// callbacks (see transport_executor.cpp). TransportExecutor holds one
// shared_ptr to this; the subscription callback captures a COPY of that
// shared_ptr by value -- never `this`, never a TransportExecutor member
// reference. If TransportExecutor is destroyed while a callback
// invocation is queued or in flight, the callback's own copy of the
// shared_ptr keeps this object (and everything it owns) alive until the
// callback itself returns and releases its copy -- a standard, provable
// C++ shared_ptr lifetime guarantee, not a timing assumption. Public so
// it is directly constructible in a lifetime-ownership unit test (see
// test/test_transport_executor_settle.cpp) without needing a live ROS
// node, executor, or subscription.
struct JointStateCallbackState
{
  std::mutex mutex;
  std::condition_variable cv;
  sensor_msgs::msg::JointState::ConstSharedPtr latest;
  uint64_t sequence{0};
};

// Pure, ROS-node-free tracker for the physical-settle decision: given a
// sequence of DISTINCT samples (the caller is responsible for only
// calling addSample() once per is_new_joint_state_sample()==true
// message -- see TransportExecutor::waitForPhysicalSettle()), tracks how
// many consecutive samples had every named joint's |velocity| below the
// threshold, the TRUE running maximum |velocity| across all samples
// seen (not the last one), and the last sample's value for diagnostics.
// Directly unit-testable without any ROS node, executor, or action
// server (see test/test_transport_executor_settle.cpp).
class SettleTracker
{
public:
  SettleTracker(
    std::vector<std::string> joint_names, double velocity_eps_rad_s,
    int required_consecutive_samples)
  : joint_names_(std::move(joint_names)), velocity_eps_rad_s_(velocity_eps_rad_s),
    required_consecutive_samples_(required_consecutive_samples)
  {
  }

  // Evaluates one sample. Returns false (and does not affect
  // consecutive/max/last) if the sample is missing velocity data or any
  // named joint.
  bool addSample(const sensor_msgs::msg::JointState & snap)
  {
    if (snap.velocity.empty()) {
      return false;
    }
    double vmax = 0.0;
    for (const auto & joint_name : joint_names_) {
      const auto it = std::find(snap.name.begin(), snap.name.end(), joint_name);
      if (it == snap.name.end()) {
        return false;
      }
      const std::size_t idx = static_cast<std::size_t>(std::distance(snap.name.begin(), it));
      if (idx >= snap.velocity.size()) {
        return false;
      }
      vmax = std::max(vmax, std::abs(snap.velocity[idx]));
    }
    last_velocity_rad_s_ = vmax;
    max_velocity_rad_s_ = std::max(max_velocity_rad_s_, vmax);
    consecutive_ = (vmax < velocity_eps_rad_s_) ? consecutive_ + 1 : 0;
    return true;
  }

  bool settled() const { return consecutive_ >= required_consecutive_samples_; }
  int consecutiveAchieved() const { return consecutive_; }
  double maxVelocityRadS() const { return max_velocity_rad_s_; }
  double lastVelocityRadS() const { return last_velocity_rad_s_; }

private:
  std::vector<std::string> joint_names_;
  double velocity_eps_rad_s_;
  int required_consecutive_samples_;
  int consecutive_{0};
  double max_velocity_rad_s_{-1.0};
  double last_velocity_rad_s_{-1.0};
};

enum class TransportExecutionState
{
  IDLE,
  PRE_SEND_VALIDATED,
  EXECUTING,
  SUCCEEDED,
  FAILED,
};

class TransportExecutor
{
public:
  TransportExecutor(rclcpp::Node::SharedPtr node, TransportExecutionParams params);

  // Structural validation (non-empty, dimensions match joint_names,
  // time_from_start strictly increasing, joint-name set matches the
  // planning group's own joints per `arm`), start-tolerance validation
  // (actual current state vs. trajectory.points.front(), by joint name,
  // against allowed_start_tolerance_rad), and controller-liveness
  // validation (controller_manager/list_controllers reports
  // controller_name ACTIVE, AND the FJT action server is available
  // within controller_wait_timeout_s). Does not send anything.
  Result preSendValidate(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    moveit::planning_interface::MoveGroupInterface & arm,
    double & max_start_error_rad_out);

  // Sends the already-validated trajectory unmodified (no retiming, no
  // tolerance fields set -- the controller's own configured defaults
  // apply, exactly as this project's controllers.yaml already defines
  // them), waits for the terminal FollowJointTrajectory result with the
  // execution-duration watchdog armed, and returns a typed Result. Must
  // be called only after preSendValidate() returned Result::SUCCESS.
  // Blocks the calling thread; see the class header for the thread model.
  Result executeAndWait(const trajectory_msgs::msg::JointTrajectory & trajectory);

  TransportExecutionState state() const { return state_; }
  int32_t lastFjtErrorCode() const { return last_fjt_error_code_; }
  const std::string & lastFjtErrorString() const { return last_fjt_error_string_; }
  double lastPlannedDurationS() const { return last_planned_duration_s_; }
  double lastWatchdogLimitS() const { return last_watchdog_limit_s_; }
  double lastExecutionElapsedS() const { return last_execution_elapsed_s_; }
  // Watchdog-cleanup-path-only telemetry (see C0.1/C0.3 class header
  // notes). Meaningless (left at their default) unless the watchdog
  // actually fired.
  bool lastWatchdogPhysicalSettleConfirmed() const { return last_settle_confirmed_; }
  // True running maximum |velocity| across every DISTINCT sample
  // evaluated during the settle wait -- not the final (lowest) sample.
  double lastWatchdogMaxVelocityObservedRadS() const { return last_settle_max_velocity_rad_s_; }
  // The last evaluated sample's velocity, retained separately for
  // diagnostic purposes -- never labeled as the maximum.
  double lastWatchdogLastVelocityObservedRadS() const { return last_settle_last_velocity_rad_s_; }
  double lastWatchdogSettleElapsedS() const { return last_settle_elapsed_s_; }
  int lastWatchdogConsecutiveAchieved() const { return last_settle_consecutive_achieved_; }
  // Whether the watchdog's cancel request was confirmed accepted for
  // THIS EXACT goal (its UUID present in the CancelResponse's
  // goals_canceling list with return_code ERROR_NONE). false does not
  // mean the arm kept moving -- only that cancellation itself could not
  // be confirmed; settle observation still runs regardless (fail-safe).
  bool lastWatchdogCancelConfirmed() const { return last_cancel_confirmed_; }

private:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using FjtGoalHandle = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  bool checkControllerActive(std::string & error) const;

  // Watchdog-cleanup-path only: blocks (bounded by stationary_timeout_s)
  // until all joints named in `joint_names` report |velocity| below
  // stationary_velocity_eps_rad_s for stationary_consecutive_samples
  // consecutive DISTINCT live /joint_states samples (see the C0.3 class
  // header note -- each evaluated sample is provably a new message, never
  // a re-read of an already-consumed one), or the timeout elapses. Never
  // called from the normal success path.
  bool waitForPhysicalSettle(
    const std::vector<std::string> & joint_names,
    double & max_velocity_observed_out,
    double & last_velocity_observed_out,
    int & consecutive_achieved_out,
    double & settle_elapsed_s_out);

  rclcpp::Node::SharedPtr node_;
  TransportExecutionParams params_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr fjt_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // C0.4: the ONLY thing the subscription callback touches (via its own
  // captured-by-value copy of this same shared_ptr, never `this` -- see
  // the class header's C0.4 note). A message is only ever evaluated once
  // by waitForPhysicalSettle(), keyed by joint_state_state_->sequence
  // strictly increasing past whatever the reader has already consumed
  // (see C0.3 note). joint_state_state_->cv wakes waitForPhysicalSettle()
  // promptly on each new message instead of fixed-cadence polling.
  std::shared_ptr<JointStateCallbackState> joint_state_state_;

  TransportExecutionState state_{TransportExecutionState::IDLE};
  int32_t last_fjt_error_code_{0};
  std::string last_fjt_error_string_;
  double last_planned_duration_s_{0.0};
  double last_watchdog_limit_s_{0.0};
  double last_execution_elapsed_s_{0.0};
  bool last_settle_confirmed_{false};
  double last_settle_max_velocity_rad_s_{-1.0};
  double last_settle_last_velocity_rad_s_{-1.0};
  double last_settle_elapsed_s_{0.0};
  int last_settle_consecutive_achieved_{0};
  bool last_cancel_confirmed_{false};
};

}  // namespace ur5e_pick_place

#endif  // UR5E_PICK_PLACE__TRANSPORT_EXECUTOR_HPP_
