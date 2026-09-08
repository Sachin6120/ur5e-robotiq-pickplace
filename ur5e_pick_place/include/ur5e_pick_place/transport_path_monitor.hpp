// transport_path_monitor.hpp — Stage-3C C1: OBSERVE-ONLY future-path
// collision monitoring for the direct-FJT TRANSPORT leg.
//
// C1 SCOPE — READ THIS BEFORE EXTENDING
//   This class watches the remaining TRANSPORT trajectory against the live
//   Stage-3B PlanningScene while TransportExecutor::executeAndWait() is in
//   flight, and RECORDS whether the robot's current state and its remaining
//   planned path are collision-valid. It NEVER cancels, stops, replans, or
//   otherwise reacts to what it observes -- see PROJECT_STATE.md/HANDOFF.md
//   Stage-3C C1 authority and transport_executor.hpp's own C0 scope note.
//   The only cancellation path that exists anywhere in the TRANSPORT leg
//   remains TransportExecutor's own watchdog cleanup (unrelated to
//   collision, unmodified by C1). Adding a cancel-on-demand method here, or
//   having this class call anything on TransportExecutor/MoveGroupInterface,
//   is Stage-3C C2/C3 and does not belong in this file.
//
// ARCHITECTURE — ONE SCENE SNAPSHOT PER TICK
//   Each monitor tick fetches exactly ONE /get_planning_scene snapshot and
//   validates every future-sampled state against that SAME snapshot, rather
//   than issuing a /check_state_validity call per future sample against
//   whatever scene happens to be live at that instant. This guarantees all
//   future samples in one tick are judged against one consistent obstacle
//   position (see the Stage-3C C1 design lock). Requested components are
//   the minimum needed for a real collision check: ROBOT_STATE,
//   ROBOT_STATE_ATTACHED_OBJECTS, WORLD_OBJECT_GEOMETRY,
//   ALLOWED_COLLISION_MATRIX, TRANSFORMS.
//
// PROGRESS ESTIMATION — ACTUAL STATE, NOT ELAPSED TIME
//   Progress along the planned trajectory is estimated every tick from the
//   ACTUAL measured /joint_states sample, projected onto the nearest
//   piecewise-linear joint-space TRAJECTORY SEGMENT (project_to_
//   nearest_segment()), not from wall/sim elapsed time and not from the
//   nearest single waypoint. A monotonic floor (MonotonicProgressTracker)
//   prevents measurement noise from walking progress backward through
//   already-executed trajectory. See PROJECT_STATE.md's Stage-3C C1 section
//   for the full rationale and the measured trajectory-resolution audit
//   that picked future_sample_dt_s's default.
//
// THREAD MODEL
//   TransportPathMonitor owns exactly one std::thread, created by start()
//   and always joined by stop() (also called defensively from the
//   destructor if the caller forgot). No detached thread is ever created.
//   The monitor's own /joint_states subscription and /collision_object
//   (dynamic_obstacle_0 freshness) subscription each use a heap-owned,
//   shared_ptr-captured-by-value callback state struct -- the SAME pattern
//   transport_executor.hpp's C0.3/C0.4 corrections established -- so a
//   message callback can never dereference a partially-or-fully-destroyed
//   TransportPathMonitor. The worker thread itself captures `this` in its
//   lambda, which is safe ONLY because stop() unconditionally joins before
//   this object's destructor can complete -- unlike the FJT action
//   callbacks in transport_executor.cpp (which rclcpp_action can invoke
//   after the client object is gone), a plain std::thread's lifetime is
//   fully bounded by join(), so this is a different, simpler, and still
//   safe pattern. The /get_planning_scene client call and the collision
//   checks all run synchronously on the worker thread; nothing here ever
//   touches MoveGroupInterface, so TRANSPORT's own execution/cancel path
//   (TransportExecutor, which does not use MoveGroupInterface either) is
//   never contended.

#ifndef UR5E_PICK_PLACE__TRANSPORT_PATH_MONITOR_HPP_
#define UR5E_PICK_PLACE__TRANSPORT_PATH_MONITOR_HPP_

#include <moveit/robot_model/robot_model.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "ur5e_pick_place/transport_executor.hpp"

namespace ur5e_pick_place
{

// --- C1 parameters (item 20: only these four plus the enable flag). -------
struct TransportMonitorParams
{
  bool enabled = true;
  double rate_hz = 10.0;
  // Chosen from the Stage-3C C1 trajectory-resolution audit recorded in
  // PROJECT_STATE.md (measured min/median/p95/max time gap and max
  // joint-space step between adjacent transport waypoints) -- see this
  // header's ARCHITECTURE note and the audit-run evidence for why 0.05 s
  // is no coarser than the observed resolution.
  double future_sample_dt_s = 0.05;
  std::string scene_service_name = "/get_planning_scene";
};

// --- Pure, ROS-free logic (directly unit-testable; see
// test/test_transport_path_monitor_logic.cpp) --------------------------

// Extracts `snap`'s positions for exactly the joints in `joint_names`, in
// that order. False if any named joint is absent or has no position value
// -- mirrors SettleTracker::addSample's own precondition checks.
inline bool extract_ordered_positions(
  const sensor_msgs::msg::JointState & snap,
  const std::vector<std::string> & joint_names,
  std::vector<double> & out)
{
  out.clear();
  out.reserve(joint_names.size());
  for (const auto & name : joint_names) {
    const auto it = std::find(snap.name.begin(), snap.name.end(), name);
    if (it == snap.name.end()) {
      return false;
    }
    const std::size_t idx = static_cast<std::size_t>(std::distance(snap.name.begin(), it));
    if (idx >= snap.position.size()) {
      return false;
    }
    out.push_back(snap.position[idx]);
  }
  return true;
}

// Result of projecting an actual joint-space sample onto the nearest
// clamped point of the piecewise-linear trajectory (item 12).
// segment_index i means the projection landed on segment [i, i+1].
struct SegmentProjection
{
  std::size_t segment_index{0};
  double fraction{0.0};             // clamped to [0, 1] within the segment
  double nearest_joint_error{0.0};  // Euclidean joint-space distance, radians
};

// Nearest-segment projection: for every trajectory segment, finds the
// clamped closest point (in joint space, Euclidean over the trajectory's
// own joint coordinates -- never Cartesian, per item 12) to `actual`, and
// returns the globally closest one. `waypoints` must have >= 2 entries, all
// the same size as `actual`.
inline SegmentProjection project_to_nearest_segment(
  const std::vector<std::vector<double>> & waypoints,
  const std::vector<double> & actual)
{
  if (waypoints.size() < 2) {
    throw std::invalid_argument("project_to_nearest_segment: need >= 2 waypoints");
  }
  const std::size_t joint_count = actual.size();
  for (const auto & wp : waypoints) {
    if (wp.size() != joint_count) {
      throw std::invalid_argument("project_to_nearest_segment: joint-count mismatch");
    }
  }

  SegmentProjection best;
  double best_dist_sq = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const auto & p0 = waypoints[i];
    const auto & p1 = waypoints[i + 1];
    double len_sq = 0.0;
    double dot = 0.0;
    for (std::size_t j = 0; j < joint_count; ++j) {
      const double d = p1[j] - p0[j];
      len_sq += d * d;
      dot += d * (actual[j] - p0[j]);
    }
    double t = (len_sq > 1e-12) ? (dot / len_sq) : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    double dist_sq = 0.0;
    for (std::size_t j = 0; j < joint_count; ++j) {
      const double closest = p0[j] + t * (p1[j] - p0[j]);
      const double diff = actual[j] - closest;
      dist_sq += diff * diff;
    }
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best.segment_index = i;
      best.fraction = t;
    }
  }
  best.nearest_joint_error = std::sqrt(best_dist_sq);
  return best;
}

inline double segment_progress_scalar(std::size_t segment_index, double fraction)
{
  return static_cast<double>(segment_index) + std::clamp(fraction, 0.0, 1.0);
}

// Item 13: progress never moves backward through already-executed
// trajectory within one monitor run. Pure, stateful, directly testable.
class MonotonicProgressTracker
{
public:
  double accept(double raw_progress_scalar)
  {
    previous_ = std::max(previous_, raw_progress_scalar);
    return previous_;
  }
  double previous() const { return previous_; }

private:
  double previous_{0.0};
};

// Splits an accepted (monotonic) progress scalar back into a clamped
// (segment_index, fraction) pair for reporting and time conversion.
inline void decompose_progress_scalar(
  double progress_scalar, std::size_t max_segment_index,
  std::size_t & segment_index_out, double & fraction_out)
{
  const double clamped = std::max(0.0, progress_scalar);
  std::size_t seg = static_cast<std::size_t>(std::floor(clamped));
  if (seg > max_segment_index) {
    seg = max_segment_index;
  }
  fraction_out = std::clamp(clamped - static_cast<double>(seg), 0.0, 1.0);
  segment_index_out = seg;
}

// Converts an (segment_index, fraction) progress pair into a
// time_from_start value via linear interpolation between that segment's
// two waypoint times -- the same linear-time-interpolation contract
// find_time_segment()/interpolate_joint_positions() use for future sampling
// (see item 14).
inline double progress_time_from_segment(
  const std::vector<double> & time_from_start_s, std::size_t segment_index, double fraction)
{
  const double t0 = time_from_start_s[segment_index];
  const double t1 = time_from_start_s[segment_index + 1];
  return t0 + fraction * (t1 - t0);
}

// Time-based bracketing segment + blend fraction for a query time within a
// trajectory's own time_from_start values (item 14's "linear time
// interpolation" contract). Clamped at both ends. This is a direct,
// self-contained replacement for
// robot_trajectory::RobotTrajectory::getStateAtDurationFromStart(), which
// was found to segfault inside moveit::core::RobotState::interpolate() in
// this installed MoveIt 2.12.4 configuration (see PROJECT_STATE.md's
// Stage-3C C1 section) -- installed-API evidence proving that path
// impossible, per item 6/14's own contingency, so future-state sampling is
// implemented directly instead.
struct TimeSegmentBlend
{
  std::size_t segment_index{0};
  double fraction{0.0};
};

inline TimeSegmentBlend find_time_segment(
  const std::vector<double> & times_s, double query_time_s)
{
  if (times_s.size() < 2) {
    throw std::invalid_argument("find_time_segment: need >= 2 time points");
  }
  if (query_time_s <= times_s.front()) {
    return {0, 0.0};
  }
  const std::size_t last_segment = times_s.size() - 2;
  if (query_time_s >= times_s.back()) {
    return {last_segment, 1.0};
  }
  for (std::size_t i = 0; i + 1 < times_s.size(); ++i) {
    if (query_time_s <= times_s[i + 1]) {
      const double span = times_s[i + 1] - times_s[i];
      const double fraction = (span > 1e-12) ? (query_time_s - times_s[i]) / span : 0.0;
      return {i, std::clamp(fraction, 0.0, 1.0)};
    }
  }
  return {last_segment, 1.0};
}

// Linear joint-space interpolation between two waypoints at the given
// (segment, fraction) blend -- the direct-sampling counterpart to
// find_time_segment(), used for every future collision-check sample.
inline std::vector<double> interpolate_joint_positions(
  const std::vector<std::vector<double>> & waypoints, const TimeSegmentBlend & blend)
{
  const auto & p0 = waypoints[blend.segment_index];
  const auto & p1 = waypoints[blend.segment_index + 1];
  std::vector<double> out(p0.size());
  for (std::size_t j = 0; j < p0.size(); ++j) {
    out[j] = p0[j] + blend.fraction * (p1[j] - p0[j]);
  }
  return out;
}

// Item 14/15: deterministic future-sample time list starting strictly after
// `start_time_s`, stepped by `dt_s`, always ending exactly at
// `total_duration_s` (added as a final sample if the last stepped value
// would otherwise undershoot it) so the very end of the remaining path is
// never skipped. Empty if dt_s <= 0 or start_time_s is already past the
// trajectory's end.
inline std::vector<double> generate_future_sample_times(
  double start_time_s, double dt_s, double total_duration_s)
{
  std::vector<double> out;
  constexpr double kEps = 1e-9;
  if (dt_s <= 0.0 || start_time_s > total_duration_s + kEps) {
    return out;
  }
  for (std::int64_t k = 0;; ++k) {
    const double t = start_time_s + static_cast<double>(k) * dt_s;
    if (t > total_duration_s + kEps) {
      break;
    }
    out.push_back(t);
  }
  if (out.empty() || out.back() < total_duration_s - kEps) {
    out.push_back(total_duration_s);
  }
  return out;
}

// Item 16: index of the first `false` entry, or nullopt if all true. Split
// out from the actual FCL collision calls so the "stop scanning at the
// first invalid future state" contract is independently testable.
inline std::optional<std::size_t> first_invalid_index(const std::vector<bool> & sample_valid)
{
  for (std::size_t i = 0; i < sample_valid.size(); ++i) {
    if (!sample_valid[i]) {
      return i;
    }
  }
  return std::nullopt;
}

// Item 11: true unless `age_ms` is a finite, non-negative value at or below
// `stale_threshold_ms`. A negative age (the recorded timestamp is in the
// future relative to the node's own clock) is physically impossible under
// one consistent clock domain and is evidence of a clock-domain mismatch
// (e.g. a publisher not sharing the node's sim-time source), not a
// genuinely fresh sample -- treated as stale/untrustworthy, never as
// valid. `age_ms == +infinity` (no sample received yet) is likewise
// stale. Pure and directly testable; the sole authority for the
// scene_stale decision in TransportPathMonitor::run().
inline bool is_obstacle_data_stale(double age_ms, double stale_threshold_ms)
{
  return !(age_ms >= 0.0 && age_ms <= stale_threshold_ms);
}

struct TickPeriodStats
{
  double min_s{0.0};
  double median_s{0.0};
  double p95_s{0.0};
  double max_s{0.0};
};

// Percentile via linear interpolation between order statistics -- same
// convention as numpy's default ("linear") -- pure and directly testable.
inline TickPeriodStats compute_stats(std::vector<double> samples)
{
  TickPeriodStats stats;
  if (samples.empty()) {
    return stats;
  }
  std::sort(samples.begin(), samples.end());
  auto percentile = [&](double p) {
      const double idx = p * static_cast<double>(samples.size() - 1);
      const std::size_t lo = static_cast<std::size_t>(std::floor(idx));
      const std::size_t hi = static_cast<std::size_t>(std::ceil(idx));
      if (lo == hi) {
        return samples[lo];
      }
      const double frac = idx - static_cast<double>(lo);
      return samples[lo] * (1.0 - frac) + samples[hi] * frac;
    };
  stats.min_s = samples.front();
  stats.max_s = samples.back();
  stats.median_s = percentile(0.5);
  stats.p95_s = percentile(0.95);
  return stats;
}

// --- Summary telemetry, valid after stop() has joined the worker. ---------
struct TransportMonitorSummary
{
  bool started{false};
  int tick_count{0};
  double rate_target_hz{0.0};
  double monitor_duration_s{0.0};
  TickPeriodStats tick_period_stats{};
  TickPeriodStats scene_latency_ms_stats{};
  TickPeriodStats compute_ms_stats{};
  int invalid_tick_count{0};
  int stale_tick_count{0};
  int max_consecutive_invalid_ticks{0};
  bool has_invalidity{false};
  double first_invalidity_monitor_elapsed_s{0.0};
  double last_invalidity_monitor_elapsed_s{0.0};
};

// --- The ROS/MoveIt-dependent orchestrator. -------------------------------
class TransportPathMonitor
{
public:
  TransportPathMonitor(
    rclcpp::Node::SharedPtr node,
    moveit::core::RobotModelConstPtr robot_model,
    TransportMonitorParams params);

  // Joins the worker thread if start() was called and stop() was not
  // (defensive -- normal callers always call stop() explicitly before this
  // object goes out of scope; see transport.cpp's Stage 4 block).
  ~TransportPathMonitor();

  TransportPathMonitor(const TransportPathMonitor &) = delete;
  TransportPathMonitor & operator=(const TransportPathMonitor &) = delete;

  // Starts the worker thread observing `trajectory` (the exact planned
  // transport JointTrajectory, sent to arm_controller unmodified -- see
  // transport.cpp). No-op if params.enabled is false or trajectory has
  // fewer than 2 points: observe-only means doing nothing is always a
  // legal, silently-recorded outcome, never a failure. May be called at
  // most once per instance.
  void start(const trajectory_msgs::msg::JointTrajectory & trajectory);

  // Requests the worker thread stop and joins it. Idempotent and always
  // safe to call (including when start() was never called, or was a
  // no-op). Blocks until the thread (if any) has fully exited.
  void stop();

  const TransportMonitorSummary & summary() const { return summary_; }

private:
  struct ObstacleStampState
  {
    std::mutex mutex;
    bool has_sample{false};
    double last_stamp_s{0.0};
  };

  void run(trajectory_msgs::msg::JointTrajectory trajectory);
  bool fetchScene(moveit_msgs::msg::PlanningScene & out) const;
  void sleepUntilNextTick(
    std::chrono::steady_clock::time_point tick_start,
    std::chrono::duration<double> period) const;

  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_;
  TransportMonitorParams params_;

  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::shared_ptr<JointStateCallbackState> joint_state_state_;
  rclcpp::Subscription<moveit_msgs::msg::CollisionObject>::SharedPtr obstacle_sub_;
  std::shared_ptr<ObstacleStampState> obstacle_state_;

  std::thread worker_;
  std::atomic<bool> stop_requested_{false};
  bool started_{false};

  TransportMonitorSummary summary_;
};

}  // namespace ur5e_pick_place

#endif  // UR5E_PICK_PLACE__TRANSPORT_PATH_MONITOR_HPP_
