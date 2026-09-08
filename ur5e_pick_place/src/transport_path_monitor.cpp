#include "ur5e_pick_place/transport_path_monitor.hpp"

#include "ur5e_pick_place/planning_scene_manager.hpp"

#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>

#include <algorithm>
#include <future>
#include <sstream>

namespace ur5e_pick_place
{
namespace
{
// Fixed Stage-3B contract identifiers (see PROJECT_STATE.md's Stage-3B
// section) -- read-only here, never written, never used to alter
// dynamic_obstacle_scene_node's own behaviour.
constexpr char kDynamicObstacleId[] = "dynamic_obstacle_0";
// Mirrors, but does NOT read or modify, dynamic_obstacle_scene_node's own
// stale_threshold_s (0.250 s) production default -- C1 observes the
// obstacle's CollisionObject stream independently and applies the same
// documented threshold value to its own read-only freshness judgement.
constexpr double kObstacleStaleThresholdMs = 250.0;
constexpr auto kSceneServiceTimeout = std::chrono::seconds(2);

std::string format_collision_pairs(const collision_detection::CollisionResult & result)
{
  std::ostringstream ss;
  for (const auto & pair : result.contacts) {
    ss << pair.first.first << "<->" << pair.first.second << "; ";
  }
  return ss.str();
}

}  // namespace

TransportPathMonitor::TransportPathMonitor(
  rclcpp::Node::SharedPtr node, moveit::core::RobotModelConstPtr robot_model,
  TransportMonitorParams params, std::shared_ptr<TransportReactiveStopSignal> stop_signal)
: node_(std::move(node)), robot_model_(std::move(robot_model)), params_(std::move(params)),
  stop_signal_(std::move(stop_signal))
{
  scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>(
    params_.scene_service_name);

  // Own /joint_states subscription, deliberately separate from
  // TransportExecutor's -- see this header's THREAD MODEL note. Same
  // heap-owned, captured-by-value callback-state pattern as
  // transport_executor.cpp's C0.4 fix.
  joint_state_state_ = std::make_shared<JointStateCallbackState>();
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [state = joint_state_state_](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->latest = std::move(msg);
      ++state->sequence;
      state->cv.notify_all();
    });

  // Read-only dynamic_obstacle_0 freshness observation (item 11) -- never
  // touches dynamic_obstacle_scene_node itself. Reliable QoS matches that
  // node's own /collision_object publisher.
  obstacle_state_ = std::make_shared<ObstacleStampState>();
  obstacle_sub_ = node_->create_subscription<moveit_msgs::msg::CollisionObject>(
    "/collision_object", rclcpp::QoS(10).reliable(),
    [state = obstacle_state_](moveit_msgs::msg::CollisionObject::ConstSharedPtr msg) {
      if (msg->id != kDynamicObstacleId) {
        return;
      }
      std::lock_guard<std::mutex> lock(state->mutex);
      state->last_stamp_s = rclcpp::Time(msg->header.stamp).seconds();
      state->has_sample = true;
    });
}

TransportPathMonitor::~TransportPathMonitor()
{
  stop();
}

bool TransportPathMonitor::fetchScene(moveit_msgs::msg::PlanningScene & out) const
{
  if (!scene_client_->wait_for_service(kSceneServiceTimeout)) {
    return false;
  }
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components =
    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
    moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
    moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
    moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
    moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS;
  auto future = scene_client_->async_send_request(request);
  if (future.wait_for(kSceneServiceTimeout) != std::future_status::ready) {
    return false;
  }
  out = future.get()->scene;
  return true;
}

void TransportPathMonitor::sleepUntilNextTick(
  std::chrono::steady_clock::time_point tick_start, std::chrono::duration<double> period) const
{
  const auto deadline = tick_start +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  while (!stop_requested_.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

void TransportPathMonitor::start(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (started_) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "M3 C1 TRANSPORT_MONITOR_CONFIG_ERROR: start() called more than once on the same "
      "TransportPathMonitor instance; ignoring.");
    return;
  }
  started_ = true;
  summary_.started = params_.enabled && trajectory.points.size() >= 2;
  if (!params_.enabled) {
    RCLCPP_INFO(
      node_->get_logger(),
      "M3 C1 TRANSPORT_MONITOR_DISABLED: transport_monitor_enabled=false; observe-only "
      "monitoring will not run for this TRANSPORT leg.");
    return;
  }
  if (trajectory.points.size() < 2) {
    RCLCPP_WARN(
      node_->get_logger(),
      "M3 C1 TRANSPORT_MONITOR_INIT_SKIPPED: transport trajectory has < 2 points "
      "(%zu); nothing to monitor. TRANSPORT execution is unaffected.",
      trajectory.points.size());
    return;
  }
  stop_requested_ = false;
  worker_ = std::thread(&TransportPathMonitor::run, this, trajectory);
}

void TransportPathMonitor::stop()
{
  stop_requested_ = true;
  if (worker_.joinable()) {
    worker_.join();
  }
}

void TransportPathMonitor::run(trajectory_msgs::msg::JointTrajectory trajectory)
{
  const auto logger = node_->get_logger();
  const auto t_monitor_start = std::chrono::steady_clock::now();

  const std::size_t n_points = trajectory.points.size();
  const std::size_t n_joints = trajectory.joint_names.size();
  std::vector<std::vector<double>> waypoints(n_points);
  std::vector<double> times(n_points);
  for (std::size_t i = 0; i < n_points; ++i) {
    waypoints[i] = trajectory.points[i].positions;
    times[i] = rclcpp::Duration(trajectory.points[i].time_from_start).seconds();
  }
  const double total_duration_s = times.back();
  const std::size_t max_segment_index = n_points - 2;

  MonotonicProgressTracker progress_tracker;
  const std::chrono::duration<double> period(1.0 / std::max(params_.rate_hz, 1e-3));

  std::vector<double> tick_periods_s;
  std::vector<double> scene_latencies_ms;
  std::vector<double> compute_ms_samples;
  std::optional<std::chrono::steady_clock::time_point> last_tick_start;

  int tick_count = 0;
  int invalid_tick_count = 0;
  int stale_tick_count = 0;
  int consecutive_invalid = 0;
  int max_consecutive_invalid_ticks = 0;
  bool has_invalidity = false;
  double first_invalidity_elapsed_s = 0.0;
  double last_invalidity_elapsed_s = 0.0;

  // Item 14: measured, not assumed -- the actual planned transport
  // trajectory's own time spacing and joint-space step size, computed once
  // per monitor run from the exact trajectory being executed. This is the
  // audit evidence future_sample_dt_s's default was chosen against (see
  // PROJECT_STATE.md's Stage-3C C1 section), logged every run so any later
  // trajectory shape drift is directly observable in evidence, not assumed.
  std::vector<double> waypoint_dt_s;
  double max_waypoint_joint_delta_rad = 0.0;
  waypoint_dt_s.reserve(n_points - 1);
  for (std::size_t i = 0; i + 1 < n_points; ++i) {
    waypoint_dt_s.push_back(times[i + 1] - times[i]);
    double delta_sq = 0.0;
    for (std::size_t j = 0; j < n_joints; ++j) {
      const double d = waypoints[i + 1][j] - waypoints[i][j];
      delta_sq += d * d;
    }
    max_waypoint_joint_delta_rad = std::max(max_waypoint_joint_delta_rad, std::sqrt(delta_sq));
  }
  const auto dt_stats = compute_stats(waypoint_dt_s);

  RCLCPP_INFO(
    logger,
    "M3 C1 TRANSPORT_MONITOR_START t_monitor_start=%.6f rate_target_hz=%.3f "
    "future_sample_dt_s=%.4f scene_service=%s trajectory_points=%zu "
    "trajectory_duration_s=%.4f waypoint_dt_min_s=%.5f waypoint_dt_median_s=%.5f "
    "waypoint_dt_p95_s=%.5f waypoint_dt_max_s=%.5f max_waypoint_joint_delta_rad=%.6f",
    node_->now().seconds(), params_.rate_hz, params_.future_sample_dt_s,
    params_.scene_service_name.c_str(), n_points, total_duration_s, dt_stats.min_s,
    dt_stats.median_s, dt_stats.p95_s, dt_stats.max_s, max_waypoint_joint_delta_rad);

  while (!stop_requested_.load()) {
    const auto tick_start = std::chrono::steady_clock::now();
    if (last_tick_start) {
      tick_periods_s.push_back(
        std::chrono::duration<double>(tick_start - *last_tick_start).count());
    }
    last_tick_start = tick_start;
    ++tick_count;

    // 1. Latest cached /joint_states sample. Best-effort, non-blocking:
    // /joint_states publishes far faster than the monitor's own rate_hz, so
    // reading whatever is already cached is always fresh enough without
    // stalling the monitor loop waiting for a brand-new message.
    sensor_msgs::msg::JointState::ConstSharedPtr snap;
    {
      std::lock_guard<std::mutex> lock(joint_state_state_->mutex);
      snap = joint_state_state_->latest;
    }
    std::vector<double> actual;
    if (!snap || !extract_ordered_positions(*snap, trajectory.joint_names, actual)) {
      RCLCPP_WARN(
        logger, "M3 C1 TRANSPORT_MONITOR_TICK tick=%d skipped=NO_JOINT_STATE_YET", tick_count);
      sleepUntilNextTick(tick_start, period);
      continue;
    }

    // 2. ONE PlanningScene snapshot for this entire tick (design lock).
    // C2 conservative freshness bound captured BEFORE requesting this snapshot.
    // Later /collision_object updates must not freshen an older scene response.
    double obstacle_stamp_before_request = -1.0;
    {
      std::lock_guard<std::mutex> lock(obstacle_state_->mutex);
      if (obstacle_state_->has_sample) {
        obstacle_stamp_before_request = obstacle_state_->last_stamp_s;
      }
    }
    const auto t_scene_req = std::chrono::steady_clock::now();
    moveit_msgs::msg::PlanningScene scene_msg;
    const bool scene_ok = fetchScene(scene_msg);
    const auto t_scene_resp = std::chrono::steady_clock::now();
    const double scene_request_latency_ms =
      std::chrono::duration<double, std::milli>(t_scene_resp - t_scene_req).count();
    scene_latencies_ms.push_back(scene_request_latency_ms);
    if (!scene_ok) {
      RCLCPP_WARN(
        logger,
        "M3 C1 TRANSPORT_MONITOR_TICK tick=%d skipped=SCENE_FETCH_FAILED "
        "scene_request_latency_ms=%.3f",
        tick_count, scene_request_latency_ms);
      sleepUntilNextTick(tick_start, period);
      continue;
    }

    // 3. dynamic_obstacle_0 freshness -- read-only observation (item 11).
    // is_obstacle_data_stale() also treats a NEGATIVE age (the recorded
    // timestamp is in the future relative to this node's own clock -- a
    // clock-domain mismatch, e.g. a publisher not sharing the node's
    // sim-time source, rather than a genuinely fresh sample) as stale,
    // never as valid -- see that function's own doc comment.
    double scene_age_ms = std::numeric_limits<double>::infinity();
    {
      std::lock_guard<std::mutex> lock(obstacle_state_->mutex);
      if (obstacle_state_->has_sample) {
        scene_age_ms = (node_->now().seconds() - obstacle_state_->last_stamp_s) * 1000.0;
      }
    }
    const bool scene_stale = is_obstacle_data_stale(scene_age_ms, kObstacleStaleThresholdMs);
    if (scene_stale) {
      ++stale_tick_count;
    }

    // 4. Local collision-checkable scene, loaded from this tick's snapshot.
    auto local_scene = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    if (!local_scene->usePlanningSceneMsg(scene_msg)) {
      RCLCPP_WARN(
        logger, "M3 C1 TRANSPORT_MONITOR_TICK tick=%d skipped=SCENE_LOAD_FAILED", tick_count);
      sleepUntilNextTick(tick_start, period);
      continue;
    }
    const moveit::core::RobotState reference_state = local_scene->getCurrentState();

    // 5. Current-state validity, from the ACTUAL measured joint values
    // (item 15) -- reported separately from future-path validity.
    moveit::core::RobotState current_actual_state(reference_state);
    for (std::size_t j = 0; j < n_joints; ++j) {
      current_actual_state.setVariablePosition(trajectory.joint_names[j], actual[j]);
    }
    current_actual_state.update(true);
    collision_detection::CollisionRequest current_req;
    current_req.contacts = true;
    current_req.max_contacts = 10;
    collision_detection::CollisionResult current_res;
    local_scene->checkCollision(current_req, current_res, current_actual_state);
    const bool current_state_valid = !current_res.collision;

    // 6. Progress projection (items 12/13): nearest trajectory segment to
    // the actual joint state, with a monotonic floor.
    const SegmentProjection raw = project_to_nearest_segment(waypoints, actual);
    const double raw_progress = segment_progress_scalar(raw.segment_index, raw.fraction);
    const double accepted_progress = progress_tracker.accept(raw_progress);
    std::size_t accepted_segment = 0;
    double accepted_fraction = 0.0;
    decompose_progress_scalar(
      accepted_progress, max_segment_index, accepted_segment, accepted_fraction);
    const double progress_time_s =
      progress_time_from_segment(times, accepted_segment, accepted_fraction);

    // 7. Future path sampling (items 14/15/16), all against local_scene.
    const double future_start_s =
      std::min(progress_time_s + params_.future_sample_dt_s, total_duration_s);
    const auto sample_times =
      generate_future_sample_times(future_start_s, params_.future_sample_dt_s, total_duration_s);

    TransportClock::time_point invalidity_detected;
    bool future_path_valid = true;
    std::optional<std::size_t> first_invalid_sample;
    double first_invalid_time_s = 0.0;
    std::string collision_pairs;
    std::size_t future_samples_checked = 0;

    for (std::size_t k = 0; k < sample_times.size(); ++k) {
      const TimeSegmentBlend blend = find_time_segment(times, sample_times[k]);
      const std::vector<double> interp_positions = interpolate_joint_positions(waypoints, blend);
      moveit::core::RobotState future_state(reference_state);
      for (std::size_t j = 0; j < n_joints; ++j) {
        future_state.setVariablePosition(trajectory.joint_names[j], interp_positions[j]);
      }
      future_state.update(true);
      ++future_samples_checked;
      collision_detection::CollisionRequest req;
      req.contacts = true;
      req.max_contacts = 10;
      collision_detection::CollisionResult res;
      local_scene->checkCollision(req, res, future_state);
      if (res.collision) {
        invalidity_detected = TransportClock::now();
        future_path_valid = false;
        first_invalid_sample = k;
        first_invalid_time_s = sample_times[k];
        collision_pairs = format_collision_pairs(res);
        break;  // item 16: stop scanning after the first invalid sample
      }
    }

    // 8. dynamic_obstacle_0 pose telemetry, best-effort.
    std::string obstacle_pose_str = "UNKNOWN";
    TransportCollisionEvidence evidence;
    const auto obstacle_it = std::find_if(
      scene_msg.world.collision_objects.begin(), scene_msg.world.collision_objects.end(),
      [](const auto & co) { return co.id == kDynamicObstacleId; });
    if (obstacle_it != scene_msg.world.collision_objects.end() &&
      !obstacle_it->primitive_poses.empty())
    {
      const auto pose = PlanningSceneManager::effectivePrimitivePose(*obstacle_it, 0);
      evidence.obstacle_pose_known = true;
      evidence.obstacle_pose = {pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w};
      std::ostringstream ss;
      ss << "[" << pose.position.x << "," << pose.position.y << "," << pose.position.z << "]";
      obstacle_pose_str = ss.str();
    }

    // C2 adds no scene query or second collision checker: these booleans,
    // contact pairs, progress and pose all belong to this same local_scene.
    const double conservative_age_ms = obstacle_stamp_before_request >= 0.0 ?
      (node_->now().seconds() - obstacle_stamp_before_request) * 1000.0 :
      std::numeric_limits<double>::infinity();
    const double snapshot_elapsed_ms =
      std::chrono::duration<double, std::milli>(TransportClock::now() - t_scene_req).count();
    const bool trigger_fresh = !scene_stale &&
      !is_obstacle_data_stale(conservative_age_ms, kObstacleStaleThresholdMs) &&
      snapshot_elapsed_ms <= kObstacleStaleThresholdMs;
    if (stop_signal_ && collision_stop_eligible(!stop_requested_.load(), trigger_fresh,
        current_state_valid, future_path_valid, !collision_pairs.empty()))
    {
      evidence.detected = invalidity_detected;
      evidence.tick = tick_count;
      evidence.scene_age_ms = scene_age_ms;
      evidence.segment = accepted_segment;
      evidence.fraction = accepted_fraction;
      evidence.progress_time_s = progress_time_s;
      evidence.nearest_joint_error_rad = raw.nearest_joint_error;
      evidence.first_invalid_time_s = first_invalid_time_s;
      evidence.first_invalid_sample = *first_invalid_sample;
      evidence.collision_pairs = collision_pairs;
      if (stop_signal_->request(evidence)) {
        const auto first = *stop_signal_->evidence();
        RCLCPP_INFO(logger,
          "M3 C2 COLLISION_TRIGGER collision_triggered=1 collision_trigger_count=1 "
          "reason=%s t_invalidity_detect=%.9f t_stop_signal_latched=%.9f "
          "invalidity_to_signal_ms=%.6f tick=%d scene_age_ms=%.6f "
          "conservative_age_ms=%.6f snapshot_elapsed_ms=%.6f current_state_valid=1 "
          "future_path_valid=0 progress_segment=%zu progress_fraction=%.9f "
          "progress_time_s=%.9f nearest_joint_error_rad=%.9f "
          "first_invalid_time_s=%.9f first_invalid_sample=%zu temporal_lead_s=%.9f "
          "collision_pairs=\"%s\" dynamic_obstacle_pose=[%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f]",
          first.reason.c_str(), transport_stamp(first.detected), transport_stamp(first.latched),
          std::chrono::duration<double, std::milli>(first.latched - first.detected).count(),
          tick_count, scene_age_ms, conservative_age_ms, snapshot_elapsed_ms,
          accepted_segment, accepted_fraction, progress_time_s, raw.nearest_joint_error,
          first_invalid_time_s, *first_invalid_sample, first_invalid_time_s - progress_time_s,
          collision_pairs.c_str(), first.obstacle_pose[0], first.obstacle_pose[1],
          first.obstacle_pose[2], first.obstacle_pose[3], first.obstacle_pose[4],
          first.obstacle_pose[5], first.obstacle_pose[6]);
      }
    } else if (!future_path_valid && stop_signal_) {
      RCLCPP_INFO(logger,
        "M3 C2 TRIGGER_INELIGIBLE tick=%d fresh=%d current_state_valid=%d has_pairs=%d",
        tick_count, trigger_fresh, current_state_valid, !collision_pairs.empty());
    }

    const auto tick_end = std::chrono::steady_clock::now();
    const double monitor_compute_ms =
      std::chrono::duration<double, std::milli>(tick_end - tick_start).count();
    compute_ms_samples.push_back(monitor_compute_ms);

    if (!current_state_valid || !future_path_valid) {
      ++invalid_tick_count;
      ++consecutive_invalid;
      max_consecutive_invalid_ticks = std::max(max_consecutive_invalid_ticks, consecutive_invalid);
      const double elapsed_s = std::chrono::duration<double>(tick_end - t_monitor_start).count();
      if (!has_invalidity) {
        has_invalidity = true;
        first_invalidity_elapsed_s = elapsed_s;
      }
      last_invalidity_elapsed_s = elapsed_s;
    } else {
      consecutive_invalid = 0;
    }

    const std::string first_invalid_sample_str =
      first_invalid_sample ? std::to_string(*first_invalid_sample) : std::string("NONE");
    RCLCPP_INFO(
      logger,
      "M3 C1 TRANSPORT_MONITOR_TICK tick=%d scene_request_latency_ms=%.3f scene_age_ms=%.3f "
      "scene_stale=%d raw_segment=%zu accepted_segment=%zu progress_fraction=%.4f "
      "nearest_joint_error_rad=%.6f current_state_valid=%d future_path_valid=%d "
      "future_samples_checked=%zu first_invalid_sample=%s first_invalid_time_s=%.4f "
      "collision_pairs=\"%s\" dynamic_obstacle_pose=%s monitor_compute_ms=%.3f",
      tick_count, scene_request_latency_ms, scene_age_ms, scene_stale ? 1 : 0,
      raw.segment_index, accepted_segment, accepted_fraction, raw.nearest_joint_error,
      current_state_valid ? 1 : 0, future_path_valid ? 1 : 0, future_samples_checked,
      first_invalid_sample_str.c_str(), first_invalid_time_s, collision_pairs.c_str(),
      obstacle_pose_str.c_str(), monitor_compute_ms);

    if (!current_state_valid) {
      RCLCPP_WARN(
        logger,
        "M3 C1 TRANSPORT_MONITOR_CURRENT_STATE_INVALID tick=%d collision_pairs=\"%s\" -- "
        "current-state invalidity is not a C2 future-path trigger.",
        tick_count, format_collision_pairs(current_res).c_str());
    }
    if (!future_path_valid) {
      RCLCPP_WARN(
        logger,
        "M3 C1 TRANSPORT_MONITOR_FUTURE_PATH_INVALID tick=%d first_invalid_time_s=%.4f "
        "collision_pairs=\"%s\" -- observation recorded; C2 eligibility and latch reported separately.",
        tick_count, first_invalid_time_s, collision_pairs.c_str());
    }

    sleepUntilNextTick(tick_start, period);
  }

  const auto t_monitor_stop = std::chrono::steady_clock::now();
  summary_.tick_count = tick_count;
  summary_.rate_target_hz = params_.rate_hz;
  summary_.monitor_duration_s =
    std::chrono::duration<double>(t_monitor_stop - t_monitor_start).count();
  summary_.tick_period_stats = compute_stats(tick_periods_s);
  summary_.scene_latency_ms_stats = compute_stats(scene_latencies_ms);
  summary_.compute_ms_stats = compute_stats(compute_ms_samples);
  summary_.invalid_tick_count = invalid_tick_count;
  summary_.stale_tick_count = stale_tick_count;
  summary_.max_consecutive_invalid_ticks = max_consecutive_invalid_ticks;
  summary_.has_invalidity = has_invalidity;
  summary_.first_invalidity_monitor_elapsed_s = first_invalidity_elapsed_s;
  summary_.last_invalidity_monitor_elapsed_s = last_invalidity_elapsed_s;

  const double achieved_rate_hz =
    (summary_.monitor_duration_s > 0.0) ?
    static_cast<double>(tick_count) / summary_.monitor_duration_s : 0.0;
  RCLCPP_INFO(
    logger,
    "M3 C1 TRANSPORT_MONITOR_STOP t_monitor_stop=%.6f monitor_tick_count=%d "
    "monitor_rate_target_hz=%.3f monitor_rate_achieved_hz=%.3f "
    "actual_tick_period_min_s=%.4f actual_tick_period_median_s=%.4f "
    "actual_tick_period_p95_s=%.4f actual_tick_period_max_s=%.4f "
    "scene_request_latency_min_ms=%.3f scene_request_latency_median_ms=%.3f "
    "scene_request_latency_p95_ms=%.3f scene_request_latency_max_ms=%.3f "
    "monitor_compute_min_ms=%.3f monitor_compute_median_ms=%.3f "
    "monitor_compute_p95_ms=%.3f monitor_compute_max_ms=%.3f "
    "invalid_tick_count=%d stale_tick_count=%d max_consecutive_invalid_ticks=%d "
    "has_invalidity=%d first_invalidity_elapsed_s=%.4f last_invalidity_elapsed_s=%.4f",
    node_->now().seconds(), tick_count, params_.rate_hz, achieved_rate_hz,
    summary_.tick_period_stats.min_s, summary_.tick_period_stats.median_s,
    summary_.tick_period_stats.p95_s, summary_.tick_period_stats.max_s,
    summary_.scene_latency_ms_stats.min_s, summary_.scene_latency_ms_stats.median_s,
    summary_.scene_latency_ms_stats.p95_s, summary_.scene_latency_ms_stats.max_s,
    summary_.compute_ms_stats.min_s, summary_.compute_ms_stats.median_s,
    summary_.compute_ms_stats.p95_s, summary_.compute_ms_stats.max_s,
    invalid_tick_count, stale_tick_count, max_consecutive_invalid_ticks, has_invalidity ? 1 : 0,
    first_invalidity_elapsed_s, last_invalidity_elapsed_s);
}

}  // namespace ur5e_pick_place
