// test_transport_path_monitor_logic.cpp — Stage-3C C1: pure-logic unit
// tests for progress projection, monotonic-floor handling, future-sample
// generation, and first-invalid-sample indexing. No ROS node, executor, or
// service is created -- everything under test here is a free function or a
// small stateful class taking plain values/messages (see
// transport_path_monitor.hpp).

#include "ur5e_pick_place/transport_path_monitor.hpp"

#include <gtest/gtest.h>

using ur5e_pick_place::MonotonicProgressTracker;
using ur5e_pick_place::SegmentProjection;

// --- project_to_nearest_segment ---------------------------------------

TEST(ProjectToNearestSegment, ExactlyOnAWaypoint) {
  const std::vector<std::vector<double>> waypoints{{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  const auto p = ur5e_pick_place::project_to_nearest_segment(waypoints, {1.0, 0.0});
  // Waypoint 1 is the end of segment 0 and the start of segment 1; either
  // projection is a valid zero-error match -- assert on the invariant
  // (zero error, fraction at a segment boundary), not a specific segment.
  EXPECT_NEAR(p.nearest_joint_error, 0.0, 1e-9);
  EXPECT_TRUE((p.segment_index == 0 && p.fraction > 0.999) ||
    (p.segment_index == 1 && p.fraction < 0.001));
}

TEST(ProjectToNearestSegment, MidSegment) {
  const std::vector<std::vector<double>> waypoints{{0.0}, {2.0}};
  const auto p = ur5e_pick_place::project_to_nearest_segment(waypoints, {0.5});
  EXPECT_EQ(p.segment_index, 0u);
  EXPECT_NEAR(p.fraction, 0.25, 1e-9);
  EXPECT_NEAR(p.nearest_joint_error, 0.0, 1e-9);
}

TEST(ProjectToNearestSegment, ClampsBeforeStart) {
  const std::vector<std::vector<double>> waypoints{{0.0}, {1.0}};
  const auto p = ur5e_pick_place::project_to_nearest_segment(waypoints, {-5.0});
  EXPECT_EQ(p.segment_index, 0u);
  EXPECT_NEAR(p.fraction, 0.0, 1e-9);
  EXPECT_NEAR(p.nearest_joint_error, 5.0, 1e-9);
}

TEST(ProjectToNearestSegment, ClampsPastEnd) {
  const std::vector<std::vector<double>> waypoints{{0.0}, {1.0}};
  const auto p = ur5e_pick_place::project_to_nearest_segment(waypoints, {5.0});
  EXPECT_EQ(p.segment_index, 0u);
  EXPECT_NEAR(p.fraction, 1.0, 1e-9);
  EXPECT_NEAR(p.nearest_joint_error, 4.0, 1e-9);
}

TEST(ProjectToNearestSegment, DegenerateZeroLengthSegmentDoesNotCrash) {
  // Repeated waypoint (zero-length segment) -- must project to fraction 0
  // at that segment's start rather than dividing by zero.
  const std::vector<std::vector<double>> waypoints{{0.0}, {0.0}, {1.0}};
  const auto p = ur5e_pick_place::project_to_nearest_segment(waypoints, {0.0});
  EXPECT_NEAR(p.nearest_joint_error, 0.0, 1e-9);
}

TEST(ProjectToNearestSegment, ThrowsOnTooFewWaypoints) {
  const std::vector<std::vector<double>> waypoints{{0.0}};
  EXPECT_THROW(
    ur5e_pick_place::project_to_nearest_segment(waypoints, {0.0}), std::invalid_argument);
}

TEST(ProjectToNearestSegment, PicksGloballyNearestSegmentNotFirst) {
  // A 2-joint rectangular path: (0,0) -> (10,0) -> (10,1) -> (0,1). A point
  // near the top edge is much closer to segment 2 ((10,1)-(0,1)) than to
  // segment 0 ((0,0)-(10,0)), even though segment 0 spans the same X range
  // -- this is only distinguishable with >= 2 joints, exercising the
  // multi-dimensional distance sum rather than a single coordinate.
  const std::vector<std::vector<double>> waypoints{
    {0.0, 0.0}, {10.0, 0.0}, {10.0, 1.0}, {0.0, 1.0}};
  const auto p = ur5e_pick_place::project_to_nearest_segment(waypoints, {5.0, 0.9});
  EXPECT_EQ(p.segment_index, 2u);
  EXPECT_NEAR(p.nearest_joint_error, 0.1, 1e-9);
}

// --- MonotonicProgressTracker (item 13) --------------------------------

TEST(MonotonicProgressTracker, NeverMovesBackward) {
  MonotonicProgressTracker tracker;
  EXPECT_NEAR(tracker.accept(0.2), 0.2, 1e-12);
  EXPECT_NEAR(tracker.accept(1.5), 1.5, 1e-12);
  // Noisy backward raw sample: accepted progress must hold at the floor.
  EXPECT_NEAR(tracker.accept(1.1), 1.5, 1e-12);
  EXPECT_NEAR(tracker.previous(), 1.5, 1e-12);
  // A genuinely larger raw sample still advances past the floor.
  EXPECT_NEAR(tracker.accept(2.0), 2.0, 1e-12);
}

TEST(MonotonicProgressTracker, StartsAtZero) {
  MonotonicProgressTracker tracker;
  EXPECT_NEAR(tracker.previous(), 0.0, 1e-12);
}

// --- decompose_progress_scalar / progress_time_from_segment -----------

TEST(DecomposeProgressScalar, MidRangeRoundTrips) {
  std::size_t seg = 999;
  double frac = -1.0;
  ur5e_pick_place::decompose_progress_scalar(2.3, 5, seg, frac);
  EXPECT_EQ(seg, 2u);
  EXPECT_NEAR(frac, 0.3, 1e-9);
}

TEST(DecomposeProgressScalar, ClampsToMaxSegmentIndex) {
  std::size_t seg = 999;
  double frac = -1.0;
  ur5e_pick_place::decompose_progress_scalar(10.0, 3, seg, frac);
  EXPECT_EQ(seg, 3u);
  EXPECT_NEAR(frac, 1.0, 1e-9);
}

TEST(ProgressTimeFromSegment, LinearBlend) {
  const std::vector<double> times{0.0, 1.0, 3.0};
  EXPECT_NEAR(ur5e_pick_place::progress_time_from_segment(times, 1, 0.5), 2.0, 1e-9);
  EXPECT_NEAR(ur5e_pick_place::progress_time_from_segment(times, 0, 0.0), 0.0, 1e-9);
  EXPECT_NEAR(ur5e_pick_place::progress_time_from_segment(times, 0, 1.0), 1.0, 1e-9);
}

// --- find_time_segment / interpolate_joint_positions (item 14, the
// direct-sampling replacement for RobotTrajectory::getStateAtDurationFrom
// Start(), which segfaults in this installed MoveIt configuration) --------

TEST(FindTimeSegment, MidSegmentBlend) {
  const std::vector<double> times{0.0, 1.0, 3.0};
  const auto blend = ur5e_pick_place::find_time_segment(times, 2.0);
  EXPECT_EQ(blend.segment_index, 1u);
  EXPECT_NEAR(blend.fraction, 0.5, 1e-9);
}

TEST(FindTimeSegment, ClampsBeforeStartAndAfterEnd) {
  const std::vector<double> times{0.0, 1.0, 3.0};
  const auto before = ur5e_pick_place::find_time_segment(times, -5.0);
  EXPECT_EQ(before.segment_index, 0u);
  EXPECT_NEAR(before.fraction, 0.0, 1e-9);
  const auto after = ur5e_pick_place::find_time_segment(times, 50.0);
  EXPECT_EQ(after.segment_index, 1u);
  EXPECT_NEAR(after.fraction, 1.0, 1e-9);
}

TEST(FindTimeSegment, ThrowsOnTooFewTimes) {
  EXPECT_THROW(ur5e_pick_place::find_time_segment({1.0}, 0.5), std::invalid_argument);
}

TEST(InterpolateJointPositions, LinearBlendPerJoint) {
  const std::vector<std::vector<double>> waypoints{{0.0, 10.0}, {2.0, 20.0}};
  const auto out = ur5e_pick_place::interpolate_joint_positions(waypoints, {0, 0.25});
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0], 0.5, 1e-9);
  EXPECT_NEAR(out[1], 12.5, 1e-9);
}

// --- generate_future_sample_times (items 14/15) ------------------------

TEST(GenerateFutureSampleTimes, StepsAndIncludesExactEnd) {
  const auto samples = ur5e_pick_place::generate_future_sample_times(0.0, 0.05, 0.12);
  ASSERT_GE(samples.size(), 3u);
  EXPECT_NEAR(samples.front(), 0.0, 1e-9);
  EXPECT_NEAR(samples.back(), 0.12, 1e-9);
  for (std::size_t i = 0; i + 1 < samples.size(); ++i) {
    EXPECT_LE(samples[i], samples[i + 1] + 1e-9);
  }
}

TEST(GenerateFutureSampleTimes, ExactMultipleDoesNotDuplicateEnd) {
  const auto samples = ur5e_pick_place::generate_future_sample_times(0.0, 0.05, 0.10);
  ASSERT_EQ(samples.size(), 3u);
  EXPECT_NEAR(samples[0], 0.0, 1e-9);
  EXPECT_NEAR(samples[1], 0.05, 1e-9);
  EXPECT_NEAR(samples[2], 0.10, 1e-9);
}

TEST(GenerateFutureSampleTimes, EmptyWhenStartPastEnd) {
  const auto samples = ur5e_pick_place::generate_future_sample_times(5.0, 0.05, 1.0);
  EXPECT_TRUE(samples.empty());
}

TEST(GenerateFutureSampleTimes, EmptyWhenDtNonPositive) {
  EXPECT_TRUE(ur5e_pick_place::generate_future_sample_times(0.0, 0.0, 1.0).empty());
  EXPECT_TRUE(ur5e_pick_place::generate_future_sample_times(0.0, -0.1, 1.0).empty());
}

TEST(GenerateFutureSampleTimes, SingleSampleWhenStartEqualsEnd) {
  const auto samples = ur5e_pick_place::generate_future_sample_times(1.0, 0.05, 1.0);
  ASSERT_EQ(samples.size(), 1u);
  EXPECT_NEAR(samples[0], 1.0, 1e-9);
}

// --- first_invalid_index (item 16) --------------------------------------

TEST(FirstInvalidIndex, AllValidReturnsNullopt) {
  EXPECT_FALSE(ur5e_pick_place::first_invalid_index({true, true, true}).has_value());
}

TEST(FirstInvalidIndex, EmptyReturnsNullopt) {
  EXPECT_FALSE(ur5e_pick_place::first_invalid_index({}).has_value());
}

TEST(FirstInvalidIndex, ReturnsFirstFalseNotLast) {
  const auto idx = ur5e_pick_place::first_invalid_index({true, true, false, false, true});
  ASSERT_TRUE(idx.has_value());
  EXPECT_EQ(*idx, 2u);
}

// --- extract_ordered_positions ------------------------------------------

TEST(ExtractOrderedPositions, HappyPath) {
  sensor_msgs::msg::JointState js;
  js.name = {"a", "b", "c"};
  js.position = {1.0, 2.0, 3.0};
  std::vector<double> out;
  ASSERT_TRUE(ur5e_pick_place::extract_ordered_positions(js, {"c", "a"}, out));
  ASSERT_EQ(out.size(), 2u);
  EXPECT_NEAR(out[0], 3.0, 1e-9);
  EXPECT_NEAR(out[1], 1.0, 1e-9);
}

TEST(ExtractOrderedPositions, MissingJointFails) {
  sensor_msgs::msg::JointState js;
  js.name = {"a", "b"};
  js.position = {1.0, 2.0};
  std::vector<double> out;
  EXPECT_FALSE(ur5e_pick_place::extract_ordered_positions(js, {"a", "z"}, out));
}

TEST(ExtractOrderedPositions, NamePresentButNoPositionValueFails) {
  sensor_msgs::msg::JointState js;
  js.name = {"a", "b"};
  js.position = {1.0};  // missing "b"'s position
  std::vector<double> out;
  EXPECT_FALSE(ur5e_pick_place::extract_ordered_positions(js, {"a", "b"}, out));
}

// --- is_obstacle_data_stale (item 11) ------------------------------------
// Regression test for a real defect found during the Stage-3C C1 closeout
// audit: a clock-domain-mismatched publisher (the C1B qualification
// harness's own REMOVE publish, not sim-time-synced) produced a hugely
// NEGATIVE age, which the original `age_ms > threshold` comparison let
// through as "not stale" (a negative number is never greater than a
// positive threshold). Never observed in production Stage-3B operation
// (all its publishers share one sim-time clock), but a real latent gap in
// the monitor's own defensive handling.

TEST(IsObstacleDataStale, FreshWithinThresholdIsNotStale) {
  EXPECT_FALSE(ur5e_pick_place::is_obstacle_data_stale(100.0, 250.0));
  EXPECT_FALSE(ur5e_pick_place::is_obstacle_data_stale(0.0, 250.0));
  EXPECT_FALSE(ur5e_pick_place::is_obstacle_data_stale(250.0, 250.0));
}

TEST(IsObstacleDataStale, PastThresholdIsStale) {
  EXPECT_TRUE(ur5e_pick_place::is_obstacle_data_stale(250.001, 250.0));
  EXPECT_TRUE(ur5e_pick_place::is_obstacle_data_stale(1e6, 250.0));
}

TEST(IsObstacleDataStale, NoSampleYetInfinityIsStale) {
  EXPECT_TRUE(ur5e_pick_place::is_obstacle_data_stale(
    std::numeric_limits<double>::infinity(), 250.0));
}

TEST(IsObstacleDataStale, NegativeAgeFromClockMismatchIsStaleNotFresh) {
  // The exact regression this audit found: a huge negative age must never
  // be reported as "not stale".
  EXPECT_TRUE(ur5e_pick_place::is_obstacle_data_stale(-1788850930945.275, 250.0));
  EXPECT_TRUE(ur5e_pick_place::is_obstacle_data_stale(-0.001, 250.0));
}

// --- compute_stats --------------------------------------------------------

TEST(ComputeStats, EmptyIsAllZero) {
  const auto stats = ur5e_pick_place::compute_stats({});
  EXPECT_NEAR(stats.min_s, 0.0, 1e-12);
  EXPECT_NEAR(stats.max_s, 0.0, 1e-12);
}

TEST(ComputeStats, MinMedianMaxOnSortedInput) {
  const auto stats = ur5e_pick_place::compute_stats({1.0, 2.0, 3.0, 4.0, 5.0});
  EXPECT_NEAR(stats.min_s, 1.0, 1e-9);
  EXPECT_NEAR(stats.max_s, 5.0, 1e-9);
  EXPECT_NEAR(stats.median_s, 3.0, 1e-9);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
