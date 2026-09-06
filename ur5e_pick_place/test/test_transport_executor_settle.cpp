// test_transport_executor_settle.cpp — focused unit tests for the C0.3
// distinct-sample settle-tracking pieces (is_new_joint_state_sample,
// SettleTracker) and the C0.4 /joint_states callback lifetime fix
// (JointStateCallbackState), all pulled out of transport_executor.hpp
// specifically so they are testable without a ROS node, executor, or
// action server.
//
// These tests exist to prove exactly the defects C0.3/C0.4 fixed:
//   1. a cached JointState message, fed repeatedly under an unchanged
//      sequence number (exactly what a stalled /joint_states stream
//      looks like), cannot be counted as six consecutive samples.
//   2. six genuinely distinct (sequence-advancing) stationary samples
//      DO satisfy the settle criterion.
//   3. the reported maximum velocity is the true running maximum across
//      every distinct sample seen, not merely the final (settled) one.
//   4. a JointStateCallbackState outlives an "owner" shared_ptr reset
//      (simulating TransportExecutor destruction) as long as any other
//      copy -- e.g. one held by a still-in-flight callback -- exists,
//      and that surviving copy's mutation is correctly visible.

#include <functional>
#include <memory>

#include <gtest/gtest.h>

#include "ur5e_pick_place/transport_executor.hpp"

namespace
{
using ur5e_pick_place::JointStateCallbackState;
using ur5e_pick_place::SettleTracker;
using ur5e_pick_place::is_new_joint_state_sample;

const std::vector<std::string> kJointNames = {"j1", "j2"};

sensor_msgs::msg::JointState make_sample(double v1, double v2)
{
  sensor_msgs::msg::JointState js;
  js.name = {"j1", "j2"};
  js.position = {0.0, 0.0};
  js.velocity = {v1, v2};
  return js;
}
}  // namespace

TEST(IsNewJointStateSample, DetectsRepeatVsAdvance)
{
  EXPECT_FALSE(is_new_joint_state_sample(5, 5));
  EXPECT_TRUE(is_new_joint_state_sample(6, 5));
  EXPECT_TRUE(is_new_joint_state_sample(1, 0));
}

// Defect A: a cached low-velocity sample, observed under an unchanged
// sequence number six times in a row (exactly what waitForPhysicalSettle
// would see against a stalled /joint_states stream, since its CV
// predicate is is_new_joint_state_sample), must not be evaluated more
// than once.
TEST(SettleTracker, CachedSampleCannotCountSixTimes)
{
  SettleTracker tracker(kJointNames, /*velocity_eps_rad_s=*/1.0e-3, /*required_consecutive=*/6);
  const auto low = make_sample(1.0e-4, 1.0e-4);

  uint64_t last_seen_seq = 0;
  const uint64_t stalled_seq = 5;  // the stream never advances past this
  int actual_add_sample_calls = 0;
  for (int poll = 0; poll < 6; ++poll) {
    if (is_new_joint_state_sample(stalled_seq, last_seen_seq)) {
      last_seen_seq = stalled_seq;
      tracker.addSample(low);
      ++actual_add_sample_calls;
    }
  }

  EXPECT_EQ(actual_add_sample_calls, 1)
    << "the gate must admit the cached sample exactly once, never on every poll";
  EXPECT_EQ(tracker.consecutiveAchieved(), 1);
  EXPECT_FALSE(tracker.settled()) << "one counted sample must not satisfy a 6-sample requirement";
}

// Six genuinely distinct (sequence-advancing) stationary samples DO
// satisfy the settle criterion.
TEST(SettleTracker, SixDistinctSamplesSatisfySettle)
{
  SettleTracker tracker(kJointNames, /*velocity_eps_rad_s=*/1.0e-3, /*required_consecutive=*/6);
  const auto low = make_sample(2.0e-4, -3.0e-4);

  uint64_t last_seen_seq = 0;
  int actual_add_sample_calls = 0;
  for (uint64_t seq = 1; seq <= 6; ++seq) {
    ASSERT_TRUE(is_new_joint_state_sample(seq, last_seen_seq));
    last_seen_seq = seq;
    tracker.addSample(low);
    ++actual_add_sample_calls;
  }

  EXPECT_EQ(actual_add_sample_calls, 6);
  EXPECT_EQ(tracker.consecutiveAchieved(), 6);
  EXPECT_TRUE(tracker.settled());
}

// The reported maximum velocity must be the true running maximum across
// every distinct sample, not the final (by-definition near-zero, once
// settled) sample -- the exact telemetry-labeling defect C0.3 fixed.
TEST(SettleTracker, MaxVelocityIsTrueMaximumNotLastSample)
{
  SettleTracker tracker(kJointNames, /*velocity_eps_rad_s=*/1.0e-3, /*required_consecutive=*/6);

  // A genuinely moving sample first (resets any prior consecutive count),
  // then six distinct stationary samples to reach settle.
  tracker.addSample(make_sample(0.35, 0.10));  // e.g. mid-cancellation motion
  for (int i = 0; i < 6; ++i) {
    tracker.addSample(make_sample(1.0e-4, 1.0e-4));
  }

  EXPECT_TRUE(tracker.settled());
  EXPECT_EQ(tracker.consecutiveAchieved(), 6);
  EXPECT_NEAR(tracker.maxVelocityRadS(), 0.35, 1e-9)
    << "the maximum must reflect the early high-velocity sample, not the final near-zero one";
  EXPECT_LT(tracker.lastVelocityRadS(), 1.0e-3)
    << "the last-sample telemetry should still separately report the final (low) velocity";
  EXPECT_GT(tracker.maxVelocityRadS(), tracker.lastVelocityRadS())
    << "max and last must be genuinely distinct quantities in this scenario";
}

// C0.4: the exact ownership guarantee the /joint_states subscription
// callback fix relies on. TransportExecutor holds one shared_ptr member
// ("owner" here); the subscription callback captures its OWN copy by
// value ("callback" here). Resetting "owner" simulates TransportExecutor
// being destroyed while a callback invocation is still queued/in
// flight. The callback must not crash, and its mutation must remain
// correctly visible through any other surviving reference ("observer").
TEST(JointStateCallbackState, SurvivesOwnerResetViaCallbackCopy)
{
  auto owner = std::make_shared<JointStateCallbackState>();
  std::shared_ptr<JointStateCallbackState> observer = owner;  // a second, independent reference

  // Mirrors TransportExecutor's actual subscription lambda: captures a
  // COPY of the shared_ptr by value, never `this`, never a reference.
  std::function<void(sensor_msgs::msg::JointState::ConstSharedPtr)> callback =
    [state = owner](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(state->mutex);
      state->latest = std::move(msg);
      ++state->sequence;
      state->cv.notify_all();
    };

  owner.reset();  // simulates TransportExecutor's own member being destroyed
  ASSERT_EQ(owner, nullptr);
  ASSERT_NE(observer, nullptr) << "another surviving reference must keep the object alive";

  auto msg = std::make_shared<sensor_msgs::msg::JointState>();
  msg->name = {"j1"};
  msg->velocity = {1.23};

  ASSERT_NO_FATAL_FAILURE(callback(msg));  // must not touch freed memory

  std::lock_guard<std::mutex> lock(observer->mutex);
  EXPECT_EQ(observer->sequence, 1u)
    << "the callback's mutation must be visible through the surviving 'observer' reference";
  ASSERT_NE(observer->latest, nullptr);
  EXPECT_DOUBLE_EQ(observer->latest->velocity.at(0), 1.23);
}
