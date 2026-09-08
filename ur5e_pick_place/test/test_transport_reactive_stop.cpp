#include <gtest/gtest.h>
#include <atomic>
#include <future>
#include <thread>
#include <vector>
#include "ur5e_pick_place/transport_reactive_stop.hpp"

using namespace ur5e_pick_place;
using namespace std::chrono_literals;

namespace
{
void activate(TransportReactiveStopSignal & s)
{
  s.arm(TransportClock::now() + 10s);
  s.goalAccepted();
}
TransportCollisionEvidence evidence(int tick)
{
  TransportCollisionEvidence e;
  e.detected = TransportClock::now();
  e.tick = tick;
  e.collision_pairs = "dynamic_obstacle_0<->pick_target";
  e.obstacle_pose = {0.45, 0.15, 0.86, 0, 0, 0, 1};
  return e;
}
}
TEST(ReactiveStop, InitiallyNotRequested) {
  TransportReactiveStopSignal s;
  EXPECT_FALSE(s.evidence());
}
TEST(ReactiveStop, BeforeGoalAcceptanceCannotLatch) {
  TransportReactiveStopSignal s;
  s.arm(TransportClock::now() + 10s);
  EXPECT_FALSE(s.request(evidence(1)));
}
TEST(ReactiveStop, DisabledCannotLatch) {
  TransportReactiveStopSignal s(false);
  activate(s);
  EXPECT_FALSE(s.request(evidence(1)));
  s.publishTerminal(TransportClock::now(), [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::NATURAL);
}
TEST(ReactiveStop, FirstEvidenceIsCopiedAndPreserved) {
  TransportReactiveStopSignal s;
  activate(s);
  auto e = evidence(7);
  ASSERT_TRUE(s.request(e));
  e.tick = 99;
  e.collision_pairs.clear();
  e.obstacle_pose[0] = 99;
  ASSERT_TRUE(s.evidence());
  EXPECT_EQ(s.evidence()->tick, 7);
  EXPECT_EQ(s.evidence()->collision_pairs, "dynamic_obstacle_0<->pick_target");
  EXPECT_DOUBLE_EQ(s.evidence()->obstacle_pose[0], 0.45);
}
TEST(ReactiveStop, SecondRequestCannotOverwrite) {
  TransportReactiveStopSignal s;
  activate(s);
  ASSERT_TRUE(s.request(evidence(1)));
  const auto t = s.evidence()->latched;
  EXPECT_FALSE(s.request(evidence(2)));
  EXPECT_EQ(s.evidence()->tick, 1);
  EXPECT_EQ(s.evidence()->latched, t);
}
TEST(ReactiveStop, ConcurrentRequestsHaveExactlyOneWinner) {
  TransportReactiveStopSignal s;
  activate(s);
  std::atomic<int> count{0}, winner{-1};
  std::vector<std::thread> threads;
  for (int i = 0; i < 32; ++i) {
    threads.emplace_back([&, i] {if (s.request(evidence(i))) {++count; winner = i;}});
  }
  for (auto & t : threads) {t.join();}
  EXPECT_EQ(count, 1);
  EXPECT_EQ(s.evidence()->tick, winner.load());
  EXPECT_EQ(s.wait(), TransportTerminalCause::COLLISION);
}
TEST(ReactiveStop, ConditionVariableWakesExecutor) {
  auto s = std::make_shared<TransportReactiveStopSignal>();
  activate(*s);
  auto f = std::async(std::launch::async, [s] {return s->wait();});
  ASSERT_TRUE(s->request(evidence(1)));
  ASSERT_EQ(f.wait_for(1s), std::future_status::ready);
  EXPECT_EQ(f.get(), TransportTerminalCause::COLLISION);
}
TEST(Arbitration, NaturalBeforeCollisionWins) {
  TransportReactiveStopSignal s;
  activate(s);
  s.publishTerminal(TransportClock::now(), [] {});
  EXPECT_FALSE(s.request(evidence(1)));
  EXPECT_EQ(s.wait(), TransportTerminalCause::NATURAL);
}
TEST(Arbitration, CollisionBeforeTerminalWins) {
  TransportReactiveStopSignal s;
  activate(s);
  ASSERT_TRUE(s.request(evidence(1)));
  s.publishTerminal(TransportClock::now(), [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::COLLISION);
}
TEST(Arbitration, ExpiredWatchdogBeforeCollisionWins) {
  TransportReactiveStopSignal s;
  s.arm(TransportClock::now() - 1s);
  s.goalAccepted();
  EXPECT_FALSE(s.request(evidence(1)));
  EXPECT_EQ(s.wait(), TransportTerminalCause::WATCHDOG);
  EXPECT_FALSE(s.evidence());
}
TEST(Arbitration, WatchdogBeforeLateTerminalWins) {
  TransportReactiveStopSignal s;
  s.arm(TransportClock::now() - 1s);
  s.publishTerminal(TransportClock::now(), [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::WATCHDOG);
}
TEST(Arbitration, WatchdogWaitCannotBeReclassified) {
  TransportReactiveStopSignal s;
  s.arm(TransportClock::now());
  s.goalAccepted();
  EXPECT_EQ(s.wait(), TransportTerminalCause::WATCHDOG);
  EXPECT_FALSE(s.request(evidence(1)));
  s.publishTerminal(TransportClock::now(), [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::WATCHDOG);
}
TEST(Arbitration, ConcurrentNaturalAndCollisionAgreeWithAcceptedRequest) {
  TransportReactiveStopSignal s;
  activate(s);
  bool accepted = false;
  std::thread a([&] {accepted = s.request(evidence(1));});
  std::thread b([&] {s.publishTerminal(TransportClock::now(), [] {});});
  a.join(); b.join();
  EXPECT_EQ(s.wait(), accepted ? TransportTerminalCause::COLLISION : TransportTerminalCause::NATURAL);
  EXPECT_EQ(s.evidence().has_value(), accepted);
}
TEST(Eligibility, FreshCurrentValidFutureInvalidWithPairsQualifies) {
  EXPECT_TRUE(collision_stop_eligible(true, true, true, false, true));
}
TEST(Eligibility, StaleCannotTrigger) {
  EXPECT_FALSE(collision_stop_eligible(true, false, true, false, true));
}
TEST(Eligibility, CurrentInvalidIsNotFutureCollisionTrigger) {
  EXPECT_FALSE(collision_stop_eligible(true, true, false, false, true));
}
TEST(Eligibility, InactiveCannotTrigger) {
  EXPECT_FALSE(collision_stop_eligible(false, true, true, false, true));
}
TEST(Eligibility, MissingPairsCannotTrigger) {
  EXPECT_FALSE(collision_stop_eligible(true, true, true, false, false));
}
TEST(Eligibility, ValidFutureCannotTrigger) {
  EXPECT_FALSE(collision_stop_eligible(true, true, true, true, true));
}
TEST(CollisionResult, CompletedStopIsNotManipulationSuccess) {
  const auto r = collision_stop_result(true, true, true, true);
  EXPECT_EQ(r, Result::TRANSPORT_COLLISION_STOPPED);
  EXPECT_FALSE(ok(r));
  EXPECT_STREQ(to_string(r), "TRANSPORT_COLLISION_STOPPED");
}
TEST(CollisionResult, CancelUnconfirmedCannotSucceed) {
  EXPECT_EQ(collision_stop_result(false, true, true, true), Result::TRANSPORT_COLLISION_CANCEL_UNCONFIRMED);
}
TEST(CollisionResult, TerminalAloneCannotProvePhysicalStop) {
  EXPECT_EQ(collision_stop_result(true, true, false, false), Result::TRANSPORT_PHYSICAL_SETTLE_TIMEOUT);
}
TEST(CollisionResult, TerminalCanceledRequired) {
  EXPECT_EQ(collision_stop_result(true, false, true, true), Result::TRANSPORT_FJT_EXECUTION_FAILED);
}
TEST(CollisionResult, StateERequired) {
  EXPECT_EQ(collision_stop_result(true, true, true, false), Result::CONFIG_ERROR);
}

// Exercise the SAME transaction called by the FJT callback, pausing after
// result_done publication but before arbiter notification (the former gap).
TEST(TerminalPublication, PublishedResultRejectsCollisionBeforeNotification) {
  TransportReactiveStopSignal s;
  activate(s);
  std::atomic<bool> result_done{false};
  std::promise<void> published, release, requesting;
  auto release_future = release.get_future();
  auto callback = std::async(std::launch::async, [&] {
    const auto observed_at = TransportClock::now();
    s.publishTerminal(observed_at, [&] {
      result_done = true;
      published.set_value();
      release_future.wait();  // controlled callback suspension, tests only
    });
  });
  published.get_future().wait();
  EXPECT_TRUE(result_done);
  auto collision = std::async(std::launch::async, [&] {
    requesting.set_value();
    return s.request(evidence(2));
  });
  requesting.get_future().wait();
  EXPECT_EQ(collision.wait_for(20ms), std::future_status::timeout);
  release.set_value();
  callback.get();
  EXPECT_FALSE(collision.get());
  const auto cause = s.wait();
  EXPECT_EQ(cause, TransportTerminalCause::NATURAL);
  EXPECT_NE(cause, TransportTerminalCause::NONE);
  EXPECT_FALSE(s.evidence());
  const int cancel_decisions = cause == TransportTerminalCause::WATCHDOG ||
    cause == TransportTerminalCause::COLLISION ? 1 : 0;
  EXPECT_EQ(cancel_decisions, 0);
}

TEST(TerminalPublication, NaturalWaiterCannotConsumeBeforeCompletePayload) {
  TransportReactiveStopSignal s;
  activate(s);
  std::promise<void> claimed, release, waiting;
  auto release_future = release.get_future();
  std::atomic<bool> result_done{false};
  int error_code = -999;
  std::string error_string;
  auto callback = std::async(std::launch::async, [&] {
    s.publishTerminal(TransportClock::now(), [&] {
      claimed.set_value();  // cause selected, payload deliberately unavailable
      release_future.wait();
      error_code = 42;
      error_string = "complete terminal payload";
      result_done = true;
    });
  });
  claimed.get_future().wait();
  auto consumer = std::async(std::launch::async, [&] {
    waiting.set_value();
    const auto cause = s.wait();
    return cause == TransportTerminalCause::NATURAL && result_done &&
      error_code == 42 && error_string == "complete terminal payload";
  });
  waiting.get_future().wait();
  EXPECT_FALSE(result_done);
  EXPECT_EQ(consumer.wait_for(20ms), std::future_status::timeout);
  release.set_value();
  callback.get();
  EXPECT_TRUE(consumer.get());
}

TEST(TerminalPublication, CallbackEntryBeforeDeadlineSurvivesBookkeepingDelay) {
  TransportReactiveStopSignal s;
  const auto deadline = TransportClock::now();
  s.arm(deadline);
  s.goalAccepted();
  // Inject the observed entry timestamp; do not sleep to fake bookkeeping.
  s.publishTerminal(deadline - TransportClock::duration(1), [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::NATURAL);
}
TEST(TerminalPublication, ExactDeadlineBelongsToWatchdog) {
  TransportReactiveStopSignal s;
  const auto deadline = TransportClock::now() + 10s;
  s.arm(deadline);
  s.goalAccepted();
  s.publishTerminal(deadline, [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::WATCHDOG);
  EXPECT_FALSE(s.request(evidence(1)));
}
TEST(TerminalPublication, AfterDeadlineBelongsToWatchdog) {
  TransportReactiveStopSignal s;
  const auto deadline = TransportClock::now() + 10s;
  s.arm(deadline);
  s.goalAccepted();
  s.publishTerminal(deadline + TransportClock::duration(1), [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::WATCHDOG);
}
TEST(TerminalPublication, CollisionWinnerSurvivesLateTerminalAndDeadline) {
  TransportReactiveStopSignal s;
  const auto deadline = TransportClock::now() + 10s;
  s.arm(deadline);
  s.goalAccepted();
  ASSERT_TRUE(s.request(evidence(1)));
  s.publishTerminal(deadline, [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::COLLISION);
  EXPECT_FALSE(s.request(evidence(2)));
  EXPECT_EQ(s.evidence()->tick, 1);
}
TEST(TerminalPublication, ConcurrentNaturalAndWatchdogHaveOneStableWinner) {
  TransportReactiveStopSignal s;
  const auto deadline = TransportClock::now();
  s.arm(deadline);
  s.goalAccepted();
  std::promise<void> start;
  auto ready = start.get_future().share();
  std::atomic<bool> result_done{false};
  auto callback = std::async(std::launch::async, [&] {
    ready.wait();
    s.publishTerminal(deadline - TransportClock::duration(1), [&] {result_done = true;});
  });
  auto consumer = std::async(std::launch::async, [&] {ready.wait(); return s.wait();});
  start.set_value();
  const auto cause = consumer.get();
  if (cause == TransportTerminalCause::NATURAL) {EXPECT_TRUE(result_done);}
  callback.get();
  EXPECT_TRUE(cause == TransportTerminalCause::NATURAL || cause == TransportTerminalCause::WATCHDOG);
  EXPECT_EQ(s.wait(), cause);
  EXPECT_FALSE(s.request(evidence(1)));
}
TEST(TerminalPublication, ConcurrentExpiredWatchdogAndCollisionCannotDoubleSelect) {
  TransportReactiveStopSignal s;
  s.arm(TransportClock::now());
  s.goalAccepted();
  std::promise<void> start;
  auto ready = start.get_future().share();
  auto collision = std::async(std::launch::async, [&] {ready.wait(); return s.request(evidence(1));});
  auto consumer = std::async(std::launch::async, [&] {ready.wait(); return s.wait();});
  start.set_value();
  EXPECT_FALSE(collision.get());
  EXPECT_EQ(consumer.get(), TransportTerminalCause::WATCHDOG);
  s.publishTerminal(TransportClock::now(), [] {});
  EXPECT_EQ(s.wait(), TransportTerminalCause::WATCHDOG);
  EXPECT_FALSE(s.evidence());
}
