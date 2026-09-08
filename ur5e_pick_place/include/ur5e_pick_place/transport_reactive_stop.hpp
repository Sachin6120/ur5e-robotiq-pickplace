// Stage-3C C2: copied collision evidence and one execution-event arbiter.
// No ROS client, goal handle, scene reference, or cancellation API lives here.
#pragma once

#include <array>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <optional>
#include <string>

#include "ur5e_pick_place/failure.hpp"

namespace ur5e_pick_place
{
using TransportClock = std::chrono::steady_clock;

inline double transport_stamp(TransportClock::time_point t)
{
  return std::chrono::duration<double>(t.time_since_epoch()).count();
}

struct TransportCollisionEvidence
{
  std::string reason{"FRESH_FUTURE_PATH_COLLISION"};
  TransportClock::time_point detected;
  TransportClock::time_point latched;
  int tick{0};
  double scene_age_ms{0.0};
  std::size_t segment{0};
  double fraction{0.0};
  double progress_time_s{0.0};
  double nearest_joint_error_rad{0.0};
  double first_invalid_time_s{0.0};
  std::size_t first_invalid_sample{0};
  std::string collision_pairs;
  // World xyz and quaternion xyzw, copied from this tick's scene.
  std::array<double, 7> obstacle_pose{};
  bool obstacle_pose_known{false};
};

inline bool collision_stop_eligible(
  bool active, bool fresh, bool current_valid, bool future_valid, bool has_pairs)
{
  return active && fresh && current_valid && !future_valid && has_pairs;
}

enum class TransportTerminalCause { NONE, NATURAL, WATCHDOG, COLLISION };

// Event claims are serialized by this mutex. The first claim wins; the
// terminal callback's entry timestamp decides its deadline classification.
// Ties at the deadline therefore belong to WATCHDOG. Detection time is
// evidence, not a backdated arbitration event. A terminal callback or later
// monitor tick cannot replace an already-selected cause.
class TransportReactiveStopSignal
{
public:
  explicit TransportReactiveStopSignal(bool enabled = true) : enabled_(enabled) {}

  void arm(TransportClock::time_point deadline)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    deadline_ = deadline;
    armed_ = true;
  }

  void goalAccepted()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = true;
  }

  bool request(TransportCollisionEvidence evidence)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_ || !armed_ || !active_ || cause_ != TransportTerminalCause::NONE) {
      return false;
    }
    const auto now = TransportClock::now();
    if (now >= deadline_) {
      cause_ = TransportTerminalCause::WATCHDOG;
    } else {
      evidence.latched = now;
      evidence_ = std::move(evidence);
      cause_ = TransportTerminalCause::COLLISION;
    }
    cv_.notify_all();
    return cause_ == TransportTerminalCause::COLLISION;
  }

  // One synchronous transaction: commit arbitration BEFORE result_done can
  // become visible, but keep wait()/request() excluded until publication is
  // complete. Thus wait() cannot return NATURAL with an unavailable payload.
  // observed_at is captured once on callback entry, not after bookkeeping.
  // publish must only publish result state; never re-enter this signal or
  // wait on another thread. Production lock order: this mutex -> result_mutex.
  // No result reader may acquire this mutex while holding result_mutex.
  template<typename Publish>
  void publishTerminal(TransportClock::time_point observed_at, Publish publish)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (cause_ == TransportTerminalCause::NONE) {
      cause_ = armed_ && observed_at >= deadline_ ?
        TransportTerminalCause::WATCHDOG : TransportTerminalCause::NATURAL;
    }
    active_ = false;
    publish();
    lock.unlock();
    cv_.notify_all();
  }

  TransportTerminalCause wait()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_until(lock, deadline_, [this] {return cause_ != TransportTerminalCause::NONE;});
    if (cause_ == TransportTerminalCause::NONE) {
      cause_ = TransportTerminalCause::WATCHDOG;
    }
    return cause_;
  }

  std::optional<TransportCollisionEvidence> evidence() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return evidence_;
  }

private:
  const bool enabled_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  bool armed_{false};
  bool active_{false};
  TransportClock::time_point deadline_{};
  TransportTerminalCause cause_{TransportTerminalCause::NONE};
  std::optional<TransportCollisionEvidence> evidence_;
};

inline Result collision_stop_result(
  bool cancel_confirmed, bool terminal_canceled, bool settled, bool state_e_captured)
{
  if (!settled) {return Result::TRANSPORT_PHYSICAL_SETTLE_TIMEOUT;}
  if (!cancel_confirmed) {return Result::TRANSPORT_COLLISION_CANCEL_UNCONFIRMED;}
  if (!terminal_canceled) {return Result::TRANSPORT_FJT_EXECUTION_FAILED;}
  if (!state_e_captured) {return Result::CONFIG_ERROR;}
  return Result::TRANSPORT_COLLISION_STOPPED;
}
}  // namespace ur5e_pick_place
