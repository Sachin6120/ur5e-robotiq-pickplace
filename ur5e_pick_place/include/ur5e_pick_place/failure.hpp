// failure.hpp — typed failure results, shared by every milestone node.
//
// The project spec requires that each of these logs at ERROR with a named
// cause, aborts the task, and returns a typed failure printed as a ONE-LINE run
// summary. Defining the enum once, here, means M2..M5 cannot quietly invent
// their own vocabulary or forget a case: adding a variant without handling it
// in to_string() is a compiler warning, not a silent gap.
//
// There must be no path where something fails and the run continues looking
// normal.

#pragma once

#include <string>

namespace ur5e_pick_place
{

enum class Result
{
  SUCCESS = 0,
  PLAN_FAILURE,             // MoveIt could not find a plan
  EXECUTE_FAILURE,          // plan found, execution rejected or aborted
  CARTESIAN_FRACTION_LOW,   // computeCartesianPath() below threshold
  TF_LOOKUP_TIMEOUT,        // tf2 lookup exceeded tf_lookup_timeout_s
  GRIPPER_GOAL_REJECTED,    // gripper action rejected, or not reached in time
  SLIP_CHECK_FAILURE,       // post-lift slip exceeded post_lift_slip_max_m
  CONFIG_ERROR              // required parameter missing or inconsistent
};

inline const char * to_string(Result r)
{
  switch (r) {
    case Result::SUCCESS:                return "SUCCESS";
    case Result::PLAN_FAILURE:           return "PLAN_FAILURE";
    case Result::EXECUTE_FAILURE:        return "EXECUTE_FAILURE";
    case Result::CARTESIAN_FRACTION_LOW: return "CARTESIAN_FRACTION_LOW";
    case Result::TF_LOOKUP_TIMEOUT:      return "TF_LOOKUP_TIMEOUT";
    case Result::GRIPPER_GOAL_REJECTED:  return "GRIPPER_GOAL_REJECTED";
    case Result::SLIP_CHECK_FAILURE:     return "SLIP_CHECK_FAILURE";
    case Result::CONFIG_ERROR:           return "CONFIG_ERROR";
  }
  // Unreachable for a well-formed Result. Deliberately NOT "UNKNOWN" as a
  // default case inside the switch: leaving the switch exhaustive means the
  // compiler flags a newly added variant instead of it silently landing here.
  return "UNHANDLED_RESULT_VALUE";
}

inline bool ok(Result r) { return r == Result::SUCCESS; }

}  // namespace ur5e_pick_place
