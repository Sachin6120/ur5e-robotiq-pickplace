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
  CONFIG_ERROR,             // required parameter missing or inconsistent
  POSE_VERIFY_FAILURE,      // execution reported SUCCESS but ground truth
                            // disagrees with the commanded pose
  GRASP_LOST_DURING_LIFT,   // actuated joint closed past
                            // expected_grip_angle + grasp_loss_threshold_rad
                            // during the lift leg -- the object left the
                            // fingers under load; transport is not attempted
  PRE_LIFT_BARRIER_TIMEOUT, // pre_lift_barrier_file was set and no external
                            // release appeared before pre_lift_barrier_timeout_s.
                            // Its OWN variant for the same reason
                            // PERCEPTION_TIMEOUT is: the configuration was
                            // fine and the grasp succeeded, an external
                            // evaluator simply never released the barrier.
                            // The lift is not attempted, and no transport,
                            // place or release occurs.
  PERCEPTION_TIMEOUT        // require_perception was set and no fresh, valid
                            // perceived object position arrived before the
                            // timeout. Deliberately its OWN variant and not
                            // CONFIG_ERROR: the configuration was fine, the
                            // sensor produced nothing usable. In strict mode
                            // this aborts before any target is composed --
                            // no motion, and explicitly no silent fall back
                            // to the configured position, which would make a
                            // perception failure indistinguishable from a
                            // perception success in the evidence.
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
    case Result::POSE_VERIFY_FAILURE:    return "POSE_VERIFY_FAILURE";
    case Result::GRASP_LOST_DURING_LIFT: return "GRASP_LOST_DURING_LIFT";
    case Result::PRE_LIFT_BARRIER_TIMEOUT: return "PRE_LIFT_BARRIER_TIMEOUT";
    case Result::PERCEPTION_TIMEOUT:     return "PERCEPTION_TIMEOUT";
  }
  // Unreachable for a well-formed Result. Deliberately NOT "UNKNOWN" as a
  // default case inside the switch: leaving the switch exhaustive means the
  // compiler flags a newly added variant instead of it silently landing here.
  return "UNHANDLED_RESULT_VALUE";
}

inline bool ok(Result r) { return r == Result::SUCCESS; }

}  // namespace ur5e_pick_place
