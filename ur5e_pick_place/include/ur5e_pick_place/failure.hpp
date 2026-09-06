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
  PERCEPTION_TIMEOUT,       // require_perception was set and no fresh, valid
                            // perceived object position arrived before the
                            // timeout. Deliberately its OWN variant and not
                            // CONFIG_ERROR: the configuration was fine, the
                            // sensor produced nothing usable. In strict mode
                            // this aborts before any target is composed --
                            // no motion, and explicitly no silent fall back
                            // to the configured position, which would make a
                            // perception failure indistinguishable from a
                            // perception success in the evidence.
  STARTUP_NOT_AT_M1,        // simulation startup did not converge at the required M1 observation pose.
  STARTUP_STATE_UNAVAILABLE, // startup state subscription timed out without genuine arm joint sample
  SCENE_INIT_FAILURE,       // table initialization or readback verification failed
  TARGET_INSERTION_FAILURE, // perceived world target insertion or readback failed
  SCENE_STALE_OR_CORRUPT,   // planning scene fingerprint or readback verification mismatch
  UNEXPECTED_COLLISION,     // collision detected during descent or before closure
  CLOSURE_ACM_FAILURE,      // C1/C2 closure contact ACM enablement failed
  ATTACH_FAILURE,           // target attachment to gripper failed
  PICKUP_CLEARANCE_FAILURE, // pickup support-clearance verification or S-removal failed
  PAYLOAD_COLLISION,        // collision detected during payload transport
  PLACEMENT_PRECONTACT_FAILURE, // placement pre-contact waypoint reached with collision or invalid fraction
  TERMINAL_SUPPORT_FAILURE, // placement support S enablement failed
  DETACH_FAILURE,           // target detachment to world failed
  FINAL_WORLD_UPDATE_FAILURE, // post-retreat world target update failed

  // --- Stage-3C C0: direct-FJT TRANSPORT execution (transport_executor.hpp) ---
  // TRANSPORT only. Every other leg still fails through the existing
  // PLAN_FAILURE/EXECUTE_FAILURE vocabulary above unchanged.
  CONTROLLER_UNAVAILABLE,      // arm_controller not ACTIVE per
                               // controller_manager/list_controllers, or its
                               // FollowJointTrajectory action server did not
                               // become available within the configured timeout
  TRANSPORT_START_TOLERANCE_VIOLATED, // actual current arm state deviates from the
                               // planned transport trajectory's first point by more
                               // than transport_allowed_start_tolerance_rad on at
                               // least one joint -- reproduces MoveIt TEM's
                               // allowed_start_tolerance, which direct FJT
                               // execution bypasses
  TRANSPORT_FJT_GOAL_REJECTED, // the direct FollowJointTrajectory goal was not
                               // accepted by arm_controller's action server
  TRANSPORT_FJT_EXECUTION_FAILED, // the FollowJointTrajectory goal reached a
                               // terminal state other than SUCCEEDED (CANCELED,
                               // ABORTED, UNKNOWN), or SUCCEEDED with
                               // error_code != SUCCESSFUL -- never reinterpreted
                               // as SUCCESS
  TRANSPORT_EXECUTION_WATCHDOG_TIMEOUT, // planned_trajectory_duration *
                               // transport_execution_duration_scaling +
                               // transport_goal_duration_margin_s elapsed before a
                               // terminal result arrived; the in-flight goal was
                               // cancelled as cleanup (not a collision trigger --
                               // Stage-3C reactive cancellation is not implemented
                               // yet, see transport_executor.hpp's C0 scope note),
                               // AND live joint-velocity evidence confirmed the arm
                               // physically settled before this was returned
  TRANSPORT_PHYSICAL_SETTLE_TIMEOUT, // C0.1: the watchdog cancelled the in-flight
                               // goal and it reached a terminal action status, but
                               // stationary_consecutive_samples consecutive DISTINCT
                               // /joint_states samples below
                               // stationary_velocity_eps never occurred within
                               // stationary_timeout_s -- FJT CANCELED/terminal is
                               // NOT physical stop (Stage-3C Phase 0.3 evidence),
                               // so this is deliberately NOT reported as the
                               // ordinary watchdog timeout above: it means physical
                               // stop could not be confirmed at all
  TRANSPORT_WATCHDOG_CANCEL_UNCONFIRMED // C0.3: physical settle WAS confirmed (the
                               // arm is stopped), but the controller never confirmed
                               // accepting this exact goal for cancellation (the
                               // CancelResponse either never arrived, was rejected,
                               // or did not list this goal's UUID in
                               // goals_canceling) -- distinct from
                               // TRANSPORT_EXECUTION_WATCHDOG_TIMEOUT, which means
                               // cancellation WAS confirmed in addition to settle;
                               // this value means the stop cannot be attributed to
                               // the watchdog's own cancel request with certainty
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
    case Result::STARTUP_NOT_AT_M1:       return "STARTUP_NOT_AT_M1";
    case Result::STARTUP_STATE_UNAVAILABLE: return "STARTUP_STATE_UNAVAILABLE";
    case Result::SCENE_INIT_FAILURE:     return "SCENE_INIT_FAILURE";
    case Result::TARGET_INSERTION_FAILURE: return "TARGET_INSERTION_FAILURE";
    case Result::SCENE_STALE_OR_CORRUPT: return "SCENE_STALE_OR_CORRUPT";
    case Result::UNEXPECTED_COLLISION:   return "UNEXPECTED_COLLISION";
    case Result::CLOSURE_ACM_FAILURE:    return "CLOSURE_ACM_FAILURE";
    case Result::ATTACH_FAILURE:         return "ATTACH_FAILURE";
    case Result::PICKUP_CLEARANCE_FAILURE: return "PICKUP_CLEARANCE_FAILURE";
    case Result::PAYLOAD_COLLISION:      return "PAYLOAD_COLLISION";
    case Result::PLACEMENT_PRECONTACT_FAILURE: return "PLACEMENT_PRECONTACT_FAILURE";
    case Result::TERMINAL_SUPPORT_FAILURE: return "TERMINAL_SUPPORT_FAILURE";
    case Result::DETACH_FAILURE:         return "DETACH_FAILURE";
    case Result::FINAL_WORLD_UPDATE_FAILURE: return "FINAL_WORLD_UPDATE_FAILURE";
    case Result::CONTROLLER_UNAVAILABLE:  return "CONTROLLER_UNAVAILABLE";
    case Result::TRANSPORT_START_TOLERANCE_VIOLATED: return "TRANSPORT_START_TOLERANCE_VIOLATED";
    case Result::TRANSPORT_FJT_GOAL_REJECTED: return "TRANSPORT_FJT_GOAL_REJECTED";
    case Result::TRANSPORT_FJT_EXECUTION_FAILED: return "TRANSPORT_FJT_EXECUTION_FAILED";
    case Result::TRANSPORT_EXECUTION_WATCHDOG_TIMEOUT: return "TRANSPORT_EXECUTION_WATCHDOG_TIMEOUT";
    case Result::TRANSPORT_PHYSICAL_SETTLE_TIMEOUT: return "TRANSPORT_PHYSICAL_SETTLE_TIMEOUT";
    case Result::TRANSPORT_WATCHDOG_CANCEL_UNCONFIRMED: return "TRANSPORT_WATCHDOG_CANCEL_UNCONFIRMED";
  }
  // Unreachable for a well-formed Result. Deliberately NOT "UNKNOWN" as a
  // default case inside the switch: leaving the switch exhaustive means the
  // compiler flags a newly added variant instead of it silently landing here.
  return "UNHANDLED_RESULT_VALUE";
}

inline bool ok(Result r) { return r == Result::SUCCESS; }

}  // namespace ur5e_pick_place
