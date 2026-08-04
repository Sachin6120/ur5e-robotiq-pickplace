// moveit_compat.hpp — MoveIt 2 API surfaces that have changed across
// releases and are worth guessing right exactly once, in one place.
//
// WHY THIS EXISTS
//   m1_joint_goal.cpp first ran into MoveGroupInterface::Plan's trajectory
//   member being named differently across MoveIt versions (trajectory_ in
//   older releases, trajectory in this Jazzy install) and worked around it
//   locally with a SFINAE pair. m2_cartesian_approach.cpp then had to guess
//   the same thing again and, on the first pass, guessed the OLD name
//   (trajectory_) — a real, repeat instance of exactly the class of bug this
//   file exists to prevent. Promoted here so M3, M4 and M5 reuse the answer
//   instead of re-deriving it, and a version skew only needs fixing once.
//
// MoveIt also moved its public headers from .h to .hpp partway through the
// 2.x series — probed here too, so every node includes this file instead of
// repeating the __has_include guard.

#pragma once

#if __has_include(<moveit/move_group_interface/move_group_interface.hpp>)
#include <moveit/move_group_interface/move_group_interface.hpp>
#else
#include <moveit/move_group_interface/move_group_interface.h>
#endif

#include <cstddef>

namespace ur5e_pick_place
{
namespace detail
{
template <typename P>
auto traj_points(const P & p, int)
-> decltype(p.trajectory_.joint_trajectory.points.size())
{
  return p.trajectory_.joint_trajectory.points.size();
}

template <typename P>
auto traj_points(const P & p, long)
-> decltype(p.trajectory.joint_trajectory.points.size())
{
  return p.trajectory.joint_trajectory.points.size();
}

template <typename P, typename T>
auto set_traj(P & p, const T & t, int) -> decltype(p.trajectory_ = t, void())
{
  p.trajectory_ = t;
}

template <typename P, typename T>
auto set_traj(P & p, const T & t, long) -> decltype(p.trajectory = t, void())
{
  p.trajectory = t;
}
}  // namespace detail

// Number of waypoints in a Plan's trajectory, regardless of whether this
// MoveIt install names the member trajectory_ or trajectory.
template <typename P>
std::size_t points_in(const P & p)
{
  return detail::traj_points(p, 0);
}

// Assign a RobotTrajectory into a Plan, regardless of member name.
template <typename P, typename T>
void set_trajectory(P & p, const T & t)
{
  detail::set_traj(p, t, 0);
}

}  // namespace ur5e_pick_place
