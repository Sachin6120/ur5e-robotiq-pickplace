# Project Start Prompt: UR5e + Robotiq 2F-85 Pick-and-Place with MoveIt 2

## Project
Build a ROS 2 pick-and-place simulation using a UR5e arm with a Robotiq 2F-85 gripper.
Pick a known object from a fixed pose on a table, place it at a second fixed pose.
All motion planning goes through MoveIt 2. No custom analytical IK solver.
Perception/object detection is out of scope for v1.

## Environment & stack
- ROS 2 Jazzy Jalisco (LTS, supported to May 2029)
- Gazebo Harmonic (LTS). Not Fortress (EOL May 2027), not Classic (already EOL)
- Arm: UR5e via official Universal_Robots_ROS2_GZ_Simulation + Universal_Robots_ROS2_Description
- Gripper: Robotiq 2F-85 -- needs URDF merge + MoveIt config via Setup Assistant,
  since UR doesn't ship an integrated gripper. Use an existing reference integration;
  report which repo and what had to change.

## Milestone -1: assemble the combined platform
1. Merge UR5e + Robotiq 2F-85 xacros (correct flange/mount transform)
2. Generate MoveIt 2 config (arm + gripper planning groups) via Setup Assistant
3. Confirm merged model spawns cleanly in Gazebo Harmonic before M0

## Milestone 0: verify the stack before writing application code
A. Gazebo version binding -- confirm binds to Harmonic, no stray old-Gazebo deps
B. Gripper controller path -- confirm controller names match moveit_controllers.yaml
C. Mimic joint handling -- confirm all mimic joints track the actuated joint in
   gz_ros2_control, cross-checked against Gazebo's own state output, not just /joint_states
Deliverable: written pass/fail note with log lines. Not "it works."

## Architecture
MoveGroupInterface (C++), two instances: arm planning group, gripper planning group.
MoveIt Task Constructor deferred until perception lands.

## Waypoints -- derived, never hardcoded
config/scene.yaml holds object pose, place pose, object dimensions.
Gazebo world spawn and TF publisher both read the same file.
- object_frame -> grasp_frame via grasp definition (approach axis, standoff, gripper roll)
- Pre-grasp = grasp pose translated along gripper's own approach axis, in grasp_frame
- Approach/retreat use computeCartesianPath(); fraction below 0.95 aborts with logged reason
- Place uses same grasp-relative composition
attachObject/detachObject are collision bookkeeping ONLY -- never evidence of a real grasp.

## Gripper targets
Robotiq 2F-85's actuated joint maps to opening width nonlinearly (4-bar linkage) --
do not reuse a "half object width minus squeeze" formula built for two independent
prismatic fingers. Derive open/close from this gripper's actual joint-to-width relationship.

## Grasping approach
Real contact/friction-based grasping. Harmonic defaults to DART physics --
ODE-era Gazebo Classic friction-tuning advice won't transfer cleanly.
Fallback (only with permission): contact-triggered attach via finger-pad contact sensors,
never a position-based teleport that ignores contact.
Every run logs its mode at startup: "GRASP MODE: friction (physics)" or
"GRASP MODE: contact-triggered attach (fallback)".

## Milestones
M-1: combined URDF+MoveIt config assembled, spawns cleanly -- note on reference repo used
M0: stack verification A/B/C -- pass/fail note with log lines
M1: controllers up, MoveIt executes a joint goal -- 20/20 planning success, logged
M2: TF-derived pre-grasp/grasp reached, no gripper yet -- TCP pose vs commanded, Gazebo ground truth
M3: friction grasp tuning -- 20 cycles, >=18 with slip <5mm from Gazebo ground truth, zero ejection/penetration
M4: full loop incl. place and retreat -- one annotated run log
M5: repeatability -- 20 cycles, CSV

## Verification standard
Any "this works" claim comes with what was measured or observed -- a rate, a run's log,
a screenshot from that same run. Never accept a claim that contradicts a screenshot,
or vice versa, without reconciling both against ground truth.

## Failure handling
No silent failures: plan failure, execute failure, Cartesian path fraction below threshold,
tf2 lookup timeout, gripper goal rejected/not reached, post-lift slip check failure --
each logs at ERROR with a named cause and aborts, returning a typed failure enum.

## Git hygiene
Before the first commit, create .claude/settings.json:
{ "attribution": { "commit": "", "pr": "" } }
