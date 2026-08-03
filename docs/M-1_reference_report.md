# M-1 Gate Report — Reference Integration Selection

**Status:** research complete, assembly not started
**Date:** 2026-08-03
**Decision:** harvest from `darshmenon/UR3_ROS2_PICK_AND_PLACE`, do not fork

---

## 1. What was searched

Requirement was a current, actively maintained integration of **UR arm + Robotiq 2F-85 +
Gazebo Harmonic + MoveIt 2 on ROS 2 Jazzy**. Three candidates surfaced.

| Repo | Arm | Gripper | Sim | Distro | Verdict |
|---|---|---|---|---|---|
| [darshmenon/UR3_ROS2_PICK_AND_PLACE](https://github.com/darshmenon/UR3_ROS2_PICK_AND_PLACE) | UR3 | Robotiq 2F-85/2F-140, OnRobot RG2/RG6 | Gazebo Harmonic (gz-sim 8.x) | Humble primary, Jazzy documented | **Selected as donor** |
| [MOGI-ROS/Week-11-12-Robot-arms](https://github.com/MOGI-ROS/Week-11-12-Robot-arms) | UR3e | via `ros2-jazzy-gripper` branch | Gazebo Harmonic | Jazzy native | Secondary opinion on the merge |
| [JuoTungChen/ROS2_pick_and_place_UR5](https://github.com/JuoTungChen/ROS2_pick_and_place_UR5) | UR5 | Robotiq | not Harmonic-specific | older | Rejected — predates Harmonic |

## 2. Why donor, not fork

The darshmenon repo is the closest functional match but carries a large payload that is
explicitly out of scope for v1:

- **MoveIt Task Constructor** is its pick-place mechanism. The project spec defers MTC
  deliberately. Forking adopts MTC silently — precisely what the spec forbids.
- **MongoDB** is a hard dependency of the MTC demo path via `warehouse_ros_mongo`.
- Perception, LLM planner (Ollama), SAC/RL training, OpenVLA, ACT, behavior trees,
  conveyor sim. None of it needed; all of it is surface area that can break a build.
- It is **UR3**, not UR5e. `ur_type` is parameterised upstream, but link lengths,
  reach envelope, and any tuned poses are UR3-specific.
- **ROS 2 Humble is the primary target.** Jazzy is documented as a secondary path
  (`ros-jazzy-ros-gz-sim`, `ros-jazzy-ros-gz-bridge`, `ros-jazzy-moveit-planners-stomp`
  added on top). Expect friction rather than a clean build.

**Harvest list** — take these, leave the rest:

1. The UR + Robotiq 2F-85 URDF/xacro merge (flange mount transform, `<mimic>` structure).
2. `moveit_controllers.yaml` gripper entry and the controller spawner ordering.
3. The `gz_ros2_control` block for the gripper.
4. Their Jazzy dependency delta (the three extra apt packages).

Build the **MoveIt config fresh via Setup Assistant** against UR5e so the two planning
groups (`arm`, `gripper`) are clean and MTC never enters the tree.

## 3. What the donor pre-answers (all still require M0 verification)

These are the donor's **claims**. They narrow the search space. They are not evidence.

**Gripper kinematics — bears on "Gripper targets" in the spec**

- Actuated joint: `finger_joint`. **5 mimic joints** hang off it.
- Reported range: `0.0` = **open**, `0.8` = **closed** (radians).
  This resolves the spec's "confirm direction, don't assume" — open is the *lower*
  bound. Still read it from `RobotModel::getJointModel("finger_joint")->getVariableBounds()`
  at runtime rather than hardcoding, and confirm the merged URDF agrees.

**Controller path — bears on M0-B**

- `gripper_controller` → `position_controllers/GripperActionController`
- `arm_controller` → `joint_trajectory_controller/JointTrajectoryController`
- `joint_state_broadcaster` → `joint_state_broadcaster/JointStateBroadcaster`
- Action interface: `/gripper_controller/gripper_cmd`, type
  `control_msgs/action/GripperCommand`, fields `{position, max_effort}`.
- **Position-controlled, not effort-controlled.** So the spec's position-space `squeeze`
  knob applies — expressed in joint-angle space, not width space, as the spec requires.
  `max_effort` in the action message is a separate ceiling, not the primary knob.
- Controllers spawn sequentially, ~10–15 s. Any M0 check must wait this out or it will
  produce a false negative.

**Mimic joints — bears on M0-C**

- Donor claims: *"Mimic joints are state-only — Gazebo Harmonic enforces the `<mimic>`
  constraints at the physics level."*
- **This claim is exactly what M0-C exists to test.** Do not accept it. If Harmonic
  enforces mimics at the physics level, the linkage tracks; if it only enforces them in
  the kinematic model, the fingers will look correct in TF and be wrong in contact.

## 4. Independent corroboration for the spec's `/joint_states` warning

The spec instructs that M0-C be cross-checked against Gazebo's own state output, *not*
`/joint_states`. The donor repo's own README contains an **unresolved** bug report that
supports this:

> `/joint_states` effort for these effort-interface joints doesn't reliably track the
> torque actually commanded through `forward_command_controller_effort` (seen off by up
> to ~15x with no consistent scale factor across joints) — likely a deeper
> gz_ros2_control state-readback issue.

Two consequences:

- Treat **effort** values on `/joint_states` in this stack as untrustworthy until proven
  otherwise. This is a live, open defect in the exact ros2_control + Harmonic
  combination being adopted.
- The donor's own `ur_force_control/ft_monitor` does contact detection by thresholding
  `finger_joint` effort **read from `/joint_states`**. Given the above, that node is
  built on a suspect signal. If the friction-grasp fallback (contact-triggered attach)
  is ever authorised, source contact from **Gazebo contact sensors**, not joint effort.

The donor also documents two sign-error fixes (2026-07-24) in gravity feedforward —
evidence the repo is actively debugged, but also that its physics-layer code has been
wrong in ways that looked plausible. Read, don't trust.

## 5. Environment facts confirmed

- **Gazebo Harmonic EOL: September 2028.** The spec's hedge resolves; safe to cite.
- Harmonic is `gz-sim` 8.x. Fortress (`ign gazebo` / gz-sim 6) is incompatible with
  Harmonic-specific world files and bridge packages — a stray Fortress dependency will
  not merely warn, it will fail. This is what M0-A checks.
- ROS 2 Jazzy requires Ubuntu 24.04 Noble.

## 6. Open questions, to be answered by recon on the target machine

Deliberately not guessed at. `scripts/00_recon.sh` collects the ground truth:

1. Exact xacro macro signature of `ur_macro.xacro` in the installed `ur_description`
   (Jazzy) — parameter names have changed across releases.
2. Which Robotiq description package is actually available on Jazzy
   (`robotiq_description` from PickNik vs. the donor's vendored copy).
3. The real flange frame name on UR5e (`tool0` vs `flange` vs `wrist_3_link`) and the
   mount transform to the gripper base.
4. Actual `finger_joint` limits and the full mimic multiplier/offset set from the parsed
   URDF — not from the donor's README table.
5. The `tool0` → finger-pad-contact-midpoint offset, needed for `grasp.tcp_offset` in
   `scene.yaml`.

Nothing in the URDF merge gets written until items 1–5 come back with real values.

---

## Sources

- [darshmenon/UR3_ROS2_PICK_AND_PLACE](https://github.com/darshmenon/UR3_ROS2_PICK_AND_PLACE)
- [MOGI-ROS/Week-11-12-Robot-arms](https://github.com/MOGI-ROS/Week-11-12-Robot-arms)
- [JuoTungChen/ROS2_pick_and_place_UR5](https://github.com/JuoTungChen/ROS2_pick_and_place_UR5)
- [PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
- [Gazebo Releases — Harmonic documentation](https://gazebosim.org/docs/harmonic/releases/)
