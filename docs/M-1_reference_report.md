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

**Simulation hardware path — resolved at M-1, not deferred to M0-B.** Not a donor
claim — verified directly against the installed `ros-jazzy-robotiq-description`
0.0.1-3 package, ahead of the merge:

`robotiq_description` exposes three mutually exclusive modes on the
`robotiq_gripper` macro: bare USB driver, `use_fake_hardware` (mock), and
`sim_ignition`. The merge uses `sim_ignition:="true"`.

- Under `sim_ignition`, the macro emits `<plugin>ign_ros2_control/IgnitionSystem</plugin>`.
  Verified this resolves: `/opt/ros/jazzy/share/gz_ros2_control/gz_hardware_plugins.xml`
  registers it as `type="ign_ros2_control::IgnitionSystem"`, a backward-compat
  alias for the same interface that drove the UR arm in the arm-only sanity run.
  **We are depending on a compat alias for a Fortress-era plugin name.** The macro
  hardcodes it, so it cannot be swapped without patching the package.
- No hand-written `gz_ros2_control` block is needed. Geometry and `ros2_control`
  both come from the single macro call (`robotiq_gripper`, params: `name`, `prefix`,
  `parent`, `*origin`, `sim_ignition:=false`, `sim_isaac:=false`,
  `use_fake_hardware:=false`, `fake_sensor_commands:=false`,
  `include_ros2_control:=true`, `com_port:=/dev/ttyUSB0`).
- `robotiq_2f_85_gripper.urdf.xacro` (the package's own demo) passes
  `use_fake_hardware`, NOT `sim_ignition`. Do not copy it verbatim — mock
  hardware echoes commands back as state with no physics, and would grasp nothing
  while looking healthy on `/joint_states`.
- Under `sim_ignition` the 5 follower joints receive only `<param name="mimic">`
  and `<param name="multiplier">` — no command or state interfaces. Mimic
  enforcement is delegated entirely to gz-sim's constraint solver, by the package
  author's explicit design. **Nothing available at M-1 proves that solver works.**
  M0-C remains the only evidence.
- Consequence for the SRDF and controller config: the gripper planning group and
  the gripper controller both claim exactly one joint,
  `robotiq_85_left_knuckle_joint`. The five followers appear in neither.
- **Open risk carried into the merge, not yet resolved:** the merged robot will
  run two different hardware plugin classes under one `controller_manager` — arm
  on `gz_ros2_control/GazeboSimSystem`, gripper on the `ign_ros2_control` alias.
  Multiple hardware components under one `controller_manager` is normal, and both
  aliases resolve to the same `GazeboSimSystemInterface`, so this should be a
  non-event — but confirm it, don't assume it: after the merged spawn,
  `ros2 control list_hardware_interfaces` must show **both** components active,
  not just the arm's. There is also precedent for Jazzy's `ros2_control` rejecting
  `<mimic>` joints that carry explicit interface declarations in the
  `<ros2_control>` block; the package's `<xacro:unless>` stripping those
  interfaces under `sim_ignition` is the right shape to avoid that, but if the
  merged model errors at spawn referencing mimic joints, that is this known
  failure mode, not a new mystery.

## 3.5 Mimic constraints are not physics-enforced, but contact still opposes them

**Status:** resolved enough to proceed. Not resolved enough to call friction
grasping validated.

*The earlier version of this section was written before a valid contact-load
run existed. Its factual claims survive unchanged; its practical implication
did not, and that is the part being replaced. This is a full replacement, not
a patch — a reader finding both versions would reasonably not know which
conclusion is live.*

**PROVEN — direct evidence**

`dartsim` does not create mimic constraints. Harmonic's default engine says so
at spawn, once per follower:

```
[Err] [Physics.cc:1808] Attempting to create a mimic constraint for joint
[robotiq_85_left_inner_knuckle_joint] but the chosen physics engine does not
support mimic constraints, so no constraint will be created.
```

Corroborated by `gz-physics#432`, the open tracking issue for dartsim mimic
support. Known engine limitation, not a model misconfiguration.

Free-space tracking works. Measured against Gazebo ground truth
(`/world/empty/model/ur5e_robotiq/joint_state`), not `/joint_states`.
Commanding the master `0.7668 -> 0.1000` rad moved all five followers to
within float noise of `multiplier × master`.

**Contact DOES oppose the linkage.** This is the finding that reverses the
earlier draft's conclusion. Probe: 40 mm rigid box at the fingertip midpoint,
master commanded to 0.8 rad (full close), well past what a 40 mm object
permits.

| quantity | value |
|---|---|
| commanded master angle | 0.800 rad |
| achieved master angle | 0.4586 rad |
| shortfall | +0.342 rad |
| controller `stalled` | `true` |
| controller `reached_goal` | `false` |
| box pose before | (0.492, 0.133, 0.367) |
| box pose after | (0.488, 0.135, 0.354) |

The linkage stopped 0.342 rad short of command and the controller reported
the stall itself. The box was retained: a few mm laterally, ~13 mm down. Not
ejected — the floor sits at z ≈ 0.03 and the box stayed at 0.354. Not driven
through.

Whatever the mimic mechanism is, contact reaction limits it. That was the
open question, and it is answered for this geometry.

**NEW OBSERVATION — one follower diverges, and only under load**

Four followers tracked `multiplier × master` to five decimal places at the
stalled pose. `robotiq_85_right_knuckle_joint` sat at `-0.431` where `-0.4586`
was expected — a 0.027 rad divergence.

The correlation worth noticing: in the earlier free-space run, all five
tracked exactly, including this joint. The divergence appears only under
contact load.

Two hypotheses, neither ruled out:

1. **Contact reaction bleeding through.** The override is soft enough that
   physics can perturb it, and this joint is where the right finger's
   reaction force shows up. If so, the coupling is partly physical rather
   than purely kinematic — which would be good news, and would partly
   explain why the stall happened at all.
2. **Solver or type artifact.** Recon established `right_knuckle` is the only
   follower declared `revolute`; the other four are `continuous`. Different
   joint types may simply be handled differently under load.

Cheap discriminating test, if it matters later: re-run the probe with no box
and command full close. If the divergence persists in free space it is (2).
The existing free-space data suggests it does not, but that run stopped at
0.1 rad rather than driving to the 0.8 limit, so it is not a clean comparison.

Not worth chasing now. Worth recording, because if grip stability later
proves asymmetric between the two fingers, this is the first place to look.

**Consequence 1 — the physics-engine decision is deferred, not taken**

The earlier draft framed `bullet-featherstone` (`gz-physics#517`, real mimic
constraints via `btMultiBodyGearConstraint`) as a likely necessity. It is not
indicated. dartsim's behaviour under contact is adequate for the thing we
needed it to do, and switching would trade a working `ros2_control`
integration for one with open defects (`gz_ros2_control#440`, `gz-sim#2729`).

**Stay on dartsim.** Revisit only if M3 slip testing fails in a way traceable
to the linkage rather than to friction parameters.

**Consequence 2 — M0-C can now be calibrated**

Replace the free-space sweep as M0-C's pass criterion. Keep it as a
subordinate check (it still catches a broken merge) but it must not be what
M0-C passes on — it returns green while the linkage is unforced.

Calibrated from the run above:

- **shortfall > 0.10 rad** on a 40 mm box commanded to 0.8. Observed 0.342, so
  this is a wide margin against "drove through" (~0) without being tuned to
  the exact number.
- **controller reports `stalled: true`**. It did so unprompted, and it is a
  free corroborating signal from a different subsystem.
- **box retained**: displacement < 30 mm, final z well clear of the floor.
  Observed ~13 mm.
- **followers within 0.05 rad** of `multiplier × master`, which admits the
  `right_knuckle` divergence rather than failing on it.

**What is NOT established**

Enumerated so this section cannot be cited as more than it is:

- **n = 1.** One box size, one approach geometry, one run.
- No lift, no transport, no slip measurement. Contact opposing closure is not
  the same as a grasp surviving acceleration and gravity. M3 remains
  entirely open.
- The ~13 mm downward box movement during closure is unexplained and worth
  watching: M3's pass criterion is 5 mm of slip. Settling as the fingers make
  contact is not the same thing as slip under transport load, but if that
  13 mm is the object sliding between the pads rather than being seated by
  them, M3 has a problem that predates any friction tuning.
- No visual confirmation of non-interpenetration. Only numeric pose. Worth
  one GUI look before M3.
- The override mechanism is still inferred, not read from source. It matters
  less now that its behaviour under contact is measured directly.

**ADDENDUM — grasp success is closing-rate dependent**

Found by accident, and it is the more actionable half of §3.5.

An `m0_verify.sh` run that omitted the pre-close step commanded the gripper
from full-open straight to 0.8 rad. It ejected the box. Restoring the
pre-close reproduced the retained result. The two runs differ in nothing but
the starting aperture.

| metric | no pre-close | pre-closed (this run) | pre-closed (probe, earlier) |
|---|---|---|---|
| travel | 0.0 → 0.79 rad | 0.44 → 0.80 rad | 0.44 → 0.80 rad |
| avg rate over window | ~0.263 rad/s | ~0.120 rad/s | ~0.120 rad/s |
| shortfall | ~0.00 | 0.3365 rad | 0.342 rad |
| controller `stalled` | `false` | `true` | `true` |
| box displacement | 363 mm, dropped to floor | 12.8 mm | 13.7 mm |
| right_knuckle divergence | — | 0.0316 rad | 0.0274 rad |

The retained result reproduces across two independent code paths — the
standalone probe and `m0_verify.sh` + `m0c_eval.py` — with every number in
agreement to the second significant figure. This is a real, repeatable
effect, not one run's noise.

**2.19× the closing rate is the difference between a grasp and an ejection.**

*Mechanism: partly resolved, partly a false dichotomy*

Two candidate mechanisms were proposed for the rate dependence.

**Hypothesis A — momentum.** The fingers arrive with enough kinetic energy to
knock the box clear rather than arrest against it.

**Hypothesis B — penetration depth per cycle.** A faster commanded rate means
a larger position step per cycle, so on the cycle where a pad first overlaps
the box the overlap is deeper, and the contact solver emits a correspondingly
larger impulse.

Test run: physics timestep halved, no-pre-close case repeated.
`empty.sdf` copied with `max_step_size` `0.001` → `0.0005`. Verified in effect
live, not assumed: `/world/empty/stats` reported `step_size: 500000 nsec`.

| | default timestep (1 ms) | halved timestep (0.5 ms) |
|---|---|---|
| box displacement | 363 mm | 360.1 mm |
| shortfall | 0.0000 rad | 0.0000 rad |
| `reached_goal` | `true` | `true` |

**Ejection is insensitive to the physics timestep. Halving it changed
nothing.**

**What this establishes**

- The ejection is not an artifact of solver integration granularity.
- Therefore **M3's 5 mm slip criterion is not timestep-confounded.** The slip
  numbers M3 measures will be physical, not solver noise layered on top. No
  reason to touch `max_step_size` for that measurement.

Both are worth having, and both are load-bearing for M3.

**What this does NOT establish**

It does not confirm Hypothesis A, and the test as designed could not have.

`gz_ros2_control` writes follower positions once per controller cycle
(`update_rate: 500` in `controllers.yaml`, i.e. every 2 ms). Halving the
physics timestep does not change the position delta per write — it only
integrates more finely between writes that are themselves unchanged. The
overlap depth created on the first contacting write is identical in both
runs, so B's actual variable was never varied.

Further: under kinematic override the followers report velocity ~0, so there
is no momentum in the ordinary sense for A to appeal to. "Closing rate" in
this system means position delta per controller write — which is B's
variable. A and B are plausibly the same mechanism under two names, which
would explain why a test targeting neither returned null.

**Not being pursued further, deliberately**

The genuine discriminator would be varying `update_rate` at fixed commanded
travel. It is not being run, because the mitigation is identical under either
hypothesis: limit the commanded closing rate. Resolving the mechanism would
not change a single downstream decision.

Recorded so that a later reader does not mistake this for an unexamined gap.

*Consequence for M3, unchanged*

If M3 ejects objects, suspect closing rate, not friction and not timestep. A
rate-dependent ejection presents as intermittent grip failure with `mu`
unchanged — some cycles hold, some do not. Against M3's criterion of 20
consecutive cycles with at least 18 under 5 mm slip, that reads as marginal
friction and invites tuning `mu` indefinitely against a variable that is not
`mu`.

*Other consequences, regardless of mechanism*

- **The pre-close is part of the grasp procedure, not a test fixture.** The
  pick-place node must pre-shape to roughly the object's width before
  approaching, then close the remainder. `config/scene.yaml` now carries
  `gripper.preclose_heuristic` and `preclose_clearance` as placeholders until
  `gripper.width_map` exists.
- **The 12–14 mm settle is now measured twice** and should be understood
  before M3, since it already exceeds M3's 5 mm slip criterion on its own.
  Most likely pad seating rather than sliding, but that is unverified — one
  GUI look distinguishes them.

**Method note**

The earlier draft's conclusion — that friction grasping could not work — was
drawn from the engine log line plus free-space data. It was wrong. The log
line was accurate and the inference from it was not: "the engine declines to
create the constraint" does not entail "nothing opposes the linkage," because
the software override turned out to be perturbable by contact.

This is the second time this project a correct observation produced a wrong
downstream conclusion. The prior instance is in §3.
`robotiq_2f_85_gripper.urdf.xacro` passes `use_fake_hardware` — a correct
reading of the file — and the conclusion drawn from it, that the package
offered only mock and USB paths and the merge would need a hand-written
`gz_ros2_control` block, was false. `sim_ignition` was in the same file the
whole time, and reading the `<hardware>` block rather than inferring from the
demo's arguments is what found it.

Both times the correcting evidence came from reading or running the artifact
itself. Neither time did more careful reasoning about the available evidence
help, because in both cases the available evidence was not wrong — it was
incomplete, and no amount of reasoning reveals what has not been looked at.

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

## 6. Open questions — answered by recon on the target machine

Recon run: 2026-08-03, `ros-jazzy-ur-description` 3.5.1, `ros-jazzy-robotiq-description`
0.0.1-3, both from apt (see §2 for full package inventory).

**1. `ur_macro.xacro` parameter names** — confirmed via
`ros-jazzy-ur-description` 3.5.1. `ur_macro.xacro:55` macro `ur_robot`, and
`ur.urdf.xacro` exposes `name`, `ur_type`, `tf_prefix`, `joint_limit_params`,
`kinematics_params`, `physical_params`, `visual_params`, `transmission_hw_interface`,
`safety_limits`, `safety_pos_margin`, `safety_k_position` as `xacro:arg`s. Unchanged
from what the donor repo assumes.

**2. Robotiq description source** — **apt**, not source-fallback:
`ros-jazzy-robotiq-description` 0.0.1-3noble.20260615.175624. `ros2_robotiq_gripper`
(PickNik source fallback) was not needed on this machine.

**3. Flange frame name + mount transform** — confirmed chain in
`ur_macro.xacro:388-403`, all fixed joints, all zero-translation:

```
${tf_prefix}wrist_3_link
  --(wrist_3-flange, xyz=0 0 0, rpy=0 -π/2 -π/2)-->  ${tf_prefix}flange
  --(flange-tool0,   xyz=0 0 0, rpy=π/2 0 π/2)-->     ${tf_prefix}tool0
```

`flange` and `tool0` differ only in orientation (ROS-Industrial convention:
`flange` carries the raw mechanical mounting axes, `tool0` re-expresses it in the
"X+ left, Y+ up, Z+ front" all-zeros tool convention). `scene.yaml` mounts the
gripper at `tool0`, consistent with the ROS-Industrial convention and the
`ur_to_robotiq_adapter.urdf.xacro` design (its `connected_to` param is left open
by the package precisely so the integrator picks the attach frame).

**4. Actuated joint + mimic set — CORRECTION to §3 above, not a confirmation.**
The donor's claim of `finger_joint` as the actuated joint **does not match** the
joint actually present in `ros-jazzy-robotiq-description` 0.0.1-3. The real master
joint, confirmed in `robotiq_2f_85_macro.urdf.xacro:232-238`:

```
joint: ${prefix}robotiq_85_left_knuckle_joint   type=revolute
  axis:  0 -1 0
  origin (from robotiq_85_base_link): xyz=0.03060114 0.0 0.05490452  rpy=0 0 0
  limit: lower=0.0  upper=0.8  velocity=0.5  effort=50
```

5 mimic joints hang off it exactly as the donor predicted, but the multiplier/type
set is:

| mimic joint | type | multiplier | offset |
|---|---|---|---|
| `robotiq_85_right_knuckle_joint` | revolute | -1 | 0 (default) |
| `robotiq_85_left_inner_knuckle_joint` | continuous | 1 (default) | 0 |
| `robotiq_85_right_inner_knuckle_joint` | continuous | -1 | 0 |
| `robotiq_85_left_finger_tip_joint` | continuous | -1 | 0 |
| `robotiq_85_right_finger_tip_joint` | continuous | 1 (default) | 0 |

**Every downstream reference to `finger_joint` is wrong for this package and must
use `robotiq_85_left_knuckle_joint`** (with `tf_prefix`/`prefix` applied same as the
arm). `config/scene.yaml` has been corrected — see below. The 0.0=open/0.8=closed
range the donor reported does hold numerically (`limit lower=0.0 upper=0.8`), so
that part of §3 stands; only the joint *name* was wrong.

**5. `tool0` → finger-pad-contact-midpoint offset** — measured against Gazebo
ground truth now that the merged URDF exists, using the same
measure-against-physical-truth-not-belief discipline as M0-C. Result was more
interesting than a single number: the fingertip midpoint sits on tool0's local
Z axis to within ~2e-7 m at every aperture tested (lateral symmetry holds
exactly), but the *distance* along that axis is NOT constant as the gripper
opens and closes — it varies 0.1093 m (open) to 0.1230 m (near-closed), 13.6 mm
of non-linear variation, larger than M3's own 5 mm slip criterion. Treating
`tcp_offset` as one fixed scalar is therefore a real approximation, not a
measured constant like items 1–4. `scene.yaml` records the full measured curve
and uses the 0.4 rad sample (0.1204 m) as a placeholder tied to similarity with
the M0-C probe's 40 mm test box, not a computed value for this project's actual
45 mm object — to be re-derived once `gripper.width_map` exists and the real
contact aperture for this object's width is known.

All of 1–4 are exact, observed values. Item 5 is real and measured, but is an
approximation until `width_map` lets it be computed at the actual grasp
aperture rather than read off a table by eye.

---

## Sources

- [darshmenon/UR3_ROS2_PICK_AND_PLACE](https://github.com/darshmenon/UR3_ROS2_PICK_AND_PLACE)
- [MOGI-ROS/Week-11-12-Robot-arms](https://github.com/MOGI-ROS/Week-11-12-Robot-arms)
- [JuoTungChen/ROS2_pick_and_place_UR5](https://github.com/JuoTungChen/ROS2_pick_and_place_UR5)
- [PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)
- [Gazebo Releases — Harmonic documentation](https://gazebosim.org/docs/harmonic/releases/)
