# DART-compatible Robotiq gripper representation — architecture design

**Status:** DESIGN ONLY. Nothing implemented, no production file modified, no
simulation run, no parameter tuned, no commit. 2026-08-25.

**Inputs treated as established** (per the task brief and the preserved
evidence): F3 Scene-A dynamic retention FAIL; friction resolved at
`mu_eff = 1.0202`; controller-gain interpolation closed; DART native SDF mimic
unavailable; DART closed-loop SDF joints unavailable; bullet-featherstone native
mimic works but migration is BLOCKED (shared-ECM conflict); split descriptions
unsupported; forking `gz_ros2_control` out of scope. Bullet is not investigated
further here.

---

## 0. Two structural facts established while reading the model

Both are new, both are load-bearing for everything below, and both were verified
against the file rather than assumed.

### 0.1 The four-bar linkage in this URDF is already fiction

Parsed joint tree of `robotiq_2f_85_macro.urdf.xacro`:

| joint | type | child | mimic |
|---|---|---|---|
| `robotiq_85_base_joint` | fixed | `robotiq_85_base_link` | — |
| `robotiq_85_left_knuckle_joint` | revolute | `left_knuckle_link` | — (MASTER) |
| `robotiq_85_right_knuckle_joint` | revolute | `right_knuckle_link` | master x -1 |
| `robotiq_85_left_finger_joint` | fixed | `left_finger_link` | — |
| `robotiq_85_right_finger_joint` | fixed | `right_finger_link` | — |
| `robotiq_85_left_inner_knuckle_joint` | continuous | `left_inner_knuckle_link` | master x +1 |
| `robotiq_85_right_inner_knuckle_joint` | continuous | `right_inner_knuckle_link` | master x -1 |
| `robotiq_85_left_finger_tip_joint` | **fixed** (TENTH OVERRIDE) | `left_finger_tip_link` | — |
| `robotiq_85_right_finger_tip_joint` | **fixed** (TENTH OVERRIDE) | `right_finger_tip_link` | — |

**Both inner-knuckle links are kinematic LEAVES — children = `[]`.** They connect
to nothing downstream. The coupler bar that would close the four-bar loop does
not exist in the model and cannot exist (DART rejects loop closures — probe I-0,
conclusive). Pad pose is therefore a function of the knuckle joint alone, per
side, through two fixed offsets.

Consequence: the inner knuckles contribute **exactly zero** kinematic fidelity.
They exist only to (a) carry a protruding collision mesh that reaches the object
first and (b) be driven by a software mimic servo that can collapse. Deleting
them costs nothing and removes defect 1 and one third of defect 3 outright.

The current model is not "a detailed Robotiq four-bar". It is **1 actuated DOF +
1 software-mimicked pad DOF + 2 software-mimicked decorative colliders**. It is
already an abstraction — just an undeclared and incorrect one.

### 0.2 Grip force was never the limiting factor — the failure is purely geometric

Pad moment arm about the knuckle axis, from the macro's own origins:
`|(_AX, _AZ)| = 57.150 mm`.

| quantity | value |
|---|---|
| master effort limit 1.0 N·m -> pad normal force | 17.50 N |
| P12.5 measured peak 0.897 N·m -> pad normal force | 15.70 N |
| required per pad for `mu_eff = 1.0202`, m = 0.15 kg | **0.72 N** |
| **margin at P12.5's peak** | **21.8x** |

A grasp with 21.8x the friction-limited force requirement still failed F3. That
independently corroborates the root-cause analysis from a direction it did not
use, and it is the quantitative justification for a **geometry-first redesign
rather than any further controller work**.

### 0.3 The shove is the closure travel

| quantity | value |
|---|---|
| aperture at pre-close 0.070 rad | 78.744 mm |
| aperture at grip 0.538015 rad (30 mm object) | 30.000 mm |
| total closure travel | 48.744 mm |
| per-side travel | 24.372 mm |
| **measured object shove** | **21.360 mm** |
| ratio shove / per-side travel | **0.876** |

The object is shoved by ~88 % of a full one-sided closure sweep. Closure is
effectively *entirely* one-sided: one pad reaches the object and drives it
across to the other. This gives the causal chain its missing quantitative link:

```
inner-knuckle protrusion (defect 1)
  -> forces a wide pre-close (0.070 rad = 78.7 mm aperture for a 30 mm object)
  -> forces 48.7 mm of closure travel
  -> ~21 mm one-sided shove (defects 5, 6)
  -> wedge + 14.43 deg pitch
  -> F3 retention FAIL
```

**The shove magnitude is set by pre-close clearance, not by architecture.**
Removing the protruding collider unlocks a tight pre-close, which collapses the
shove *regardless of whether closure is symmetric*. This materially changes the
architecture trade-off in §2.

---

## 1. The governing constraint: symmetry is impossible in a 1-DOF DART tree

In a kinematic tree, a joint moves its entire subtree as one rigid body relative
to its parent. Two jaws moving *relative to each other* therefore require two
joints. One DOF gives exactly one moving body relative to the base. Making both
pads move symmetrically from a single command requires one of:

1. a kinematic constraint (mimic / gearing) — **unavailable under DART**;
2. a closed loop — **unavailable under DART (probe I-0, conclusive)**;
3. both pads in one subtree — then they cannot move relative to each other, so
   aperture cannot change.

**Therefore: under DART, model-level symmetric two-sided jaw motion with one
controllable DOF is impossible.** Not difficult — impossible. Every candidate
below is a different way of paying for that, and any design claiming both
properties is wrong.

The honest fork is:

* **1 DOF** -> one moving jaw, one fixed reference jaw. Asymmetry is a declared
  model property. Object is displaced by the full closure clearance.
* **2 DOF** -> two independently actuated jaws driven from one setpoint.
  Symmetry is a *controller* property, never a model guarantee, and is not
  maintained under contact.

---

## 2. Architecture comparison

Criteria abbreviations: DART = DART-compatible; SymTrue = symmetry is a true
model property; Par = pad parallelism across the aperture; Det = contact
determinism; Blast = code/config blast radius.

| | **A. 1-DOF prismatic parallel jaw** (one moving jaw, fixed reference jaw) | **B. 1 actuated jaw + transformed fixed counterpart** | **C. 2 actuated jaws, one symmetric command, no mimic** | **D. Cosmetic Robotiq shell + decoupled deterministic pads** | **E. 1-DOF prismatic + arm-compensated closure** |
|---|---|---|---|---|---|
| **DART compatibility** | PASS — prismatic joints are natively supported, no constraint of any kind | PASS | PASS — two ordinary actuated joints | PASS | PASS |
| **DOF count / type** | 1 prismatic (actuated) | 1 prismatic/revolute (actuated) | 2 prismatic (both actuated) | 1 prismatic + N fixed cosmetic | 1 prismatic + arm DOFs |
| **SymTrue** | **NO** — declared one-sided (impossible per §1) | NO | **NO** — commanded only; under contact one jaw stalls while the other continues | NO | Effectively yes *in world frame*, via arm motion — but not a gripper-model property |
| **Pad parallelism** | **EXACT at every aperture, by construction** (prismatic cannot rotate the pad) | Exact if prismatic | **EXACT at every aperture** | Exact (pads are the prismatic children) | Exact |
| **Contact determinism** | **HIGH** — flat box pads, single closing axis, only 2 colliders in the whole gripper | HIGH | HIGH per jaw, but final state depends on contact order | HIGH | HIGH, minus arm/gripper timing coupling |
| **Aperture mapping** | **exactly linear**, `aperture = A0 - q`, trivially invertible | linear | `aperture = A0 - q_l - q_r`, linear | linear | linear |
| **Collision geometry** | 2 box pads. Everything else collision-free | 2 boxes | 2 boxes | 2 boxes + cosmetic meshes carry NO collision | 2 boxes |
| **Contact sensors** | preserved, one per pad, unchanged topics | preserved | preserved | preserved | preserved |
| **ros2_control changes** | **minimal** — 1 joint, `effort_controllers/GripperActionController` unchanged, `GripperCommand` + `stalled` preserved | minimal | **LARGE** — no stock gripper controller accepts 2 joints (verified: both `gripper_controllers` and `parallel_gripper_controller` expose singular `joint`). Requires JTC (loses `stalled`) or a custom controller | minimal | minimal |
| **MoveIt impact** | group stays 1 joint; units rad->m; SRDF `disable_collisions` pruned | same | group becomes 2 joints; `moveit_controllers` retargeted | same as A | same as A + closure becomes a coordinated motion |
| **Visual impact** | jaw blocks, or vendor meshes rigidly attached as decoration | same | same | **best** — keeps the vendor look exactly | same as A |
| **Blast radius** | **moderate** — URDF, ros2_control block, 4 MoveIt configs, scene.yaml units, geometry util, m3_grasp thresholds | moderate | **large** — all of A *plus* the gripper controller, `gripper_close_and_hold`, stall semantics, every analyzer keyed to one joint | moderate (= A + cosmetic xacro) | large — adds arm/gripper synchronization during closure |
| **Scientific defensibility** | **HIGH** — declared abstraction, every property provable offline | HIGH | HIGH on paper; weakened by contact-dependent asymmetry that is *not* how a single-motor 2F-85 behaves | **HIGHEST for communication** — honest physics, familiar appearance | HIGH but the compensation is an extra claim to defend |
| **Regression burden** | full gripper-dependent re-validation (V0–V10) | same | same + controller re-validation | same as A | same as A + new coupled-motion validation |
| **Key risks** | residual one-sided shove bounded by pre-close clearance (~7 mm predicted, vs 21.4 mm today) | same, with no compensating benefit | **recreates a soft-coupling problem in a new place**: nothing forces the jaws to agree under load; loses hard-won `stalled` machinery | none beyond A | new timing-coupling failure mode during closure — precisely the class that has cost this project the most |

### 2.1 Verdict on candidate C specifically (as the brief asks)

C does **not** recreate the *existing* failure — there is no follower servo
tracking a master's measured position at 500/s, so defect 2 (servo collapse)
cannot occur. But it creates a *different* soft-coupling problem of the same
family: two jaws that agree only because they were told to. Under contact each
jaw stalls independently, so the final geometry is contact-order dependent —
which is **less** faithful to a real single-motor 2F-85 (where the jaws are
rigidly geared and cannot disagree), not more. Combined with the verified
absence of any stock multi-joint gripper controller, C buys a *commanded*
symmetry that the model never enforces, at the highest blast radius of any
candidate. **Rejected.**

### 2.2 Is a simplified parallel-jaw abstraction more defensible than the detailed model?

**Yes, decisively** — and the argument does not rest on taste:

1. The detailed model **is not simulating a four-bar** (§0.1). The coupler does
   not exist; the inner knuckles are leaves. It is already an abstraction.
2. Its parallel-jaw property holds at **exactly one angle** (`fingertip_grasp_theta`)
   under the TENTH OVERRIDE. `gripper_geometry.py` still carries
   `_assert_parallel_jaw()`, which asserts the *superseded* continuous-mimic
   model — a live inconsistency in the codebase.
3. It injects a protruding collider that **dominates first contact** (right inner
   knuckle touches 5.86 s before either fingertip).
4. It makes grasp geometry **object-dependent** through a derived xacro arg with
   a documented silent-failure mode ("changing `object.size` without re-deriving
   this angle produces a pad tilted at the WRONG angle at grasp, silently").
5. **None of its extra DOFs carry load.**

So the "detailed" model is detailed in *appearance* only. Replacing it with a
declared, exactly-specified abstraction strictly increases scientific honesty:
it removes fiction, it does not add any.

---

## 3. RECOMMENDATION — Architecture A (+ D's cosmetic layer)

> **A single-DOF prismatic parallel-jaw gripper: one actuated moving jaw, one
> fixed reference jaw, two flat box pads, and no other collision geometry.
> Vendor Robotiq meshes retained as visual-only decoration.**

Chosen because it eliminates all six model-level defects, is provably correct
offline, keeps every ros2_control/MoveIt/action interface the project has already
validated, and — given §0.3 — recovers most of the benefit of symmetry through
pre-close tightening, which symmetry was only ever a proxy for. Candidate C's
extra DOF is not purchased by any measurement currently in evidence.

### 3.1 Exact kinematic concept

```
tool0
 └─ (fixed) gripper_base_link                      [visual: robotiq_base.dae; NO collision]
     ├─ (fixed)     jaw_fixed_link                 [visual; NO collision]
     │   └─ (fixed) pad_fixed_link                 [BOX collision + contact sensor]
     └─ (PRISMATIC) jaw_moving_link  <-- the one actuated DOF
         └─ (fixed) pad_moving_link                [BOX collision + contact sensor]
```

Exactly **two** collision bodies exist in the entire gripper. Every other link is
visual-only. There are no continuous joints, no mimic tags, no followers, and no
object-dependent xacro arguments anywhere in the model.

### 3.2 Controllable DOF

One: `gripper_jaw_joint`, `type="prismatic"`, axis along the gripper closing
axis (the local axis that maps to the world closing axis under
`grasp.gripper_roll`).

* `q = 0` -> fully open; increasing `q` -> closing. **Sign convention preserved**
  from the current master joint, so every `squeeze` / `preclose_margin` /
  tolerance expression keeps its sign and changes only units.
* `limit lower="0.0" upper="0.085"`, `velocity="0.1"` (m/s — carried over
  unchanged in numeric value and rationale from the OVERRIDE 2026-08-10
  stall-overshoot analysis, which bounds *displacement* per stall interval and so
  transfers directly), `effort="20.0"`.
* **Effort is now a force in newtons, directly comparable to physics.** Required
  per pad is 0.72 N (`m·g / (2·mu_eff)`, m = 0.15 kg, `mu_eff` = 1.0202). 20 N is
  a ~28x ceiling, deliberately generous because §0.2 shows force is not the
  binding constraint. This replaces a torque whose mapping to pad force ran
  through a lever arm the model does not actually simulate.

### 3.3 How left/right pads move — stated honestly

They do **not** move symmetrically. `pad_fixed_link` never moves relative to
`gripper_base_link`; `pad_moving_link` translates by exactly `q`. This is a
**declared, measured model property**, not an accident, and per §1 it is the only
option available under DART at one DOF.

Mitigation, which §0.3 shows is the effective one: the object's closure
displacement is bounded above by the pre-close clearance,
`shove <= aperture_preclose - width`. With the protruding inner knuckle gone, the
pre-close clearance is limited only by pad-face clearance during descent — set by
perception error (worst 1.6136 mm, repeatability 0.000 mm) plus margin, giving an
aperture of ~37 mm for the 30 mm object and a **predicted worst-case shove of
~7 mm, against 21.36 mm measured today**. If V5 shows that residual is
unacceptable, candidate C is the pre-costed upgrade — see §3.15.

### 3.4 How pad orientation stays parallel

By construction and without any constraint: a prismatic joint contributes **pure
translation**, so `R(pad_moving) ≡ R(pad_fixed) ≡ R(gripper_base)` for all `q`.
Pad-face normals are exactly antiparallel at every aperture, to machine
precision, at every instant, under any contact load. This is strictly stronger
than the original continuous-mimic model (parallel only while a software servo
tracked) and eliminates defect 4 at the root.

Because object pitch was measured to equal pad tilt (`pitch = master -
fingertip_grasp_theta`, slope 1.0007, residual < 0.03 deg), removing pad tilt
removes the measured pitch-driving term entirely.

### 3.5 aperture(q)

```
aperture(q) = A0 - q          A0 = 0.085 m      q in [0, 0.085]
q_for_width(w) = A0 - w
```

Exactly linear, exactly invertible, no bisection, no mesh parsing at runtime,
no `theta_grasp` dependence, no `pad_z` dependence. `A0 = 85.000 mm` is retained
deliberately: it preserves the mesh-validated published stroke that
`gripper_geometry.py` currently reproduces to 1 micron, so the aperture scale
keeps its existing provenance.

**Two error classes vanish with this joint type:**

* `tcp_offset` becomes a **constant**. Today it varies 109.326 -> 122.812 mm across
  the stroke (13.5 mm), and `tcp_offset_delta_m()` exists solely to patch the
  resulting descent-depth error. A prismatic jaw moving perpendicular to the tool
  axis does not translate the pad along z at all.
* `aperture_m_fixed_tip()`'s documented "NOT INDEPENDENTLY CONFIRMED" branch is
  deleted rather than validated.

Lateral grasp-centre offset, exposed analytically instead of empirically:

```
grasp_centre_offset_m(q) = q / 2      (along the closing axis, toward the fixed jaw)
```

Exact, closed-form, and unit-testable offline. Compare with today's
`pad_centre_offset = 0.031573 m`, which is empirical and carries a documented,
still-unexplained 18.14 mm disagreement with the mesh-measured value. **The model
carries no object-specific parameter of any kind** — the single most valuable
robustness property of this design, since it removes the entire class of silent
failure that `fingertip_grasp_theta` introduced.

### 3.6 How 30 mm / 45 mm width logic maps in

Unchanged upstream, simpler downstream. `scene_xacro_args.resolve_closing_axis()`
and `resolve_grasp_width_m()` are **kept exactly as they are** — they already
correctly derive the closing axis from `grasp.gripper_roll` and the object pose
(verified live: roll 1.5708 -> `axis_index = 0` -> `width_m = 0.030`), and that
derivation is the fix from the M6 investigation. Only the consumer changes:

| today | proposed |
|---|---|
| `theta_for_width(0.030) = 0.538015 rad` (bisection) | `q_for_width(0.030) = 0.055 m` (subtraction) |
| `theta_for_width(0.045) = 0.402893 rad` | `q_for_width(0.045) = 0.040 m` |
| emitted as `fingertip_grasp_theta` xacro arg into the URDF | **not emitted at all — the model is object-independent** |
| `preclose_margin_rad = 0.4678679450464813` | `preclose_clearance_m`, e.g. 0.007 (aperture = width + 7 mm) |

Note a pre-existing inconsistency this retires: `scene.yaml` documents an expected
grip angle of 0.5378679450464813 rad, while `theta_for_width` now returns
0.538014762810753 — a 1.47e-4 rad drift between the stored constant and its own
derivation. Under the proposed scheme the grip command is recomputed from width
by subtraction, so no stored constant can drift from its derivation.

### 3.7 Collision geometry strategy

* **Exactly two colliders**: `pad_fixed_link` and `pad_moving_link`, each a
  `<box>` sized to the measured pad face — 38.004 mm tall (from
  `PAD_FACE_Z_MAX_M - PAD_FACE_Z_MIN_M`, mesh-measured), pad width from the same
  mesh, thickness ~4 mm. Boxes, not meshes: a flat pad gains nothing from mesh
  collision and loses contact determinism.
* Pad `<surface>` friction parameters **carried over byte-identical** from the
  current fingertip blocks, so the `mu_eff = 1.0202` calibration remains the
  applicable material result (see §7 for the cheap re-confirmation).
* **Every other link carries no `<collision>` at all** — base, knuckles, fingers,
  inner knuckles. This is what kills defect 1: there is no longer any body that
  *can* reach the object before a pad.

### 3.8 Contact sensor strategy

Two contact sensors, one per pad, at `update_rate 1000` (unchanged), publishing on
topics that preserve the existing suffix convention so
`scripts/perception/milestone_f1_harness.py`'s
`sensor/left_finger_tip_contact/contact` and `.../right_finger_tip_contact/contact`
subscriptions keep working with a name substitution only. The six `diag/*`
sensors on knuckles/fingers/inner-knuckles are **deleted** — their links no longer
have collision geometry, so they could only ever report nothing.

Bilateral-engagement checks (an F2 criterion) survive unchanged in meaning: one
sensor per pad, both must report contact.

### 3.9 ros2_control implications

Deliberately minimal, and the main reason A beats C:

* `<ros2_control>` block: **one** joint, `command_interface position` +
  `command_interface effort`, `state_interface position/velocity/effort` — the
  same shape as today's master.
* **All five follower joint blocks and every `<param name="mimic">` are deleted.**
* Controller **type unchanged**: `effort_controllers/GripperActionController`,
  with its `gains` block (p 50.0, d 2.0) carried over as a starting point and
  re-baselined in V1.
* `control_msgs::action::GripperCommand` and the `stalled` / `reached_goal`
  semantics are **preserved**, so `gripper_close_and_hold()`,
  `stall_velocity_threshold = 0.05`, `stall_timeout = 0.2` and the M3 stall work
  all survive. (`stall_velocity_threshold` is now m/s and must be re-derived
  against the new velocity noise floor in V1 — it is a measured quantity, not a
  ported constant.)
* Verified constraint driving this choice: **no stock multi-joint gripper
  controller exists** — `gripper_controllers` and `parallel_gripper_controller`
  both expose a singular `joint` parameter.

### 3.10 MoveIt implications

Small and mechanical:

* `<group name="gripper">` still contains exactly **one** joint — only the name
  changes. The `gripper_eef` end-effector definition is untouched.
* `group_state` "open" value changes units (rad -> m).
* `joint_limits.yaml` gains the prismatic limits.
* `disable_collisions` entries referencing deleted links are pruned; because only
  two links now carry collision, the ACM shrinks substantially.
* No planning-group, kinematics, or Pilz-limits change. The arm is untouched.

### 3.11 P12.5

**Retire it — but not yet, and not silently.**

P12.5 is characterized against the *current* linkage's contact dynamics. Those
dynamics are exactly what this redesign replaces, so its characterization cannot
transfer, and §0.2 shows it was operating at ~21.8x the friction-limited force
requirement — it was never solving a force problem. Do **not** port it.

Sequence: freeze P12.5 as historical evidence, re-baseline the new model with the
plain effort `GripperActionController`, and add force shaping only if V5/V6
measurement demands it. Formally retire it once V10 passes without it. It remains
what it always was: characterized, never validated.

### 3.12 Production files that would need modification

| # | file | change |
|---|---|---|
| 1 | `ur5e_robotiq_description/urdf/parallel_jaw_gripper.urdf.xacro` | **NEW** — the macro above |
| 2 | `ur5e_robotiq_description/urdf/ur5e_robotiq.urdf.xacro` | include new macro behind `gripper_model:=robotiq_linkage\|parallel_jaw`, default `robotiq_linkage` until V7 |
| 3 | `ur5e_robotiq_description/urdf/vendor/robotiq_gripper.ros2_control.xacro` | one-joint block; delete 5 follower blocks + all mimic params |
| 4 | `ur5e_robotiq_description/config/controllers.yaml` | `gripper_controller.joint` name; re-baseline gains |
| 5 | `ur5e_robotiq_moveit_config/config/ur5e_robotiq.srdf` | group joint name; `group_state` units; prune `disable_collisions` |
| 6 | `ur5e_robotiq_moveit_config/config/joint_limits.yaml` | prismatic limits |
| 7 | `ur5e_robotiq_moveit_config/config/moveit_controllers.yaml` | joint name |
| 8 | `ur5e_robotiq_moveit_config/config/ros2_controllers.yaml` | joint name |
| 9 | `config/scene.yaml` | `gripper.actuated_joint`; `tcp_offset` -> constant; `pad_centre_offset` -> analytic; `squeeze`, `grasp_tolerance_*`, `grasp_loss_threshold_*`, `preclose_*` -> metres |
| 10 | `config/scene_xacro_args.py` | `xacro_gripper_args()` stops emitting `fingertip_grasp_theta`; **`resolve_closing_axis` / `resolve_grasp_width_m` untouched** |
| 11 | `scripts/lib/gripper_geometry.py` | add linear `aperture_m(q)`, `q_for_width`, `grasp_centre_offset_m`; **retain all existing functions** so historical evidence stays readable; retire the stale `_assert_parallel_jaw()` |
| 12 | `ur5e_pick_place/src/m3_grasp.cpp` | threshold unit renames; drop `fingertip_grasp_theta` plumbing; default `actuated_joint` |
| 13 | `ur5e_pick_place/launch/m3_grasp.launch.py`, `ur5e_robotiq_description/launch/ur5e_robotiq_sim_control.launch.py` | xacro-arg plumbing |

**Joint renaming is recommended, not optional.** `robotiq_85_left_knuckle_joint`
would otherwise keep its name while changing type (revolute -> prismatic) and unit
(rad -> m). A silent unit change under an unchanged name is exactly the trap that
produced both the `grasp_width_axis` and `fingertip_grasp_theta` incidents.
Rename to `gripper_jaw_joint`; the rename is mechanical and caught at build time,
whereas the unit collision would be caught only by a wrong measurement.

### 3.13 Files that must remain untouched

* All perception: `scripts/perception/*`, detector thresholds and sync (FROZEN),
  camera geometry in the URDF (FROZEN).
* All physics/world configuration: friction, solver, step size, engine selection.
* P12.5 controller sources — frozen historical.
* `evidence/**` — append-only, never edited.
* `docs/HANDOFF_RGBD_PERCEPTION.md` §8–9 measurements.
* `ur5e_pick_place/src/transport.cpp` placement logic.
* `scripts/lib/slip.py`, `scripts/lib/gz_settle.py` analysis semantics —
  parameterize the joint name; do not rewrite the analysis.
* MoveIt `kinematics.yaml`, `pilz_cartesian_limits.yaml`, `sensors_3d.yaml`.
* The vendored `robotiq_2f_85_macro.urdf.xacro` — **kept in place, unmodified**,
  as the rollback target and historical reference.

### 3.14 Migration sequence

1. Add the new macro behind `gripper_model`, default **off**. Build. Run **V0**
   (pure offline).
2. Flip the arg in a scratch launch only. **V1**, **V2**.
3. **V3**, **V4**, **V5** — descent and closure, no lift.
4. Implement the redesigned G0 gate (§5). **V6**.
5. **V7**, **V8** — F2 Scene A, then A–D.
6. **V9** — classical regression.
7. **V10** — one F3 Scene-A lift.
8. Only then flip the production default, update docs, formally retire P12.5.

### 3.15 Rollback strategy

* **One-flag rollback**: `gripper_model:=robotiq_linkage` restores the current
  model exactly. The vendored macro is never deleted or edited.
* Config changes additive where possible (new keys beside old, selected by the
  same flag), so `scene.yaml` remains valid for both models.
* Work on a branch; no commit to `main` until V7 passes.
* All evidence append-only, so a rollback never destroys a measurement.
* **Pre-costed upgrade path**, if V5's residual shove proves unacceptable:
  promote the fixed jaw to a second actuated prismatic joint (candidate C),
  replace `GripperActionController` with a 2-joint JTC or a custom controller,
  and re-run from V1. Trigger declared in advance: **V5 closure translation
  > 8 mm, or V6 fails on object displacement.**

---

## 4. Validation ladder and pass criteria

Predeclared before any run. **Scene A is not permitted until V0–V6 have passed.**
Criteria use existing project provenance where it exists; where a threshold is
genuinely new it is marked NEW and its basis given. No criterion may be relaxed
after seeing a result.

| gate | what runs | pass criteria | provenance |
|---|---|---|---|
| **V0** | offline geometry only, no sim | `aperture(q) = A0 - q` to 1e-9 m at >=5 apertures, checked against pad-link poses derived from the generated URDF; pad-normal antiparallelism exact (symbolic + numeric, tol 1e-12 rad); `grasp_centre_offset(q) = q/2` to 1e-12; `aperture(0) = 85.000 mm +- 0.001`; `q_for_width` inverts to 1e-9 for 30 and 45 mm; **grep proves zero object-dependent xacro args remain** | 85 mm stroke already validated to 1 micron in `gripper_geometry.py` |
| **V1** | free-space open->close->open, no object | `\|q - q_cmd\| <= 0.5 mm` at rest; 3 repeats, spread <= 0.2 mm; zero contacts; **joint census: exactly 1 actuated gripper joint, 0 mimic tags, 0 continuous joints**; re-derive `stall_velocity_threshold` against the measured velocity noise floor | mirrors the M3 stall-threshold derivation method (`docs/gripper_stall_velocity_noise_20260806.log`) |
| **V2** | pad pose sweep, Gazebo ground truth | pad-face normals antiparallel within 1e-6 rad at **every** sample across the full stroke; moving-pad travel = `q` within 0.1 mm; fixed-pad travel = 0 within 0.05 mm | direct falsifier of defect 4. NEW — verifies parallelism and *declared* one-sidedness, explicitly **not** symmetry |
| **V3** | Scene-A descent, no closure | min pad->object distance >= 2.0 mm throughout; **zero** contact events on any gripper link before `CLOSE_BEGIN`; assert no non-pad link owns collision geometry | F2's existing "no premature descent contact"; 2.0 mm sits inside the measured F2 real-STL clearance band 1.198–7.680 mm |
| **V4** | first contact identity | first contact is pad<->object, on either pad. **Any other first contact = FAIL** | direct falsifier of defect 1 (today: right inner knuckle leads by 5.86 s) |
| **V5** | closure seating | object pitch change <= **1.0 deg**; object translation <= `(aperture_preclose - width) + 1.0 mm`, predeclared <= **8.0 mm**; both pads in contact at end of closure | vs 14.43 deg and 21.36 mm measured today. Translation bound is the §0.3 geometric prediction; 1.0 deg is NEW, allowing contact transients on a model with no tilt-driving term |
| **V6** | new G0 gate (§5) | proof-lift 10 mm; **zero** table<->object contacts across the full 1.0 s dwell; object z drift <= 0.5 mm; both pads in continuous contact | NEW — replaces "stationary" with "held". See §5 |
| **V7** | F2 Scene A | all existing F2 criteria unchanged: Cartesian fraction 1.0, P1->P2 = 0.0000 mm, no premature contact, bilateral engagement, grasp-only stop, `position_source=perceived` | frozen F2 criteria |
| **V8** | F2 Scenes A–D | as V7, all four scenes | frozen F2 matrix |
| **V9** | classical regression | `position_source=configured`, transport SUCCESS, placement error <= **1.0 mm** | M4 measured 0.162 mm; loosened once to admit a model change, stated explicitly |
| **V10** | one F3 Scene-A lift | G0(new) -> L1 -> L2; slip <= **5.000 mm**; `retained=yes`; no drop evidence | frozen F3 threshold, unchanged |

---

## 5. Redesigned F3 G0 prerequisite (concept only — not implemented)

**The defect.** G0 currently begins in the final 0.5 s before `LIFT_BEGIN` and
tests *quiescence*. An object resting on the table passes trivially — it is
stationary because the **table** holds it, not the grasp. Measured: the grasp
carried only **17.9 %** of the object's weight at G0. G0 therefore certifies a
*stationary* object, not a *held* one, and the F3 slip metric was measuring
retention of something that was never lifted.

**The redesign.** A valid G0 must certify two independent properties before the
baseline is taken:

* **(a) Table-clear.** A short *proof lift* (~10 mm, well below the F3 lift) is
  executed **before** the baseline. G0 is then sampled in the held state.
  Criterion: **zero** table<->object contact events across the entire dwell.
  Absence of an uncaptured stream is "not available", never "no contact" —
  consistent with the existing measurement plan's contact discipline.
* **(b) Load-bearing.** The grasp must carry essentially all the weight:
  * both pads report continuous contact for the whole dwell;
  * object CoM vertical drift <= 0.5 mm across the dwell (it is not sinking);
  * where force data exists, summed pad normal force >= `m·g / mu_eff` = **0.72 N**
    for m = 0.15 kg — a criterion that is only expressible because §3.2 makes the
    actuated effort a force in newtons.

Only when (a) and (b) hold **continuously** for the dwell is G0 valid; F3 then
starts from that pose. Failure to establish either is **INDETERMINATE**, not
FAIL — the same discipline the current plan applies to inadequate coverage.

This converts G0 from "the object is not moving" to "the object is being held",
which is the property F3 was always supposed to presuppose.

---

## 6. Results invalidated by this change (must be regenerated)

Everything whose value depends on gripper geometry or contact:

* the M3/M5 20/20 slip sweep;
* F2's four-scene matrix and the 2026-08-23 durable regeneration;
* **all seven F3 experiments**, including the P12.5 Scene-A lift;
* the classical regression;
* P12.5's characterization (and P12.5 itself — §3.11);
* the descent pre-close value `0.070 rad` and `preclose_margin_rad`;
* `tcp_offset = 0.120405` and `pad_centre_offset = 0.031573`;
* `fingertip_grasp_theta` (ceases to exist);
* the quantitative relations **53.3 mm/rad** clearance-vs-mimic-error and
  **pitch = master - fingertip_grasp_theta** — these describe the *old* model
  and are retained as findings *about it*, not as properties of the new one.

## 7. Findings that remain conceptually valid

* Perception A–E in full: camera geometry, metric depth, detector, camera-frame
  3D, TF world position, worst error 1.6136 mm, repeatability 0.000000000 mm.
  **Entirely independent of the gripper** — do not re-run.
* F1/F2 perception-derived *position* accuracy, for the same reason.
* Friction: `mu_eff = 1.0202`, and the conclusion **friction is not the cause**.
  The pad `<surface>` parameters carry over byte-identical, so the material
  result stands. Contact *patch* changes (mesh -> box), so a cheap
  re-confirmation is recommended, not a full recalibration.
* All mechanism findings, which are the *reasons* for this redesign and remain
  true statements about the model that produced them: the object is
  table-supported at G0 and the grasp carries 17.9 % of its weight; G0 certifies
  a stationary rather than a held object; closure is one-sided and wedges the
  object; mimic error governs descent clearance; pad tilt equals master
  overshoot; DART enforces no mimic constraint; DART rejects closed loops;
  bullet-featherstone migration is blocked.
* Every harness-fault finding (five in the compat probe, plus the ignored
  `<topic>` elements on `ApplyJointForce` and the Contact system).

## 8. Should the detailed Robotiq model remain visual-only?

**Yes.** Keep the vendor `.dae` meshes on collision-free links so the robot still
*looks* like a 2F-85 in RViz and Gazebo, and state plainly in the macro header
that the visual shell is decorative and that the simulated mechanism is a
declared parallel-jaw abstraction. This costs nothing scientifically — the
meshes already carry no load — and preserves communication value.

The one honest caveat to document: the cosmetic knuckle/finger meshes will
*translate* with the moving jaw rather than *rotate* as the real linkage does, so
the animation is approximate. That is a labelled cosmetic approximation, which is
categorically different from the current situation, where the *physics* is
approximate and the label is missing.

## 9. Is a production change now justified?

**Yes — and the evidence for it is unusually strong**, but it is a decision for
the maintainer, not for this agent:

1. The confirmed root cause of the F3 failure is **geometric**, and §0.2 shows
   the grasp had **21.8x** the force it needed — so no controller work can fix it.
2. The only two ways to obtain a *real* kinematic constraint under this stack
   (DART native mimic, DART closed loops) are conclusively unavailable, and the
   bullet route is BLOCKED with its sole escape traced to infeasible.
3. The current model's linkage is **already fiction** (§0.1) — the change removes
   fiction rather than adding abstraction.
4. Every defect 1–6 is eliminated by construction, provably, offline, before any
   simulation is run.

The cost is real and must be stated: a full re-validation of the gripper-dependent
evidence base (§6). That cost is unavoidable under *any* fix, including doing
nothing and continuing to measure a wedging gripper.

## 10. Smallest next implementation task, if authorized

**Write `ur5e_robotiq_description/urdf/parallel_jaw_gripper.urdf.xacro` and wire
it behind `gripper_model:=parallel_jaw`, default OFF, then run V0 only.**

Deliberately bounded: it produces zero behavioural change with the default flag
(so the current model and all its evidence stay reproducible), it is verifiable
entirely offline, and it either passes V0's exact geometric identities or it does
not. Nothing else — no controller change, no `scene.yaml` change, no simulation —
until V0 passes.

---

*Design only. No production file modified, no simulation run, no parameter tuned,
no commit. HEAD `7b875a4`.*
