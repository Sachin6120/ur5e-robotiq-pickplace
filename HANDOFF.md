# HANDOFF.md

> READ THIS SECTION FIRST. The section immediately below, "2026-08-31
> Stage-2D Planar Pose Generalization — CURRENT AUTHORITY", is the sole
> current-authority statement of repository state.
> Every other authority label anywhere else in this file is superseded and
> has been relabelled
> accordingly; their content is retained as historical evidence, not current
> state — do not act on any instruction inside a superseded section without
> checking it against the section below first.

## 2026-08-31 Stage-2D Planar Pose Generalization — CURRENT AUTHORITY

### Milestone status

- Repository: `~/ur5e_pickplace`, branch `stage2-orientation-generalization`,
  HEAD `0562222` (`geometry: raise parallel-jaw fixed-side grasp TCP
  clearance to 2.0 mm`). No other production code or configuration changed
  after that commit.
- **`GRASP_TCP_FIXED_SIDE_CLEARANCE_M` (`scripts/lib/parallel_jaw_geometry.py`)
  is 2.0 mm, raised from 1.5 mm, commit `0562222`.** This is the current
  production value. `DECLARED_CLEARANCE_M` (4.0 mm), the aperture split
  (`FIXED_SIDE_CLEARANCE_M`/`MOVING_SIDE_CLEARANCE_M`), the pre-close
  aperture, the final close target, `q_for_width`, controllers, perception,
  and URDF/xacro are all unchanged by this commit.
- Stage-2D extends Stage-2C: the physical spawn XY and yaw are both
  independently decoupled from the configured pick pose (combined planar
  offset + yaw), while manipulation still consumes perceived XYZ and
  perceived yaw together. Three cases: D1 (spawn offset +30/+30 mm, spawn
  yaw +30 deg), D2 (-30/-30 mm, -30 deg), D3 (+30/-30 mm, +45 deg).

### D3 failure at the former 1.5 mm clearance, and its root cause

- D3 failed deterministically at 1.5 mm: `EXECUTE_FAILURE`, MoveIt error
  -4, after a Cartesian path with `cartesian_fraction = 1.0000` and no
  reported collision. Evidence: `evidence/stage2d_pose/D3`,
  `D3_retry1_diagnostics` (identical IK solution, identical halt point,
  reproduced on retry).
- **The controller-gain hypothesis was investigated and falsified.** A
  read-only source trace and disassembly of `gz_ros2_control`'s runtime
  diagnostic `position_proportional_gain = 0.1` found: (1) it is a
  compiled plugin default (`libgz_hardware_plugins.so`), never set by any
  commit in this repository (`git log -S"position_proportional_gain"`
  touches only docs/scripts, never a value-setting site); (2) the control
  law `target_vel = gain * update_rate * (cmd - pos)` has **no saturation**
  in `GazeboSimSystem::write()` (0 `minsd`/`maxsd` instructions in that
  function); (3) at the observed shoulder_lift tracking error (0.0707 rad)
  the law would command 3.53 rad/s, already **exceeding shoulder_lift's own
  URDF velocity limit** (pi rad/s) -- the joint was railed, not
  under-driven; (4) the fixed pad sustained a **~301 N steady-state contact
  force (670 N peak) for 6.3 s** while stationary -- proof of ample
  actuation authority, not a gain deficit; (5) a lag-only explanation
  bounds the possible tracking error at <=6.96 mrad given the trajectory's
  peak speed, an order of magnitude (8.5-10.2x) below the observed
  shoulder_lift/wrist_1 errors.
- **The true cause is a hard mechanical contact**: the fixed pad's bottom
  face landing flat on the object's top face during descent. Contact
  evidence (`evidence/stage2d_pose/D3_retry1_diagnostics/contact_pad_fixed.csv`):
  first contact at descent +1.822 s (descending at ~60 mm/s in the samples
  immediately prior, decelerating through the 17 ms window in which
  contact force built up -- a rigid-stop signature, not a controlled
  approach), a 4-point, 22.000 mm x 59.2 um contact sliver at the
  object's fixed-side face, closing-axis position exactly -15.0000 mm,
  contact normal (0,0,-1), depth 6.3-8.3 um, sustained ~301 N median /
  670 N peak normal force for 6.3 s (persisting past the abort). Travel
  stopped at 58.578% of the 100 mm descent (58.578 mm), matching the
  reported ~58.4%. The joint-space errors are the kinematic image of this
  rigid stop: shoulder_lift's lever arm (~0.525 m) times its 0.0707 rad
  error is ~37.1 mm, wrist_1 contributes ~5.9 mm in the same sense, total
  ~43 mm against a measured 41.4 mm shortfall (matches within the
  linearisation error).
- **Predicted closing-axis margin** (perceived point vs. measured ground
  truth, projected onto the case's own closing axis) at the former 1.5 mm
  clearance: D3 = **-0.0759 mm** (overlap -- matches the observed 59.2 um
  contact sliver once the object's own 20 um displacement at impact is
  accounted for), D1 = +0.1701 mm, D2 = +1.1690 mm. D1's margin was
  already thin; only D3's yaw (closing-axis error nearly parallel to its
  45 deg closing axis) pushed it negative.
- MoveIt has no way to see this class of failure: `gripper_base_link`,
  `jaw_moving_link`, and `jaw_fixed_link` have visual geometry but **no
  collision geometry** in the planning scene, so `cartesian_fraction`
  reads 1.0 and no collision is ever reported despite the hard contact.

### 2.0 mm production change: validation sequence and results

Validated in this order, each a single run/case, no retries or tuning:

| Step | Cases | Override | Result |
|---|---|---|---|
| 1 | D1, D2, D3 | diagnostic (`c_fixed_m=0.002`) | **3/3 PASS**, zero fixed-pad and zero moving-pad contact before `GRIPPER_CLOSE` in all three (`evidence/stage2d_pose/D{1,2,3}_clearance2mm_diag`) |
| 2 | D3 | **production default** (no override) | **PASS**; `control_setup.json["fixed_side_clearance_m_override"] = null`; resolved `grasp_tcp_offset_vec.x = -0.025500`; zero pad contact before close (`evidence/stage2d_pose/D3_production_default_2mm`) |
| 3 | Scene-A/O0 x2 | production default | **2/2 PASS**, all 11 gates, zero pad contact, resolved TCP offset -0.025500 confirmed in both (`evidence/stage1_scene_a_production_2mm_confirmation`, `..._run2`) |

Predicted closing-axis margin at 2.0 mm, from each run's own perceived
point and measured ground truth: D3 = **+0.4241 mm** (confirmed
identically in both the diagnostic-override and production-default D3
runs), D1 = +0.6701 mm, D2 = +1.6690 mm. The diagnostic-override and
production-default D3 runs are numerically indistinguishable (resolved TCP
offset, achieved aperture, gripper result all match; `tcp_error_m` differs
only at sub-micron/noise scale), confirming the two code paths in
`m3_grasp.launch.py` resolve identically.

**Stage-2D D1/D2/D3 are therefore qualified at the 2.0 mm production
default. Stage-2D is COMPLETE.**

### Accepted placement trade-off (not "no regression")

Raising the fixed-side clearance moves the grasp TCP 0.5 mm relative to
the object (`preclose_pose_offset_m(0.030)`: 0.0260 m -> 0.0255 m), which
measurably shifts where the object sits in the gripper and therefore where
it is placed. This is a real, characterised, accepted trade -- not the
absence of a regression:

| case | placement position @1.5 mm | @2.0 mm | delta |
|---|---:|---:|---:|
| Scene-A/O0 (2 runs) | 1.9408 mm (R1-R5 mean) | 2.3284 / 2.3288 mm | **+0.3876 / +0.3880 mm** |
| D1 | 1.4612 mm | 1.9854 mm | **+0.5241 mm** |
| D2 | 1.7727 mm | 2.4335 mm | **+0.6608 mm** |

All remain comfortably inside the `placement_pos_err_mm_max = 10.0 mm` gate
(`scripts/perception/stage2a_analyzer.py`) -- Scene-A at ~23%, D1/D2 at
~20-24% of the gate. The two Scene-A runs agree to 0.0004 mm, confirming
the shift is a deterministic geometric consequence of the TCP move, not
run-to-run noise. Orientation, slip, aperture, and TCP-error metrics show
no coherent shift at 2.0 mm, only ordinary run-to-run variation --- the
mechanism is specific to placement position.

### Stage-2C (C1-C3): not rerun, 3x diagnostic addendum only

Stage-2C's C1-C3 (see the superseded section below) ran under the former
1.5 mm clearance and were **not rerun** for this closure -- their
closing-axis margins only improve at 2.0 mm, so no case is put at risk:

| case | spawn yaw | closing-axis error (3x) | margin @1.5 mm (3x-derived) | margin @2.0 mm (3x-derived) |
|---|---:|---:|---:|---:|
| C1 | +30 deg | +0.4446 mm | +1.0554 mm | **+1.5554 mm** |
| C2 | -30 deg | +0.1120 mm | +1.3880 mm | **+1.8880 mm** |
| C3 | +45 deg | +0.4594 mm | +1.0406 mm | **+1.5406 mm** |

(Margins computed from each case's own `PERCEPTION_POSITION_USED` log line
against `init_settled_pose.json` ground truth -- existing evidence, no new
run.) These C1-C3 margins are still historical 3x diagnostic evidence, not
960x720 production qualification. Their perceived-yaw consumption,
configured/spawned-yaw decoupling, axial mod-180 semantics, and physical
manipulation results remain valid for the actual 2880x2160 runs. Their
numerical XYZ and yaw-error values are resolution-specific. Their recorded
placement-position numbers (1.535-1.583 mm, see the table in the
superseded section below) were measured under the **former 1.5 mm**
configuration; based on the D1/D2/Scene-A pattern above they would be
expected to shift by roughly +0.4-0.7 mm if rerun, but this is not
measured and the original evidence is preserved unmodified as the
authoritative C1-C3 record.

### Next stage

Stage-2D orientation/pose generalization is complete. Do not rerun the
recorded D1/D2/D3 cases, tune thresholds/controllers/clearance further, or
launch a new manipulation campaign from this milestone without a
separately authorized next-stage objective.

Two gaps are already flagged above and neither is scheduled or authorized:
(1) full SO(3) orientation change (roll/pitch, not just yaw) remains
diagnostic-only, per Stage-2C's own "Semantics and limitations" note
below; (2) the gripper has no MoveIt collision geometry, so this entire
failure class stays invisible to planning-time checks regardless of
clearance headroom. A third, narrower candidate: at the 34 mm pre-close
aperture (unchanged), the object's actual physical gap from the moving pad
at pre-close is `preclose_aperture - width - c_fixed_m` = 4.0 mm - 2.0 mm
= **2.0 mm, now numerically equal to the fixed side** -- a derived
physical quantity, not the named `MOVING_SIDE_CLEARANCE_M` constant, which
remains its own unchanged 3.5 mm (it feeds only `DECLARED_CLEARANCE_M`'s
sum, not this per-side accounting). There is no further headroom on the
fixed side without widening `DECLARED_CLEARANCE_M` and the pre-close
aperture, so the next time a case shows a closing-axis overlap, the
durable fix is addressing the ~1.3-1.6 mm closing-axis perception bias at
its source, not a third clearance increase.

## 2026-08-31 Stage-2C Orientation Generalization — SUPERSEDED

Superseded by the Stage-2D section above: Stage-2D closes with the
production fixed-side clearance raised to 2.0 mm (commit `0562222`), which
C1-C3 below did not run under. Every yaw/perception conclusion below
remains valid and was not rerun; see the Stage-2D section's "Stage-2C
addendum" above for the corrected closing-axis margins at 2.0 mm. The
placement-position numbers in the table immediately below are historical,
recorded under the **former 1.5 mm** clearance.

### Milestone status

- Repository: `~/ur5e_pickplace`, branch `stage2-orientation-generalization`,
  HEAD `eb74c27` (`test: make Stage-2C grasp tilt yaw-invariant`). No
  production code or configuration changed after that commit.
- Stage-2C is closed on three independent full-cycle cases with configured
  pick yaw 0 deg and place yaw 0 deg, while spawned yaw was independently
  set to +30 deg, -30 deg, and +45 deg. Every case used fresh perceived yaw,
  one attempt, and no retry or tuning.
- Perception yaw is sourced from fresh `/object_detector/pose_world` data.
  Configured yaw remains in the generated case configuration but is
  deliberately decoupled from spawned ground-truth yaw.
- Object orientation is axial (modulo 180 deg). All yaw comparisons use
  `axial_difference()`.

### Validated Stage-2C cases (diagnostic 2880x2160)

The C1-C3 evidence is not a 960x720 production position or yaw-error
qualification. Separate 960x720 Stage-2B perception evidence remains a
distinct dataset and must not be conflated with these cases.

| Case | Spawn yaw | Yaw error (3x) | Position error (3x) | Aperture | Grasp tilt | Lift slip | Transport slip | Placement position | Axial placement yaw (3x) | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| C1 | +30 deg | 0.0490 deg | 0.459 mm | 30.005 mm | 0.0506 deg | 0.0121 mm | 0.00931 mm | 1.574 mm | 0.0557 deg | PASS |
| C2 | -30 deg | 0.0490 deg | 0.452 mm | 30.008 mm | 0.0497 deg | 0.00833 mm | 0.0104 mm | 1.535 mm | 0.00691 deg | PASS |
| C3 | +45 deg | 0.000018 deg | 0.459 mm | 30.000 mm | 0.0623 deg | 0.0117 mm | 0.0102 mm | 1.583 mm | 0.0389 deg | PASS |

Evidence directories:
`evidence/stage2c_orientation/C1_retry1_config0_spawnp30_place0_perceived_3x/`,
`evidence/stage2c_orientation/C2_config0_spawnm30_place0_perceived_3x/`, and
`evidence/stage2c_orientation/C3_config0_spawnp45_place0_perceived_3x/`.

### Semantics and limitations

- Grasp tilt is authoritative world-up/upright tilt and is yaw-invariant.
  Full SO(3) orientation change remains diagnostic only.
- CORRECTION (2026-08-31 provenance audit): C1-C3 ran at diagnostic
  **2880x2160**, not the 960x720 launch default. The committed
  `run_stage2a_yaw_case.py` and `run_stage2c_yaw_case.py` wrappers did not
  forward resolution arguments, but each recorded C1-C3 runtime command used
  a transient inline Python shim that appended
  `camera_width:=2880 camera_height:=2160` to
  `ur5e_robotiq_sim_control.launch.py` before process creation. The `_3x`
  evidence-directory names are therefore accurate. The effective 3x
  image/intrinsic lattice is independently confirmed by the approximately
  half-pixel XYZ signature in the preserved artifacts. The ~0.45 mm XYZ
  results and numerical yaw-error values are resolution-specific and must not
  be used for 960x720 production qualification. Perceived-yaw consumption,
  configured/spawned-yaw decoupling, axial-yaw semantics, and physical
  manipulation results remain valid for the actual 3x runs. Separate 960x720
  Stage-2B evidence remains separate and must not be conflated with C1-C3.
- The Stage-2B perception chain remains frozen.
- The negative control (configured 0 deg, spawned +30 deg,
  `use_perceived_yaw=false`) aborted during descent with `EXECUTE_FAILURE`
  before grasp closure. It is comparative/incomplete evidence only; the
  abort is not attributed to yaw mismatch.
- Known non-blocking cleanup: `configured_object_yaw_deg` is recorded as
  NaN when `use_perceived_yaw=false`.

### Next stage

Stage-2C orientation generalization is complete. Do not rerun the recorded
cases, tune thresholds/controllers, or launch a manipulation campaign from
this milestone without a separately authorized next-stage objective.

## 2026-08-30 Stage-1 P200 Requalification + Stage-2A Yaw-Feasibility Complete — SUPERSEDED

### Production state

- Repository: `~/ur5e_pickplace`, branch `stage2-orientation-generalization`,
  HEAD `e37383e` (`control: raise parallel-jaw grasp gain to validated
  value`). Every commit hash cited as "HEAD" in a section below this one
  (e.g. `a056023`) is stale by several commits; treat `e37383e` as current.
- `parallel_jaw_gripper_controller`'s `gains.gripper_jaw_joint.p` =
  **200.0** is **production** in
  `ur5e_robotiq_description/config/controllers.yaml` (commit `e37383e`,
  `control: raise parallel-jaw grasp gain to validated value`). **It is no
  longer diagnostic-only.** Every "P=200 remains diagnostic-only" /
  "P=200 does not authorize a production gain change" statement in the
  sections below this one describes a state that no longer holds — it was
  true when written and is superseded now.
- D10 XYZ position estimator (selected-component subpixel centroid + 10%
  symmetric trimmed-mean depth) is production, unchanged since commit
  `a056023`.
- Fixed-side clearance remains **1.5 mm**
  (`GRASP_TCP_FIXED_SIDE_CLEARANCE_M` in `parallel_jaw_geometry.py`),
  unchanged — no clearance override is in production.
- `thresholds.plan_attempts = 1` in `config/scene.yaml`, unchanged.
- No diagnostic override (`p_gain_override`, `fixed_side_clearance_m_override`)
  is required, or was used, for any result recorded in this section.

### Stage-1: requalified under production P=200

- **G1-G5 production-default P=200 regression: 5/5 PASS.** Evidence:
  `evidence/stage1_g{1..5}_production_p200_regression_run*/`.
- **Scene-A production-default P=200 repeatability: 5/5 PASS** — five
  consecutive clean full cycles, no retries/tuning/config changes between
  them, identical production configuration confirmed in every run's
  `control_setup.json`. Evidence:
  `evidence/stage1_scene_a_final_repeatability_20260830/R1..R5/`. Perception
  error identical every run (1.6134 mm); lift slip 0.0068-0.0103 mm;
  transport slip 0.0034-0.0044 mm; placement position error 1.88-1.99 mm;
  placement yaw error <= 0.055 deg; measured gripper effort exactly 5.000 N
  through lift and transport in every run; zero descent collisions; bilateral
  pad seating before lift in every run.
- **Stage-1 (position generalization G1-G5 + Scene-A repeatability) is
  requalified end-to-end under production P=200.** This supersedes the
  "Scene-A production-default repeatability remains pending" status that
  stood before 2026-08-30, and supersedes the older P=50 versions of both
  campaigns recorded further down this file (kept there as historical
  evidence of the P=50 baseline, not as current state).

### Stage-2A: configured-yaw manipulation feasibility COMPLETE

**O0-O4 perception-driven-XY, production P=200, full-cycle: 5/5 physical
PASS** (yaw = 0, +15, -15, +30, -30 deg; object centre still CONFIGURED from
`scene.yaml`, not perceived — see "Current limitation" below). Evidence:
`evidence/stage2a_o{0,1,2,3,4}_perceived_force_authority_full_cycle_control*/`.

| Case | Yaw | Percept err | Aperture | Peak tilt | Lift slip | Stored verdict |
|---|---:|---:|---:|---:|---:|---|
| O0 | 0 deg | 1.6134 mm | 29.9995 mm | 0.0550 deg | 0.0072 mm | PASS |
| O1 | +15 deg | 1.3895 mm | 29.9995 mm | 0.0382 deg | 0.0185 mm | PASS |
| O2 | -15 deg | 1.4293 mm | 29.9995 mm | null (window artifact, see below) | null (window artifact, see below) | stored FAIL, physical PASS |
| O3 | +30 deg | 1.3826 mm | 29.9995 mm | 0.0662 deg | 0.0101 mm | PASS |
| O4 | -30 deg | 1.4200 mm | 29.9995 mm | 0.0675 deg | 0.0075 mm | PASS |

- **O2-O4 are NOT blocked.** They have run and passed. The "O2-O4 remain
  BLOCKED" statement in the 2026-08-29 "Stage-2A Orientation" section below
  is superseded.
- **The O1 configured-center / zero-clearance / force-authority diagnostic
  controls are historical, completed work**, not a pending next task — they
  are what led to the P=200 force-authority fix now in production. The
  "Exact next task — one O1 configured-center diagnostic control" instruction
  in the 2026-08-29 "Stage-2A Orientation" section below has already been
  carried out and must **not** be rerun.
- **Configured-yaw manipulation feasibility (+/-15 deg, +/-30 deg) is
  therefore COMPLETE.**

### Known evidence artifacts (documented; do not read as weakening the PASS verdicts above)

- **G1 (`_run2`) and O2's stored `cycle_metrics.json` verdicts read `FAIL`**,
  solely because `stage2a_analyzer.py`'s fixed `LIFT_BEGIN - 0.8s` pre-lift
  quiescence window overlaps the intentional P=200 force-seating
  displacement, so it declines to compute `lift_slip_mm`/`max_grasp_tilt_deg`
  (nulls them) rather than reporting a bad number. This is a **measurement-
  window artifact of the generic analyzer, not a physical failure** — the
  analyzer itself was not modified and must not be.
  - **G1 direct ground truth** (0.6 s late-anchored quiescent window, right-
    anchored at `LIFT_BEGIN`): lift slip **0.00492 mm**, peak tilt
    **0.0444 deg** — both far inside gates.
  - **O2 direct ground truth** (0.25 s late-anchored window, already recorded
    in the O2 section below): lift slip **0.0125 mm**, peak tilt
    **0.0244 deg**, transport slip **0.0088 mm** — all inside gates.
  - Every other G-pose and O-case resolved the standard 0.8 s window cleanly;
    this artifact affects only these two runs.
- **Some `evidence/stage2a_o{0,3}_..._run{1..4}/` directories are
  infrastructure/preflight aborts, not repeated manipulation attempts.**
  They contain only `control_setup.json` + `scene_case.yaml` (two of them
  also a partial `gz_pose_stream.csv`) — no `m3_grasp.log`, no contact CSVs —
  meaning the harness exited during Gazebo/controller startup, before
  `m3_grasp` ever launched. The numeric suffixes are harness bookkeeping, not
  tuned re-attempts. The PASS results cited above are each the run in its
  family that actually completed (`_run5` for O0, `_run3` for O3; O1/O2/O4
  completed on their first attempt).

### Current limitation — why Stage-2A is not full orientation generalization

XYZ is perception-driven (D10); **yaw is still read from
`config/scene.yaml`'s `object.pick_pose.yaw`**, not estimated from sensor
data. `m3_grasp.cpp`'s perception substitution
(`T_world_grasp.setOrigin(...)`) replaces translation only; rotation is
untouched — confirmed by direct source read, not inferred. Stage-2A therefore
proves the manipulation pipeline *tolerates* a range of configured object
yaws; it does not prove yaw can be *perceived*. True perception-driven yaw
generalization is not yet validated.

### Next stage

- **Stage-2B = yaw perception implementation and perception-only
  qualification.** Begin with an **isolated mask-orientation estimator plus
  unit tests, before any ROS topic wiring**: a header-only second-moment
  estimator (`atan2` of the mask's central moments) added alongside
  `d10_trimmed_mean.hpp`, exercised by synthetic-mask gtests (angle sweep,
  degenerate/near-square rejection, sub-pixel-shift robustness) with no node,
  no topic, and no simulation. Only once that unit suite is green does
  topic/node wiring (`object_detector` -> new `pose_camera` topic ->
  `object_position_world` -> `m3_grasp`) begin.
- **Stage-2B design caution:** object yaw is **axial, not directional** — a
  2-fold-symmetric rectangle's orientation is observable only mod 180
  degrees. Any yaw comparison (estimator-vs-ground-truth in tests, or
  perceived-vs-configured delta inside `m3_grasp`) MUST use the **shortest
  axial difference, canonicalised into `[-90, +90)` degrees** — NOT ordinary
  `wrap_pi`/`[-180, +180)` angle wrapping, which would report a spurious
  ~180 degree error for the identical physical orientation.
- **Stage-2C = manipulation using perceived yaw, with configured yaw
  deliberately decoupled from spawned yaw** (spawn the object at `yaw_true`
  in Gazebo while leaving `scene.yaml`'s configured yaw at 0), so a passing
  cycle is proof the yaw came from perception, not from the config file.
  Stage-2C must not start before Stage-2B's perception-only qualification
  passes its own acceptance criteria.

Do not tune, do not modify `controllers.yaml`/`scene.yaml`, do not launch a
new manipulation campaign, and do not push, until Stage-2B is authorized as
a separate task.

## 2026-08-29 Stage-2A O2 Perceived-Target Force-Authority Full Cycle — PASS — superseded 2026-08-30 (historical: P=200 was diagnostic-only when this ran; it is production now — see current-authority section above)

Exactly one O2 (-15 deg) full cycle was completed with the D10 perceived
target, production 1.5 mm fixed-side clearance, diagnostic P=200 N/m,
commanded 5 N maximum effort, and corrected full-manifold contact recorders.
It completed approach, pre-close, descent, close, lift, transport, place,
release, and retreat. No O3/O4 run or tuning occurred.

- Evidence: `evidence/stage2a_o2_perceived_force_authority_full_cycle_control/`.
- Perception error was 1.4293 mm; its closing-axis component was +0.6946 mm.
  Minimum measured fixed-side descent clearance was +0.7560 mm; there was no
  pre-close/descent contact or object movement.
- Moving/fixed contacts began at sim 42.007/42.017 s and bilateral seating was
  present before `LIFT_BEGIN` at 42.664 s. At lift begin fixed/moving gaps were
  -0.000991/+0.000031 mm. Aperture was 29.9995 mm and joint effort remained
  exactly 5 N through grasp hold, lift, and transport.
- The generic full-cycle analyzer reports O2 `FAIL` only because its fixed
  `LIFT_BEGIN - 0.8 s` baseline spans the last 0.812 mm of the intended
  closing-seat displacement, exceeding its 0.5 mm quiescence limit. It therefore
  leaves lift slip and tilt unavailable; this is a measurement-window issue,
  not a failed lift. A recorded late pre-lift 250 ms quiescent window and the
  post-lift 800 ms quiescent window both have 0.000 mm spread: direct
  ground-truth relative lift slip is 0.0125 mm, lift peak tilt 0.0244 deg,
  and transport peak/retained tilt 0.0848/0.0847 deg. Transport slip is
  0.0088 mm.
- Moving contact was continuous until intended release. Fixed contact had the
  established 17 isolated one-tick reappearance signatures (4 lift, 10
  transport, 3 place), without a corresponding pose, slip, tilt, or retention
  response.
- The object remained retained through intended release and settled upright at
  `[0.448296, 0.199120, 0.772500]` m: 1.9174 mm placement position error and
  0.0274 deg yaw error.

**O2 decision:** PASS on the directly recorded physical gates. P=200 remains
diagnostic-only; do not change `controllers.yaml` or infer a real-hardware
safety conclusion. Stop here; O3-O4 require separate authorization.

## 2026-08-29 Stage-2A O1 Perceived-Target Force-Authority Lift-Only — PASS — superseded 2026-08-30 (historical: P=200 was diagnostic-only when this ran; it is production now — see current-authority section above)

Exactly one guarded O1 (+15 deg) perception-driven diagnostic control was
completed. It used the D10 production perception path (`require_perception`),
the production 1.5 mm fixed-side clearance, P=200 N/m, commanded 5 N maximum
effort, corrected full-manifold pad contact recorders, and `lift_only:=true`.
It stopped after the full post-lift dwell; no transport, place, release, or
O2-O4 run occurred.

- Evidence: `evidence/stage2a_o1_perceived_force_authority_lift_only_control/`.
- Perception used `[0.450936, -0.148973, 0.772499]` m instead of the configured
  centre, a 1.3895 mm XY error with a +1.1698 mm closing-axis component. This
  left a measured minimum fixed-side descent clearance of +0.2868 mm; there
  was no descent contact or object translation.
- Moving/fixed pad contacts first appeared at sim 46.852/46.857 s (fixed 5 ms
  after moving). At `LIFT_BEGIN`, geometric gaps were -0.000810/+0.000326 mm
  (fixed/moving), with four-point manifold summed forces +6.6982/-5.0000 N.
  Final aperture was 29.9995 mm and the informational grasp check was within
  tolerance.
- The object translated 0.3311 mm during close, predominantly 0.3198 mm
  toward the fixed pad, achieving bilateral seating. Measured gripper effort
  was exactly 5.000 N throughout lift and dwell.
- Lift slip was **0.0193 mm**, peak and retained tilt were **0.0111 deg**, the
  object rose 119.996 mm, and it was retained throughout dwell (no drop).
  The moving manifold was continuous. The fixed manifold had four isolated
  one-physics-tick omissions with an immediate ~10.05 N reappearance; no
  pose, slip, tilt, or retention response accompanied them, so they are a
  transient fixed-pad reseat signature rather than sustained contact loss.
- This improves directly on the prior P=50 perceived O1 run (30.2718 mm
  aperture, 1.5656 mm lift slip, 3.3757 deg peak tilt, and fixed seating only
  after lift began). The perception error itself is unchanged; close authority
  is the causal difference.

**Lift-only decision:** PASS. The force-authoritative close tolerates this
measured D10 O1 perception error in this simulation lift-only control. P=200
remains diagnostic-only and does not authorize a production gain change or a
real-hardware safety conclusion. Do not tune or run O2-O4 from this result.

## 2026-08-29 Stage-2A O1 Force-Authority Lift-Only Control — PASS — superseded 2026-08-30 (historical: P=200 was diagnostic-only when this ran; it is production now — see current-authority section above)

Exactly one guarded diagnostic control was completed at O1 (+15 deg), with
the configured scene centre as target (never perceived XY or live Gazebo GT),
the production 1.5 mm fixed-side clearance, `parallel_jaw` geometry, P=200
N/m, commanded max effort 5 N, and `lift_only:=true`. It executed approach,
pre-close, descent, close, Stage-3 lift, and the full 2 s post-lift dwell;
it stopped before transport, place, release, or retreat.

- Primary evidence:
  `evidence/stage2a_o1_force_authority_lift_only_control_run1/`.
- `m3_grasp` result: `SUCCESS`; `lift_result=SUCCESS`;
  `lift_only_stop_reached=yes`; `transport_attempted=no`;
  `place_release_attempted=no`.
- At `LIFT_BEGIN`, the geometric fixed/moving pad face gaps were
  `-0.000934/-0.000062 mm`, and both corrected four-point manifold streams
  were active (fixed/moving summed closing-axis forces `+6.8004/-5.0000 N`).
- Ground-truth lift slip was **0.0636 mm** and peak object tilt was
  **0.0167 deg**; retained post-lift tilt was also 0.0167 deg. The object rose
  119.995 mm, stayed upright, and did not drop.
- Gripper measured effort was exactly 5.000 N in every recorded lift and
  dwell sample; final aperture was 29.9995 mm (`WITHIN TOLERANCE`).
- Bilateral seating was continuous through the 2 s post-lift dwell. During
  the 1.724 s lift, the fixed-pad stream had four one-physics-tick absences
  at sim 46.104-46.106, 46.517-46.519, 46.855-46.857, and 47.351-47.353 s.
  Each immediately resumed at about +10.05 N while the moving pad remained
  present; no object slip, tip, or drop accompanied them. This is a
  transient fixed-pad reseat signature, not persistent loss of grasp.
- The generic Stage-2A full-cycle analyzer writes `FAIL` because perception,
  transport, and placement metrics are intentionally unavailable in this
  configured-centre lift-only control. That is not the lift-only verdict.

**Lift-only decision:** PASS. The force-authoritative close remains securely
grasped through lift under this one simulation control. P=200 remains a
diagnostic-only gain; this does not establish real-hardware safety or
authorize a production gain change. No additional run, tuning, or O2-O4 case
is authorized by this result alone.

Harness-only change used by this control: the existing
`scripts/perception/run_stage2a_yaw_case.py` now exposes default-off
`--lift-only`, forwarding the already-existing `m3_grasp.launch.py`
`lift_only:=true` mode. Production defaults and controller configuration are
unchanged.

## 2026-08-29 Stage-2A Orientation — D10 Integrated, O1 Diagnostic Control Next — superseded 2026-08-30 (historical: HEAD was `a056023` here, now `e37383e`; "O2-O4 remain BLOCKED" and the "Exact next task" below are both superseded — O2-O4 have run and passed, and the O1 configured-center diagnostic has already been carried out. See current-authority section above)

### Repository and production state

- Repository: `~/ur5e_pickplace`
- Branch: `stage2-orientation-generalization`
- HEAD: `a056023` — `perception: use robust D10 object position estimator`
- Production object position estimator:
  - XY = selected connected-component subpixel centroid `(u, v)`;
  - Z = deterministic 10% symmetric trimmed mean of finite, positive depths
    under that selected mask;
  - the centroid is back-projected using that D10 depth;
  - segmentation, component selection, topics, and `PointStamped` interface
    are unchanged.
- `ur5e_pick_place` built successfully with tests enabled. All six focused D10
  tests passed (exact trimming, odd/even and small sample counts, non-finite
  filtering, insufficient finite samples, and deterministic input-order
  behavior).

### Verified Stage-2A state

- Post-D10 perception-only yaw regression P0-P4: **5/5 PASS**.
- Every tested yaw has positive predicted fixed-side pre-close clearance.
- Evidence root: `evidence/stage2a_orientation/`.
- O2-O4 remain **BLOCKED** and must not run until O1 is resolved.

The post-D10 O1 (+15 degrees) full manipulation rerun produced:

| Metric | Result |
|---|---:|
| manipulation node | `SUCCESS` |
| perception error | 1.3895 mm |
| Cartesian fraction | 1.0000 |
| Stage-2 TCP error | approximately 0 mm |
| achieved aperture | 30.2718 mm |
| peak grasp tilt | **3.3757 deg — FAIL** |
| lift slip | **1.5656 mm — FAIL** |
| transport slip | 0.0364 mm — PASS |
| placement position error | 1.8865 mm — PASS |
| placement yaw error | 0.022 deg — PASS |
| authoritative case verdict | **FAIL** |

Current evidence: `evidence/stage2a_orientation/O1/`. Preserved historical
comparison: `evidence/stage2a_orientation_pre_d10_baseline/O1/`.

### O1 forensic conclusion

The historical pre-D10 O1 and current post-D10 O1 have different failure
mechanisms:

- **Historical O1 mechanism eliminated:** its larger closing-axis perception
  error caused fixed-pad interference during descent. The object moved and
  tipped approximately 90 degrees before final close, and the jaw was forced
  to an approximately 45.4 mm aperture. D10 reduced the closing-axis error and
  restored positive pre-close clearance; the new O1 remains stationary and
  upright through pre-close and descent and reaches a 30.2718 mm aperture.
- **Current O1 mechanism:** lift-onset asymmetric seating. Residual perception
  error is approximately 1.170 mm along the closing axis and 0.750 mm along
  the orthogonal axis. At lift start the moving pad is touching while the
  fixed pad retains an approximately 0.272 mm gap. Bilateral seating occurs
  just after lift begins, and the object then rolls about the closing axis and
  moves relative to the wrist by `[-0.272, +1.297, +0.833]` mm (norm 1.5656
  mm). Peak tilt is transient at 3.3757 degrees, but approximately 2.8 degrees
  remains through the post-lift dwell and transport. Additional transport
  slip is only 0.0364 mm, so the object is stable after seating. Placement
  contact returns it upright.
- No measured joint-effort or direct pad-contact stream was preserved in this
  O1 evidence. Contact timing/identity above was reconstructed geometrically
  from the object, wrist, and jaw pose stream plus the known pad geometry.
  The next control must record those signals directly.

### Exact next task — one O1 configured-center diagnostic control

Run **exactly one** diagnostic O1 control with all of these constraints:

- yaw: +15 degrees;
- manipulation target: the configured object center from the scene;
- do **not** use perceived XY for the manipulation target;
- do **not** query or substitute live Gazebo ground truth as the target;
- retain the same TCP, clearance, gripper close command, commanded effort,
  friction, controller, and arm trajectories;
- do not tune any production or physics parameter;
- record continuously from before descent through the post-lift dwell:
  - object pose;
  - TCP and/or `wrist_3_link` pose;
  - gripper joint position, velocity, and measured effort if available;
  - fixed-pad contact stream;
  - moving-pad contact stream.

Decision question: **does configured-center targeting remove the lift-onset
tilt/slip?** If it does, residual perceived XY misregistration is confirmed as
the initiating cause. If the same roll persists despite centered targeting,
investigate pad/contact geometry or force-generation behavior next. Do not
tune during this control, do not retry it silently, and do not run O2-O4.

## 2026-08-28 Baseline Frozen — Stage-1 Commits + Local Tag — superseded 2026-08-30 (historical: this froze the P=50 baseline; Stage-1 was requalified under production P=200 on 2026-08-30 — see current-authority section above)

The READ-ONLY working-tree audit, the D6/D7 forensic resolution, and the
controlled baseline cleanup (all recorded in prior sessions this same day)
are complete and are now committed. This supersedes the previous "Session
End — Next Task: READ-ONLY WORKING-TREE AUDIT" section; that audit is done,
not pending.

- Generalization Stage-1 G1-G5 is validated (full detail in the section
  immediately below) and the campaign is **frozen** — do not re-run G1-G5 to
  "add" evidence; any further pose sweep is Stage-2 scope (see below).
- 5/5 Scene-A perception-driven repeatability is validated (its own section
  further down).
- One **post-cleanup Scene-A regression** cycle was run on 2026-08-28 after
  the cleanup, through the unmodified `scripts/perception/run_5_cycles.py`
  harness: result SUCCESS, perception error 1.6134 mm, selected q =
  `[-0.572304 -0.909075 1.525371 0.954500 1.570796 0.998492]` (identical to
  the repeatability campaign's own selection), Cartesian fraction 1.0000,
  TCP error 0.000281 mm, aperture 29.9995 mm, max tilt 0.0573°, lift slip
  0.0154 mm, transport slip 0.0257 mm, placement error 1.9557 mm, final
  orientation error 0.0542°. **This is a confirmation that the cleaned
  working tree still reproduces the validated cycle — it is NOT an
  additional Stage-1 repeatability or generalization data point and must not
  be counted toward either campaign's N.**
- The historical G5 15.001 s transport-planning stall remains an
  **unexplained, non-reproduced transient planner/runtime anomaly** — still
  OPEN, not fixed. The KDL goal-sampling-starvation hypothesis was refuted
  (offline `IKConstraintSampler` reproduction: 6000/6000 success). Nothing
  in this session's cleanup or regression run bears on this anomaly one way
  or the other.
- `thresholds.plan_attempts` remains `1` in `config/scene.yaml` — still a
  **diagnosability change only**, not a claimed fix for the stall.
- `thresholds.tf_lookup_timeout_s` remains `15.0` — its own comment in
  `config/scene.yaml` records that this value was **never approached** by
  any validated run (worst observed upper bound 2.381 s) and must not be
  described as validated, only as the deadline in force during the final G5
  qualification.
- `static_scene_tf`'s periodic `/tf_static` re-publish timer is **unchanged
  and undemonstrated** — its own comment in `static_scene_tf.cpp` records
  that no consumer in this repository requires it (every lookup uses
  `TimePointZero`) and that no evidence shows it changed an outcome. It is
  retained because Stage-1 validated the binary containing it, not because
  it was shown to fix anything. **Do not describe either the timer or the
  15 s timeout as a validated fix in any future summary.**
- The current production baseline requires `gripper_model:=parallel_jaw`
  with `use_perceived_position:=true require_perception:=true` — every
  Stage-1/repeatability/regression result was measured under this
  configuration. The launch-file **defaults** remain `robotiq_linkage` /
  `use_perceived_position:=false` for backward compatibility; that default
  path has no current validation evidence and must not be assumed
  equivalent.
- Raw bulk experiment evidence (`evidence/`, ~7.7 GB) is **intentionally
  excluded from Git** (`.gitignore`) and stays local-only. Durable results
  live in this file and `PROJECT_STATE.md`, not as committed raw data. Only
  curated summaries (README + MANIFEST.sha256 + summary CSV/JSON), committed
  individually by explicit path, are ever intended to enter Git.
- Three commits establish this baseline on `rgbd-perception`:
  `feat: establish validated perception-driven manipulation baseline`,
  `tools: add manipulation validation and diagnostic utilities`,
  `docs: freeze Stage-1 validation authority` (this commit). A local
  (unpushed) annotated tag `stage1-generalization-pass` is intended as the
  final step, pointing at this commit or later, once a post-commit build
  re-verification passes; check `git tag -l` / `git log --oneline -8` for
  the actual current tip. Nothing is to be pushed.
- **Historical next research phase:** Generalization Stage 2 had not started
  when this 2026-08-28 baseline was frozen. The 2026-08-29 current-authority
  section above supersedes that resume point. Position generalization (G1-G5)
  and repeatability remain closed.

## 2026-08-28 Generalization Stage-1 — RESOLVED — G1-G5 ALL PASS — historical P=50 baseline (superseded by the 2026-08-30 P=200 requalification above; the G1-G5 PASS result itself still stands, at the gain in force at the time)

This section supersedes the G5 verdict in the "2026-08-27 Generalization
Stage-1 — Poses G1-G4" section immediately below (kept intact for its G1-G4
data and the original G5 failure's root-cause chain, which remains relevant
evidence, not an active blocker). G5 was re-attempted twice after that
session and now qualifies as a full Stage-1 PASS. Position generalization
across G1-G5 is experimentally validated, with the planner-attempt
configuration difference explicitly documented below.

### G5 final qualification result — PASS

G5 = [0.480, -0.120, 0.7725]. Full lifecycle
(pregrasp -> descent -> grasp -> lift -> transport -> place -> release),
`use_perceived_position:=true require_perception:=true gripper_model:=parallel_jaw`,
same validated `parallel_jaw` perception-driven baseline as G1-G4.

| criterion | threshold | measured | verdict |
|---|---|---|---|
| result | SUCCESS | SUCCESS | PASS |
| perception error | < 3.0 mm | 1.4959 mm | PASS |
| deterministic selector | valid unique pregrasp | `[-0.489138 -0.866632 1.447793 0.989635 1.570796 1.081659]` | PASS |
| Cartesian fraction | >= 0.95 | 1.0000 | PASS |
| Stage-2 TCP error | < 2.0 mm | 0.000650 mm | PASS |
| grasp aperture | 30 +/- 1 mm | 29.9995 mm | PASS |
| max grasp tilt | < 2.0 deg | 0.0626 deg | PASS |
| lift slip | < 1.0 mm | 0.0335 mm | PASS |
| transport slip | < 1.0 mm | 0.0472 mm | PASS |
| placement error | < 10.0 mm | 1.8906 mm | PASS |
| final orientation error | < 5.0 deg | 0.0511 deg | PASS |

Transport planning time: **16.1 ms** (vs. G1-G4's 25-36 ms). Full cycle result: **PASS**.

Evidence: `evidence/g5_qualification_20260828_000018/` (`README.md`,
`MANIFEST.sha256`, `run_combined.log`, `cycle_metrics.json`, `m3_grasp.csv/.log`,
`gz_pose_stream.csv`, `joint_states.csv`, `mg_proc.csv`, `slip_analysis.txt`,
`tools/`).

### Historical original G5 anomaly — OPEN, unexplained, not a manipulation failure

The original 2026-08-27 G5 trial (documented in full below) failed transport
planning with `PLAN_FAILURE` after a full 15.001 s planning attempt, AFTER
perception, grasp, and lift had already passed cleanly at the same quality bar
as G1-G4. This anomaly:

- **did not reproduce** across two subsequent independent G5 executions
  (a guarded diagnostic retry at `plan_attempts=20`,
  `evidence/g5_diagnostic_retry_20260827_234234/`, transport planned in
  21.4 ms; and this final qualification cycle at `plan_attempts=1`, transport
  planned in 16.1 ms);
- has **no established root cause** and is retained as an **OPEN MoveIt/OMPL
  runtime-planning risk**, not a closed item;
- is **not classified as a manipulation-generalization failure** — perception,
  grasp, and lift passed at the original anomaly's own quality bar, and both
  reproduction attempts passed the full cycle including transport.

### Offline diagnosis — KDL goal-sampling starvation REFUTED as the cause

An offline investigation (prior session, not reproduced here) found that
standalone KDL IK calls at the production 5 ms timeout show real seed
sensitivity (~13-16% single-call failure rate). However, a production-like
reproduction of MoveIt's actual `IKConstraintSampler` goal-sampling path
(same constraint sampler, same retry structure) succeeded **6000/6000**
trials. KDL goal-sampling starvation is therefore **refuted** as the original
G5 anomaly's cause. The anomaly's true cause remains unknown.

### `plan_attempts`: 20 -> 1 — diagnosability change ONLY, not a fix

`config/scene.yaml` `thresholds.plan_attempts` was changed from `20` to `1`.
This is treated strictly as a **diagnosability** change: at `count <= 1`,
MoveIt's `ModelBasedPlanningContext::solve()` takes the single-planner path
and calls `logPlannerStatus()` on failure, which the `count > 1`
`ompl::tools::ParallelPlan` branch never does — so a future recurrence of the
original anomaly would now actually emit an OMPL `PlannerStatus` instead of
disappearing silently. **Do not read the successful 16.1 ms G5 transport plan
at `plan_attempts=1` as evidence that this change fixed the historical
stall** — the stall never reproduced even at the original `plan_attempts=20`
setting, so its absence here proves nothing about the parameter change's
effect on it. No PlannerStatus evidence has yet been emitted, because no
failure has yet recurred to trigger it.

### Generalization Stage-1 caveats — read before citing this as closed

1. **Configuration is not strictly homogeneous across the five poses.** G1-G4
   were executed with `plan_attempts=20`; the final G5 qualification used
   `plan_attempts=1`. This is a real, documented difference in the frozen
   configuration, not merely a labeling issue. Re-running one additional
   G-pose at `plan_attempts=1` would NOT make the five-pose campaign
   configuration-homogeneous — it would only add a second non-uniform data
   point; only a full G1-G5 re-run at one common setting would.
2. **G5's slip/tilt values come from a reconstructed observer.** The original
   `/tmp/m3_diag_observer.py` used for G1-G4 was destroyed when `/tmp` was
   wiped by a reboot. G5's final qualification measured lift slip, transport
   slip, and grasp tilt via `evidence/g5_qualification_20260828_000018/tools/gz_observer.py`
   and `analyze_g5.py`, built from the repo's own validated primitives
   (`scripts/lib/slip.py`, `scripts/lib/sample_pose.py`), with definitions
   documented in that evidence directory's `README.md` and self-tests
   confirming the slip metric's correctness. The reconstructed results agree
   closely with the G1-G4 distributions (e.g. transport slip 0.0472 mm vs.
   G1-G4's 0.0238-0.0427 mm range) but this agreement corroborates the
   reconstruction rather than proving definitional identity with the original
   observer.

**Stage-1 conclusion:** Position generalization across G1-G5 is
experimentally validated, with the planner-attempt configuration difference
explicitly documented above.

## 2026-08-27 Generalization Stage-1 — Poses G1-G4 — historical (superseded first by the 2026-08-28 G1-G5 RESOLVED section, then by the 2026-08-30 P=200 requalification above)

This section is the first durable record of the Generalization Stage-1 campaign; the underlying raw evidence for G1-G3 already existed on disk (`evidence/generalization_stage1/pose_G{1,2,3}/`) but had not previously been summarized here. G4 was run in this session using the unmodified perception-driven `parallel_jaw` baseline and the identical harness methodology as G1-G3 (per-pose driver script, one mechanical substitution of pose coordinates/labels only, no logic change). Harness scripts used: `/tmp/run_pose_g{1,2,3,4}.py` (not committed to the repo — recovered from `/tmp` for G1-G3, authored for G4 by direct mechanical substitution of G3's script) plus the shared observer `/tmp/m3_diag_observer.py` and `scripts/perception/milestone_f1_harness.py`. All four trials used `use_perceived_position:=true require_perception:=true gripper_model:=parallel_jaw`, full lifecycle (pregrasp -> descent -> grasp -> lift -> transport -> place -> release), object spawned and ground-truth-confirmed at each pose's configured XYZ before perception ran.

Stage-1 acceptance criteria (all must hold): `result==SUCCESS`; perception error < 3.0 mm; deterministic selector produces a valid unique pregrasp; Cartesian fraction >= 0.95; Stage-2 TCP error < 2.0 mm; grasp aperture 30 +/- 1 mm; grasp/object tilt < 2.0 deg; lift slip < 1.0 mm; transport slip < 1.0 mm; placement error < 10.0 mm; final orientation error < 5.0 deg.

| Pose | Configured XYZ (m) | Verdict | Perception err | Cartesian frac | Stage-2 TCP err | Aperture | Tilt | Lift slip | Transport slip | Placement err | Orientation err |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| G1 | [0.450, -0.100, 0.7725] | **PASS** | 1.7628 mm | 1.0000 | 0.000322 mm | 29.9995 mm | 0.0888° | 0.0146 mm | 0.0340 mm | 2.1960 mm | 0.0552° |
| G2 | [0.450, -0.200, 0.7725] | **PASS** | 1.4716 mm | 1.0000 | 0.000702 mm | 29.9995 mm | 0.0570° | 0.0242 mm | 0.0427 mm | 1.9693 mm | 0.0557° |
| G3 | [0.500, -0.150, 0.7725] | **PASS** | 1.7284 mm | 1.0000 | 0.000653 mm | 29.9995 mm | 0.0745° | 0.0521 mm | 0.0238 mm | 1.9821 mm | 0.0509° |
| G4 | [0.400, -0.150, 0.7725] | **PASS** | 1.5116 mm | 1.0000 | 0.000322 mm | 29.9995 mm | 0.0697° | 0.0393 mm | 0.0277 mm | 2.0162 mm | 0.0463° |

All four poses PASS every Stage-1 criterion with wide margin. Full per-run evidence (Gazebo pose/joint streams at high rate, pad contact captures, node logs, `cycle_metrics.json`, `m3_grasp.csv`) is under `evidence/generalization_stage1/pose_G{1,2,3,4}/`. Post-run verification for G4: `git diff --check` clean, zero stale ROS/Gazebo processes confirmed via `pgrep -af`.

No production or configuration file was modified to run G4 (verified: tracked `git status --short` diff is unchanged from session start). This campaign is evidence-gathering only.

### G5 = [0.480, -0.120, 0.7725] — FAIL, campaign stopped here

`/tmp/run_pose_g5.py` was produced by the same mechanical pose/label substitution from the validated G4 driver (no harness-logic change; diff-verified). Pregrasp selection, descent, grasp, and lift all completed normally and matched the same quality bar as G1-G4:

- Perception error: 1.4959 mm (< 3.0 mm)
- Selected pregrasp q: `[-0.489138 -0.866632 1.447793 0.989635 1.570796 1.081659]`, D_descent=0.6609, D_transit=4.2473
- Cartesian fraction: 1.0000
- Stage-2 TCP error: 0.000343 mm
- Grasp aperture: 29.9995 mm, tilt: 0.0049°
- Lift slip: 0.0301 mm

**Stage 4 (transport) FAILED**: MoveIt/OMPL RRTConnect returned `PLAN_FAILURE: no plan to the pose one standoff (0.100 m) above object.place_pose` after a 15 s planning attempt. The object was still held in the gripper at failure time and was never released or dropped (confirmed by the m3_grasp log's explicit "nothing has been released" statement and `object_released=false` in evidence). Transport slip, release result, and placement metrics are therefore not computable/applicable and are recorded as `null` with an explanatory note in `cycle_metrics.json` rather than fabricated.

This is consistent with the transport-side stochastic OMPL/RRTConnect planning-failure behavior already documented earlier in this file for perception-mode transitions (see the 2026-08-26 section on the perception-mode `PLAN_FAILURE`, there attributed to pregrasp planning; here the same class of failure recurs at the transport-standoff plan). It is evidence of intermittent/pose-dependent planner behavior in the transport leg, not a regression in perception, grasp, or lift, all of which passed cleanly in the same run.

Per instruction, the trial was **not retried and no tuning was attempted**. Evidence preserved at `evidence/generalization_stage1/pose_G5/` (full logs, `gazebo_pose_stream.csv`, `gripper_joint_stream.csv`, `m3_grasp.csv`, `cycle_metrics.json` with `verdict: FAIL`). Zero stale ROS/Gazebo/MoveIt processes confirmed after teardown; `git diff --check` clean; no tracked file was modified by this trial (tracked `git status --short` diff identical to before the trial — same 20 modified files as at session start). **The Generalization Stage-1 campaign stops here; no further poses were run.**

## 2026-08-27 Repeatability Campaign — historical P=50 baseline (superseded by the 2026-08-30 P=200 Scene-A repeatability requalification above; the 5/5 PASS result itself still stands, at the gain in force at the time)

This section supersedes prior authority entries below. It records the final validated Scene-A perception-driven pick-and-place baseline and 5/5 repeatability campaign results.

### Validated Scene-A Baseline & Repeatability Campaign Results

- **Scene-A perception-driven pick/place repeatability:** **5/5 PASS**
- **Repeatability verdict:** **CONFIRMED PASS**
- **Perception error:** mean **1.6134 mm**, max **1.6134 mm**
- **Deterministic selector:** chose the exact same pregrasp in all 5 runs:
  `[-0.572304, -0.909075, 1.525371, 0.954500, 1.570796, 0.998492]`
- **Cartesian fraction:** **1.0000** in all 5 runs
- **grasp_tcp_offset:**
  - `X = -0.026 m`
  - `Z = 0.049 m`
- **Grasp aperture:** **29.9995 mm** in all 5 runs
- **Lift slip:** mean **0.0303 mm**, max **0.0471 mm**
- **Transport slip:** mean **0.0465 mm**, max **0.0815 mm**
- **Placement error:** mean **2.0200 mm**, max **2.0819 mm**
- **Orientation error:** mean **0.0340°**, max **0.0537°**
- **Stage-2 TCP error:** mean **0.000473 mm**, max **0.000713 mm**
- **Full-open parallel-jaw release fix:** **VALIDATED**
- **allowed_start_tolerance:** **0.01**
- **Required DART baseline:** `parallel_jaw`

### Key Root-Cause Chain

1. **IK Branch Selection:** Bad pregrasp IK branch selection previously caused excessive trajectory demand and joint tolerance violations. The deterministic selector fixed branch selection.
2. **Grasp Descent Tipping:** Caused by fixed-pad overlap from the old `X = -0.027 m` offset. Adjusting to `X = -0.026 m` restored positive clearance and guaranteed an upright grasp.
3. **Release Bug:** The full-open release failure was resolved by commanding full-open `q = 0`.

## 2026-08-26 Execution Handoff — historical (MoveIt execution-watchdog / GOAL_TOLERANCE_VIOLATED debugging, long resolved; superseded by all sections above)

The validation-wrapper lifecycle hang is fixed and validated. Perception-mode
pre-grasp `PLAN_FAILURE` is cleared as stochastic OMPL/RRTConnect behaviour;
do not retune planning. The established perception-mode shoulder-pan
path-constraint exclusion remains intact.

The MoveIt execution watchdog fix is validated with runtime
`trajectory_execution` parameters:

- `allowed_execution_duration_scaling = 1.2`
- `allowed_goal_duration_margin = 1.5`
- `allowed_start_tolerance = 0.01`

The actual remaining failure is `arm_controller`
`GOAL_TOLERANCE_VIOLATED`: `shoulder_lift_joint` ends approximately
`0.093015 rad` from goal. Cartesian x2 time scaling, x0.5 velocities, and
x0.25 accelerations are confirmed active.

Correction to the point-count premise: MoveIt reports 12 Cartesian samples,
but the actual post-scaling FJT contains 20 points. Therefore a 12-vs-20 knot
density experiment is invalid. The exact captured failing trajectory is 20
points, `3.777624982 s` long, with positions/velocities/accelerations
populated, effort empty, and `arm_controller.interpolation_method = splines`.
Its start state is:

```
[-0.571294174, -1.268085081, 2.127037774,
 -2.428692762, -1.570836761, -2.142085821]
```

Evidence: `evidence/perception_watchdog_confirmation_20260826_152825` and
`evidence/trajectory_density_ab_20260826`. The historical 20-point E2 FJT was
not recoverable; it is not an exact reproducible baseline.

### Exact next task

Run **one** isolated direct replay of the captured failing 20-point
post-scaling `JointTrajectory` directly through `arm_controller` using
`FollowJointTrajectory`, from the exact same settled start state. No
perception, MoveIt planning, complete pick/place cycle, or parameter tuning.

- If it reproduces ~0.093 rad shoulder error: investigate
  `ros2_control -> gz_ros2_control -> Gazebo`.
- If it succeeds: investigate runtime context/load/state differences.

### 2026-08-26 direct isolated replay — completed, do not rerun automatically

One arm-only, no-MoveIt/no-perception replay sent the captured 20-point FJT
unchanged after direct restoration to its recorded first point. The start-state
maximum position error was `1.04e-7 rad`; maximum start velocity was
`5.76e-6 rad/s`. `arm_controller` was active at 500 Hz with `splines`.

The replay did **not** reproduce the historical goal-tolerance signature.
Instead it aborted earlier with `PATH_TOLERANCE_VIOLATED` at 3.058 s simulated
time: shoulder-lift controller error `+0.200083561 rad` versus its 0.20 rad
path tolerance. The action ended at 3.092 s sim / 3.101 s wall with error code
`-4`. Its physical shoulder velocity did not collapse in the 2.10--2.25 s
window (it increased from 0.01164 to 0.02413 rad/s). The requested-final
shoulder error at terminal state was `-0.219729496 rad` (actual minus requested
final); the controller's post-abort reference was reset, so its final reported
error is not the requested-goal error.

This confirms an FJT/controller-to-simulation failure without MoveIt or
perception, but the changed failure signature means it is not an exact
reproduction of the original `-0.093015 rad` goal-tolerance result. Evidence:
`evidence/direct_cartesian_replay_20260826/run/` (outbound goal, preflight,
controller and ROS joint streams, terminal result) and
`evidence/direct_cartesian_replay_20260826/gz_native_joint_state.log`.

## Historical Objective (2026-08-24, superseded — bullet-engine migration probe; see current-authority section at top of file)

**Run the corrected bullet-featherstone project-compatibility probe, then
decide only A / B / C.** Full statement in the dated "engine-capability probes"
section at the end of this file, and in
`evidence/bullet_engine_probes_20260824/README.md`.

The Robotiq right-jaw mimic / contact-geometry defect is diagnosed and the
friction hypothesis is CLOSED, but the audit's recommended fix was falsified by
probe and the replacement route is not yet established. **No compatibility or
migration verdict has been reached.**

Friction calibration run 2 (`RUN_ID=20260824_122918_4279`) returned a formal
**VALID** verdict on 2026-08-24 with all four validity gates passed and
**mu_eff = 1.0202** at the frozen 2 mm criterion, **0.99969** from the predeclared
cubic estimator. Effective pad<->object friction is RESOLVED and is not what loses
the object. Full result in the dated section at the end of this file.

Order of work, as predeclared: the `right_knuckle_joint` 0.53 rad mimic collapse
first; then the inner-knuckle top-edge contact and the descent geometry that lets
it lead; then the F3 retention criterion itself, which must require the object to
be clear of the table and load-bearing in the grasp before G0 is taken.

The root-cause analysis of dynamic grasp retention is COMPLETE and accepted
(2026-08-24). Its conclusions are recorded in the dated section at the end of
this file and in
`evidence/f3_p12_5_lift_scene_A/analysis_root_cause_20260824/ROOT_CAUSE_ANALYSIS.md`.

No manipulation trial, no Scene A rerun, no Scenes B-D, no controller gain
tuning, no threshold change, no friction change, no physics change, no grasp or
geometry change, and no automatic rerun of anything is authorized. The Robotiq
right-jaw fix is identified but must NOT be started until the friction
hypothesis is formally closed.

## Historical Status Snapshot (2026-08-24, superseded — see current-authority section at top of file)

```
MILESTONE F2 — perception-derived grasp    — PASS (2026-08-23)
MILESTONE F3 — Scene A P12.5 lift trial    — FAIL (2026-08-23)
             — Scenes B-D                  — NOT RUN, and must not be run
ROOT-CAUSE ANALYSIS of the F3 failure      — COMPLETE, accepted (2026-08-24)
FRICTION CALIBRATION run 1                 — INDETERMINATE (2026-08-24)
                                             descriptive reading mu_eff ~ 1.0
                                             VERDICT UNCHANGED, not revisited
FRICTION CALIBRATION run 2 instrument      — CORRECTED, gates 8/8, FROZEN
FRICTION CALIBRATION run 2 measurement     — VALID (2026-08-24)
                                             mu_eff = 1.0202 (cubic 0.99969)
                                             all 4 gates PASS
FRICTION HYPOTHESIS                        — CLOSED
ROBOTIQ MIMIC AUDIT (read-only)             — COMPLETE (2026-08-24)
SDF CLOSED-LOOP FEASIBILITY (I-0)           — FAIL, conclusive
NATIVE SDF MIMIC under bullet-featherstone  — PASS, conclusive (<=6e-8 rad)
PROJECT-COMPATIBILITY PROBE                 — NOT TESTED (harness built, unrun)
ENGINE MIGRATION VERDICT                    — NONE REACHED
```

### F3 Scene A P12.5 Scientific Lift Result (`RUN_ID=20260823_190610_13717`)

The controlled, frozen P12.5 Scene-A lift-only experiment completed all execution
boundaries under strict pre-lift barrier synchronization and evaluated to **FAIL**:

- **Verdict:** **`FAIL`** (G2 retention breach: $26.054\text{ mm} > 5.000\text{ mm}$);
- **F2 Prerequisite Gate:** **`PASS`** (perception top surface $[0.4510, -0.1487, 0.7950]$, Cartesian fraction $1.0$, TCP error $0.0000\text{ m}$, `TIMED_OUT_HELD` at $0.5169\text{ rad}$, barrier armed at sim $t=69.703\text{ s}$);
- **Reference Grip Coordinate:** $q_{\text{ref}} = 0.789144\text{ rad}$ extracted from final $0.5\text{ s}$ median of loaded PID baseline ($[71.589, 72.089]\text{ s}$) and latched;
- **Controller Switch & Ownership:** Strict switch executed at sim $t=76.166\text{ s}$ ($P_0$); `p12_5_hybrid_hold_controller` held exclusive master effort claim; restored cleanly to `gripper_controller` at teardown;
- **G0 Baseline Validity:** **`G0_RELIABLE: true`** ($[105.665, 106.165]\text{ s}$, 53 samples, max gap $15.0\text{ ms}$, p2p $1.218\text{ mm}$, master range $0.01465\text{ rad}$);
- **Lift Timing:** `LIFT_BEGIN` at sim $t=106.165\text{ s}$, `LIFT_DONE` at sim $t=108.347\text{ s}$ (duration $2.182\text{ s}$);
- **Relative Slips:**
  - **G0 $\rightarrow$ L1 slip:** **$8.048\text{ mm}$** ($0.008048\text{ m}$)
  - **G0 $\rightarrow$ L2 slip:** **$26.054\text{ mm}$** ($0.026054\text{ m}$) — **Threshold $5.000\text{ mm}$ FAILED**
  - **L1 $\rightarrow$ L2 slip:** **$18.079\text{ mm}$** ($0.018079\text{ m}$) — object continued slipping during the 2.0 s post-lift dwell
- **Maximum Instantaneous Wrist-Relative Displacement:** $7.187\text{ mm}$ during lift transit, $26.272\text{ mm}$ during full post-lift dwell;
- **Orientation Change:** G0 $\rightarrow$ L1: $0.1417\text{ rad}$ ($8.12^\circ$); G0 $\rightarrow$ L2: **$0.3783\text{ rad}$ ($21.68^\circ$)**;
- **World-Z Motion:** Gross lift at L1 $= +113.09\text{ mm}$ ($Z=0.8886\text{ m}$ vs G0 $0.7755\text{ m}$); net lift at L2 $= +96.30\text{ mm}$ (dropped $16.79\text{ mm}$ between L1 and L2);
- **Contact Dynamics:**
  - Bilateral fingertip contact remained continuous throughout all windows (Left: 41,172 msgs, max gap $4\text{ ms}$; Right: 40,809 msgs, max gap $5\text{ ms}$ — G1 PASS);
  - Right inner-knuckle contact was lost during post-lift dwell at sim $t=108.758\text{ s}$ ($0$ msgs in L2) as the object slid down into distal fingertip contact;
- **Controller Effort & Saturation:** $27,724 / 27,724$ cycles reconstructed at $10^{-10}\text{ N}\cdot\text{m}$ (G3 PASS); **0 / 27,724 saturated cycles** ($0.00\%$, range $0.167\text{--}0.897\text{ N}\cdot\text{m}$);
- **Finding:** P12.5 successfully suppressed limit cycles during static hold ($34.4\%$ band power vs PID), but did **NOT** provide sufficient normal force/friction to prevent gross slip under vertical acceleration;
- **Milestone Gating:** Scenes B-D must NOT proceed; controller gain interpolation should not continue without new evidence;
- **Durable Evidence:**
  - Analysis JSON: `evidence/f3_p12_5_lift_scene_A/A/lift_analysis_20260823_190610_13717.json`
  - Retention JSON: `evidence/f3_p12_5_lift_scene_A/A/A_retention_20260823_190610_13717.json`
  - Verdict file: `evidence/f3_p12_5_lift_scene_A/A/VERDICT_20260823_190610_13717.txt`
  - Protocol log: `evidence/f3_p12_5_lift_scene_A/A/protocol_20260823_190610_13717.log`
  - Streams: `p12_5_terms_*.csv`, `pose_*.csv`, `gz_joint_*.csv`, `master_joint_*.csv`, `contact_*.log`

### Validated boundary

```
RGB-D -> detector -> camera-frame 3D -> TF2 world position
      -> perception-derived pre-grasp -> Cartesian descent
      -> physical grasp -> STOP
```

No perception-derived lift, transport, place, or release is validated or
claimed.

### Frozen production candidate

Descent pre-close command `0.070000 rad`, produced by `config/scene.yaml`
`grasp.preclose_margin_rad: 0.4678679450464813` against the unchanged expected
grip angle `0.5378679450464813 rad`. Validated across scenes A-D plus a strict
no-object regression and a full classical regression. Do not tune it.

### F2 headline evidence

| Scene | perception error | achieved pre-close [rad] | min right/left inner clearance [mm] | fraction | P1->P2 | TCP error [mm] | F2 stop |
|---|---:|---:|---:|---:|---:|---:|---|
| A | 1.613648 mm | 0.060133924 | 1.198 / 10.471 | 1.0 | 0.000000 mm | 0.000296 | yes |
| B | 1.5767 mm | 0.060137029 | 6.841 / 7.142 | 1.0 | 0.000000 mm | 0.000339 | yes |
| C | 1.5141 mm | 0.0601319 | 7.680 / 7.285 | 1.0 | 0.000000 mm | 0.000702 | yes |
| D | 1.1333 mm | 0.0601414 | 3.603 / 7.535 | 1.0 | 0.000000 mm | 0.000488 | yes |

All four used `position_source=perceived`. Full detail, including the strict
no-object and classical regressions, is in the dated F2 sections below and in
`docs/HANDOFF_RGBD_PERCEPTION.md` §11.3.

### Open warnings carried into F3

1. **Closure seating ~21.6-22.3 mm in every scene.** Undiagnosed. It occurs
   after the accepted approach-disturbance interval, so F2 explicitly excluded
   it — but it is the single most likely cause of an F3 lift-retention failure.
2. **Final close returns `TIMED_OUT_HELD`, never `REACHED_GOAL`**, in all four
   scenes and in classical regression. Historically accepted as "held".
3. **Thin timeout headroom.** At `0.070 rad` roughly 0.223 s remains before the
   5 s first-result boundary; the sweep showed `0.045 rad` would cross it.
4. **The follower/mesh clearance model is validated only to useful accuracy**,
   not proven: measured Scene-A clearance was `1.198 mm` against a predicted
   `1.488 mm` — 0.290 mm optimistic.
5. **`perceived_position_timeout_s=2.0` untested under heavier simulation load.**
6. **Perception bias is uncorrected by design.** The whole ~1.1-1.6 mm error
   budget is Milestone C mask discretization, systematic across all scenes.
7. **Published point semantics:** visible TOP SURFACE, not the geometric
   centre, and no orientation/yaw is estimated. Any F3 consumer must respect
   this.

### Current Git state — measured 2026-08-23

Verified with `git status --short`, `git diff --check`, `git diff --stat`,
`git branch --show-current`, `git rev-parse HEAD`. Re-measure rather than
trusting this block if the working tree has since changed.

- Branch `rgbd-perception`, HEAD `7b875a4a0283b9faf34cc0288e9d5673a4a2f518`
  (= tag `m6-30mm-stable`). No upstream configured for this branch.
- `origin/m6-width-30mm` also at `7b875a4`; `main` and `origin/main` at
  `9c26214`. Remote `git@github.com:Sachin6120/ur5e-robotiq-pickplace.git`.
- **Nothing committed, nothing pushed.** All A-F2 work is working-tree only.
- `git diff --check`: clean.
- **Tracked modified: 10 files** (+881 / −22):
  `config/scene.yaml`,
  `ur5e_pick_place/CMakeLists.txt`,
  `ur5e_pick_place/package.xml`,
  `ur5e_pick_place/include/ur5e_pick_place/failure.hpp`,
  `ur5e_pick_place/src/m3_grasp.cpp`,
  `ur5e_pick_place/launch/m3_grasp.launch.py`,
  `ur5e_robotiq_description/CMakeLists.txt`,
  `ur5e_robotiq_description/launch/ur5e_robotiq_sim_control.launch.py`,
  `ur5e_robotiq_description/urdf/ur5e_robotiq.urdf.xacro`,
  `ur5e_robotiq_description/urdf/vendor/robotiq_2f_85_macro.urdf.xacro`.

  The last two of those beyond the original F1 set of eight are the F2 work:
  `config/scene.yaml` carries the single `preclose_margin_rad` scalar, and the
  vendor macro carries the opt-in `enable_diagnostic_contacts` observers
  (default false; `<sensor type="contact">` elements only, no geometry,
  surface, inertia, joint, controller, or physics change).
- **Untracked kept (58 entries total in `git status --short`):** `AGENTS.md`,
  `CLAUDE.md`, `HANDOFF.md`, `PROJECT_STATE.md`,
  `docs/HANDOFF_RGBD_PERCEPTION.md`, `scripts/perception/`,
  `ur5e_pick_place/src/object_detector.cpp`,
  `ur5e_pick_place/src/object_position_world.cpp`,
  `ur5e_robotiq_description/worlds/`, `m3_grasp.csv`, and the M8/M9/M10 and
  `prod_reg` capture scripts and marker directories.

### Evidence durability audit — 2026-08-23

Audited `docs/`, `runs/`, `scripts/perception/`, the repository root, and
`/tmp`. No evidence was moved, renamed, or altered by this audit.

**A. Durable RAW evidence that survives**

- `runs/prod_reg_test_c_roll90_width30_{pose,traj}_20260823_004208_6512.csv`
  (1.0 MB + 2.4 MB) and the matching
  `docs/prod_reg_test_c_roll90_width30_*_20260823_004208_6512.{log,csv}` set
  (sim, move_group, grasp, poserecorder, trajrecorder, stallmonitor, stalls,
  and both fingertip contact streams), plus the marker directory.
  This is the classical full-cycle regression run **on the F2 code base**:
  `result=SUCCESS position_source=configured f2_stop_reached=no
  transport_result=SUCCESS`. It proves the F2 code additions did not regress
  the classical lifecycle.
  **Note its scope precisely:** `preclose_achieved=0.2280`, i.e. the OLD
  `preclose_margin_rad: 0.30` configuration. It predates the `0.070 rad`
  correction and is NOT evidence for the frozen candidate.
- The 27 `docs/*markers_*/` directories and their `runs/` counterparts for
  M8/M9/M10 and the four `prod_reg` configurations — the classical baseline.

**B. Durable SUMMARIZED evidence that survives**

- The F1 and F2 measurement tables and dated narrative sections in this file.
- `docs/HANDOFF_RGBD_PERCEPTION.md` §§8-11, including §11.3's A-D matrix.
- `scripts/perception/milestone_{d,e,f1}_{harness,truth}.py` — the harnesses
  themselves survive, so the procedure is reproducible even though its output
  is not. F2 reused the F1 harness with runtime flags.

**C. MISSING evidence — do not reconstruct**

Every `/tmp` path referenced by the F1/F2 sections is gone; `/tmp` currently
holds no project directories at all:
`/tmp/ur5e_f1_guarded_results/`, `/tmp/ur5e_f2_results/`,
`/tmp/ur5e_f2_initialized_results/`, `/tmp/f2_scene_b_sync/`,
`/tmp/f2_A_0070_results/`, `/tmp/f2_A_0070_sync/`, `/tmp/f2_0070_generalize/`.

Lost with them: the A-D `0.070 rad` node logs, CSVs and truth JSON; the
synchronized 1 kHz joint/follower traces and all eight contact streams; the
strict no-object run; and the final classical regression under the frozen
`0.070 rad` value. `m3_grasp.csv` in the repository root is **not** a
substitute — it holds one row from 2026-08-22 15:48 with the pre-F1 header and
no `position_source` column.

Nothing has been fabricated or reconstructed to replace any of this. The same
loss pattern was recorded for Milestone B in
`docs/HANDOFF_RGBD_PERCEPTION.md` §8.9.

**Conclusion.** F2 raw runtime evidence is not durable; F2 PASS currently rests
on the recorded measurement tables/documentation. That is an evidence-quality
limitation, not grounds to silently revoke the validated result.

Contributing cause, for whoever plans F3: `.gitignore` excludes `runs/`,
`docs/*.log`, `docs/*.csv` and `docs/*.txt`, and the perception harnesses
default their output to `/tmp`. Evidence therefore survives only on the local
disk, and only when a harness was pointed outside `/tmp`. Any F3 run should
write its evidence under the repository from the start.

### Exact recommended next step

SUPERSEDED 2026-08-24. The F2 durability question was settled by the
regeneration recorded at the end of this file; the sentence that stood here
("decide the F2 durability question before starting F3") was two milestones
stale and has been replaced.

**The next task is to prospectively correct the friction-calibration
measurement infrastructure so the initial quiescence window is structurally
recorded before ramp motion.** Then, in order:

1. regression-test the corrected validity gates;
2. freeze the prospective amendment;
3. execute exactly ONE new friction-calibration measurement;
4. no parameter sweep, no automatic rerun.

If the valid repeat confirms `mu_eff ~ 1`, close the friction hypothesis and
proceed to investigation and engineering of the Robotiq right-jaw
mimic/contact-geometry defect.

Do not commit or push without explicit approval.

## Historical record — Milestone F1 validation (superseded by F2)

The remainder of this section is the F1 checkpoint as written. Its "Current Git
state" and "Exact recommended next step" subsections are historical and were
correct at the time; use the measured Git state above instead.

### Completed

- **Strict perception mode.** `require_perception` (default false). With
  `use_perceived_position:=true require_perception:=true`, a perception timeout
  is a typed `PERCEPTION_TIMEOUT` failure — no fallback to `scene.yaml`.
- **`PERCEPTION_TIMEOUT`** added to `Result` in `failure.hpp` (not overloading
  `CONFIG_ERROR`).
- **`position_source` in the CSV**, placed immediately after `result`. Values:
  `configured`, `perceived`, `fallback_configured`, `perception_timeout`. Also
  in the one-line RUN SUMMARY. The evidence now states the source instead of
  leaving it to be inferred from commanded coordinates.
- **`pregrasp_only`** (default false): stops after the pre-grasp pose is
  executed and verified. Four stage guards added (pre-close, Cartesian descent,
  gripper close, transport). Separate from `close_and_hold_only`, which stops
  after contact.
- **M1 observation pose + stationarity gate.** When `use_perceived_position` is
  true, the node drives to M1 (joint values from `scene.yaml` via the launch
  file — NOT redefined in code) and requires all six arm joints below
  `stationary_velocity_eps` for `stationary_consecutive_samples` consecutive
  `/joint_states` samples before perceiving.
- **Freshness gate.** Only a `PointStamped` stamped strictly after the M1
  stationarity boundary is accepted; stale and invalid samples are counted and
  logged. One sample is frozen, then the subscription goes out of scope.
- **Pre-grasp ground-truth verification** against `pregrasp_pose_error_max_m`
  (default 0.010), using the same Gazebo-not-TF mechanism as the classical
  Stage-2 check.
- **Evaluation harness**: `scripts/perception/milestone_f1_harness.py` and
  `milestone_f1_truth.py`, same freeze-then-truth two-process discipline as D/E.

### NOT completed

- Documentation of F1 in `docs/HANDOFF_RGBD_PERCEPTION.md` was not written.
- Perception-driven grasp/contact/lift/transport/place (F2 or later) has not
  started.

### Guarded-binary validation update — 2026-08-23

- **Strict no-object PASS.** `result=PERCEPTION_TIMEOUT`,
  `position_source=perception_timeout`, no fallback, `NO_MOTION` logged, and no
  planning or execution request occurred after the timeout. The plan/execute
  requests earlier in the log were solely the required move to M1. A
  post-failure `/joint_states` sample matched M1 within 0.09 mrad on every arm
  joint and showed the arm stationary.
- **Scenes A-D PASS on the guarded binary.** All used
  `position_source=perceived`, met the frozen target/achieved/orientation/object
  bounds, and performed no descent, gripper command, or transport.
- **Fallback observability PASS.** With perception unavailable and
  `require_perception:=false`, the run logged `PERCEPTION_FALLBACK` and recorded
  `position_source=fallback_configured`; configured pre-grasp verification
  error was 0.0393 mm.

| Scene | perceived→truth pre-grasp | achieved TCP error | orientation error | object displacement |
|---|---:|---:|---:|---:|
| A | 1.6134 mm | 0.1344 mm | 0.053010° | 0.0000 mm |
| B | 1.5767 mm | 0.1685 mm | 0.079557° | 0.0000 mm |
| C | 1.5145 mm | 0.0978 mm | 0.076356° | 0.0000 mm |
| D | 1.1339 mm | 0.1675 mm | 0.061282° | 0.0000 mm |

Freshness rejection remained active (`rejected_stale=1` in A-C; D's first
post-boundary observation was already fresh). Raw sensor logs/JSON, CSVs, and
truth JSON are under `/tmp/ur5e_f1_guarded_results/` for this session.

### Confirmed findings

- **Classical regression PASSES.** `use_perceived_position:=false`, all new
  options at defaults: `result=SUCCESS`, `position_source=configured`, full
  lifecycle including transport, `tcp_error_m=6.25e-07`, object displacement
  349.095 mm (= the pick-and-place itself: 0.200 − (−0.150) = 0.350 m). The
  signature matches historical `prod_reg` test C evidence
  (`TIMED_OUT_HELD`, `within_tolerance=0`, `preclose_achieved=0.227955`
  identical; achieved grip angle 0.6187 vs historical 0.6108/0.6140). **No
  regression attributable to the new code.**
- **Perception receipt at M1 works.** Point received in frame `world`, finite,
  identical to the Milestone E value.
- **The freshness gate genuinely fires** — `rejected_stale=1` in scenes A, B
  and C.
- **Perception genuinely drives the target.** Scene B logged
  `delta=[0.351274 -0.099071 -0.000000]` against the configured centre: the arm
  went where perception said, not where `scene.yaml` said.
- **The semantic conversion is correct in practice**: top `0.795000` →
  centre `0.772500` (= 0.795 − 0.045/2) in every scene.
- **DEFECT, mine, confirmed by log timestamps.** In strict mode with no object,
  the node correctly logged `PERCEPTION_TIMEOUT` and
  `position_source=perception_timeout` and did NOT fall back — but it then
  planned and executed a move to the CONFIGURED pre-grasp:
  `PERCEPTION_TIMEOUT` at wall `1787436946.819810`, `Execute request accepted`
  at `1787436946.849219`, `Execute request success!` at `1787436955.255497`.
  Root cause: Stage 1's `setPoseTarget`/`plan`/`execute` had no `ok(result)`
  guard, because before this change no failure could be raised inside that
  scope. This violates "NO PERCEPTION = NO MOTION".

### Remaining uncertainty

- That `perceived_position_timeout_s=2.0` is sufficient under heavier
  simulation load. It was ample in every run here (~10 chances at 5 Hz).
- Nothing beyond pre-grasp is established for perception-driven manipulation.

### Files modified this session

Production:
- `ur5e_pick_place/include/ur5e_pick_place/failure.hpp` — `PERCEPTION_TIMEOUT`.
- `ur5e_pick_place/src/m3_grasp.cpp` — F1 parameters, config validation, M1
  move, stationarity gate, relocated + freshness-gated perception block, strict
  mode, `pregrasp_only` verification and stage guards, `position_source` in CSV
  and summary, and the Stage-1 `may_move` guard (the fix).
- `ur5e_pick_place/launch/m3_grasp.launch.py` — new arguments and parameters.

Evaluation-only (new):
- `scripts/perception/milestone_f1_harness.py`
- `scripts/perception/milestone_f1_truth.py`

**Codex's reviewed semantic conversion was NOT altered.** The line
`sample->point.z - object_height_m / 2.0` is byte-identical to the reviewed
version; the substitution is still `setOrigin()` only, so the configured grasp
rotation is retained. Frozen components — `object_detector.cpp`,
`object_position_world.cpp`, camera XACRO/config, camera TF, detector
thresholds, D estimator, E transform, `transport.cpp`, gripper controller,
physics, object geometry — were not touched.

### Tests performed, and their results

| Test | Config | Result |
|---|---|---|
| Classical regression | `use_perceived_position:=false` | **PASS** — `SUCCESS`, `position_source=configured`, full lifecycle, matches historical evidence |
| Live M1 perception receipt | camera stack at M1 | **PASS** — `world` frame, finite, matches Milestone E |
| Scene A `(0.45,-0.15)` | strict + pregrasp_only | **PASS on guarded binary** |
| Scene B `(0.80,-0.25)` | strict + pregrasp_only | **PASS on guarded binary** |
| Scene C `(0.80, 0.25)` | strict + pregrasp_only | **PASS on guarded binary** |
| Scene D `(0.18,-0.22)` | strict + pregrasp_only | **PASS on guarded binary** |
| No-object strict mode | strict + pregrasp_only, no object | **PASS on guarded binary** — timeout/source correct, no post-timeout plan or execution, arm remained at M1 |
| Fallback mode (Part 15) | `require_perception:=false`, no object | **PASS** — `position_source=fallback_configured` |

Historical four-scene results on the pre-fix binary, superseded by the guarded
matrix above but retained as comparison, all `position_source=perceived`:

| Scene | perceived top world | perceived→truth pre-grasp | achieved TCP err | orientation err | object displacement |
|---|---|---:|---:|---:|---:|
| A | `0.450965, −0.148707, 0.795` | 1.6134 mm | 0.1527 mm | 0.054° | 0.0000 mm |
| B | `0.801274, −0.249071, 0.795` | 1.5767 mm | 0.1108 mm | 0.059° | 0.0000 mm |
| C | `0.801274, 0.250819, 0.795` | 1.5145 mm | 0.1645 mm | 0.057° | 0.0000 mm |
| D | `0.180755, −0.219154, 0.795` | 1.1339 mm | 0.2009 mm | 0.095° | 0.0000 mm |

All within the predeclared bounds (target ≤5 mm, achieved ≤10 mm, object ≤1 mm),
and the target errors are exactly Milestone E's errors propagated through the
unchanged classical geometry. In every scene `executed=no`,
`attempted_transport=no`, `gripper_result=N/A`, `preclose_result=N/A` — no
descent, no gripper command, no transport.

### Git state AS OF F1 — historical, superseded

Superseded by "Current Git state — measured 2026-08-23" above. The count of
eight below was correct at the end of F1; F2 subsequently modified
`config/scene.yaml` and
`ur5e_robotiq_description/urdf/vendor/robotiq_2f_85_macro.urdf.xacro`, bringing
the measured total to ten. Always re-measure with `git status --short` rather
than trusting any recorded count.

Branch `rgbd-perception`, HEAD `7b875a4` (= tag `m6-30mm-stable`), no upstream.
**Nothing committed, nothing pushed.** `git diff --check` clean.

Tracked modified (8, at F1): `ur5e_pick_place/CMakeLists.txt`,
`ur5e_pick_place/package.xml`, `ur5e_pick_place/include/ur5e_pick_place/failure.hpp`,
`ur5e_pick_place/src/m3_grasp.cpp`, `ur5e_pick_place/launch/m3_grasp.launch.py`,
`ur5e_robotiq_description/CMakeLists.txt`,
`ur5e_robotiq_description/launch/ur5e_robotiq_sim_control.launch.py`,
`ur5e_robotiq_description/urdf/ur5e_robotiq.urdf.xacro`.

Untracked kept: `AGENTS.md`, `CLAUDE.md`, `HANDOFF.md`, `PROJECT_STATE.md`,
`docs/HANDOFF_RGBD_PERCEPTION.md`, `scripts/perception/`,
`ur5e_pick_place/src/object_detector.cpp`,
`ur5e_pick_place/src/object_position_world.cpp`,
`ur5e_robotiq_description/worlds/`, `m3_grasp.csv`, and the 37 M8/M9/M10 and
`prod_reg` evidence entries. All simulation processes were shut down cleanly.

### Recommended next step AS OF F1 — historical, superseded

Both actions below were subsequently completed: the F1 checkpoint was written
into `docs/HANDOFF_RGBD_PERCEPTION.md` §10, and F2 was scoped, run and passed.

Write the detailed F1 checkpoint in `docs/HANDOFF_RGBD_PERCEPTION.md`, then
define any F2 grasp/contact scope separately. Do not infer perception-driven
grasp success from this pre-grasp-only PASS. Do not commit or push without
explicit approval.

## Important Existing Handoffs
Detailed milestone-specific notes may exist under:

- `docs/HANDOFF_M3.md`
- `docs/HANDOFF_RGBD_PERCEPTION.md`

Read the relevant detailed handoff when working on that area.

## Source of Truth
Use, in this order:

1. Current repository files
2. `git status`
3. relevant `git diff`
4. logs and recorded evidence
5. ROS / Gazebo measurable state
6. milestone-specific handoff documentation

Do not rely only on previous AI conversation claims.

## Before Starting Work
1. Read `AGENTS.md`.
2. Read this `HANDOFF.md`.
3. Inspect `git status`.
4. Inspect relevant existing diffs.
5. Read the milestone-specific handoff relevant to the task.
6. Do not overwrite unrelated existing work.

## Git Safety
Do not automatically:

- reset
- clean
- discard changes
- commit
- push

Ask for explicit approval before committing or pushing.

## Current Development Direction
RGB-D perception is validated through Milestone E (world-frame position).
The next direction is perception-driven manipulation: replacing the classical
pipeline's hardcoded object location with the perceived one.

Before starting another implementation change, establish the current verified
state from repository evidence. For milestones A-E specifically, that state is
recorded in `docs/HANDOFF_RGBD_PERCEPTION.md` — read it rather than re-running
the experiments.

## Validation Principle
A simulation that appears to work is not considered verified until the relevant physical and software behavior is supported by evidence.

## End-of-Task Handoff
At the end of a significant task, report:

### Objective
What was being investigated or implemented.

### Findings
Confirmed root cause, evidence, or important observations.

### Files Inspected
Relevant files reviewed.

### Files Modified
Files changed during the task.

### Validation
Commands, tests, simulations, or evidence actually used.

### Result
What is confirmed.

### Remaining Uncertainty
Anything not yet verified.

### Git Status
Relevant modified/untracked files.

### Recommended Next Step
The smallest useful next action.

---

## 2026-08-23 — Milestone F2 stopped on Scene B (FAIL)

### Objective and Changes

Added an explicit perception-derived approach-and-grasp stop mode which reuses
the classical pre-close, Cartesian descent, and direct-effort close/hold path,
but never calls lift/transport/place/release.

- `m3_grasp.cpp`: added `grasp_only` (default false), an explicit
  `pregrasp_only && grasp_only` `CONFIG_ERROR`, a stop immediately before
  `lift_transport_place()`, frozen perceived-target logs, and structured
  stage/no-lift CSV and run-summary evidence.
- `m3_grasp.launch.py`: exposed and passed `grasp_only:=false`.
- `docs/HANDOFF_RGBD_PERCEPTION.md`: added the detailed accepted F1 section.
- Frozen perception, geometry, gripper, controller, physics, and transport
  components were not modified for F2.

### Validation and Evidence

- Python launch syntax: PASS.
- Targeted `ur5e_pick_place` build: PASS; only the pre-existing warning in
  `gz_topic_utils.hpp`.
- Classical regression C, all new modes false: full cycle `SUCCESS`,
  `position_source=configured`, transport `SUCCESS`, post-run gripper health
  gate OK. Evidence suffix: `20260823_004208_6512`.
- Scene A strict F2: perceived top `[0.450965,-0.148707,0.795000]`, object
  reference/grasp target `[0.450965,-0.148707,0.772500]`; truth delta
  `[+0.965,+1.293,0.000]` mm, Euclidean `1.6134 mm`. Pre-grasp/descent passed;
  achieved grasp TCP error `0.00047 mm`; close `TIMED_OUT_HELD` at
  `0.616788 rad`; existing contact streams showed bilateral `pick_target`
  contact and the final object sample was settled between the contact sides.
  Lift/transport/place-release were explicitly not attempted. Warning: closure
  seated the object `21.5773 mm` (`[-21.3642,-0.0121,+3.0253] mm`) and tilted
  it about Y. It did not drop, but this is a material anomaly.
- Scene B strict F2: fresh perceived target frozen; truth error `1.5767 mm`.
  Pre-grasp passed, but the unchanged free-air pre-close returned
  `TIMED_OUT_HELD` at `0.313260 rad` rather than `REACHED_GOAL` (target
  `0.2379 rad`). The existing guard returned `GRIPPER_GOAL_REJECTED` before
  descent. Final close/lift/transport/place/release were not attempted and
  object displacement was `0.0000 mm`. Evidence: `/tmp/ur5e_f2_results/`.

### Result and Safest Next Action

Milestone F2 is FAIL because Scene B did not reach grasp. Per the frozen stop
discipline, Scenes C/D and the post-change strict no-object rerun were not run;
no tuning was attempted. The last strict no-object evidence remains the guarded
F1 PASS and is not claimed as a post-F2 regression.

Next, diagnose why the unchanged free-air pre-close intermittently reports
`TIMED_OUT_HELD` using existing action/joint telemetry. Do not tune perception,
geometry, effort, physics, or thresholds until that divergence is explained.

---

## 2026-08-23 — F2 Scene-B pre-close diagnosis

### Diagnostic objective

Explain why Scene B's free-air pre-close sampled `0.313260 rad` and returned
`TIMED_OUT_HELD` for a `0.2379 rad` target. Diagnostic only; no F2 continuation
or behavioral changes.

### Evidence inspected

- Current `gripper_close_and_hold()` and Stage 1.5 control flow.
- `controllers.yaml` action tolerances/stall configuration and the URDF joint
  velocity limit.
- Current classical regression, F2 A/B logs and CSVs, the F1 harness's
  sequential-run setup, and historical 30 mm production-regression logs.
- Existing object-pose/contact evidence and pre-grasp geometry.

### Proven root cause

The Scene-B failure was caused by **sequential gripper state plus a fixed
timeout shorter than the physically required reverse travel**, not contact.

1. Scene A's final close sampled `0.616788 rad`, then unconditionally issued a
   hold goal at `0.8 rad`. This goal intentionally remains driven.
2. The sequential F1/F2 harness removes/spawns only the object. It neither
   reopens the gripper nor resets/reactivates its controller between scenes.
3. Removing Scene A's object removes the obstruction, allowing the retained
   `0.8 rad` goal to close the joint to approximately `0.8 rad` before B's
   pre-close.
4. Scene B commands the unchanged `0.2379 rad` goal. The joint velocity limit
   is `0.1 rad/s`; entering the controller's `0.01 rad` goal tolerance from
   `0.8 rad` requires `(0.8 - 0.2479) / 0.1 = 5.521 s`, exceeding the helper's
   fixed `5.0 s` result wait.
5. After five seconds, velocity-limited motion predicts about `0.300 rad`.
   Gazebo sampled `0.313260 rad` (13.26 mrad difference, consistent with goal
   startup/sampling dynamics), while the joint was still travelling toward the
   correct target.
6. The helper labels any missing first result `TIMED_OUT_HELD`, samples Gazebo,
   then sends the same goal again. B logged its result 5.860 s after pre-grasp
   execution completed: the extra ~0.86 s is consistent with completing the
   remaining ~0.65 s of travel during that second send. The helper retains the
   first-send `TIMED_OUT_HELD` classification even if the hold send succeeds.

Therefore `0.313260 rad` is an in-flight timeout sample, not an equilibrium or
a controller-reported stall. There was no goal/target mismatch.

### Hypotheses ruled out

- Object contact: B remained at the pre-grasp pose, 100 mm above the object;
  the object stayed exactly stationary, and descent never began.
- Table contact or planned collision: the pre-grasp execution completed and
  MoveIt accepted the collision-checked pose; the timeout angle matches the
  velocity/time calculation rather than a geometric stop.
- Stale/incorrect Gazebo position feedback: the sampled value is quantitatively
  consistent with five seconds of travel from the retained closed state.
- Effort equilibrium/saturation: the joint continued in the commanded opening
  direction and completed the residual travel during the second send.
- Wrong target/action goal: code and logs both show `0.2379 rad`, `50` max
  action effort, and the correct master joint/action server.
- Perception or Scene-B geometry: pre-close is independent of perceived XYZ
  after the arm reaches the free-air pose.

### Repeatability and historical comparison

Ten located historical 30 mm clean-start runs reached the same `0.2379 rad`
target and returned `REACHED_GOAL` near `0.2280 rad`; the current classical
regression and F2 Scene A did likewise. Scene B is prior-state-dependent, not
evidence of a deterministic free-air obstruction. No live reproduction was
needed because the existing sequential run supplies the discriminating sample
and the numbers close against the configured velocity and timeout.

### Relationship to Scene-A seating anomaly

**Clearly related at the control-mechanism level.** Scene A's 21.5773 mm
seating occurred while the same hold-at-`0.8` behavior continued driving into
contact. Once the object was removed, that retained command closed the now-free
gripper fully and created Scene B's long reverse-travel starting state. The B
failure itself was not physical contact and does not explain all of A's contact
geometry, but both consequences originate from the persistent close/hold goal.

### Smallest recommended action

Before resuming F2, propose (do not silently implement) an explicit,
evidence-checked gripper initialization between independent scene trials: send
the existing open command, wait for its result, verify Gazebo joint position
and controller responsiveness, then begin M1. This restores equivalent initial
state without changing controller, effort, PID, timeout, or grasp behavior.
Separately, improve diagnostic naming/logging so a first-send timeout followed
by a successful second send is not mistaken for a physical stall.

---

## 2026-08-23 — deterministic F2 trial initialization and stopped revalidation

The evaluation harness now initializes each independent trial immediately
after removing the previous object and before spawning the next object or
launching `m3_grasp`. It reads the Gazebo master joint, verifies the controller
and action, sends the canonical `0.0 rad` / max-effort-50 open goal, requires
`reached_goal=true` and `stalled=false`, then verifies Gazebo within the existing
0.01 rad tolerance. Failure freezes `TRIAL_INIT_*` JSON evidence and exits
before M1 or arm motion. A harness-only `--init-only` mode was added.

Initialization-only validation deliberately established the existing closed
target first. The corrected test passed `0.797012 -> 0.003787 rad`, with
`REACHED_GOAL` and a responsive controller/action; no MoveIt, perception,
object, or arm motion ran. An initial parser attempt failed closed with
`GAZEBO_START_POSITION_UNAVAILABLE`; the instrumentation was mechanically
corrected to read Gazebo's `axis1.position` layout.

Strict initialized F2 evidence (`/tmp/ur5e_f2_initialized_results/`):

- Scene A: init `0.003754 -> 0.003754 rad`; target error `1.6134 mm`;
  pre-close `REACHED_GOAL` at `0.227935`; descent fraction 1.0; grasp TCP error
  `0.000294 mm`; final close `TIMED_OUT_HELD` at `0.618012`; bilateral contact;
  approach/descent disturbance `0.2528 mm`; final settled displacement
  `21.5885 mm`; no lift/transport/place. Scene A passes, but seating persists
  versus the prior `21.5773 mm` (+0.0112 mm).
- Scene B: init `0.797112 -> 0.003954 rad`, directly validating sequential
  reset. Its former blocker disappeared: pre-close `REACHED_GOAL` at
  `0.227938`. Target error `1.5767 mm`; descent fraction 1.0; grasp TCP error
  `0.000339 mm`; final close `TIMED_OUT_HELD` at `0.598100`; bilateral contact;
  no lift/transport/place. **Scene B nevertheless FAILS:** object translation
  after descent and before final closure was `1.6211 mm`, above the frozen
  `<=1 mm` limit. Final settled displacement was `21.6596 mm`.

C/D and strict no-object were not run. No tuning occurred. F2 remains FAIL.
The passed post-F2-code classical regression remains valid because production
manipulation code was untouched; only the evaluation harness changed.

Next: diagnose Scene B's approach/descent disturbance using the captured phase
pose and existing contact logs, without tuning initialization, perception,
geometry, controllers, physics, or thresholds.

---

## 2026-08-23 — Scene-B pre-close disturbance diagnosis

### Objective and evidence inspected

Diagnostic only: explain the initialized Scene-B object's reported `1.6211 mm`
motion before final close. Inspected the raw F2 B node log/CSV/JSON, both raw
500 Hz fingertip-contact streams, the asynchronous phase-capture code, the
actual Robotiq collision Xacro/meshes as represented by
`gripper_geometry.py`, Scene A as a control, and the historical position-only
descent capture in `docs/HANDOFF_M3.md`. No simulation or tuning was performed.

### Phase-by-phase motion

All positions below are Gazebo world positions in metres:

- P0 settled: `[0.800000000, -0.250000000, 0.772500000]`
- P1 after pre-grasp: `[0.800000000, -0.250000000, 0.772499999949]`
- P2 after descent / before close trigger: `[0.801621091470,
  -0.249990256780, 0.772499937010]`
- P3 after final close/settling: `[0.778557363, -0.249966325, 0.775557769]`

P1-P0 was effectively zero. P2-P1 was `[+1.621091, +0.009743,
-0.000063] mm`, Euclidean `1.621121 mm`: the failed value is almost purely
world +X and arose between pre-grasp and the post-descent sample, not from
table settling. P3-P2 was `[-23.063728, +0.023932, +3.057832] mm`, Euclidean
`23.265564 mm`; P3-P0 was `[-21.442637, +0.033675, +3.057769] mm`, Euclidean
`21.659588 mm`. The latter two belong to final closure and are not the F2
approach criterion under diagnosis.

### Timing, contact, and capture audit

The ordered node evidence is: perception frozen at sim `104.800`; pre-grasp
execution completes at wall `1787439708.417436`; pre-close completes at
`1787439710.879587`; descent starts at `1787439710.882239`, executes from
`1787439710.882600` to `1787439712.637445`, and logs success at
`1787439712.638105`; stage-2 ground truth completes at `1787439713.754982`;
final close then begins; F2 stops at `1787439723.878873`.

The harness polls every 0.5 s for `execution reported SUCCESS`, synchronously
queries one Gazebo pose, and production intentionally waits 1.0 s after that
message before its stage-2 ground-truth query and final close. Thus P2 was
triggered in the intended post-descent/pre-close window, although the harness
did not retain the pose message's own stamp or query wall timestamps. This is
strong evidence, not a timestamp-complete proof of the boundary.

The first recorded object contact on the left fingertip sensor was sim
`120.432`; the right followed at `120.466`. Correlation with detector
camera stamps maps sim `120.4` to approximately wall `1787439717.093`, roughly
3.34 s after stage-2 verification/final-close start. Both collision names are
the fixed-joint-lumped fingertip collision against `pick_target::link::c`.
Therefore neither fingertip sensor records contact before or during descent;
left fingertip contacts first during final close. These sensors do not observe
finger, inner/outer-knuckle, base, or wrist collisions.

### Geometry and diagnosis

At the achieved pre-close angle `0.227938 rad`, with the actual fixed-tip
design angle `0.402893 rad`, the collision-mesh-derived pad aperture varies
from `68.756` to `81.986 mm` over the pad's vertical extent (`75.217 mm` at
its centroid). Against the actual 45 mm closing-axis object dimension, nominal
clearance is `11.878` to `18.493 mm` per side (`15.108 mm` nominal). A
fingertip side-face collision is therefore not expected from static aperture
alone, even including the `1.5767 mm` perception offset.

The motion is almost entirely along world +X, the configured closing axis for
the frozen roll, rather than Z; P0/P1 stability rules out spawn/table settling.
The first fingertip contacts occur only well into final close, yet P2 already
contains the shift. Together with the project's prior independent 0.2 s
trajectory capture—which directly established object motion during Cartesian
descent—this **strongly supports real physical contact during descent by a
non-fingertip collision shape**. Candidate shapes are the finger/knuckle
assembly; the 45 mm-height configuration has approximately 27.5 mm nominal
inner-knuckle/top clearance, making the inner knuckle less likely, but the
current run recorded no global/non-fingertip contact stream and cannot prove
which exact link touched first. Static aperture does not account for the full
3-D swept mesh or dynamic contact deflection, so it cannot name the link.

Scene A used identical orientation, pre-close angle, target semantics and
descent, but its target bias and resulting approach occurred at a different
arm configuration and produced only `+0.252618 mm` X motion (`0.2528 mm`
total). The approximately 6.4x A/B difference is therefore consistent with a
configuration-dependent dynamic contact/swept-volume event, not a changed
command or static object property. Scene A's final seating was not diagnosed.

### Classification and next action

Root cause classification: **STRONGLY SUPPORTED** — real, closing-axis physical
disturbance during descent from an unsensored gripper collision. Ruled out:
table settling, pre-pregrasp motion, Z-dominant settling, perception changing
during execution, final fingertip closure as the source of P2-P1, and a simple
insufficient static fingertip aperture. Exact first non-fingertip collision
link and the phase sample's precise sim timestamp remain unknown.

Smallest next experiment: one unchanged initialized Scene-B F2 run with a
synchronized high-rate trace of object pose, TCP/link poses, gripper joint
state, explicit phase timestamps, and contact reporting for every nearby
Robotiq collision (not just fingertip sensors). Stop at F2. Do not tune before
that trace identifies the first colliding geometry.

---

## 2026-08-23 — synchronized Scene-B collision diagnosis: PROVEN

### Instrumentation and physical isolation

The world Contact system exposed only the two existing fingertip topics, not a
global contact stream. Added opt-in, evaluation-only contact observers for the
left/right outer-knuckle, fixed-joint-lumped finger, and inner-knuckle collision
meshes. `enable_diagnostic_contacts` defaults false. The conditional blocks add
only `<sensor type="contact">` elements: no geometry, surface, mass, inertia,
joint, controller, or physics field changes. Default Xacro expansion contains
zero `diag_left`/`diag_right` tokens; enabled expansion contains all six.

Collision inventory: each outer-knuckle link has its own mesh collision and is
a revolute child of `robotiq_85_base_link`; each finger mesh is a fixed child
and Gazebo lumps it into its outer-knuckle link; each fingertip mesh is another
fixed child/lump and retains its existing contact sensor; each inner-knuckle
mesh is a separately simulated continuous/mimic child of the gripper base. The
Robotiq base/palm mesh is fixed through the adapter and may be lumped toward
`wrist_3_link`; it had no pre-existing contact sensor. All eight distal
collision candidates were observed in this run. The palm/adapter is well above
the proven first-contact location and cannot pre-empt the timestamped distal
contact.

Raw synchronized evidence is under `/tmp/f2_scene_b_sync/`: Gazebo pose/info,
master joint-state, all eight contact streams, m3 log/CSV, and harness JSON.
Pose and contact records carry direct simulation timestamps; joint state is at
1 kHz. Production manipulation was not modified.

### Reproduction and exact first event

One unchanged initialized Scene-B `grasp_only` run reproduced the failure.
Initialization passed `0.003954 -> 0.003787 rad`; perception, pre-grasp,
pre-close (`REACHED_GOAL`, `0.227937 rad`), Cartesian fraction 1.0, descent,
F2 stop, and no-lift/transport/place evidence all remained intact.

The exact first robot/object collision was at sim `117.195000 s`:

```
collision1 = ur5e_robotiq::robotiq_85_right_inner_knuckle_link::
             robotiq_85_right_inner_knuckle_link_collision
collision2 = pick_target::link::c
```

First contact points were at approximately `[0.785004, -0.244709,
0.794994]` and `[0.785003, -0.237522, 0.794996] m`, on the object's world −X
top edge. Contact normals had a strong `+X` component (`~+0.60`) and downward
Z component (`~-0.80`). The left inner knuckle did not contact until sim
`118.955`; fingertips began only at `121.457` left and `121.463` right. The
finger and outer-knuckle observer streams stayed empty.

At the nearest pose/joint sample, sim `117.196 s`:

- object `[0.800092298, -0.250000020, 0.772499993] m`, already
  `[+0.092298,-0.000020,-0.000007] mm` from its stationary baseline;
- reconstructed TCP `[0.801293021,-0.249071775,0.776978304] m`, orientation
  quaternion `[0.9999999998,-0.0000092232,0.0000110349,-0.0000133390]`
  (`x,y,z,w`);
- right-inner-knuckle link `[0.788591265,-0.249073663,0.856536024] m`,
  quaternion `[0.957545436,-0.000012677,0.288282391,-0.000010114]`;
- master angle `0.235739 rad`, velocity `+0.100000 rad/s`. Adjacent 1 ms
  samples alternate approximately `+/-0.1 rad/s` under contact. Gazebo's
  joint-state message did not publish master effort, so effort is unavailable;
  the unchanged action max effort was 50.

The stationary/noise floor was below 0.001 mm. First contact is
`117.195 s`; the first available pose sample exceeding 0.001, 0.01, and 0.05
mm alike is `117.196 s` (`0.092298 mm`). Thus motion follows contact by 1 ms,
within the pose/contact measurement resolution. By `0.1 mm` threshold the
object was `0.759907 mm` displaced at `117.213 s`.

The new harness P2-P1 was `[+1.434478,-0.003538,-0.000031] mm`, Euclidean
`1.434482 mm`, versus prior `[+1.621091,+0.009743,-0.000063] mm`,
`1.621121 mm`. Same sign, dominant axis, phase, first geometry, and frozen
criterion failure; reproducibility is HIGH (X magnitude differs 0.187 mm,
11.5%).

### Proven mechanism and continuation

Mechanism **C — inner-knuckle collision**, root cause **PROVEN**. During the
last approximately 4.48 mm of descent, the right inner-knuckle collision's
sloped lower surface reaches the object's −X/top edge. Its oblique contact
normal has a strong closing-axis component and wedges the object toward world
+X. This explains why large nominal fingertip aperture is irrelevant: the
first contact is a different, more proximal mesh.

F2 remains FAIL. No correction was implemented. Smallest candidate correction
for a separately authorized experiment: alter the collision-free approach
geometry so the right inner knuckle clears the top edge while preserving the
same final classical grasp target; compare that against changing pre-close
configuration only after geometric analysis. Do not select or implement either
without a controlled proposal.

---

## 2026-08-23 — Scene-B collision-free approach design (analysis only)

Used the actual binary STL collision meshes, the synchronized Scene-B link
transforms, and the URDF joint kinematics. No Gazebo run, production edit,
parameter tuning, or F2 rerun was performed. Dense triangle-surface sampling
was used for clearance; the proven contact sensor supplies the penetration
cross-check.

### Current geometry

At the proven configuration (`master=0.235739 rad`) the theoretical first
right-inner-knuckle/top-edge touch occurs at TCP Z `0.777059 m`, only
`0.081 mm` above the measured contact-sample TCP Z `0.776978 m`. This agreement
is within the pose/surface discretization and 1 ms sampling. The contact sensor
reported up to `0.014 mm` depth; the static sampled mesh at the next pose frame
is approximately `0.081 mm` below the top plane and `0.110 mm` into the box's
X extent.

During the final 10 mm of descent, predicted right-inner-knuckle clearance is
approximately: 4.9 mm at TCP 10 mm above final, 3.0 mm at +8 mm, 1.2 mm at
+6 mm, zero at +4.559 mm, then interference. At the unchanged final TCP pose,
the current pre-close geometry has about `2.089 mm` right-side interference;
the left inner knuckle has only `2.164 mm` clearance.

### Candidate comparison

- **A, lateral offset:** shifting the gripper toward world −X increases right
  clearance, but reduces left clearance equally. At the final Z, a zero-margin
  right clearance needs 2.089 mm shift and leaves only 0.075 mm left; equalized
  maximum bilateral clearance is about 0.038 mm. For right margins 1/2/3/5 mm,
  shifts 3.089/4.089/5.089/7.089 mm would produce left interference of about
  0.925/1.925/2.925/4.925 mm. Converging to the unchanged final pose recreates
  the original interference. A alone is not viable.
- **B, two-stage descent/lateral convergence:** it can postpone contact but
  cannot reach the unchanged final TCP at the current angle collision-free;
  the endpoint itself interferes. Viable only when combined with C, so it is
  larger and riskier than C alone.
- **C, more-open descent configuration:** actual right-inner mesh prediction
  at the unchanged final TCP gives maximum master angles of approximately
  `0.174`, `0.156`, `0.138`, and `0.105 rad` for 1/2/3/5 mm right-side margins.
  At those angles left-inner clearance is approximately 5.25/6.20/7.16/8.96
  mm. At `0.130 rad`, the proposed robust point, predicted clearance is about
  3.5 mm right and 7.6 mm left. Finger, fingertip, and outer-knuckle minimum
  clearances at the final pose are all above 24 mm (outer knuckles above
  57 mm), so no new distal collision is predicted. Final close still drives
  from this open state to the existing production target/contact behavior.
- **D, open only late in descent:** geometrically possible, but adds
  synchronized gripper/arm motion and a new transition boundary. It changes
  more control flow than selecting the already-existing pre-close target and
  has higher regression risk. No smaller robust lateral-only solution exists.

### Ranking and exact next experiment

Rank: C first; D second; B third (only with C); A fourth/infeasible alone.

Recommend exactly one controlled experiment: retain the same perceived object
target, pre-grasp, straight vertical Cartesian descent, final TCP XYZ and
orientation, then use `0.130 rad` as the descent pre-close target instead of
`0.2379 rad`. This is an opening change of approximately `0.108 rad` and
predicts 3.5 mm right-inner and 7.6 mm left-inner clearance at the worst/final
descent pose. No lateral offset or intermediate waypoint. Final close remains
the existing direct-effort close toward the unchanged classical grasp
configuration.

This proposal is grasp-frame-relative and therefore applies unchanged to A-D;
it is not a Scene-B world-coordinate patch. The worst validated perception
coordinate bias (1.274 mm) still leaves over approximately 2.2 mm of the
predicted 3.5 mm limiting-side margin. It also preserves a clean classical
policy primitive for later PPO comparison. This is analysis only and is not a
validated correction; F2 remains FAIL.

---

## 2026-08-23 — One controlled Scene-B pre-close correction trial

Exactly one authorized manipulation value changed: `grasp.preclose_margin_rad`
in `config/scene.yaml` changed from `0.30` to `0.4078679450464813`. With the
unchanged Scene-B expected grip angle `0.5378679450464813 rad` and unchanged
`max(0, expected - margin)` calculation, this commands the authorized descent
pre-close target of exactly `0.130 rad`. No motion geometry, final target,
controller, physics, perception, initialization, or acceptance value changed.
`ur5e_pick_place` built successfully and the pre-run diff was the one scalar.

The initialized Scene-B grasp-only trial passed its initialization boundary
(`0.003754 -> 0.003854 rad`, `REACHED_GOAL`, controller responsive), froze a
fresh perceived target (`position_source=perceived`), and reached pre-close as
`REACHED_GOAL` at `0.120134 rad` (inside the existing 0.01 rad tolerance).
Cartesian descent was 1.0 and the unchanged final grasp TCP target/achieved
translation was approximately
`[0.801273610,-0.249070619,0.772499862] ->
[0.801273604,-0.249070616,0.772500551] m`, error `0.00068975 mm`.
The synchronized final wrist/tool orientation was effectively the commanded
`[x=1,y=0,z=0,w=0]` (sub-microdegree numerical error).

Phase capture gave identical P1 and P2:

- P1 after pre-grasp: `[0.800000000,-0.250000000,0.772499999949] m`;
- P2 after descent/before close: the same position;
- P2-P1: `[0,0,0] mm`, total `0.000000 mm`.

All eight monitored Robotiq collision streams were empty throughout descent.
The first right-inner contact was only during final closure at sim `114.211 s`
(master joint about `0.171068 rad`), followed by left-inner at `115.238 s`,
right fingertip at `117.674 s`, and left fingertip at `118.001 s`. Left/right
finger and outer-knuckle streams remained empty for the whole trial. Thus the
previous premature right-inner descent contact is absent. There is no direct
range sensor, so minimum clearance is not measured metrically; synchronized
no-contact evidence is consistent with the precomputed STL estimate of about
`3.5 mm` at the `0.130 rad` command.

The existing final close/hold returned `TIMED_OUT_HELD` at `0.520668 rad`, was
within the unchanged grasp tolerance, and established bilateral fingertip
contact. The object then seated by `21.6523 mm` overall, an existing separate
grasp-quality issue outside this correction experiment. `F2 STOP` was reached;
lift, transport, place, and release attempt flags were all false.

**Scene-B corrected F2 trial PASS.** This is not an overall F2 PASS. A-D and
strict no-object/general regression remain pending separate authorization; no
other scene or regression was run in this task.

---

## 2026-08-23 — F2 correction generalization: Scene A FAIL / stopped

The frozen `0.130 rad` commanded pre-close correction was retained exactly
(`preclose_margin_rad=0.4078679450464813`). Both affected packages rebuilt and
`git diff --check` passed; no production/configuration edits were made for this
validation.

Per the stop-on-first-failure protocol, only Scene A was run. Initialization
passed (`0.003754 -> 0.003787 rad`, controller responsive). Fresh perception
was frozen as top-surface world
`[0.450965037,-0.148706724,0.794999921] m` against truth
`[0.450000000,-0.150000000,0.795000000] m` (1.6136 mm error), with
`position_source=perceived`. Pre-close commanded `0.130000 rad` and returned
`REACHED_GOAL` at `0.120140 rad`. Cartesian fraction was 1.0 and final grasp
TCP translation error was `0.000687447 mm`.

The frozen phase samples were:

- P1 after pre-grasp: `[0.450000000,-0.150000000,0.772499999949] m`;
- P2 after descent/before final close:
  `[0.451002468740,-0.150125420169,0.772499988119] m`;
- P2-P1: `[+1.002469,-0.125420,-0.000012] mm`;
- Euclidean approach disturbance: **`1.010284 mm`**, exceeding the frozen
  `<=1.0 mm` maximum.

Synchronized contacts identify a premature collision rather than pose noise:
the right-inner-knuckle collision first contacted `pick_target` at sim
`71.634 s`, while the master joint remained at the pre-close configuration.
Final closure did not begin until approximately sim `72.99 s`. Left inner
contact (`74.490 s`) and bilateral fingertip contact (`77.070/77.075 s`) were
during final closure; finger and outer-knuckle streams remained empty.

The final close returned `TIMED_OUT_HELD` at `0.558695 rad`, within the
unchanged grasp tolerance, and the F2 stop flags proved no lift, transport,
place, or release. Those later successes do not override the approach failure.

**MILESTONE F2 — FAIL.** Scene A is the first failing evidence, so Scenes C/D,
strict no-object, and classical regression were intentionally not run. The
corrected Scene-B PASS remains valid but the `0.130 rad` correction has not
generalized within the frozen A-D criterion. No tuning or redesign was
performed. Recommended next step: separately diagnose the Scene-A right-inner
contact geometry at the unchanged `0.130 rad` pre-close before proposing any
new correction.

---

## 2026-08-23 — Scene-A/B real-mesh diagnosis and next-command design

Measurement/static analysis only. Gazebo was not launched and the live
`0.130 rad` configuration was not changed. Inputs were the preserved 1 ms
joint/contact traces, pose traces, actual Robotiq collision STLs, and URDF
kinematics.

### Scene-A first contact

The contact message is exact at sim `71.634 s`. It identifies
`robotiq_85_right_inner_knuckle_link_collision` against
`pick_target::link::c`, with contact points
`[0.434998592,-0.160372804,0.795001248]`,
`[0.435002850,-0.142606077,0.794996885]`, and
`[0.435001528,-0.137172804,0.794997985] m`. Normals were approximately
`[+0.747,0,-0.665]`, `[+0.676,0,-0.737]`, and `[+0.605,0,-0.796]`; maximum
reported initial penetration was `0.012111 mm`.

Gazebo pose messages bracket the event at `71.626/71.642 s`, so there is no
pose message stamped exactly `71.634`. The last pre-contact pose has the object
at `[0.450000000,-0.150000000,0.772500000] m` and real-STL clearance
`0.127790 mm`. Midpoint interpolation gives TCP
`[0.450987840,-0.148722800,0.776473289] m`, orientation approximately
`[0.9999999997,-0.000016580,-0.000013514,-0.000012798]` (`x,y,z,w`,
`0.002856 deg` error), and about `3.973 mm` descent remaining. Interpolated
right-inner link pose is
`[0.438289986,-0.148724415,0.856031632] m`, quaternion
`[0.958503691,-0.000019540,0.285080119,-0.000007540]`.

The master sample at the exact contact timestamp was `0.127938649 rad` at
`+0.100000001 rad/s`; master effort is unavailable from the Gazebo joint-state
message (action max effort remained 50). Crucially, the actual right-inner
follower joint was `-0.578377959 rad`, not `-0.12794 rad`, and was moving at
`+0.100000023 rad/s` (relaxing/opening). Left inner was `+0.429284526 rad`;
right outer was `-0.147062226 rad`.

### Scene B at the equivalent descent height

At sim `112.537 s`, with `3.944 mm` descent remaining, Scene B had TCP
`[0.801284320,-0.249084580,0.776443962] m`, right-inner link
`[0.788585710,-0.249082330,0.856002180] m`, master `0.127867549 rad`, and
actual right-inner follower `-0.520609564 rad`. Exact STL-to-box clearance was
`2.565088 mm` right and `9.135236 mm` left. At B's final pre-close sample
(`113.743 s`) the follower had relaxed further to `-0.451405601 rad`; minimum
final clearance was `5.013598 mm` right and `8.171369 mm` left.

### Why A contacts and B clears

With ideal rigid/mimic kinematics, identical object geometry, final orientation,
and grasp-relative target, the geometry is translation invariant. Perception
does not explain the observed direction: world top-position errors were A
`[+0.965037,+1.293276,-0.000079] mm` and B
`[+1.273610,+0.929381,-0.000138] mm`; in the grasp frame
(`R=diag(1,-1,-1)`) these are A
`[+0.965037,-1.293276,+0.000079] mm` and B
`[+1.273610,-0.929381,+0.000138] mm`. B actually has the larger +closing-axis
target error.

The measured non-invariance is follower configuration. At equivalent descent
height A's right inner was about `0.05777 rad` more closed than B's; at the
final comparison its worst measured residual is about `0.12697 rad` larger.
The sloped STL surface converts that rotation into millimetres of top-edge
clearance. TCP orientation/execution differences are only millidegrees and
sub-millimetre and are secondary. The right-inner histories show both joints
still dynamically relaxing rather than matching the master mimic target.
Therefore master angle alone is not a deterministic proxy for actual
right-inner geometry in these traces.

### Conditional real-STL command sweep

The sweep preserves the worst measured Scene-A follower residual and shifts
all gripper joints 1:1 with each candidate master-command change. It evaluates
the full straight descent; the final pose is the minimum-clearance point once
the mesh is collision-free. `contact` means triangle/AABB intersection, not a
rounded zero-distance pass.

| command rad | A right mm | A left mm | B right mm | B left mm |
|---:|---:|---:|---:|---:|
| 0.130 | contact | 8.455 | 5.014 | 8.171 |
| 0.125 | contact | 8.731 | 5.294 | 8.448 |
| 0.120 | contact | 9.006 | 5.574 | 8.723 |
| 0.115 | contact | 9.282 | 5.854 | 8.999 |
| 0.110 | contact | 9.556 | 6.133 | 9.274 |
| 0.100 | contact | 10.105 | 6.691 | 9.823 |
| 0.090 | 0.354 | 10.652 | 7.248 | 10.371 |
| 0.085 | 0.637 | 10.924 | 7.526 | 10.644 |
| 0.080 | 0.921 | 11.197 | 7.804 | 10.916 |
| 0.075 | 1.205 | 11.469 | 8.081 | 11.189 |
| 0.070 | 1.488 | 11.740 | 8.358 | 11.461 |
| 0.065 | 1.771 | 12.011 | 8.634 | 11.732 |
| 0.060 | 2.054 | 12.282 | 8.910 | 12.003 |
| 0.055 | 2.337 | 12.552 | 9.186 | 12.274 |
| 0.050 | 2.620 | 12.822 | 9.461 | 12.544 |
| 0.045 | 2.902 | 13.091 | 9.736 | 12.813 |
| 0.040 | 3.184 | 13.360 | 10.011 | 13.082 |

At the recommended `0.070 rad`, exact minimum distances in the worst A model
are: right inner `1.488`, left inner `11.740`, right/left fingertip
`1.527/5.095`, fingers `44.787/47.755`, outer knuckles `57.625/58.008`, and
base `28.478 mm`. Thus opening farther does not create an opposite-side or
proximal collision in the actual meshes; the right inner remains limiting.

### Perception envelope, margins, and close timing

The observed A-D grasp-frame lateral envelope is closing-axis X
`+0.755..+1.274 mm` and orthogonal Y `-1.293..-0.819 mm` (largest lateral norm
`1.6136 mm`; no statistical confidence claim). Re-evaluating the exact mesh at
each observed full XY error vector gives worst right-inner margins:

| command rad | nominal A mm | worst observed-error vector mm | margin class |
|---:|---:|---:|---|
| 0.080 | 0.921 | 0.618 | <1 mm |
| 0.075 | 1.205 | 0.901 | <1 mm |
| 0.070 | 1.488 | 1.185 | >=1 mm |
| 0.065 | 1.771 | 1.469 | >=1 mm |
| 0.060 | 2.054 | 1.752 | >=1 mm |
| 0.055 | 2.337 | 2.035 | >=2 mm |
| 0.050 | 2.620 | 2.318 | >=2 mm |
| 0.045 | 2.902 | 2.601 | >=2 mm |
| 0.040 | 3.184 | 2.883 | <3 mm |

Using the observed approximately `command-0.00986 rad` reached sample and the
frozen `0.1 rad/s` limit, expected-contact timing grows from about `4.177 s`
at `0.130` to `4.777 s` at `0.070` (+`0.600 s`). Commands
`0.065/0.060/0.055/0.050/0.045` predict about
`4.827/4.877/4.927/4.977/5.027 s`; therefore the 2 mm option at `0.055` has
only about 73 ms before the existing 5 s first-result boundary, and `0.045`
crosses it. Expected object contact remains reachable; target `0.8 rad` itself
is not expected in a grasp because contact/hold intervenes.

### Recommendation and limitation

Recommend exactly one next controlled value: **`0.070 rad`**, universally for
A-D, not scene-specific. It is the smallest tested change that clears the
worst measured A geometry by at least 1 mm after applying every observed A-D
perception-error vector, while keeping about 0.223 s before the 5 s result
boundary. Predicted nominal A/B right-inner clearances are `1.488/8.358 mm`;
worst observed-error-vector clearance is `1.185 mm`.

Confidence is high that the A/B difference is actual follower-state geometry,
not perception or world translation. Confidence in the `0.070` prediction is
conditional, not proven: the sweep assumes follower angles shift 1:1 with the
master-command delta, while the recorded follower histories demonstrably do
not track the master deterministically. The next experiment should therefore
be exactly one initialized Scene-A grasp-only run at `0.070 rad`, with the same
synchronized contact/pose evidence plus all four knuckle joint positions and
velocities. It must stop on contact/disturbance failure and must not proceed to
C/D or tune further.

---

## 2026-08-23 — Single Scene-A `0.070 rad` follower-validation trial

Exactly one authorized Scene-A grasp-only trial was run. The only behavior
change was `config/scene.yaml` `grasp.preclose_margin_rad`, from the prior
working value `0.4078679450464813` to `0.4678679450464813`. With the unchanged
30 mm expected grasp angle `0.5378679450464813 rad`, this produces exactly
`0.0700000000000000 rad`. The targeted `ur5e_pick_place` build and
`git diff --check` passed before execution.

Initialization passed (`0.003786969 -> 0.003786969 rad`, open target `0.0`),
the controller/action was responsive, M1 stationarity passed, and fresh
perception was used (`position_source=perceived`). Perceived top was
`[0.450965037,-0.148706724,0.794999921] m`; truth top was
`[0.450000000,-0.150000000,0.795000000] m`. The error vector was
`[+0.965037,+1.293276,-0.000079] mm`, norm `1.613648 mm`.

Pre-close returned `REACHED_GOAL` at master `0.060133924 rad` for the exact
`0.070000 rad` command. At the first sample meeting the action tolerance (sim
`56.538 s`) the directly measured joint positions/velocities were:

- left outer/master: `+0.060133924 rad`, `+0.100000001 rad/s`;
- right outer: `-0.209804969 rad`, `+0.029001224 rad/s`;
- left inner: `+0.369909958 rad`, `+0.100000000 rad/s`;
- right inner: `-0.647405182 rad`, `+0.100000000 rad/s`.

The followers were therefore still dynamically relaxing when the master
entered tolerance; the master result alone did not describe the collision
geometry. At the minimum-clearance sample near the end of descent (sim
`58.493 s`), right inner was `-0.526192201 rad` at `+0.100000000 rad/s`.
At descent completion (sim `58.678 s`) it was `-0.519798068 rad`, close to the
offline model's assumed `-0.518377959 rad` (difference `0.001420109 rad`).

The unchanged Cartesian descent achieved fraction `1.0000`. Frozen poses were:

- P1: `[0.450000000,-0.150000000,0.772499999949] m`;
- P2: `[0.450000000,-0.150000000,0.772499999949] m`;
- P2-P1: `[0,0,0] mm`, total `0.000000 mm`.

There was no monitored robot/object contact during descent. Final closure
began at approximately sim `59.627 s`; first right-inner contact was sim
`59.956 s`, left-inner `61.425 s`, right fingertip `64.170 s`, and left
fingertip `64.398 s`, all after final-close start. Finger and outer-knuckle
contact streams remained empty. Exact real-STL reconstruction over the actual
recorded descent poses gives minimum right-inner clearance `1.198288 mm` at
sim `58.493 s` and left-inner clearance `10.471042 mm`. The right-inner result
is `0.289712 mm` below the predicted `1.488 mm`, but positive and consistent
with the measured end-of-descent follower angle; the conditional follower/mesh
model is validated to useful accuracy for this trial.

The unchanged final TCP target was
`[0.450965037,-0.148706724,0.772499921] m`; reconstructed achieved TCP was
`[0.450965035,-0.148706722,0.772500217] m`, translation error
`0.000296 mm` and orientation error below the trace's displayed precision
(`0.000000 deg`). Final close began after descent and returned
`TIMED_OUT_HELD` at master `0.504874 rad`; the helper consumed its existing
5 s initial-result boundary and 5 s hold boundary (about `10.117 s` wall-clock
total), without changing timeout or velocity. Bilateral object contact was
established and the existing final-close semantics regarded the result as
successful. The known large Scene-A closure/seating motion persisted
(`21.6094 mm`) and remains a separate unresolved grasp-quality warning.

Structured evidence proves `f2_stop_reached=yes`, with
`lift_attempted=no`, `transport_attempted=no`, and
`place_release_attempted=no`. No fallback occurred.

**Scene-A `0.070 rad` controlled trial: PASS.** This validates `0.070 rad` as
a Scene-A candidate and validates the offline follower model at the
collision-critical end of descent; it does **not** establish overall F2 PASS.
No B/C/D, strict no-object, classical regression, or F3 work was run. Preserved
evidence is under `/tmp/f2_A_0070_results` and `/tmp/f2_A_0070_sync`.

Recommended next step: under separate authorization, run the unchanged
`0.070 rad` candidate through the remaining established F2 generalization
matrix, beginning with the already-defined next scene and stopping at the
first failure; retain direct follower/contact instrumentation. Do not tune the
value during that validation.

---

## 2026-08-23 — Milestone F2 `0.070 rad` generalization: COMPLETE / PASS

The frozen universal `0.070000 rad` descent pre-close candidate was validated
unchanged through B, C, and D after the accepted Scene-A run. Every scene used
verified per-trial opening, fresh strict perception, `position_source=perceived`,
the unchanged Cartesian descent/final-close behavior, and `grasp_only=true`.
No scene was retried or tuned.

| Scene | perceived top XYZ [m] | perception error | achieved pre-close [rad] | right-inner at descent end [rad] | minimum right/left inner clearance [mm] | fraction | P1->P2 [mm] | TCP error [mm] | final close | seating displacement [mm] | verdict |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---|
| A | `[0.450965037,-0.148706724,0.794999921]` | 1.613648 mm | 0.060133924 | -0.519798068 | 1.198288 / 10.471042 | 1.0 | `[0,0,0]` | 0.000296 | TIMED_OUT_HELD, 0.504874 | 21.6094 | PASS |
| B | `[0.801273610,-0.249070619,0.794999862]` | 1.5767 mm | 0.060137029 | -0.413621276 | 6.841447 / 7.142378 | 1.0 | `[0,0,0]` | 0.000339 | TIMED_OUT_HELD, 0.447886 | 21.6564 | PASS |
| C | `[0.801273610,0.250818749,0.794999862]` | 1.5141 mm | 0.0601319 | -0.399647024 | 7.680157 / 7.285271 | 1.0 | `[0,0,0]` | 0.000702 | TIMED_OUT_HELD, 0.502070 | 21.6191 | PASS |
| D | `[0.180754568,-0.219154460,0.794999862]` | 1.1333 mm | 0.0601414 | -0.482581096 | 3.602755 / 7.534626 | 1.0 | `[0,0,0]` | 0.000488 | TIMED_OUT_HELD, 0.453811 | 22.2618 | PASS |

Clearance timestamps and measured state at the right-inner minima were:

- B sim `49.081 s`: master `0.067985920 rad`, right inner
  `-0.420530181 rad` at `+0.100000000 rad/s`;
- C sim `120.984 s`: master `0.067769755 rad`, right inner
  `-0.404842245 rad` at `+0.100000000 rad/s`;
- D sim `204.146 s`: master `0.068010675 rad`, right inner
  `-0.486968949 rad` at `+0.043890068 rad/s`.

As in A, these measured follower states—not the master alone—were used with
the real collision STL. All monitored inner/finger/fingertip/outer-knuckle
contacts began after final closure started. Finger and outer-knuckle streams
remained empty; inner and bilateral fingertip contacts established the grasp.
All four structured summaries record `f2_stop_reached=yes` and
`lift_attempted=no transport_attempted=no place_release_attempted=no`.

The strict no-object regression passed on the current guarded binary:
`PERCEPTION_TIMEOUT`, `position_source=perception_timeout`, no fallback,
`NO_MOTION`, and no pre-grasp/descent/close/lift/transport/place. Planning and
execution messages preceding the timeout were exclusively the required M1
move; none occurred afterward.

One initialized classical full-cycle regression then passed with perception
disabled and both F2 stop modes false: `result=SUCCESS`,
`position_source=configured`, pre-close `REACHED_GOAL` at `0.0601339 rad`,
grasp `TIMED_OUT_HELD` at `0.507414 rad`, Cartesian fraction `1.0`, and
`transport_result=SUCCESS`. This confirms the universal pre-close correction
does not break the established classical lifecycle.

The approximately 21.6--22.3 mm final-closure seating motion appears in every
scene and remains an explicit, separate grasp-quality warning. It occurs after
the accepted disturbance interval and was not tuned or reclassified here.

**MILESTONE F2 — PERCEPTION-DERIVED GRASP PASS.** This establishes fresh
perception -> pre-grasp -> collision-free descent -> physical close/hold ->
grasp-only stop across A-D. It does not establish lift, transport, place, or
release from a perception-derived grasp. Evidence is preserved under
`/tmp/f2_A_0070_{results,sync}` and `/tmp/f2_0070_generalize/`.

Recommended next milestone, only under separate authorization: F3 should test
perception-derived grasp -> lift while retaining follower/contact/object-relative
pose evidence and treating the persistent closure-seating displacement as a
known warning. Do not infer transport or place from F2.

---

## 2026-08-23 — documentation consistency and evidence-durability audit

Documentation-only task. No manipulation, perception, controller, physics,
geometry, threshold, or acceptance value was changed; no simulation was
launched; no F2 rerun; nothing committed or pushed. No evidence artifact was
moved, renamed, deleted, or edited.

### Inconsistencies verified before editing, all four real

1. `PROJECT_STATE.md` "Immediate Next Step" asserted "F2 remains FAIL and
   validation is paused" (former line 176) and again "F2 remains FAIL" (194),
   then "Milestone F2 ... is PASS" (196). Stale text had been appended rather
   than replaced.
2. `HANDOFF.md` "Current Objective" (line 3) and "Current Status" (16) still
   described F1 as current. A new agent reading from line 1 would have
   concluded F2 was unstarted.
3. `HANDOFF.md` recorded "Tracked modified (8)"; the measured count is 10.
4. All `/tmp` F1/F2 evidence paths are gone. Confirmed by direct listing:
   `/tmp` contains no project directories.

### Corrections applied

- `PROJECT_STATE.md`: added a "Current Milestone Status" block naming F2 PASS
  as the latest verified milestone and F3 as next; updated the perception
  milestone summary from "opt-in consumer ... NOT yet validated" to the F1/F2
  PASS record; split the contradictory "Immediate Next Step" into "Current
  Verified State", a corrected "Immediate Next Step", and "Historical F2
  Failure and Root-Cause Investigation". No failure history was deleted — the
  Scene-B pre-close timeout, the inner-knuckle wedging mechanism, the failed
  `0.130 rad` candidate, and the follower-vs-master lesson are all retained
  and explicitly labelled historical.
- `HANDOFF.md`: replaced the top-level objective/status with the F2 PASS /
  F3 NOT STARTED state, the validated boundary, the frozen `0.070 rad`
  candidate, the A-D headline table, and the seven open warnings; added the
  measured Git state and this evidence audit; demoted the F1 material to
  "Historical record — Milestone F1 validation (superseded by F2)" with its
  Git-state and next-step subsections marked historical rather than removed.
- `config/scene.yaml`: comments only. Documented why
  `preclose_margin_rad` is `0.4678679450464813` (commands `0.070000 rad`),
  the inner-knuckle mechanism it corrects, the failed `0.130 rad` attempt, the
  follower-tracking caveat, and the ~0.223 s timeout headroom. Marked
  `grasp.squeeze` and `grasp.position_controlled` as stale under the
  direct-effort baseline. **No scalar was changed**; verified by diffing the
  value lines and by re-parsing the file (`preclose_margin_rad` unchanged at
  `0.4678679450464813`, `squeeze` `0.02`, `position_controlled` `true`,
  `max_effort` `50.0`). `position_controlled: true` is factually wrong for the
  current stack but is read by no code (verified by grep); it was left as-is
  because changing it is a value change, not a documentation fix.

### Durability decision

**Recommendation: REGENERATE DURABLE F2 EVIDENCE FIRST (Option B).**

Reason: zero durable raw evidence exists for the frozen `0.070 rad` candidate.
The one surviving F2-era raw run, suffix `20260823_004208_6512`, is the
classical regression at the OLD `0.2379 rad` pre-close; it proves the F2 code
additions did not regress the classical lifecycle and nothing more. Every
measurement that actually constitutes the F2 PASS — the A-D matrix, the
follower/contact traces the clearance figures were reconstructed from, the
strict no-object run, and the final classical regression under `0.070 rad` —
exists only as tables in this file and in
`docs/HANDOFF_RGBD_PERCEPTION.md` §11.3.

Supporting reasons:

- **Thesis/PPO baseline.** F2 is the classical policy primitive a later
  classical-vs-PPO comparison measures against. A baseline whose raw data
  cannot be produced on request is weak, and this is precisely the kind of
  result an examiner or reviewer asks to see behind.
- **The rerun is cheap and mechanical.** The harnesses survive
  (`scripts/perception/milestone_f1_{harness,truth}.py`, reused by F2 with
  runtime flags), the configuration is frozen, and the acceptance criteria are
  predeclared. It is execution, not diagnosis, and no tuning is permitted.
- **A non-reproduction would be critical to learn now, not during F3.** The
  project has confirmed history of simulation behaviour degrading over session
  runtime. If the frozen candidate does not reproduce from a clean start, that
  must surface before F3 builds on it.
- **The instrumentation overlaps F3 anyway.** F3 needs the same synchronized
  follower/contact/object-pose capture, so the rerun doubles as an F3 harness
  rehearsal.

Precondition for the rerun, and the actual root cause of the loss: point all
harness output at a repository path instead of `/tmp`, and confirm it is not
swallowed by `.gitignore`, which currently excludes `runs/`, `docs/*.log`,
`docs/*.csv` and `docs/*.txt`. Without that change a rerun would lose its
evidence exactly the same way.

If F3 is authorized before the rerun, that is a legitimate call — F2's PASS is
not being revoked here — but the F2 baseline should then be cited explicitly as
documentation-backed rather than artifact-backed.

---

## 2026-08-23 — F2 durable evidence regeneration: PASS REPRODUCED

Evidence-regeneration task, not optimization. The frozen F2 configuration was
run exactly as accepted and nothing was tuned. This section ADDS regenerated
reproducibility evidence; it does not replace the original F2 validation
recorded above, which stands on its own record.

### What was run

Scenes A-D of the frozen F2 matrix, `grasp_only:=true
use_perceived_position:=true require_perception:=true`, through the existing
validated `scripts/perception/milestone_f1_harness.py`. Boundary respected in
every scene: `f2_stop_reached=yes`, `lift_attempted=no`,
`transport_attempted=no`, `place_release_attempted=no`.

Clean state: each scene tore down and relaunched the ENTIRE stack — simulator,
move_group, perception nodes, controllers — with `gz_assert_clean_slate`
required to pass first, on top of the harness's own verified gripper
initialization. That is a stronger guarantee than the original sequential run
and directly addresses the historical ~0.8 rad leak. All four initializations
returned `REACHED_GOAL` with the controller responsive, verified open at
0.00385-0.00395 rad.

### Durable evidence root

`evidence/f2_0070_regeneration_20260823_114505/`

Verified with `git check-ignore -v` before the matrix was run: the root and
every artifact type in it are NOT ignored. It is on the repository filesystem,
not `/tmp`. ~770 MB, uncompressed, with a `MANIFEST.sha256` over all 149 files
and a `README.md` recording configuration, protocol, layout and limitations.

### Regenerated A-D matrix

| Scene | source | perception error | achieved pre-close [rad] | right-inner at descent end [rad] | min clearance R/L [mm] | fraction | P1->P2 | TCP error [mm] | final close | seating [mm] | premature contact |
|---|---|---:|---:|---:|---:|---:|---:|---:|---|---:|---|
| A | perceived | 1.6134 mm | 0.0601333 | -0.488294 | 3.148 / 10.840 | 1.0 | 0.0000 mm | 0.000671 | TIMED_OUT_HELD @0.4748 | 21.59 | none |
| B | perceived | 1.5767 mm | 0.0601340 | -0.346623 | 10.826 / 7.655 | 1.0 | 0.0000 mm | 0.000493 | TIMED_OUT_HELD @0.3901 | 21.65 | none |
| C | perceived | 1.5145 mm | 0.0601319 | -0.419677 | 6.738 / 6.968 | 1.0 | 0.0000 mm | 0.000296 | TIMED_OUT_HELD @0.4792 | 21.58 | none |
| D | perceived | 1.1339 mm | 0.0601352 | -0.452537 | 5.429 / 7.534 | 1.0 | 0.0000 mm | 0.000502 | TIMED_OUT_HELD @0.4514 | 22.27 | none |

Deltas against the accepted historical figures: perception error within
0.00061 mm, pre-close within 6.2e-6 rad, TCP error within 0.00041 mm, seating
within 0.039 mm. Every frozen criterion is satisfied, including the unchanged
1.0 mm P1->P2 limit, which all four scenes met at exactly 0.0000 mm.

### Independently re-derived, not taken from the node's own claims

Every acceptance check above was recomputed from the raw streams by
`tools/analyze_scene.py`. First object contact per stream, in simulation time,
occurs after descent completion AND after the stage-2 ground-truth check in all
four scenes — i.e. during final closure, never during descent. Left/right
finger and outer-knuckle streams stayed empty for entire runs, as in the
accepted result. Bilateral fingertip engagement was recorded in all four.

The wall/simulation clock correlation the original diagnosis had to infer from
camera stamps is now direct: `pose_*.csv` records both clocks on every row.

### Three honest limitations

- **Clearance figures come from a REBUILT tool.** The original clearance
  instrument did not survive; `tools/clearance.py` reimplements the described
  method against the same real collision STLs and the measured link poses. All
  values are positive and above 3 mm, and they track the measured follower
  states — where this run's right-inner sat less closed than the historical
  one, clearance is correspondingly larger. Regenerated cross-check, not a
  byte-for-byte reproduction.
- **`milestone_f1_truth.py`'s pre-grasp comparison does not apply to a
  `grasp_only` run.** It was written for `pregrasp_only`, where the object never
  moves; here it samples the object after closure has seated it, so its
  "perception vs truth" figure is an ordering artifact. The perception accuracy
  quoted above is the frozen perceived point against the known spawn position —
  the same quantity the accepted record uses. Truth JSONs retained unedited.
  Worth fixing before F3, which will have the same ordering.
- **Scene C's console transcript was lost** to an interrupted tool session; the
  run completed normally and every artifact was written. Its recorder tails were
  cut by the interrupt rather than the script's teardown, so those streams stop
  shortly after the run. Nothing inside the measured window is affected.

### Result

**F2 PASS REPRODUCED — durable raw evidence regenerated.**

This confirms reproducibility of the accepted F2 result on a clean-start stack.
It does not extend the validated boundary. The ~21.6-22.3 mm closure seating
persists in all four scenes and remains an explicit F3 warning. **F3 has not
started and its status is unchanged.** Nothing was committed or pushed.

---

## 2026-08-24 — F3 root-cause analysis COMPLETE, and friction calibration run 1

Two pieces of work, both offline or robot-free, both accepted. No simulation of
the manipulation pipeline was run, no production file was modified, nothing was
committed.

### 1. The valid P12.5 F3 Scene-A lift stands as FAIL

```
RUN_ID       = 20260823_190610_13717
F3 Scene A   = FAIL
G0->L2 slip  = 26.054 mm   against the frozen 5.000 mm criterion
```

This verdict is not revised by anything below. The controller reconstructed
27,724/27,724 rows with zero clip saturations and G0 was a valid stationary
baseline by the frozen gate. Evidence:
`evidence/f3_p12_5_lift_scene_A/A/lift_analysis_20260823_190610_13717.json`.

### 2. Root-cause analysis — accepted as the current scientific diagnosis

Full document, with every table and all reproduction scripts:
`evidence/f3_p12_5_lift_scene_A/analysis_root_cause_20260824/ROOT_CAUSE_ANALYSIS.md`

Confirmed findings:

- **The majority of the slip occurs after the lift motion has stopped.**
  8.6 % during acceleration, 18.5 % over the rest of the lift, and **72.9 %
  (19.009 mm) with the wrist provably stationary** (max |v| 0.21 mm/s).
- **Lift acceleration is not the dominant cause.** Peak MEASURED wrist vertical
  acceleration 0.1386 m/s^2 = 1.4 % of g; required hold force rises from
  1.4700 N to 1.4908 N, a 1.4 % dynamic increment.
- **The pre-lift object was substantially table-supported.** Its centre height
  matches the table-resting identity `z = 0.750 + sum_i |R[2,i]| h_i` to
  +-0.015 mm over 3437 samples spanning the whole hold INCLUDING G0, and the
  gripper supplies only **17.9 % of the object's weight at G0** (0.6 % at S2)
  against 100.0 % once airborne. The object clears the table at LIFT_BEGIN
  +0.153 s and slip begins 0.270 s later. G0 was stationary because the table
  held the object, not because the grasp did.
- **Bilateral fingertip contact remained present throughout the slip** —
  99.2 % / 99.3 % of 1 ms ticks — with a time-mean summed normal force of
  7.546 N, i.e. 5.13x the 1.470 N required at the configured mu.
- **Right inner-knuckle support was lost** during the post-lift dwell
  (sim t = 108.758 s). Its joint slews -0.221 rad back toward its mimic target
  as the object stops blocking it: a consequence and an escape indicator, not a
  cause.
- **The repeated ~21.6-22.3 mm closure seating is mechanically significant and
  is now identified.** It is not a translation artifact: the object is shoved
  -21.36 mm and pitched +14.43 deg into a wedge during closure. Reproducible
  across six runs and four controllers to 0.01 deg / 0.01 mm / 0.0001 rad.
- **The evidence strongly implicates asymmetric Robotiq mimic / contact / grasp
  geometry.** `right_knuckle_joint` tracks its mimic target to 1e-4 rad in free
  space, breaks at first object contact, saturates at **0.53 rad** (67 % of
  range) for the entire grasp, hold and lift, and returns to 1.1e-5 rad the
  instant the object is released. The closure is one-sided: left jaw 0.789 rad,
  right jaw 0.259 rad. Pad normal forces are asymmetric ~4:1; the right pad
  makes a 0.2 mm-wide line contact; the right inner knuckle bears on the
  object's top edge (z_obj = +22.50 mm) and touches it FIRST, before either
  fingertip. This is the same dartsim constraint-priority behaviour the project
  already diagnosed and fixed for the fingertip joints (TENTH OVERRIDE in
  `robotiq_2f_85_macro.urdf.xacro`); the three remaining velocity-servoed mimic
  joints were never covered by that fix.

Standing decisions carried forward:

- **Controller gain interpolation remains STOPPED.** A 2x gain change plus a
  20 s hold moved the slip by 4 % (27.211 -> 26.054 mm), and the identical
  wedge state appears under PID, H25, P12.5 and at zero commanded effort.
- **P12.5 remains the characterized reference controller** — frozen, not
  validated. Keeping it fixed is what makes the upstream variables measurable.
- **Friction must remain FROZEN.** No friction value may be changed.

Also established, and correcting an earlier statement in this file's own
record: the controller's +-50 N.m clip is NOT the actuation ceiling. The master
`left_knuckle_joint` has a **1.0 N.m URDF effort limit**
(`robotiq_2f_85_macro.urdf.xacro:460`) which DART enforces as a hard torque cap.
P12.5 peaked at 0.897 N.m = 89.7 % of that real ceiling, and the PID baseline it
replaced was railed at exactly 1.000000 N.m for 231/1000 samples. Measured
master-effort -> left-pad normal-force gain is 11.9-12.4 N per N.m (r = +0.958
raw, +0.994 at 0.1 s smoothing) across two operating points.

Ruled out: inertial loading; insufficient normal gripping force; controller
gain and controller authority as primary; hypothesis H-A (that the original
slip was caused by an unestablished G0); a dynamic/transient failure mode; and
per-point Coulomb utilisation as a usable diagnostic (it exceeds 1.0 by up to
81x during a fully static hold, because an LCP's per-point force distribution
over a redundant contact set is not unique).

### 3. Friction calibration run 1 — formal verdict INDETERMINATE

Experiment-local, robot-free: `evidence/friction_calibration_20260824/`
(`README.md` carries the protocol frozen BEFORE launch, above the RESULTS line;
`PROVENANCE.txt` carries provenance and the execution record).

```
RUN_ID = 20260824_015110_11233     ONE measurement, as predeclared
FORMAL VERDICT = INDETERMINATE
```

**The verdict is INDETERMINATE and must NOT be retroactively changed to PASS.**

Descriptive reading, explicitly NOT the formal measurement:

- slip onset at approximately **45 degrees**;
- **mu_eff approximately 1.0** (cubic back-extrapolation theta0 = 44.9883 deg,
  mu = 0.99959; frozen 2 mm criterion would give 45.578 deg, mu = 1.020);
- **sliding occurred before tipping** — the object's tilt stayed below 0.004 deg
  through slip onset and first exceeded 5 deg at 48.13 deg, by which point it
  had already slid 327.8 mm and was leaving the plate;
- reference-insensitive: four different reference windows all give 45.1700 deg;
- pre-slip stationarity 0.71 um over 1521 samples from 0 to 44 deg; contact
  continuity 1.0000 over the valid interval;
- **this strongly suggests the configured friction IS being realized**, and it
  definitively excludes the mu ~ 0.2 branch and the multiplicative combination
  rule.

**Why the verdict is nevertheless INDETERMINATE:** the frozen quiescence window
`Q = [2.0, 4.0] s` of native simulation time contains **zero samples**, because
recorders cannot start before the world's topics exist and the first recorded
native sample is at **sim_t ~ 5.257 s**. The empty window produced a NaN
reference which cascaded into the slip criterion never evaluating and the
contact gate being computed over the wrong interval. Two of three validity
gates therefore failed. That is an instrumentation defect, not a physics
finding, and no frozen threshold was changed and the run was NOT repeated.

Declared in advance and still unresolved: DART's own default friction
coefficient is also 1.0, so this measurement cannot distinguish "configured
values honoured" from "tags ignored, engine default in force". Separating them
needs a second ramp material, which is a second variable. Not on the critical
path: either way the pad<->object pair delivers mu = 1.0.

Consequence for the ranking: root-cause candidate #4, "insufficient effective
friction", moves from PLAUSIBLE to **RULED OUT at the material level**, subject
to the INDETERMINATE verdict above. The mu_eff ~ 0.195 figure flagged in
`ROOT_CAUSE_ANALYSIS.md` Sec 4.5 was labelled there as "a flag, not a
measurement" and is now falsified.

### 4. Exact next task

**Prospectively correct the friction-calibration measurement infrastructure so
the initial quiescence window is structurally recorded before ramp motion.**
Anchor the window to the first recorded sample rather than to an absolute
simulation time, or hold the settle until the recorded stream itself proves
2.0 s of coverage before commanding omega. Both are recording-side fixes; no
criterion, material value, geometry or omega changes.

Then, in order:

1. regression-test the corrected validity gates;
2. freeze the prospective amendment;
3. execute exactly ONE new friction-calibration measurement;
4. no parameter sweep, no automatic rerun.

If the valid repeat confirms `mu_eff ~ 1`, **close the friction hypothesis** and
proceed to investigation and engineering of the **Robotiq right-jaw
mimic/contact-geometry defect** — the 0.53 rad mimic collapse first, then the
inner-knuckle top-edge contact and the descent geometry that lets it lead, then
the F3 retention criterion itself, which must require the object to be clear of
the table and load-bearing in the grasp before G0 is taken.

### 5. Evidence paths

| Path | What it holds |
|---|---|
| `evidence/f3_p12_5_lift_scene_A/` | the valid P12.5 lift, 1.4 GB, RUN_ID 20260823_190610_13717 |
| `evidence/f3_p12_5_lift_scene_A/analysis_root_cause_20260824/` | root-cause analysis, new this session |
| `evidence/friction_calibration_20260824/` | friction calibration run 1, 70 MB, new this session |
| `evidence/f3_scene_A_20260823_125623/` | original PID F3 lift, INDETERMINATE, slip 27.211 mm |
| `evidence/f3_close_hold_scene_A_20260823_132557/` | 150 s close-hold, NOT_SETTLED |
| `evidence/f3_controller_ablation_scene_A_20260823_141550/` | zero-effort ablation |
| `evidence/f3_constant_effort_scene_A_20260823_143824/` | constant-effort attempt, stopped at strict switch |
| `evidence/f3_h25_hybrid_hold_infrastructure/` | H25 static hold, INDETERMINATE |
| `evidence/f3_p12_5_hybrid_hold/` | P12.5 static hold, INDETERMINATE |
| `evidence/f2_0070_regeneration_20260823_114505/` | F2 durable regeneration, PASS reproduced |

Total `evidence/` is now ~6.6 GB, untracked, repo-local, and confirmed NOT
git-ignored. Nothing in it may be deleted or regenerated.

### 6. New evaluation-only files created this session

Under `evidence/f3_p12_5_lift_scene_A/analysis_root_cause_20260824/`:
`ROOT_CAUSE_ANALYSIS.md`, `rc_kinematics.py`, `rc_contacts.py`,
`rc_cross_experiment.py`, their three `.json` outputs, and four
`rc_contact_*.npy` per-message contact aggregates.

Under `evidence/friction_calibration_20260824/`:
`README.md`, `PROVENANCE.txt`, `world/friction_calib.sdf`,
`tools/run_friction_calibration.sh`, `tools/pose_recorder.py`,
`tools/joint_recorder.py`, `tools/analyze_friction_calibration.py`,
`tools/diagnostic_reading.py`, and `A/` with 14 raw evidence files (62 MB
contact log, pose/joint/dynpose CSVs, topic census, manifest, protocol and sim
logs, and the two result JSONs).

All are evaluation-only. None is production code. None was committed.

### 7. Working-tree state that must NOT be reset

Unchanged by this session and still uncommitted:

- **12 tracked modified files, +1136 / -52.** These carry all of the A-F2 work
  plus the F3 `lift_only` path and the default-off pre-lift barrier
  (`transport.hpp`, `transport.cpp`, `m3_grasp.cpp`, `m3_grasp.launch.py`,
  `failure.hpp`, `scene.yaml`, both `CMakeLists.txt`, `package.xml`, the sim
  launch file, and the two URDF/XACRO files).
- **52 untracked entries**, including `AGENTS.md`, `CLAUDE.md`, `HANDOFF.md`,
  `PROJECT_STATE.md`, `docs/HANDOFF_RGBD_PERCEPTION.md`, the three `docs/F3_*`
  documents, `scripts/perception/`, `ur5e_robotiq_description/worlds/`, the two
  perception `.cpp` sources, the M8/M9/M10 and `prod_reg` capture scripts and
  marker directories, and all of `evidence/`.

`git diff --check` is clean. Nothing has ever been committed on this branch
(HEAD is still `7b875a4`). **Do not reset, clean, checkout over, or stash any
of this.** A `git reset --hard` or `git clean -fd` here would destroy the entire
A-F3 body of work and ~6.6 GB of irreplaceable evidence.

### 8. Known stale statements elsewhere, NOT yet repaired

Left alone deliberately so no historical record is rewritten. A future session
may correct them:

- `docs/F3_P12_5_LIFT_PLAN.md` line 3 and Sec 12 still say the scientific lift
  is "NOT RUN, NOT authorized". It was authorized and run; the plan body is a
  frozen predeclaration and must not be edited, but its status header now
  misstates fact.
- `HANDOFF.md` has no dated narrative sections for the five intermediate F3
  controller experiments; their record lives only in the per-directory
  `README.md` and `PROVENANCE.txt` files under `evidence/`.
- Five of the eight `evidence/*` directories still lack a `MANIFEST.sha256`.

---

## 2026-08-24 (later) — friction-calibration instrument corrected, gates regression-tested, amendment frozen

Offline and robot-free. **No simulator was launched.** No production file was
modified, no physics/friction/geometry/threshold/criterion value was changed,
nothing was committed. Steps 1-3 of the four-step "Exact next task" above are
complete; step 4 is not authorized and was not run.

### Run 1 is untouched

```
evidence/friction_calibration_20260824/   RUN_ID 20260824_015110_11233
FORMAL VERDICT = INDETERMINATE            — UNCHANGED
```

Run 1 was **not rerun and not reinterpreted**. Its raw evidence was never
re-analysed. Its directory was opened read-only twice: once to read the recorded
CSV/contact-log *header format* so synthetic fixtures could be written faithfully,
and once to `sha256sum` the world SDF and the two recorders. Newest mtime under
that directory is still `2026-08-24 01:56`, hours before this session. Nothing in
it was modified.

### The defect, and the correction

Run 1's quiescence window was frozen at an **absolute** `Q = [2.0, 4.0] s` of sim
time. Recorders cannot attach before the world's topics exist, so the first
recorded sample was at `sim_t ~ 5.257 s` and `Q` held **zero samples** — a NaN
reference, a slip criterion that could never fire, and a contact gate computed
over an interval that did not exist. The run script compounded it: its settle loop
waited for `sim_t >= 5.0 s` read from the recorded stream, which the very first
recorded sample already satisfied.

The corrected instrument lives in a NEW directory so run 1 stays intact:

```
evidence/friction_calibration_run2_20260824/
```

`AMENDMENT-1`, frozen prospectively in that directory's `AMENDMENT.md`, makes
three recording-side changes and nothing else:

1. **`Q` is anchored to recorded data** — `Q_start = t_rec0 + 1.0 s`,
   `Q_end = Q_start + 2.0 s`, where `t_rec0` is the sim time of the first recorded
   `pick_target` sample in the primary pose stream. The window **width stays 2.0 s**,
   exactly as predeclared; only the anchor moves. This is option 1 of run 1's
   README §R6, written before this session.
2. **`omega` is not commanded until coverage is proven** — the run script holds at
   `theta = 0` until the recorded file itself shows `sim_t >= max(Q_end + 0.5, 5.0)`,
   counts the rows actually inside `Q`, and **aborts before commanding omega** if
   there are fewer than the 100 the frozen gate needs. Run 1's 5.0 s settle floor
   is retained as a lower bound, so the settle can only lengthen. It then writes
   `A/window_<RUN_ID>.json` recording the window, the proven coverage, the tilt
   angle before the command, and the sim time at which omega was commanded.
3. **A fourth validity gate, `window_precedes_ramp`** — requires tilt inside `Q` to
   be within `1e-4 rad` of zero, and (when the artifact exists) that omega was
   commanded at a recorded sim time `>= Q_end` with the artifact agreeing with the
   window derived from the raw stream. It can only make a result *harder* to
   declare VALID.

Plus a robustness guard: an empty `Q` is now reported as an explicit recording
defect naming the window and the actual stream span, instead of silently producing
NaN and surfacing as two unrelated-looking gate failures.

Byte-identical to run 1, sha256-verified in `FREEZE.sha256`:
`world/friction_calib.sdf`, `tools/pose_recorder.py`, `tools/joint_recorder.py`.
Unchanged in value: all friction and material properties, all geometry,
`omega = 0.0349065850398866 rad/s`, the 65 deg stop, every quiescence/monotonic/
contact/slip/tipping threshold, the cubic secondary estimator, the §9
interpretation table, and **ONE measurement, no sweep, no automatic rerun**.

### Regression test of the corrected gates — 8/8 PASS

`regression/run_regression.py` -> `regression/gate_regression_result.json`.
Eight cases against **synthetic** streams with known ground truth, written in the
exact recorder format, every one starting at `sim_t = 5.257 s` to reproduce run 1's
late-start condition. **No run-1 data is read anywhere in the suite.**

| # | Fixture | Expected | Got |
|---|---|---|---|
| c1 | nominal, theta0 = 45.0 deg | VALID | VALID, theta_slip 45.5947, cubic 45.0002, mu 1.00001 |
| c2 | same data, run 1's absolute window | INDETERMINATE | INDETERMINATE, 0 samples in Q |
| c3 | not quiescent (p2p 0.96 mm) | gate 1 fails | gate 1 failed |
| c4 | non-monotonic ramp (-4.7e-4 rad) | gate 2 fails | gate 2 failed |
| c5 | contact dropout | gate 3 fails | gate 3 failed, 80.2 % occupancy |
| c6 | ramp motion inside Q | gate 4 fails | gate 4 failed, **gate 1 passed** |
| c7 | artifact says omega preceded Q_end | gate 4 fails | gate 4 failed |
| c8 | nominal, theta0 = 30.0 deg | VALID, mu = 0.5774 | VALID, mu 0.57737 |

Three of these are load-bearing:

- **c2 also executes run 1's analyzer VERBATIM** — a byte-identical copy, run
  unmodified — against the same synthetic fixture. It returns INDETERMINATE with
  `quiescence` and `contact_continuity` failed and `p2p_mm = null`: run 1's exact
  failure signature, reproduced on data the corrected analyzer evaluates cleanly.
  That is independent confirmation the diagnosed mechanism is the real one.
- **c6 is the reason gate 4 exists.** `p_rel` is computed in the plate's own
  measured frame and removes rigid-body ride exactly, so an object riding a tilting
  plate still looks perfectly quiescent — gate 1 PASSES. Only gate 4 catches it.
- **c8 recovers a different mu.** The corrected analyzer is not biased toward
  `mu ~ 1`: given a 30 deg ground truth it returns 0.57737 against 0.57735.

**Honest limit, stated in the amendment too:** the fixtures use the same
`d = k(theta - theta0)^3` form the secondary estimator fits, so c1/c8's 0.0002 deg
recovery verifies the *instrument and the gate plumbing*, not the physics. It says
nothing about what mu the simulator will produce. Tipping and the
INDETERMINATE_BOUNDED path are untouched by this amendment and are not covered.

### Files created (all evaluation-only, none production, none committed)

Under `evidence/friction_calibration_run2_20260824/`: `AMENDMENT.md`,
`FREEZE.sha256`, `PROVENANCE.txt`, `tools/` (2 corrected + 2 verbatim-copied),
`world/friction_calib.sdf` (verbatim copy), `regression/make_fixture.py`,
`regression/run_regression.py`, `regression/gate_regression_result.json`,
`regression/fixtures/` (~25 MB), and an empty `A/` awaiting the measurement.
The root is confirmed NOT git-ignored. Total ~25 MB.

### Working tree

Unchanged by this session: still 12 tracked modified files, `git diff --check`
clean, HEAD still `7b875a4`, nothing committed. The section "Working-tree state
that must NOT be reset" above still applies in full, now plus this directory.

### Exact next step

**Step 4: execute exactly ONE friction-calibration measurement — REQUIRES EXPLICIT
AUTHORIZATION.** It launches a Gazebo simulator.

```
bash evidence/friction_calibration_run2_20260824/tools/run_friction_calibration.sh
python3 evidence/friction_calibration_run2_20260824/tools/analyze_friction_calibration.py
```

Predeclared reading, fixed in `AMENDMENT.md` §5 before the run:

- **VALID with mu_eff ~ 1** -> close the friction hypothesis, proceed to the
  Robotiq right-jaw mimic / contact-geometry defect (0.53 rad mimic collapse first,
  then the inner-knuckle top-edge contact and the descent geometry that lets it
  lead, then the F3 retention criterion itself, which must require the object to be
  clear of the table and load-bearing in the grasp before G0 is taken).
- **VALID with mu_eff well below 1** -> investigate friction-parameter propagation
  before touching any grasp physics; run 1's descriptive reading is superseded.
- **INDETERMINATE** -> record and investigate. No silent retry, no threshold change.
- Run 1's formal verdict stays INDETERMINATE in every case.

Still prohibited without explicit authorization: Scene A reruns, Scenes B-D,
controller gain interpolation, friction changes, physics changes, grasp or geometry
changes, and starting the Robotiq fix before the friction hypothesis is formally
closed.

---

## 2026-08-24 (later) — friction calibration run 2: FORMAL VERDICT **VALID**, mu_eff = 1.0202. Friction hypothesis CLOSED.

Exactly ONE measurement, executed with the frozen corrected instrument under
explicit authorization. No parameter sweep, no rerun, no retry. The protocol,
thresholds, friction, geometry, ramp rate, physics and analyzer were not changed
before or after seeing the result — the freeze was sha256-verified intact both
immediately before the launch and again after the analysis. The UR5e was not run,
Scene A was not run, no production file was modified, nothing was committed.

```
RUN_ID          = 20260824_122918_4279
FORMAL VERDICT  = VALID
mu_eff          = 1.0202398  (frozen 2 mm criterion)
mu from theta0  = 0.9996946  (predeclared cubic estimator)
```

### All four validity gates PASS

| Gate | Requirement | Measured | |
|---|---|---|---|
| 1 quiescence | ≥ 100 samples, p2p < 0.20 mm, no gap > 20 ms | 118 samples, p2p **0.0000 mm**, max gap 17.0 ms | PASS |
| 2 monotonic ramp | no step < −1e-4 rad | most negative step **0.0** rad over 38,945 samples | PASS |
| 3 contact continuity | ≥ 95 % of 20 ms bins, Q_start → t* | **100.00 %** of 1,334 bins over [6.215, 32.879] s | PASS |
| 4 window precedes ramp *(new)* | tilt in Q ≤ 1e-4 rad; omega commanded ≥ Q_end; artifact agrees | tilt p2p **0.0** rad over 2,001 joint samples; omega at 10.078 s ≥ 8.215 s; artifact agrees | PASS |

Run 1 failed gates 1 and 3 on an empty window. Run 2 passes all four on a window
that contains data.

### The correction did exactly what it was built to do

The first recorded primary-pose sample landed at **sim_t = 5.215 s** — Run 1's was
5.257 s. The late start is **structural, not a one-off**, and Run 1's absolute
`Q = [2.0, 4.0] s` would have been empty again. Anchoring produced
`Q = [6.215, 8.215] s` with **118 recorded samples**, against Run 1's zero.

Proof that ramp motion began only after Q_end, from three independent records:

- the recorded `window_20260824_122918_4279.json` artifact: omega commanded at
  recorded `sim_t = 10.078 s`, i.e. **1.863 s of simulation time after Q_end**,
  with `tilt_before_ramp_rad = 0.000000000`;
- the recorded `tilt` joint stream itself: over the 2,001 joint samples inside Q,
  peak-to-peak and maximum absolute tilt are both **exactly 0.0 rad**;
- the object: across all 118 in-window pose samples there is exactly **ONE unique
  position** — bit-identical, 0.0 µm in x, y and z — and 110 further samples
  between Q_end and the ramp command are also at 0.0 µm with theta = 0.

The exactly-zero peak-to-peak is genuine perfect rest, not a stalled stream: the
same stream later records the full 70 deg ramp and a 327 mm slide.

### Result

- **Sustained-slip onset** `t* = 32.879 s`, the first sample satisfying both frozen
  conditions (d ≥ 2.0 mm; non-decreasing to 0.05 mm over the next 0.25 s with mean
  rate ≥ 5.0 mm/s). `d(t*) = 2.0396 mm`.
- **theta_slip = 45.5740 deg** (0.795416 rad). **mu_eff = tan(theta_slip) = 1.02024.**
- **Secondary estimator**, `d = k(theta − theta0)^3` fitted over d ∈ [0.2, 5.0] mm,
  16 samples, residual 7.6e-4: **theta0 = 44.9912 deg, mu = 0.99969.**
- **Uncertainty**: random, one pose sample × omega = **±0.034 deg**; systematic
  criterion overshoot, one-sided positive = **+0.583 deg** (the raw criterion fires
  above true onset by construction; the cubic estimator removes it).
- **Displacement is pure down-slope sliding**: at t* the plate-frame displacement is
  (+2.0396, 0.0000, −0.0004) mm — no lift-off, no lateral wander.
- **Sliding decisively preceded tipping.** phi stayed below **0.0086 deg** for the
  entire ramp up to t*, and was 0.0022 deg at t* itself. phi first exceeded the
  5 deg tipping threshold at sim_t = 34.154 s, theta = 48.124 deg — **1.275 s after
  slip onset, by which point the object had already slid 326.9 mm** and was leaving
  the plate. Predicted theta_tip = 56.3099 deg was never reached in a static sense.
  The bounded-outcome path was correctly not taken.

### Comparison with run 1's descriptive reading

Run 1's formal verdict remains **INDETERMINATE** and is NOT retroactively changed.
Its descriptive numbers were explicitly labelled "not the formal measurement".
Run 2 is an independent measurement — fresh simulator launch, new RUN_ID, corrected
instrument — and it reproduces them to within the stated uncertainty:

| Quantity | Run 1 (descriptive) | Run 2 (**formal, VALID**) | delta |
|---|---:|---:|---:|
| theta_slip at 2 mm | 45.578 deg | **45.5740 deg** | 0.004 deg |
| mu_eff | 1.020 | **1.02024** | 0.0002 |
| theta0, cubic | 44.9883 deg | **44.9912 deg** | 0.003 deg |
| mu from theta0 | 0.99959 | **0.99969** | 0.0001 |
| first phi > 5 deg | 48.13 deg | 48.124 deg | 0.006 deg |
| slid distance at that moment | 327.8 mm | 326.9 mm | 0.9 mm |

All deltas are far inside the ±0.034 deg random uncertainty. What was a
high-confidence but formally unusable reading is now a formal result.

### Conclusions

1. **Effective pad<->object friction is RESOLVED: mu_eff = 1.00 (1.0202 raw,
   0.99969 corrected).** The configured friction is being realized.
2. **The friction hypothesis is CLOSED.** Root-cause candidate #4, "insufficient
   effective friction", is RULED OUT at the material level — now on a VALID
   measurement rather than on run 1's INDETERMINATE one. The 5.13x normal-force
   surplus measured during the F3 lift cannot be explained away by weak friction.
3. **The next engineering task is the Robotiq right-jaw mimic / contact-geometry
   defect**, exactly as predeclared. Every remaining confirmed cause of the F3
   failure is geometric or mechanical: object not load-bearing before the lift
   (dominant), wedged grasp geometry, and mimic collapse under contact load.

**Unchanged limitation, declared before run 1 and still true:** DART's own default
friction coefficient is also 1.0, so this measurement still cannot distinguish
"configured values honoured" from "tags ignored, engine default in force".
Separating them needs a second ramp material — a second variable, a separate
predeclared experiment, and **not on the critical path**: either way the
pad<->object pair delivers mu = 1.0 and friction is not what loses the object.

### Evidence

| Path | What |
|---|---|
| `evidence/friction_calibration_run2_20260824/A/` | 73 MB, 14 files, `MANIFEST.sha256`, RUN_ID 20260824_122918_4279 |
| `.../A/friction_calibration_result.json` | the formal VALID verdict |
| `.../A/window_20260824_122918_4279.json` | the recorded window artifact |
| `.../A/protocol_20260824_122918_4279.log` | execution record, all 4 recorders ALIVE at teardown |
| `.../AMENDMENT.md`, `.../FREEZE.sha256` | the prospective amendment and its freeze |
| `.../regression/` | the 8/8 gate regression, synthetic fixtures |
| `.../descriptive/post_hoc_descriptive.json` | read-only supplement: tipping timeline, µm stationarity. NOT the formal measurement |
| `evidence/friction_calibration_20260824/` | run 1, INDETERMINATE, untouched |

Run 1 verified untouched after the run: no file under its directory modified since
02:00, verdict still reads INDETERMINATE.

### State at stop

No simulator, controller or ROS process left running. HEAD still `7b875a4`,
`git diff --check` clean, 12 tracked modified files at +1136/−52 — the same figure
recorded before this session — and 52 untracked entries. Nothing committed, nothing
pushed. The section "Working-tree state that must NOT be reset" still applies in
full.

---

## 2026-08-24 (later) — Robotiq mimic audit, and three engine-capability probes

Read-only audit plus three simulator-capability probes. **No production file was
modified, no engine was migrated, the UR5e was not run, Scene A was not run, no
F2/F3 evidence was regenerated, nothing was committed.** No manipulation result
was produced and none is claimed.

```
ROBOTIQ MIMIC AUDIT (read-only)             COMPLETE
SDF CLOSED-LOOP FEASIBILITY (I-0)           FAIL, conclusive
NATIVE SDF MIMIC under bullet-featherstone  PASS, conclusive
PROJECT-COMPATIBILITY PROBE                 NOT TESTED (harness built, not run)
ENGINE MIGRATION VERDICT                    NONE REACHED
```

Durable artifacts, copied out of the session scratchpad because `/tmp` does not
survive a session — the same failure mode that lost the original F2 raw evidence:

```
evidence/bullet_engine_probes_20260824/   244 MB, README.md, MANIFEST.sha256
    i0_probe/       closed-loop feasibility        FAIL
    mimic_probe/    native SDF mimic               PASS
    compat_probe/   project compatibility          NOT TESTED, ready to rerun
```

### 1. The audit's central finding

The SDF `<mimic>` tags **do** survive into the SDF Gazebo loads for all three
followers — but `libgz-physics7-dartsim-plugin.so` contains **zero** mimic
symbols, and the F3 run's own log confirms `gz::physics::dartsim::Plugin` is what
loaded. **The native constraint is inert; the sole enforcement is
gz_ros2_control's software velocity servo.** There is no kinematic constraint
anywhere in this gripper — three soft velocity servos wearing a constraint's name.

Measured consequences, recomputed from preserved evidence (all 11 published
`right_knuckle` values in `ROOT_CAUSE_ANALYSIS.md` §6.1 reproduced exactly):

- **All three followers deviate from the first recorded sample** (sim_t 23.399,
  errors +0.6455 / -0.4454 / -0.3025 rad) in free space, with no contact and with
  self-collision disabled. Not contact-induced.
- `right_inner_knuckle` sits ~0.49 rad closed while the gripper is open, so it
  protrudes and makes first object contact at t=61.73, **5.86 s before either
  fingertip**, pushing down-and-sideways along `[+0.899, ~0, -0.439]`.
- **Mimic error controls descent clearance**: across the four F2 scenes,
  clearance = 53.30*q + 29.28, **r = +0.998**. Scene A is tightest because its
  error is largest.
- **Object pitch = master - fingertip_grasp_theta**, slope **1.0007**, residual
  < 0.03 deg at two independent master angles 6.4 deg apart (0.678191 -> 8.0522
  deg; 0.789360 -> 14.4262 deg). The 14.43 deg wedge is the object lying flat
  against the TENTH OVERRIDE's welded pad, tilted by the master's overshoot past
  the design angle. Not an emergent wedging artifact.
- `scripts/lib/gripper_geometry.py` still documents the pre-TENTH-OVERRIDE
  "parallel jaw at every theta" property as the thing that makes it exact. That
  has been false since 2026-08-12 and should be corrected regardless of what
  happens next.

### 2. Probe I-0 — SDF closed loops: FAIL, conclusive

The audit's recommended fix (close the four-bar in SDF) is **not implementable**.
`gz-physics` 7.6.0's dartsim plugin refuses any joint whose child link already has
a parent joint, which every closed loop necessarily creates:

```
[Err] [SDFFeatures.cc:1111] ... but the child link already has a parent joint of type [RevoluteJoint].
[Wrn] [EntityManagementFeatures.cc:469] No joint named [jointD_loop] for modelID [2]
```

The SDF validates and the joint is then **silently dropped**, leaving an open
chain with an extra unconstrained DOF. Rejected identically for revolute joints,
fixed joints, and reversed parent/child. bullet-featherstone rejects it too
(`SDFFeatures.cc:290`, "multiple parent joints"). Only plain `bullet` accepts it,
and that plugin has no mimic support at all.

**Reverting the TENTH OVERRIDE via loop closure is dead with it.**

### 3. Probe 2 — native SDF mimic under bullet-featherstone: PASS, conclusive

Smallest representative model, master + follower with the production joints' exact
limits, carrying the same `<axis><mimic>` construct sdformat already emits. No
ros2_control, no mimic servo, nothing commanding the follower.

| | Result |
|---|---|
| Load | clean, zero Err/Wrn |
| Free space, full 0.8 rad sweep | max abs error **1.0e-9 rad** |
| Follower disturbed +20 / -20 / **+500 N.m** | error 3.3e-8 / 6.0e-8 / **3.0e-8 rad** |
| What happened under load | the follower **dragged the master with it**, by an identical amount |
| Recovery | error to 0-2.6e-8 rad, no hysteresis |

A hard bidirectional kinematic constraint, not a motor. 500 N.m is 500x the
master's own 1.0 N.m URDF effort limit. Caveat: at 500 N.m the pair was dragged
~0.04 rad past declared position limits.

dartsim control — the engine says so itself, once, not spam:

```
[Err] [Physics.cc:1808] Attempting to create a mimic constraint for joint [follower_joint]
but the chosen physics engine does not support mimic constraints, so no constraint will be created.
```

Functionally: under 500 N.m the follower moved 134.755 deg and the master 0.000
deg. Free DOF; the tag has zero effect.

Two harness faults were caught by controls and the affected runs redone —
`ApplyJointForce` silently ignored its `<topic>` element (the first disturbance
commands went nowhere and looked like perfect constraint rejection), and the
follower vanishes from `joint_state` without a mimic tag. Valid final runs are
`bf_mimic3` / `bf_nomimic3`; earlier directories predate the fix and **their
disturbance windows are void and must not be cited.**

### 4. Probe 3 — project compatibility: NOT TESTED

Built to answer classes 1-7 (gz_ros2_control, native-mimic coexistence, contact
sensing, joint effort/wrench, RGB-D, timing, MoveIt boundary) in one spawn. **Run
once; failed to reach any test.** The failure was **entirely probe-harness**, and
says nothing about the engine:

1. The URDF passed as `-p robot_description:="$(cat ...)"` broke `rcl` argument
   parsing (`arguments.c:352`) and `robot_state_publisher` aborted.
2. `/robot_description` was therefore never published, so
   `ros_gz_sim create -topic robot_description` waited forever.

**Both faults are fixed and `bash -n` verified, but not yet exercised:** RSP now
starts from `--params-file rsp_params.yaml` (YAML round-trips, embedded URDF
re-parses as valid XML) and the spawn uses `-string`, matching what the production
launch file already does.

**All capability classes stand at NOT TESTED. No compatibility or migration
verdict has been reached — not A, not B, not C.**

One incidental static finding: both engines carry `JointTransmittedWrench` symbols
(dartsim 188, bullet-featherstone 260), so joint-effort readback is not obviously
blocked. A symbol count, not a functional result.

### 5. Standing answers, unchanged by the compatibility gap

- **Would native mimic allow removing the gz_ros2_control software mimic?** Yes —
  on an engine that implements it. It is 7 orders of magnitude tighter (6e-8 rad
  under load vs the measured 0.53 rad collapse).
- **Could the TENTH OVERRIDE be reverted?** Not via loop closure — that route is
  dead. Possibly via native mimic on the fingertip joints, which would restore the
  parallel jaw at every angle and remove the `pitch = master - theta_ft` term at
  its source. Untested.
- **Would the gripper be mechanically closer to a real 2F-85?** Closer, not close.
  The four-bar stays an open tree on every viable engine, so the linkage remains
  an approximation held by constraints rather than by mechanism.
- **What would have to be regenerated after a migration?** Everything measured
  under DART: the M3/M5 20/20 baseline, F2 and its four-scene matrix, the F2
  durable regeneration, all seven F3 experiments, the classical regression, and
  the friction calibration (whose declared DART-default-mu ambiguity becomes a
  different open question under a different contact model). P12.5's
  characterisation would not transfer.
- **What survives regardless?** The conceptual findings: that the object is
  table-supported at G0 and the grasp carries 17.9% of its weight; that G0
  certifies a stationary object rather than a held one; that closure is one-sided
  and wedges the object; that mimic error governs descent clearance; that pad tilt
  equals master overshoot; and that friction is not the cause. Those are
  mechanisms, not measurements, and they would need re-measuring but not
  re-deriving.

### 6. Exact next task

1. Run the corrected probe under bullet-featherstone (~10-15 min; exceeds the
   10-minute tool cap, so run it detached and poll).
2. Run the identical probe under dartsim as the control, so every PASS/FAIL is
   attributable to the engine and not to the harness. The previous probe had an
   undiagnosed fault where JointController never moved the master under dartsim —
   expect it again and diagnose it rather than work around it.
3. Build the compatibility matrix: Capability | DART current | bullet probe |
   Required? | Blocking?, each PASS / PARTIAL / FAIL / NOT TESTED.
4. Decide **only**: A BULLET MIGRATION TECHNICALLY VIABLE / B BLOCKED /
   C INDETERMINATE. Do not migrate.

If the outcome is B, the fallback to design is the smallest DART-compatible
gripper simplification that preserves the existing evidence base. Independently of
all of this, the F3 retention criterion still needs amending so G0 requires the
object to be clear of the table and load-bearing before the baseline is taken.

Prohibited without fresh authorization: migrating the engine, modifying any
production file, running the UR5e or Scene A, regenerating F2/F3 evidence,
committing or pushing.

### 7. Working tree

Unchanged by all of this: HEAD still `7b875a4`, `git diff --check` clean, 12
tracked modified files at +1136/-52, 52 untracked entries plus the new
`evidence/bullet_engine_probes_20260824/`. The section "Working-tree state that
must NOT be reset" above still applies in full.

---

## 2026-08-24 (later) — project-compatibility probe EXECUTED. Verdict **B — BULLET MIGRATION BLOCKED**.

Supersedes "Probe 3 — project compatibility: NOT TESTED" above. Read-only with
respect to production: **zero production files modified, no engine migrated,
nothing committed.** HEAD still `7b875a4`.

### 1. Three further harness faults, found before any result was trusted

The previous section recorded two fixed faults. Three more were latent, and each
would on its own have produced a false FAIL blamed on the engine:

3. `probe.urdf` / `rsp_params.yaml` / `probe_check.sdf` hardcoded the
   `gz_ros2_control` `<parameters>` path to a **dead session scratchpad**
   (`.../93e6ad59-.../compat_probe/controllers.yaml`). controller_manager would
   have received no configuration and all seven classes would have read FAIL.
4. `gz-sim`'s SystemLoader does **not** search `LD_LIBRARY_PATH` for system
   plugins: `gz_ros2_control-system` failed with "Could not find shared library".
   Confirmed by minimal reproduction that `GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib`
   fixes it.
5. The probe's `controllers.yaml` had **no `gains:` block**. `effort_controllers/
   GripperActionController` computes effort through a PID; without gains it
   commands zero effort and the master cannot move. Production's
   `ur5e_robotiq_description/config/controllers.yaml` carries `p: 50.0, d: 2.0`.
   The README's claim that the probe config "mirrors production" was wrong.

Also, not a fault but a trap worth recording: the contact sensor's `<topic>`
element is **ignored** by the Contact system, exactly like `ApplyJointForce`'s
`<topic>` in the earlier mimic probe. Corrected to the advertised topic; contact
still yielded 0 bytes under **both** engines, so the scenario never produced a
contact. Contact sensing is **NOT TESTED**, not FAIL.

### 2. The decisive experiment: a 2x2 under bullet-featherstone, with a dart control

Byte-identical harness across all five runs. Production-faithful joint values,
verified against `robotiq_2f_85_macro.urdf.xacro:460,474` — master
`velocity=0.1 effort=1.0`, followers `effort=50`.

| Run | Engine | `<mimic>` | master effort | gripper action | master final (rad) |
|---|---|---|---|---|---|
| dart | dartsim | yes | 1.0 | **rc=0 SUCCEEDED**, 7 s | **0.6963**, `reached_goal: true`, effort 0.301 |
| bullet_run2_gains | bullet-fs | yes | 1.0 | rc=124 | **5.2e-09 — frozen** |
| bullet_e50 | bullet-fs | yes | **50.0** | rc=124 | **-2.4e-09 — frozen** |
| bullet_nomimic | bullet-fs | no | 1.0 | rc=124 | 0.04656 — stalled |
| bullet_nomimic_e50 | bullet-fs | **no** | **50.0** | rc=124 (harness 60 s cap) | **0.6644 — actuating normally** |

Under bullet-featherstone the master actuates **only** when the mimic constraint
is absent AND the effort ceiling is raised 50x. With the production mimic
construct present, no amount of torque moves it: 50x headroom moved the final
position from 5.2e-09 to -2.4e-09.

The last row's rc=124 is the harness's own 60 s cap, not a capability failure —
the joint travelled 0 -> 0.6644 rad at its 0.1 rad/s velocity limit and was still
tracking when the cap fired. That cell must be judged by position, not by rc.

Master effort readback: dartsim ~0.0015-0.0035 N.m; bullet saturates at ±2.0 N.m
while barely moving.

### 3. The finding that reframes the question

**`gz_ros2_control` reads the URDF `<mimic>` tags directly and installs its own
software mimic servo — in BOTH engines, regardless of ros2_control mimic params:**

```
[gz_ros_control]: Joint 'follower_a_joint'is mimicking joint 'gripper_master_joint' with multiplier: -1
```

The probe was built on the premise "NO ros2_control mimic params anywhere", so
that the native constraint would be the sole coupling. **That premise is false.**
"Native mimic only" is not reachable while a single description feeds both gz-sim
and ros2_control.

Under bullet the native constraint is real (followers tracked master to 5.2e-09,
consistent with the earlier probe's 1e-9 rad); the software servo and the hard
constraint fight, and the assembly locks. Under dartsim the native constraint is
inert — the engine says so at `Physics.cc:1808` — so only the servo acts and the
master moves.

### 4. Independently important: dartsim's coupling does nothing at all

Dart run WITH mimic tags ended at `follower_a 8.6e-14`, `follower_b -0.9000`,
`follower_c +0.9000`. But `follower_b` has multiplier **+1** (expected +0.696) and
`follower_c` **-1** (expected -0.696) — both **sign-inverted and saturated at their
limits**. The no-mimic bullet run shows those are exactly where free, uncoupled
joints fall under gravity.

**dartsim's followers behaved indistinguishably from having no coupling at all.**
This reproduces the production defect in a minimal model: followers deviating in
free space, with no contact — the measured origin of the right-inner-knuckle wedge.

### 5. Compatibility matrix

| # | Capability | DART (current) | bullet probe | Required? | Blocking? |
|---|---|---|---|---|---|
| 1 | gz_ros2_control coexistence | PASS | **PASS** (CM up, 20 services, 4 cmd + 18 state ifaces, 3 controllers active, arm reached 0.4999997/0.2999997) | yes | no |
| 2 | native-mimic coexistence | FAIL (inert; servo does not hold) | **FAIL** (constraint real, locks master) | yes | **YES** |
| 3 | contact sensing | NOT TESTED (0 bytes) | NOT TESTED (0 bytes) | yes | unknown |
| 4 | joint effort / wrench | PASS (readback ~0.003 N.m) | PARTIAL (state exported; readback saturates ±2.0; command side fails under mimic) | yes | **YES** (command side) |
| 5 | RGB-D | PASS (733/714/257/2112) | **PASS** (695-734/733-752/171-299/2112) | yes | no |
| 6 | timing at 1 ms | PARTIAL (RTF 0.2436-0.9997, variable) | **PASS** (RTF 0.9966-0.9999, steady) | yes | no |
| 7 | MoveIt boundary | PASS (production) | NOT TESTED (no MoveIt in harness) | yes | unknown |

bullet-featherstone is **better** than dartsim on timing, equal on RGB-D and
controller coexistence, and fails the one class the migration exists to fix.

### 6. Verdict — **B — BULLET MIGRATION BLOCKED**

For the configuration this project needs. The migration's purpose was a real
kinematic constraint on the gripper linkage via native SDF mimic. Under
`gz_ros2_control`, which the project requires, enabling native mimic makes the
gripper master unactuatable — in three independent variants, one with 50x the
production torque ceiling. The only bullet configuration in which the master
actuates has **no mimic at all**, which delivers nothing DART does not already give.

**The one identified escape, NOT TESTED:** feed `gz_ros2_control` a description
**without** `<mimic>` tags while gz-sim receives one **with** them, so the software
servo is never installed and the native constraint acts alone. The production
launch currently feeds one description to both. Whether divergent descriptions are
possible in this stack is untested. Until it is, B stands.

Classes 3 and 7 stay NOT TESTED and cannot be resolved by this harness; they do not
affect the verdict, which rests on class 2 alone.

### 7. Exact next task

1. Test the escape route: can gz-sim and `gz_ros2_control` be given divergent
   descriptions (mimic tags for the former, none for the latter)? If yes, re-run
   the 2x2 — that single question is what separates B from A.
2. If it cannot, adopt the class-B fallback already named: design the smallest
   DART-compatible gripper simplification preserving the existing evidence base.
3. Independently of the engine question, amend the F3 retention criterion so G0
   requires the object to be clear of the table and load-bearing before the
   baseline is taken. This is unchanged and still outstanding.

Evidence: `evidence/bullet_engine_probes_20260824/compat_probe_run_20260824/`
(73 MB, README.md, MANIFEST.sha256, 143 files, all five runs plus every harness
variant). The earlier `compat_probe/` directory is retained; its `NOT TESTED`
status is superseded.

Prohibited without fresh authorization, unchanged: migrating the engine, modifying
any production file, running the UR5e or Scene A, Scenes B-D, controller gain
interpolation, friction or physics changes, regenerating F2/F3 evidence,
committing or pushing.

### 8. Working tree

Unchanged by all of this: HEAD `7b875a4`, 12 tracked modified files at +1136/-52,
untracked entries plus `evidence/`. Production URDF/xacro mtimes remain
2026-08-23 01:13 — untouched. The section "Working-tree state that must NOT be
reset" still applies in full.

---

## 2026-08-24 (later) — split-description escape route traced: INFEASIBLE without forking `gz_ros2_control`. Verdict **B unchanged**.

Answers the "exact next task" item from the section above: whether gz-sim could
receive a description WITH native `<mimic>` while `gz_ros2_control` receives one
WITHOUT, so its software servo never installs and bullet's native constraint acts
alone. **No simulation was run for this** — the question was answered first, as
instructed, by tracing the actual installed binaries
(`/opt/ros/jazzy/lib/libgz_ros2_control-system.so`,
`/opt/ros/jazzy/lib/libgz_hardware_plugins.so`, `libsdformat14.so.14`), and the
trace shows the split does not exist as a reachable configuration, so no probe was
built per the standing "stop if infeasible" instruction.

### Trace result

`GazeboSimROS2ControlPlugin::Configure()` (sim-side) takes only its own `<plugin>`
SDF sub-tree and the spawned model's `EntityComponentManager` as input. An
exhaustive scan of every SDF-tag-shaped string literal in the binary found exactly
five configurable elements: `parameters`, `controller_manager_name`,
`hold_joints`, `position_proportional_gain`, `remapping` — no `robot_param`, no
`robot_description`, nothing that could source an alternate description. No
`rclcpp` parameter-client or topic-subscription symbols exist in the binary
either; it never talks to `robot_state_publisher`.

`GazeboSimSystem::initSim()` (hardware-side, owns the "is mimicking joint"
message) receives the **same live `EntityComponentManager`** physics operates on
— there is exactly one ECM per spawned model. It detects mimic relationships via
`sdf::JointAxis::Mimic()` (confirmed exported by `libsdformat14.so.14`, alongside
`SetMimic`), the identical accessor gz-sim's own `Physics`/`SDFFeatures` system
uses to attempt constraint creation (the same one that produced
`[Err] [Physics.cc:1808] ... does not support mimic constraints` under dartsim).
No `urdf::Model` parsing is ever invoked for this despite `liburdfdom_model.so`
being a transitive link.

**Conclusion: `gz_ros2_control` has no code path capable of consuming a
description different from the one physics is running.** Physics and the
software servo both read mimic status off the same ECM, populated once at spawn
by the single URDF/SDF string handed to `ros_gz_sim create`.

### Independent empirical corroboration, no new run needed

The already-completed 2x2 is a perfect natural control for exactly this question:

| Run | `<mimic>` at spawn | servo installed | physics attempted constraint |
|---|---|---|---|
| dart, bullet_run2_gains, bullet_e50 | present | yes (3x) | yes |
| bullet_nomimic, bullet_nomimic_e50 | absent | no | no |

Perfect 1:1 correlation across all five prior runs — the servo installs if and
only if physics sees `<mimic>`, regardless of engine. Consistent with "single ECM,
single source of truth" and inconsistent with any theory of independent
mimic-awareness.

### Verdict — unchanged

```
J: B — BULLET MIGRATION BLOCKED
```

The only route that would remove the conflict is forking and rebuilding
`gz_ros2_control` to drop its ECM-level mimic query — modifying and maintaining a
patched vendored ROS package, not a scratch-only probe, and out of today's scope.
Per instruction: do not investigate bullet further. Recommend the smallest
DART-compatible gripper simplification instead, per the class-B fallback already
named.

### Full answers A-J, evidence

`evidence/bullet_engine_probes_20260824/split_description_feasibility_20260824/`
— README.md (full A-J trace) and the raw string-scan supporting the "no
alternate-description tag" claim. No production file modified, no engine
migrated, no simulation launched.

### Working tree, process state

HEAD `7b875a4` unchanged. `git diff --check`: clean. 12 tracked modified files,
+1136/-52, unchanged. `gz`/`robot_state_publisher`/`controller_manager`: 0
processes, unchanged (none were started this section).

---

## 2026-08-25 — checkpoint: gripper redesign is DESIGN-ONLY, zero implementation started

State preservation only. No new work performed this section beyond verification.

### What V0 work was completed

**None.** No V0 offline geometry check was written or run. The prior section
("DART-compatible Robotiq gripper representation — architecture design")
produced `docs/GRIPPER_REDESIGN_DESIGN.md` — a design document only. Its own
§10 names the smallest next implementation task
(`ur5e_robotiq_description/urdf/parallel_jaw_gripper.urdf.xacro` behind
`gripper_model:=parallel_jaw`, default OFF, then run V0) but that task was
**not started**: confirmed by direct check, the file does not exist and no
`gripper_model` string appears anywhere under `ur5e_robotiq_description/`.

### Files changed this section

Exactly one: `docs/GRIPPER_REDESIGN_DESIGN.md` (new, ~599 lines — the
architecture comparison, recommendation, and validation ladder). Nothing else
was written, edited, or generated.

### What remains unfinished

Everything in the design's own migration sequence (§3.14), starting from step 1:
write the new macro, wire it behind the flag (default off), build, then run V0
only. No macro, no flag, no build, no V0 through V10 — none of it has begun.

### Was any validation actually run?

**No.** No `colcon build`, no xacro processing, no Gazebo launch, no ROS node,
no offline Python check beyond the read-only analysis already used to write the
design document (aperture arithmetic, joint-tree parsing, force-margin
calculation — all computed from existing files, none of it a "V0 run" in the
sense the validation ladder defines, since no new macro exists yet to validate).

### Exact next task

Per `docs/GRIPPER_REDESIGN_DESIGN.md` §10, if and when authorized: write
`ur5e_robotiq_description/urdf/parallel_jaw_gripper.urdf.xacro`, wire it behind
`gripper_model:=parallel_jaw` with the production default left at
`robotiq_linkage` (i.e., unchanged current behavior), then run **V0 only**
(offline geometry/aperture identities — no simulation). Do not proceed past V0
without a fresh checkpoint.

### git status / diff --check

```
HEAD: 7b875a4 (unchanged)
git diff --check: clean (exit 0)
tracked diffstat: 12 files changed, 1136 insertions(+), 52 deletions(-)  (unchanged)
```

`git status --short`: same 12 modified tracked files as every prior checkpoint,
plus the untracked set already on record (AGENTS.md, CLAUDE.md, HANDOFF.md,
PROJECT_STATE.md, `docs/*` capture scripts and marker directories, `evidence/`,
`scripts/perception/`, two new C++ sources, `ur5e_robotiq_description/worlds/`),
plus exactly one new untracked file this section: `docs/GRIPPER_REDESIGN_DESIGN.md`.

### ROS/Gazebo processes

**Zero.** Verified directly with `pgrep -af`: no `gz`, no
`robot_state_publisher`, no `controller_manager`, no `move_group`, no ROS2
daemon. (An earlier broad `pgrep -f` pass returned false-positive counts because
this session's own Bash-tool shell wrapper command line coincidentally contains
the substrings `"ros2 "`, `"_ros2_daemon"`, and `"move_group"` in its snapshot
path/eval text — `pgrep -af` on each pattern showed the match was that wrapper
process, not a real ROS process. No real process running.)

### Nothing committed, nothing pushed.
