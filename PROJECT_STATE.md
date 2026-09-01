# PROJECT_STATE.md

## Project

UR5e + Robotiq 2F-85 pick-and-place simulation.

## Repository

`~/ur5e_pickplace`

## Main Stack

- ROS 2 Jazzy
- Gazebo Sim
- MoveIt 2
- ros2_control
- URDF/XACRO
- Python/C++ ROS 2 nodes

## Purpose

Develop and validate a reliable UR5e + Robotiq 2F-85 pick-and-place pipeline in simulation, with evidence-based testing of robot motion, grasping, object transport, placement, and perception.

## 2026-09-01 Stage-2D PlanningSceneManager Regression & Full Generalization Closeout — CURRENT AUTHORITY

- **Repository**: branch `stage2-orientation-generalization`, HEAD `a32a68b` (`docs: record PlanningSceneManager integration and verified Scene-A baseline`).
- **Milestone Outcome**: Full Stage-2D regression suite is **100% COMPLETE & VERIFIED** under the production MoveIt `PlanningSceneManager` collision lifecycle.
  - **Scene-A Lifecycle-Integrated Baseline** ($0\text{ mm}, 0\text{ mm}, 0^\circ$): **PASS** (`evidence/perception_validation_pj_20260901_021651`)
  - **D1 Case** ($+30\text{ mm}, +30\text{ mm}, +30^\circ$): **PASS** (`evidence/stage2d_pose/D1_planning_scene_20260901_0222`)
  - **D2 Case** ($-30\text{ mm}, -30\text{ mm}, -30^\circ$): **PASS** (`evidence/stage2d_pose/D2_planning_scene_20260901_0226`)
  - **D3 Case** ($+30\text{ mm}, -30\text{ mm}, +45^\circ$): **PASS** (`evidence/stage2d_pose/D3_planning_scene_20260901_0230`)
  - *(Note: Scene-A serves as the lifecycle-integrated baseline; D1, D2, and D3 constitute the 3/3 Stage-2D planar pose regression cases).*

- **D3 Critical Defect Closure**:
  - *Historical 1.5 mm Failure*: Governing fixed-side perception error projection $\approx 1.5759\text{ mm}$ resulted in predicted negative margin $\approx -0.0759\text{ mm}$, producing a rigid mechanical collision ($59.2\ \mu\text{m}$ contact sliver, $\sim 301\text{ N}$ force) halting descent at $58.4\%$ travel (`EXECUTE_FAILURE`).
  - *Current 2.0 mm Production Regression*: Governing projection is identically $1.5759\text{ mm}$, yielding a predicted positive clearance margin of $\mathbf{+0.4241\text{ mm}}$ with $M_{\text{model,working}} = 1\times 10^{-6}\text{ mm}$. Physical execution verified **0 fixed-pad and 0 moving-pad pre-close contacts**, Cartesian descent fraction $1.0000$, and full cycle `SUCCESS`. Direct regression proof that the defect remains eliminated under MoveIt collision management.

- **Verified Regression Metrics Summary**:
  - **D1 Case** ($+30\text{ mm}, +30\text{ mm}, +30^\circ$): Perception error $1.3900\text{ mm}$, yaw error $-0.0215^\circ$, descent fraction $1.0000$, TCP error $0.00028\text{ mm}$, grasp aperture $30.0022\text{ mm}$, pickup separation $+4.977\text{ mm}$, pickup clone contacts $0$, lift slip $0.0101\text{ mm}$, transport slip $0.0044\text{ mm}$, placement pre-contact separation $+4.994\text{ mm}$, placement error $2.0300\text{ mm}$, placement yaw error $0.0342^\circ$, final upright tilt $0.0001^\circ$.
  - **D2 Case** ($-30\text{ mm}, -30\text{ mm}, -30^\circ$): Perception error $1.2849\text{ mm}$, yaw error $-0.2371^\circ$, descent fraction $1.0000$, TCP error $0.00070\text{ mm}$, grasp aperture $30.0034\text{ mm}$, pickup separation $+4.981\text{ mm}$, pickup clone contacts $0$, lift slip $0.0101\text{ mm}$, transport slip $0.0027\text{ mm}$, placement pre-contact separation $+4.940\text{ mm}$, placement error $2.2234\text{ mm}$, placement yaw error $0.0055^\circ$, final upright tilt $0.000006^\circ$.
  - **D3 Case** ($+30\text{ mm}, -30\text{ mm}, +45^\circ$): Perception error $1.6104\text{ mm}$, yaw error $+0.000016^\circ$, governing fixed-side projection $1.5759\text{ mm}$, predicted fixed-side margin $+0.4241\text{ mm}$, descent fraction $1.0000$, TCP error $0.00071\text{ mm}$, pre-close pad contacts fixed=$0$/moving=$0$, grasp aperture $29.9995\text{ mm}$, pickup separation $+4.959\text{ mm}$, pickup clone contacts $0$, lift slip $0.0107\text{ mm}$, transport slip $0.0071\text{ mm}$, placement pre-contact separation $+4.968\text{ mm}$, placement error $1.9793\text{ mm}$, placement yaw error $0.0380^\circ$, final upright tilt $0.000013^\circ$, teardown SIGKILL count $0$.

- **PlanningScene Lifecycle Management**:
  - Permanent Table Isolation: $P = (\text{table} \leftrightarrow \text{base\_link\_inertia})$
  - Grasp Closure Contact Exemptions: $C_1 = (\text{pick\_target} \leftrightarrow \text{pad\_fixed\_link})$, $C_2 = (\text{pick\_target} \leftrightarrow \text{pad\_moving\_link})$
  - Table Support Exception: $S = (\text{pick\_target} \leftrightarrow \text{table})$
  - Unified lifecycle validated across all runs: World Target $\rightarrow$ Collision-protected descent $\rightarrow$ $C_1/C_2$ closure $\rightarrow$ Attachment with $S$ $\rightarrow$ 5 mm support clearance $\rightarrow$ Cloned collision check without $S$ $\rightarrow$ Live $S$ removal $\rightarrow$ 115 mm lift & transport $\rightarrow$ 95 mm protected descent $\rightarrow$ Pre-contact check $\rightarrow$ 5 mm terminal stroke with $S$ $\rightarrow$ Release & detach $\rightarrow$ World target retreat.

- **Current Project Status**:
  - Stage-1 perceived XYZ/D10: **COMPLETE / VERIFIED**
  - Stage-2A configured yaw: **COMPLETE / VERIFIED**
  - Stage-2B yaw perception: **COMPLETE / VERIFIED**
  - Stage-2C perceived yaw manipulation: **COMPLETE / VERIFIED**
  - Stage-2D planar XY+yaw generalization: **COMPLETE / VERIFIED**
  - 2.0 mm production clearance correction: **COMPLETE / VERIFIED**
  - MoveIt/FCL model-margin audit: **COMPLETE / VERIFIED**
  - PlanningSceneManager infrastructure: **COMPLETE / VERIFIED**
  - PlanningSceneManager integration: **COMPLETE / VERIFIED**
  - Scene-A lifecycle baseline: **COMPLETE / VERIFIED**
  - D1/D2/D3 PlanningScene regression: **COMPLETE / VERIFIED**
  - *Shadow Estimator*: Remains diagnostic/non-production (not promoted to production).

- **Next Development Boundary**:
  - The current planar pose generalization and PlanningScene collision lifecycle regression phase is **CLOSED**.
  - No further manipulation experiments are needed or authorized for Stage-2D qualification.
  - Future tasks should establish new milestone research objectives (e.g., full SO(3) roll/pitch orientation handling, MoveIt gripper visual-collision URDF modeling, or multi-object scenes).

## 2026-08-31 Stage-2D Planar Pose Generalization — SUPERSEDED

Superseded by the 2026-09-01 section above: PlanningSceneManager is now integrated and live-validated on Scene-A with full collision lifecycle and 2.0 mm clearance. The Stage-2D results below are retained as historical qualification of D1/D2/D3 under the 2.0 mm clearance prior to PlanningSceneManager integration.

- Repository: branch `stage2-orientation-generalization`, HEAD `0562222`
  (`geometry: raise parallel-jaw fixed-side grasp TCP clearance to 2.0 mm`).
- **`GRASP_TCP_FIXED_SIDE_CLEARANCE_M` (`scripts/lib/parallel_jaw_geometry.py`)
  is now 2.0 mm, raised from 1.5 mm.** This is now the production value;
  every earlier statement in this file or `HANDOFF.md` naming 1.5 mm as
  current is superseded. `DECLARED_CLEARANCE_M` (4.0 mm), the aperture
  split, the pre-close aperture, the final close target, controllers,
  perception, and URDF are all unchanged.
- Stage-2D extended Stage-2C with combined planar XY-offset + yaw cases:
  D1 (+30/+30 mm, yaw +30 deg), D2 (-30/-30 mm, yaw -30 deg), D3
  (+30/-30 mm, yaw +45 deg). D3 failed deterministically at the former
  1.5 mm clearance (`EXECUTE_FAILURE` during Cartesian descent,
  `evidence/stage2d_pose/D3`, `D3_retry1_diagnostics`).
- **D3 root cause is a fixed-pad/object-top hard contact during descent,
  not a controller-authority problem.** A read-only source-and-disassembly
  audit of `gz_ros2_control`'s `position_proportional_gain` (0.1, a
  compiled plugin default never set by this project's own code) found no
  saturation in its position-command path, a commanded velocity (3.53
  rad/s) already exceeding shoulder_lift's own URDF limit at the observed
  tracking error, and a sustained ~301 N contact force for 6.3 s on the
  fixed pad — the controller-gain hypothesis is **falsified**. The joint
  errors (shoulder_lift +0.0707 rad, wrist_1 -0.0590 rad) are the
  kinematic image of a rigid stop 41.4 mm short of the 100 mm descent
  target, matching a measured 22.000 mm x 59.2 um contact sliver on the
  fixed pad's inner face at closing-axis position -15.0000 mm.
- **Predicted closing-axis margin, computed from each case's own
  perceived point vs. measured ground truth:** D3 = -0.0759 mm (overlap,
  matches the observed failure) at 1.5 mm, **+0.4241 mm** at 2.0 mm; D1 =
  +0.1701 mm -> +0.6701 mm; D2 = +1.1690 mm -> +1.6690 mm.
- **2.0 mm validated before the production change, in order:** D1/D2/D3
  all PASS with zero fixed-pad and zero moving-pad contact before
  `GRIPPER_CLOSE` under the diagnostic override
  (`evidence/stage2d_pose/D{1,2,3}_clearance2mm_diag`); D3 re-run through
  the actual production default (no override) PASSes identically,
  confirming the resolved TCP offset X = -0.025500 m and
  `fixed_side_clearance_m_override: null`
  (`evidence/stage2d_pose/D3_production_default_2mm`); two independent
  Scene-A/Stage-1 cycles at the production default PASS with all 11 gates
  and zero pad contact
  (`evidence/stage1_scene_a_production_2mm_confirmation{,_run2}`).
- **Stage-2D D1/D2/D3 are qualified at the 2.0 mm production default.
  Stage-2D is COMPLETE.**
- **Accepted placement trade-off (not "no regression"):** raising the
  clearance measurably shifts placement position, since the grasp TCP
  moves 0.5 mm relative to the object. Scene-A: +0.388 mm (two
  independent cycles, +0.3876 mm and +0.3880 mm, agreeing to 0.4 um).
  D1: +0.524 mm. D2: +0.661 mm. All remain comfortably inside the 10 mm
  `placement_pos_err_mm_max` gate (`stage2a_analyzer.py`).
- **Stage-2C (C1-C3) was NOT rerun.** Their perceived-yaw consumption,
  configured/spawned-yaw decoupling, axial-yaw semantics, and physical
  manipulation results remain valid for their actual diagnostic 2880x2160
  runs. Their numerical XYZ and yaw-error values are resolution-specific and
  are not 960x720 production qualification. Their closing-axis fixed-side margins
  increase by the full 0.5 mm at 2.0 mm (C1 +1.055 -> +1.555 mm, C2
  +1.388 -> +1.888 mm, C3 +1.041 -> +1.541 mm, computed from their own
  existing evidence). Their recorded placement-position numbers
  (1.535-1.583 mm) were measured under the **former 1.5 mm** clearance and
  are retained as historical; see `HANDOFF.md`'s Stage-2C section
  (now superseded) for the full addendum.
- Detailed case metrics, the D3 mechanism evidence, and the full
  validation chain are recorded in the current Stage-2D section of
  `HANDOFF.md`.
- **NEXT:** no next-stage objective is authorized yet. Two open,
  already-flagged gaps: full SO(3) orientation change (roll/pitch) remains
  diagnostic-only; the gripper has no MoveIt collision geometry, so
  fixed-pad/object contact stays invisible to planning-time checks. At the
  34 mm pre-close aperture (unchanged), the object's derived physical gap
  from the moving pad is now 2.0 mm too (`preclose_aperture - width -
  c_fixed_m`) — numerically equal to the fixed side, though
  `MOVING_SIDE_CLEARANCE_M` itself (a separate named constant, feeding
  only `DECLARED_CLEARANCE_M`'s sum) remains its own unchanged 3.5 mm. No
  further headroom remains on the fixed side without widening
  `DECLARED_CLEARANCE_M`; the durable fix for any future overlap is the
  underlying ~1.3-1.6 mm closing-axis perception bias, not a third
  clearance increase. See `HANDOFF.md`'s Stage-2D "Next stage" subsection
  for detail.

## 2026-08-31 Stage-2C Orientation Generalization — SUPERSEDED

Superseded by the Stage-2D section above: Stage-2D closes with the
production fixed-side clearance raised to 2.0 mm, which C1-C3 below did
not run under. The C1-C3 resolution claims in this historical section are
also superseded by the provenance correction below: these were diagnostic
2880x2160 runs, not 960x720 production runs. Their yaw-path and physical
manipulation conclusions remain valid for the actual 3x runs; their numerical
XYZ and yaw-error values are not 960x720 qualification.

- Repository: branch `stage2-orientation-generalization`, HEAD `eb74c27`.
- Stage-2C is closed with three independent perceived-yaw full cycles:
  C1 spawned +30 deg, C2 spawned -30 deg, and C3 spawned +45 deg. Each
  retained configured pick/place yaw 0 deg, used fresh `pose_world` yaw,
  production thresholds/controllers, one planning attempt, and no retries or
  tuning. All three are PASS.
- Authoritative semantics: configured yaw is decoupled from spawned ground
  truth; yaw is sourced from `/object_detector/pose_world`; yaw comparisons
  are axial/mod 180 via `axial_difference()`.
- Results (diagnostic 2880x2160 only): perceived yaw error <= 0.049 deg,
  position error ~0.45 mm,
  aperture ~30.00 mm, yaw-invariant grasp tilt <= 0.063 deg, lift and
  transport slip ~0.01 mm, placement position 1.5-1.6 mm, and axial
  placement yaw <= 0.056 deg. Full SO(3) change is diagnostic only.
- CORRECTION (2026-08-31 provenance audit): C1-C3 ran at diagnostic
  2880x2160, not the 960x720 launch default. The committed wrappers did not
  forward resolution arguments; the recorded runtime commands used a
  transient inline Python shim that appended
  `camera_width:=2880 camera_height:=2160` before process creation. The `_3x`
  evidence-directory names are accurate. The ~0.45 mm XYZ results and
  numerical yaw-error values are resolution-specific and must not be used
  for 960x720 production qualification. Perceived-yaw consumption,
  configured/spawned-yaw decoupling, axial-yaw semantics, and physical
  manipulation results remain valid for the actual 3x runs. Separate 960x720
  Stage-2B evidence remains separate and must not be conflated with C1-C3.
  The Stage-2B perception chain remains frozen.
- Negative control (configured 0 deg, spawned +30 deg, perceived yaw OFF)
  aborted during descent before grasp closure. It is comparative/incomplete
  evidence only; yaw mismatch is not identified as the abort cause.
- Known non-blocking telemetry cleanup: `configured_object_yaw_deg` is NaN
  when `use_perceived_yaw=false`.
- Detailed case metrics and evidence paths are recorded in the current
  Stage-2C section of `HANDOFF.md`.

## Historical Milestone Status (superseded)

```
HISTORICAL STATE — 2026-08-30 (superseded by the section above):

  PRODUCTION: parallel-jaw P=200 (controllers.yaml, commit e37383e) is now
  production, not diagnostic-only. D10 XYZ estimator is production. Fixed-
  side clearance 1.5 mm, plan_attempts=1, both unchanged. HEAD = e37383e.

  STAGE-1: requalified end-to-end under production P=200.
    G1-G5 production-default P=200 regression: 5/5 PASS.
    Scene-A production-default P=200 repeatability: 5/5 PASS
      (evidence/stage1_scene_a_final_repeatability_20260830/R1..R5).
    Supersedes the P=50 baselines recorded further below (kept as history).

  STAGE-2A: configured-yaw manipulation feasibility COMPLETE.
    D10 perception-only yaw regression P0-P4: 5/5 PASS.
    O0-O4 perception-driven-XY, P=200, full cycle: 5/5 physical PASS
      (O2's stored verdict reads FAIL only from the analyzer's pre-lift
      window artifact -- direct ground truth is PASS; see HANDOFF.md).
    O2-O4 are NOT blocked; the O1 configured-center diagnostic already ran.
    LIMITATION: XYZ is perception-driven; yaw is still read from
    scene.yaml's object.pick_pose.yaw, not estimated. Not yet true
    perception-driven yaw generalization.

  NEXT: Stage-2B -- yaw perception implementation and perception-only
  qualification. Start with an isolated mask-orientation estimator + unit
  tests (no ROS topic wiring yet). Yaw is axial/mod-180: use the shortest
  axial difference canonicalised into [-90, +90) deg, NOT ordinary wrap_pi.
  Stage-2C (manipulation with perceived yaw, configured yaw deliberately
  decoupled from spawned yaw) follows only after Stage-2B qualifies.
  Full detail: HANDOFF.md 2026-08-30 current-authority section.

  --- superseded below, kept as history ---

  Generalization Stage-1 — RESOLVED — G1-G5 ALL PASS (2026-08-28, P=50)
  Position generalization across five distinct object poses experimentally
  validated. Planner-attempt configuration difference (G1-G4 at
  plan_attempts=20, G5 qualification at plan_attempts=1) explicitly
  documented; see HANDOFF.md 2026-08-28 section for caveats.
  Superseded by the 2026-08-30 P=200 requalification above.

  Scene-A Perception-Driven Pick & Place Repeatability — PASS (2026-08-27, P=50)
  5/5 consecutive clean perception-driven cycles PASSED with parallel_jaw baseline.
  Superseded by the 2026-08-30 P=200 requalification above.

  Post-cleanup Scene-A regression — PASS (2026-08-28), CONFIRMATION ONLY
  One cycle via the unmodified run_5_cycles.py harness after the baseline
  cleanup below: all 11 Stage-1 criteria PASS. Does NOT add to either
  campaign's N; see HANDOFF.md 2026-08-28 "Baseline Frozen" section.
```

## 2026-08-30 Stage-1 P200 Requalification + Stage-2A Complete — CURRENT AUTHORITY

Branch `stage2-orientation-generalization`, HEAD `e37383e` (`control: raise
parallel-jaw grasp gain to validated value`). `parallel_jaw_gripper_controller`
p=200.0 is production in `controllers.yaml`, not diagnostic-only. D10 XYZ
estimator, 1.5 mm fixed-side clearance, and `plan_attempts=1` are unchanged.

Stage-1 is requalified end-to-end under production P=200: G1-G5 regression
5/5 PASS, and Scene-A repeatability 5/5 PASS (five consecutive clean cycles,
no retries/tuning between them; evidence
`evidence/stage1_scene_a_final_repeatability_20260830/R1..R5`). This
supersedes the P=50 baselines recorded further below.

Stage-2A configured-yaw manipulation feasibility is COMPLETE: O0-O4
(perception-driven XY, configured yaw 0/+-15/+-30 deg, production P=200)
all 5 physically PASS. O2's stored `cycle_metrics.json` verdict reads FAIL
only because the generic analyzer's fixed pre-lift quiescence window
overlaps the intentional P=200 force-seating motion (a known, documented
measurement-window artifact -- the analyzer itself is unmodified); direct
ground-truth re-measurement is PASS (lift slip 0.0125 mm, tilt 0.0244 deg).
The same artifact affects G1's P=200 regression run; its direct ground truth
is likewise PASS (lift slip 0.00492 mm, tilt 0.0444 deg). O2-O4 are NOT
blocked, and the O1 configured-center/zero-clearance/force-authority
diagnostics are historical completed work that led to the P=200 fix, not a
pending task. Some `evidence/stage2a_o{0,3}_..._run{1..4}/` directories are
infrastructure/preflight aborts (harness exited before `m3_grasp` launched,
no `m3_grasp.log`), not repeated manipulation attempts.

**Current limitation:** XYZ is perception-driven; object yaw is still read
from `config/scene.yaml`'s `object.pick_pose.yaw`, not estimated from sensor
data (`m3_grasp.cpp`'s perception substitution replaces translation only).
True perception-driven yaw generalization is not yet validated.

**Next:** Stage-2B (yaw perception implementation + perception-only
qualification) — begin with an isolated mask-orientation estimator and unit
tests before any ROS topic wiring. Design caution: yaw is axial/mod-180 for
a 2-fold-symmetric object; use the shortest axial difference canonicalised
into `[-90, +90)` degrees, NOT ordinary `wrap_pi`/`[-180,+180)` wrapping.
Stage-2C (manipulation with perceived yaw, configured yaw deliberately
decoupled from spawned yaw) follows only once Stage-2B qualifies on its own
acceptance criteria. Full detail: `HANDOFF.md`'s 2026-08-30 current-authority
section.

## 2026-08-29 Stage-2A Orientation — D10 Integrated; O1 Lift-Onset Failure — superseded 2026-08-30 (historical: this predates the P=200 force-authority fix now in production; O2-O4 are NOT blocked and have since run and passed; HEAD is now e37383e, not a056023 — see the current-authority block above)

Repository authority AT THE TIME was branch `stage2-orientation-generalization` at
commit `a056023` (`perception: use robust D10 object position estimator`). The
production estimator back-projects the selected connected component's
subpixel centroid with a deterministic 10% symmetric trimmed mean of finite,
positive masked depths. The affected package built successfully and all
focused D10 tests passed.

The post-D10 perception-only yaw regression P0-P4 passed 5/5, and every tested
yaw now has positive predicted fixed-side clearance. The subsequent full O1
manipulation rerun at +15 degrees did not qualify despite the manipulation
node returning `SUCCESS`: peak tilt was 3.3757 degrees and lift slip was
1.5656 mm. Transport slip (0.0364 mm), placement position error (1.8865 mm),
and placement yaw error (0.022 degrees) passed.

Forensic comparison with the preserved pre-D10 O1 evidence established two
distinct mechanisms. The historical O1 suffered pre-close mechanical
interference during descent and tipped before final close; D10 eliminated that
collision. The current O1 remains upright through pre-close, descent, and
close, then develops tilt/slip at lift onset while seating from unilateral to
bilateral pad contact. Residual XY misregistration plus the asymmetric initial
contact is the current failure classification. Once seated, the object is
stable through transport.

O2-O4 were **BLOCKED at the time this section was written**; they have since
run and passed (2026-08-30, see current-authority block above), and the O1
configured-center diagnostic control described below has already been
carried out — it led directly to the P=200 force-authority fix now in
production. This paragraph is retained as historical description of the
reasoning at the time, not as a pending instruction.

## 2026-08-28 Baseline Frozen — Stage-1 Commits + Local Tag

Session-end state, recorded for a clean continuation. Supersedes the
previous "Session End — Next Task: READ-ONLY WORKING-TREE AUDIT" entry --
that audit ran, its findings were reviewed, and the resulting cleanup is now
committed. Full detail: `HANDOFF.md` §"2026-08-28 Baseline Frozen".

- Generalization Stage-1 G1-G5 remains validated and is now **frozen**
  (section below).
- The READ-ONLY working-tree audit, the D6 (`static_scene_tf` re-publish
  timer)/D7 (`tf_lookup_timeout_s`) forensic resolution, and the controlled
  baseline cleanup (legacy `allowed_start_tolerance: 0.05` reverted, stale
  parallel_jaw comments corrected, dead `ur5e_robotiq.srdf` duplicate
  removed, `.gitignore` hardened against raw evidence/generated artifacts)
  all ran to completion in prior sessions and are recorded in `HANDOFF.md`.
- The cleaned tree is committed as three commits on `rgbd-perception`
  (production baseline, tooling, docs). A local (unpushed) annotated tag
  `stage1-generalization-pass` is intended as the final step of this same
  effort, once a post-commit build re-verification passes -- see
  `HANDOFF.md` for exact hashes and `git log --oneline -8` / `git tag -l`
  for the actual current state.
- The historical G5 15.001 s transport-planning stall remains an
  **unexplained, non-reproduced transient planner/runtime anomaly** -- OPEN,
  not fixed. `thresholds.plan_attempts` remains `1` (diagnosability only).
  `thresholds.tf_lookup_timeout_s` remains `15.0` (never approached by any
  validated run). Neither is a claimed fix for anything.
- Raw bulk evidence (`evidence/`, ~7.7 GB) is intentionally excluded from
  Git and stays local-only; only curated summaries are ever committed.

**Historical next-task note:** at the 2026-08-28 freeze, Generalization Stage 2
had not started. It is now COMPLETE for configured-yaw feasibility (Stage-2A,
2026-08-30) and both this file's top "LATEST VERIFIED STATE" block and
`HANDOFF.md`'s 2026-08-30 current-authority section supersede this old resume
point. Position generalization (Stage 1) and repeatability remain closed, and
are now also requalified under production P=200.

## 2026-08-28 Generalization Stage-1 — RESOLVED — G1-G5 ALL PASS

Supersedes the G5 verdict in the "2026-08-27 Generalization Stage-1" section
immediately below (kept for its G1-G4 data and the original G5 failure's
root-cause chain). G5 was re-attempted twice after that session and now
qualifies as a full Stage-1 PASS.

**G5 final qualification** = [0.480, -0.120, 0.7725]: perception error
1.4959 mm, selected pregrasp `[-0.489138 -0.866632 1.447793 0.989635
1.570796 1.081659]`, Cartesian fraction 1.0000, Stage-2 TCP error 0.000650 mm,
grasp aperture 29.9995 mm, max grasp tilt 0.0626 deg, lift slip 0.0335 mm,
transport slip 0.0472 mm, placement error 1.8906 mm, final orientation error
0.0511 deg, transport planning time 16.1 ms. Full cycle result: **PASS**.
Evidence: `evidence/g5_qualification_20260828_000018/`.

**Historical original G5 anomaly** (transport `PLAN_FAILURE` after 15.001 s,
occurring after perception/grasp/lift had already passed) did **not
reproduce** across two subsequent independent G5 executions. Its cause
remains **unexplained** and is retained as an **OPEN MoveIt/OMPL
runtime-planning risk** -- it is NOT classified as a manipulation-
generalization failure, since perception, grasp, and lift passed at the
anomaly's own quality bar and both reproduction attempts passed the full
cycle including transport.

**Offline diagnosis**: standalone KDL IK calls at the production 5 ms timeout
show seed sensitivity (~13-16% single-call failure), but a production-like
reproduction of MoveIt's actual `IKConstraintSampler` goal-sampling path
succeeded 6000/6000. KDL goal-sampling starvation is therefore **refuted** as
the original G5 anomaly's cause.

**`plan_attempts`: 20 -> 1** -- a **diagnosability change only**. At
`count <= 1` MoveIt's single-planner path calls `logPlannerStatus()` on
failure, which the `count > 1` `ParallelPlan` branch never does. Do NOT read
the successful 16.1 ms G5 transport plan at `plan_attempts=1` as evidence
that this change fixed the historical stall -- the stall never reproduced
even at the original `plan_attempts=20` setting, so its absence here proves
nothing about the parameter change's effect on it.

**Caveats before citing this as closed**:
1. G1-G4 ran at `plan_attempts=20`; the final G5 qualification ran at
   `plan_attempts=1`. The five-pose campaign is therefore **not strictly
   configuration-homogeneous**. Re-running one additional G-pose at
   `plan_attempts=1` would NOT fix this -- only a full G1-G5 re-run at one
   common setting would.
2. G5's lift slip / transport slip / tilt were measured with a
   **reconstructed** observer (the original `/tmp/m3_diag_observer.py` was
   lost when `/tmp` was wiped). Definitions are documented in
   `evidence/g5_qualification_20260828_000018/README.md`; results agree
   closely with the G1-G4 distributions, which corroborates but does not
   prove definitional identity with the original observer.

**Stage-1 conclusion**: Position generalization across G1-G5 is
experimentally validated, with the planner-attempt configuration difference
explicitly documented above. Full detail: `HANDOFF.md` §"2026-08-28
Generalization Stage-1 — RESOLVED".

## 2026-08-27 Generalization Stage-1 — G1-G4 PASS, G5 FAIL (transport planning), campaign stopped — SUPERSEDED, kept for root-cause detail

Four Generalization Stage-1 trials at distinct object poses (G1 [0.450,-0.100], G2
[0.450,-0.200], G3 [0.500,-0.150], G4 [0.400,-0.150], all Z=0.7725) each ran the
full unmodified perception-driven `parallel_jaw` baseline end-to-end
(pregrasp->descent->grasp->lift->transport->place->release) and **PASSED every
Stage-1 acceptance criterion** with wide margin.

**G5 [0.480, -0.120, 0.7725] FAILED** at Stage 4 (transport): perception,
pregrasp selection, descent, grasp, and lift all passed cleanly (perception
error 1.4959 mm, Cartesian fraction 1.0, TCP error 0.000343 mm, lift slip
0.0301 mm), but MoveIt/OMPL RRTConnect returned `PLAN_FAILURE` planning the
transport standoff above the place pose. The object remained held, never
dropped or released. No retry or tuning was performed. Not a perception,
grasp, or lift regression -- a transport-leg planner failure, consistent with
previously documented stochastic OMPL/RRTConnect behavior. **Campaign stopped
here per instruction; no further generalization poses were run.**

Full detail: `HANDOFF.md` §"2026-08-27 Generalization Stage-1". Evidence:
`evidence/generalization_stage1/pose_G{1,2,3,4,5}/` (G5's `cycle_metrics.json`
has `verdict: FAIL` with `null` fields for the metrics that never occurred).

## 2026-08-27 Repeatability Campaign & Final Scene-A Baseline

The Scene-A perception-driven pick-and-place repeatability campaign evaluated to **5/5 PASS** (**CONFIRMED PASS**).

### Confirmed Baseline Metrics
- **Perception-driven pick/place repeatability:** **5/5 PASS**
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
- Bad pregrasp IK branch caused excessive trajectory demand and joint tolerance violations.
- Deterministic selector fixed branch selection.
- Grasp descent tipping was caused by fixed-pad overlap from the old `X = -0.027 m` offset.
- `X = -0.026 m` restored positive clearance and upright grasp.
- Release bug was fixed by commanding full-open `q = 0`.

## Current Execution Investigation — 2026-08-26

The Scene-A perception validation wrapper lifecycle hang is **fixed and
validated**. It detects `m3_grasp` completion, cleans up its ROS/Gazebo/MoveIt
children, and returns deterministically. A perception-mode pre-grasp
`PLAN_FAILURE` was investigated and cleared as stochastic OMPL/RRTConnect
behaviour; the established perception-mode path-constraint fix remains intact.

The MoveIt trajectory-execution watchdog correction is **validated**. Runtime
`trajectory_execution` parameters are:

- `allowed_execution_duration_scaling = 1.2`
- `allowed_goal_duration_margin = 1.5`
- `allowed_start_tolerance = 0.01`

The current real failure is downstream: `arm_controller` returns
`GOAL_TOLERANCE_VIOLATED` for `shoulder_lift_joint`, with final error about
`0.093015 rad`. The deterministic Cartesian scaling is confirmed active:
timestamps x2, velocities x0.5, accelerations x0.25.

Important correction: MoveIt reports **12 Cartesian samples**, but the actual
post-scaling `FollowJointTrajectory` message contains **20 points**. The
previous 12-vs-20 point-density hypothesis is therefore invalid. The captured
failing trajectory has 20 points, duration `3.777624982 s`, populated
positions/velocities/accelerations, empty effort, and runtime controller
interpolation method `splines`. Its start state is:

```
[-0.571294174, -1.268085081, 2.127037774,
 -2.428692762, -1.570836761, -2.142085821]
```

Relevant evidence:

- `evidence/perception_watchdog_confirmation_20260826_152825`
- `evidence/trajectory_density_ab_20260826`

The historical 20-point E2 FJT message was not recoverable and must not be
treated as an exact reproducible baseline.

**Exact next task:** run exactly one isolated direct replay of the captured
20-point post-scaling `JointTrajectory` through `arm_controller` using
`FollowJointTrajectory`, from the same settled start state. Do not run
perception, MoveIt planning, a full pick/place cycle, or tune parameters. If
it reproduces the ~0.093 rad shoulder error, investigate
`ros2_control -> gz_ros2_control -> Gazebo`; if it succeeds, investigate
runtime context/load/state differences.

The validated perception-driven boundary is:

```
RGB-D -> detector -> camera-frame 3D -> TF2 world position
      -> perception-derived pre-grasp -> Cartesian descent
      -> physical grasp -> STOP
```

No perception-derived lift, transport, place, or release has been validated.

F3 Scene A P12.5 controlled scientific lift-only experiment (`RUN_ID=20260823_190610_13717`)
completed the full execution boundary under strict barrier synchronization and
evaluated to **FAIL**:
- F2 prerequisite passed; stationary G0 established (p2p 1.218 mm);
- P12.5 strict switch and exclusive ownership confirmed (0/27,724 saturated cycles);
- G0->L1 slip = 8.048 mm; G0->L2 slip = 26.054 mm (threshold 5.000 mm breached);
- L1->L2 slip = 18.079 mm (continuous slipping during 2.0 s post-lift dwell);
- Maximum post-lift wrist-relative displacement = 26.272 mm;
- G0->L2 orientation change = 0.3783 rad (21.68 deg); gross world-Z lift = 113.09 mm;
- Bilateral fingertip contact remained continuous throughout;
- Right inner-knuckle contact lost during post-lift dwell (sim t=108.758 s);
- P12.5 suppressed limit cycles during static hold but did NOT solve dynamic lift retention;
- Scenes B-D must NOT proceed; gain interpolation should not continue without new evidence.
- Evidence: `evidence/f3_p12_5_lift_scene_A/A/lift_analysis_20260823_190610_13717.json`.

Frozen production candidate from F2: descent pre-close command
`0.070000 rad`, produced by `config/scene.yaml`
`grasp.preclose_margin_rad: 0.4678679450464813` against the unchanged
expected grip angle `0.5378679450464813 rad`. Do not tune this value.

Evidence status: **F2 durable raw evidence was regenerated on 2026-08-23 and
the result reproduced.** The original F2 validation's raw artifacts were
written to `/tmp` and are gone; the frozen matrix was therefore rerun, without
tuning, into repository-local non-ignored storage:

```
evidence/f2_0070_regeneration_20260823_114505/    (~770 MB, MANIFEST.sha256)
```

All four scenes passed every frozen criterion on a clean-start stack, with
`position_source=perceived`, Cartesian fraction 1.0, P1->P2 = 0.0000 mm, no
premature descent contact, bilateral fingertip engagement, and the F2 stop
respected. Perception error, pre-close and seating reproduce the accepted
figures to within noise. Read that directory's `README.md` for the protocol,
the layout and three stated limitations. The original F2 measurements are
preserved unchanged in `HANDOFF.md` and `docs/HANDOFF_RGBD_PERCEPTION.md`
§11.3; the regenerated set is reproducibility evidence, not a replacement.

Still not durable: the strict no-object regression and the final `0.070 rad`
classical regression were not part of this regeneration.

## Verified Historical Baseline

The project has previously demonstrated a validated M3/M5 pick-and-place baseline with 20/20 successful cycles.

Treat this as historical evidence, not automatic proof that the current uncommitted working tree still preserves the same behavior.

## Current Development Area

Current development is focused on:

- RGB-D perception (primary)
- object detection
- grasp/contact behavior
- gripper/object interaction
- simulation reliability
- regression testing

### Verified perception milestones

A (camera + observation pose), B (metric depth), C (sensor-only detector),
D (camera-frame 3D position) and E (world-frame position via TF2) are all
validated and PASS. C's thresholds and sync are FROZEN; A's camera geometry is
FROZEN.

The perception chain now publishes a validated world-frame object position:

```
RGB-D -> detector mask -> pinhole back-projection -> camera_optical_frame point
      -> TF2 -> object_detector/position_world  (PointStamped, frame_id = world)
```

Accuracy at the four validated object positions: worst Euclidean error
1.6136 mm, worst single coordinate 1.2736 mm, repeatability 0.000000000 mm.
The TF transform contributes no error of its own — world errors are the exact
re-expression of the camera-frame errors.

A validated opt-in consumer exists. `m3_grasp` substitutes the perceived
position for the configured one under `use_perceived_position:=true` (plus
`require_perception`, `pregrasp_only` and `grasp_only` for strict F1/F2
validation). The default is false, and with it false the classical pipeline is
unchanged — re-confirmed by full classical regression runs on 2026-08-23.

Milestone F1 (perception-derived pre-grasp) **PASS** on 2026-08-23. The guarded
binary passed strict no-object/no-motion behavior, the frozen four-scene matrix,
and fallback-source observability.

Milestone F2 (perception-derived approach and grasp) **PASS** on 2026-08-23,
and supersedes F1 as the current validated boundary. Do not treat lift,
transport, place, or release driven by perception as validated.

Semantics that the next milestone must respect: the published point is the
object's VISIBLE TOP-SURFACE position, not its geometric centre, and no
orientation/yaw is estimated.

Detailed evidence: `docs/HANDOFF_RGBD_PERCEPTION.md`, sections 8 and 9.

## Current Working Tree

There are active uncommitted changes and experimental files.

These may include changes related to:

- `ur5e_pick_place`
- `ur5e_robotiq_description`
- simulation launch/configuration
- URDF/XACRO
- object detection
- Gazebo worlds
- experiment/capture scripts
- recorded experiment evidence

Treat existing changes as intentional until reviewed.

Do not reset, clean, overwrite, or discard them automatically.

## Current Evidence Rule

Do not treat any of the following alone as proof of pick-and-place success:

- node reports SUCCESS
- action returns SUCCESS
- trajectory completes
- gripper closes
- simulation visually appears correct

Where relevant, verify against measurable evidence such as:

- robot joint state
- gripper joint state
- object pose
- contact state
- Gazebo state
- TF
- controller state
- recorded experiment evidence

## Current Agent Roles

### ChatGPT
Project planning, robotics reasoning, review, research interpretation, milestone decisions, and coordination between agents.

### Claude Code
Primary debugging and repository investigation.

### Codex
Focused implementation, code changes, refactoring, and well-scoped engineering tasks.

### Antigravity
Independent audit, second-opinion investigation, and verification of important results.

These roles are defaults, not absolute restrictions.

Avoid making multiple agents independently repeat the same investigation unless an independent second opinion is intentionally required.

## Agent Startup

Before substantial work:

1. Read `AGENTS.md`.
2. Read `PROJECT_STATE.md`.
3. Read `HANDOFF.md`.
4. Inspect `git status`.
5. Inspect relevant existing diffs.
6. Read the relevant milestone-specific documentation.
7. Work only from the actual repository/evidence state.

## Detailed Historical Information

Do not expand this file into a complete project diary.

Keep detailed experiment results and milestone history in `docs/`.

Relevant handoff documents currently include:

- `docs/HANDOFF_M3.md`
- `docs/HANDOFF_RGBD_PERCEPTION.md`

## Updating This File

Update this file only when the project state materially changes, for example:

- a milestone is validated
- a major bug is confirmed and resolved
- architecture changes
- the primary development objective changes
- a new baseline becomes verified

Keep this file concise.

Do not record every command or minor experiment here.

## Current Verified State

Milestone F2 (perception-derived approach and grasp) is **PASS**. The universal
`0.070 rad` descent pre-close command passed A-D with Cartesian fraction 1.0,
zero measured P1->P2 disturbance, no monitored premature contact, sub-micrometre
final TCP translation error, physical close/contact, and the explicit
grasp-only stop. Real-STL minimum right-inner clearances were A/B/C/D
`1.198/6.841/7.680/3.603 mm`; actual follower states, not the master alone,
remain the authoritative geometry evidence. Strict no-object passed with
`PERCEPTION_TIMEOUT` and no post-timeout manipulation, and a full classical
regression passed with `position_source=configured` and transport SUCCESS.

F2 establishes no lift, transport, place, or release from a perception-derived
grasp. All scenes retain a separate approximately 21.6--22.3 mm final-closure
seating warning; it occurs after the accepted approach-disturbance interval and
was correctly identified as the most likely threat to F3 retention.

RESOLVED 2026-08-24: that seating is not a translation artifact. The 2026-08-24
root-cause analysis showed it is the object being SHOVED -21.36 mm and PITCHED
+14.43 deg into a wedge by a one-sided closure, reproducible across six runs and
four controllers to 0.01 deg / 0.01 mm. It is the direct mechanical antecedent
of the F3 lift failure.

Milestones A-E are established; read `docs/HANDOFF_RGBD_PERCEPTION.md` rather
than re-running those experiments, unless current evidence contradicts them.

## Checkpoint 2026-08-25 — gripper redesign is DESIGN-ONLY, implementation not started

`docs/GRIPPER_REDESIGN_DESIGN.md` (new, ~599 lines) records the full
architecture design: candidate comparison (A-E), recommended single-DOF
prismatic parallel-jaw architecture, exact production-change surface,
validation ladder V0-V10, and a redesigned F3 G0 concept. **Nothing from it has
been implemented.** Verified directly: no
`ur5e_robotiq_description/urdf/parallel_jaw_gripper.urdf.xacro` exists, no
`gripper_model` string appears anywhere under `ur5e_robotiq_description/`, no
build/xacro/simulation was run. HEAD unchanged at `7b875a4`, `git diff --check`
clean, tracked diffstat unchanged (12 files, +1136/-52), zero ROS/Gazebo
processes running (confirmed with direct `pgrep -af`, not a broad grep --
this session's own Bash-tool wrapper process line contains substrings that
false-match naive `-f` patterns like "ros2 "/"move_group"; none were real).

## Immediate Next Step

Per the design doc's own §10: **write the new macro, wire it behind
`gripper_model:=parallel_jaw` with production default left at
`robotiq_linkage` (unchanged), then run V0 only** (offline geometry/aperture
identities -- no simulation). Do not proceed past V0 without a fresh
checkpoint. Full sequence, validation criteria, and rollback strategy are in
`docs/GRIPPER_REDESIGN_DESIGN.md`; this section is a pointer to it, not a
restatement.

Superseded pointer, kept for continuity: the paragraph immediately below
predates the design doc and describes *why* this path was chosen (the doubly
confirmed bullet-migration BLOCKED verdict). It remains accurate as
background; the actionable next step is the paragraph above.

**Design the smallest DART-compatible gripper simplification that preserves the
existing evidence base.** Do not investigate bullet further; do not migrate
engines; do not modify production files without fresh authorization.

Verdict B (BULLET MIGRATION BLOCKED) is now doubly confirmed:

```
VERDICT: B — BULLET MIGRATION BLOCKED, for the configuration this project needs.
```

1. (2026-08-24, execution) A 2x2 experiment under bullet-featherstone with a
   dartsim control showed the gripper master is unactuatable under bullet when
   native `<mimic>` is present with `gz_ros2_control` in the loop — in three
   variants, including one with 50x the production torque ceiling.
2. (2026-08-24, later, static trace) The proposed escape route — give gz-sim a
   description WITH `<mimic>` and `gz_ros2_control` one WITHOUT, so its software
   servo never installs — was traced against the actual installed binaries
   (`libgz_ros2_control-system.so`, `libgz_hardware_plugins.so`,
   `libsdformat14.so.14`) rather than assumed. **It is architecturally
   infeasible**: `gz_ros2_control`'s mimic-servo reads `sdf::JointAxis::Mimic()`
   directly off the SAME `EntityComponentManager` physics runs on — there is no
   code path for it to consume a different description. No probe was built,
   per the standing "stop if infeasible" instruction; the already-completed 2x2
   independently corroborates this (servo installs if and only if physics sees
   `<mimic>`, regardless of engine, across all five prior runs).

The only way to remove the conflict would be forking and rebuilding
`gz_ros2_control` to drop its ECM-level mimic query — maintaining a patched
vendored ROS package, not a scratch probe, and out of scope without a fresh,
explicit decision to take that on.

**Next task, per the class-B fallback named in the prior verdict:**

1. design the smallest DART-compatible gripper simplification that preserves the
   existing M3/M5/F2 evidence base — likely candidates to evaluate: tightening
   the software mimic servo's own gains/limits so followers track better without
   a native constraint, or a reduced-DOF gripper representation that removes the
   inner-knuckle geometry responsible for the wedge, evaluated against the
   already-measured 53.3 mm/rad clearance-vs-mimic-error and
   pitch = master - fingertip_theta relationships;
2. independently of the engine question, amend the F3 retention criterion so G0
   requires the object to be clear of the table and load-bearing before the
   baseline is taken. Unchanged and still outstanding since the F3 root-cause
   session.

Evidence:
`evidence/bullet_engine_probes_20260824/compat_probe_run_20260824/` (73 MB, the
executed 2x2) and
`evidence/bullet_engine_probes_20260824/split_description_feasibility_20260824/`
(the static trace, README.md with full A-J answers).

Still prohibited without explicit authorization: migrating the engine, modifying
any production file, Scene A reruns, Scenes B-D, controller gain interpolation,
friction changes, physics changes, regenerating F2/F3 evidence, and forking or
rebuilding `gz_ros2_control`. P12.5 remains the frozen, characterized reference
controller — characterized, not validated.

## Historical F2 Failure and Root-Cause Investigation

Superseded by the PASS above. Retained because the root causes are real,
were measured, and constrain any future change to the descent configuration.
None of the statements below describe the current state.

F2 first FAILED at Scene B on a free-air pre-close. Diagnosed: Scene A left the
gripper driven toward `0.8 rad`; the sequential harness did not reopen/reset it,
and reversing from approximately `0.8` to `0.2379 rad` at the frozen
`0.1 rad/s` limit cannot enter the `0.01` rad goal tolerance within the helper's
5 s wait. The `0.313260 rad` `TIMED_OUT_HELD` value was an in-flight sample, not
contact or equilibrium. Fixed by adding verified gripper-open initialization
between independent trials (harness only): initialization-only passed
`0.797012 -> 0.003787 rad`; sequential Scene B passed
`0.797112 -> 0.003954 rad`, and its prior pre-close timeout vanished.

F2 then FAILED again on the approach-disturbance criterion. The proven
mechanism was the right inner knuckle's oblique lower surface contacting the
object's -X top edge during the last few millimetres of descent and wedging it
along the closing axis — not the fingertips, and not perception error.

The first correction candidate, `0.130 rad`, passed Scene B but A-D
generalization stopped at Scene A (`1.010284 mm`, premature right-inner
contact). Offline diagnosis showed the identical master target did not produce
equivalent gripper geometry: at equivalent descent height Scene A's right-inner
follower was about `0.05777 rad` more closed than Scene B's, despite equivalent
master angles. This dynamic follower-state difference — not world translation
and not the perception-error direction — is the measured cause of A contacting
while B clears.

**Standing lesson: master joint position alone is not a deterministic proxy for
follower/collision geometry while the coupled joints are still relaxing.** The
accepted `0.070 rad` value was chosen from a real-collision-STL sweep evaluated
against directly measured follower states, and validated scene by scene.
