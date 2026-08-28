# F3 — Controlled P12.5 Scene-A Lift: Predeclared Plan

Status: **frozen plan, infrastructure implemented & pre-experiment validated, scientific lift NOT RUN, NOT authorized.** No manipulation has been performed under this document. The pre-lift barrier (§7.1) and evaluation tooling (§7.2) are implemented and pre-experiment validated (§8).

Acceptance criteria are `docs/F3_HOLD_CRITERIA_AUDIT.md` §6 (G1–G10). Checkpoint
and metric definitions are `docs/F3_MEASUREMENT_PLAN.md`, reused unchanged.

## 0. Recorded verdicts are not rewritten

```
H25   20260823_160017_30609       INDETERMINATE
P12.5 20260823_172848_26905       INDETERMINATE
F3 Scene A 20260823_130143_24582  INDETERMINATE
```

---

## 1. P12.5 frozen for the next experiment only

P12.5 is selected as the hold controller **for the single experiment described
here**. This is explicitly **not** a claim that P12.5 is globally validated, and
not a production change: the controller remains experiment-local, loaded
`--inactive`, and claims the master effort interface only between the strict
switch and the strict restore.

```
tau = clip(0.495831801156754 + 12.5*(q_ref - q) - 2.0*qdot, -50, +50)
```

Feedforward, Kd, Ki = 0, the ±50 N·m clip, 1 ms physics, 500 Hz ros2_control
update, geometry, contacts, friction, Scene A, perception, and the frozen
0.070000 rad pre-close are all unchanged and must not be tuned.

Selection rationale, from `20260823_172848_26905` vs `20260823_160017_30609`:

- static retention comfortably inside the existing 5 mm criterion (window medians
  0.30–0.41 mm; maximum instantaneous 0.97 mm);
- substantially lower 6–12 Hz band power than H25 on all four declared signals
  (24.9 % vs 40.4 % on T_go.X; 18.2 % vs 33.0 % on master velocity; 14.8 % vs
  34.2 % and 15.0 % vs 32.6 % on summed fingertip normal/tangential force, each
  relative to its own run's PID baseline);
- substantially improved contact-force behaviour (strict percentile exceedances
  1/16 vs 7/16; banded 0/16 vs 1/16; maximum normal force ×0.992 of the same-run
  PID maximum vs ×1.031);
- no divergence, contact escape, or drop in either run;
- increased positional wander (≈2× H25) remains well inside the retention
  budget, using 6–8 % of it.

---

## 2. Scientific question

> Does the P12.5 hybrid hold retain the object through the actual lift transient
> while preserving a valid pre-lift G0 baseline?

Two questions, both open, both unanswerable from static-hold evidence:

1. **Retention through acceleration.** Both hybrid-hold runs measured constant
   load. The failed F3 lift slipped 27.2109 mm; the holds retained to
   0.18–0.41 mm. The failure mode lives in the transient.
2. **G0 validity.** The original F3 trial was INDETERMINATE because G0 was
   materially dynamic (3.4116 mm relative peak-to-peak, 0.039405 rad master
   motion), invoking the predeclared unreliable-baseline rule.

The measured cause of (2) is structural, not stochastic. In the original trial
the close result was logged at wall `1787482973.409358625` and `LIFT_BEGIN` at
`1787482973.409379809` — **21 microseconds apart**. The frozen G0 window
`[LIFT_BEGIN − 0.500, LIFT_BEGIN]` therefore fell entirely inside the close's own
settling transient, immediately after a `TIMED_OUT_HELD` result. Any protocol
that interposes an established, measured hold between close and lift addresses
this by construction.

---

## 3. Experiment definition

Canonical Scene A `(0.45, -0.15)`. Perception required. Frozen 0.070000 rad
pre-close. Existing validated F2 path to the physical grasp. `lift_only = true`.
**No transport, no place, no release.** Exactly one manipulation trial.

### 3.1 Native simulation-time sequence

| Phase | Definition | Source |
|---|---|---|
| F2 path | perception → pre-grasp → Cartesian descent → pre-close → physical close | existing, unchanged |
| **BARRIER ARMED** | m3_grasp announces pre-lift readiness and blocks before `lift_transport_place()` | **new, §7** |
| Prerequisite gate | current-run grasp proof at the barrier | new, evaluation-only, §7 |
| `B` | loaded-PID baseline `[B0, B0 + 2.000]` | existing `run_scene_A_h25.sh` |
| `q_ref` | coordinate median of the master joint over `[B0 + 1.500, B0 + 2.000]`; ≥100 samples, endpoints ±0.010 s | existing `compute_q_ref.py`, unchanged |
| latch | publish to `pending_q_ref`, then `latch_q_ref` must return `success=True` | existing, unchanged |
| switch | strict `STRICT=2`, `activate_asap`, 5 s; P12.5 active, `gripper_controller` inactive, P12.5 sole claimant of `robotiq_85_left_knuckle_joint/effort` | existing, unchanged |
| `P0` | first of ten consecutive post-switch native rows reproducing the latched q_ref and frozen law | existing `establish_h0.py` (`--kp 12.5`) |
| **Pre-lift hold** | `[P0, P0 + 20.000]` — **the frozen 20 s horizon, unchanged**, with `W1/W2/S1/S2` exactly as defined in `validate_h25_horizon.py` | existing, unchanged |
| **BARRIER RELEASED** | at the first native sample with `sim_t ≥ P0 + 20.000` | new, §7 |
| `LIFT_BEGIN` | `M3 STAGE 3 LIFT_BEGIN` marker, ≈ barrier release + one 20 ms poll | existing `transport.cpp` |
| `G0` | `[LIFT_BEGIN − 0.500, LIFT_BEGIN]` | `F3_MEASUREMENT_PLAN.md`, **unchanged** |
| `LT` | lift transient `[LIFT_BEGIN, LIFT_DONE]`; 0.120 m along −approach at velocity/acceleration scaling 0.1; ≈2.18 s expected | existing, unchanged |
| `LIFT_DONE` | `M3 STAGE 3 LIFT_DONE` marker | existing |
| grasp-loss check | existing post-`LIFT_DONE` check (see §6 caveat) | existing |
| `L1` | `[LIFT_DONE, LIFT_DONE + 0.250]` | `F3_MEASUREMENT_PLAN.md`, **unchanged** |
| `L2` | `[LIFT_DONE + 1.500, LIFT_DONE + 2.000]` | `F3_MEASUREMENT_PLAN.md`, **unchanged** |
| dwell | existing `grasp.slip_sample_dwell_s = 2.0`, covering `L2` exactly | existing, unchanged |
| `LIFT_ONLY_STOP` | `M3 F3 LIFT_ONLY_STOP`; TRANSPORT_BEGIN never reached | existing |
| restore | strict switch back to `gripper_controller`; verify PID active, P12.5 inactive, PID sole claimant | existing, unchanged |

### 3.2 Why the pre-lift hold is exactly 20.000 s

Not an invented duration. It is the **already-frozen horizon**, chosen so that:

- `W1/W2/S1/S2` apply unchanged and are directly comparable, sample for sample,
  with the preserved static run `20260823_172848_26905`;
- the run contains an **internal replication** of the static-hold result at zero
  extra experimental cost, partially addressing the standing n = 1 limitation;
- `G0` falls inside `S2 = [P0 + 18, P0 + 20]`, an interval whose stationarity
  under a P12.5 hold has already been directly measured (p2p 1.3925 mm, master
  range 0.015139 rad, displacement 0.3005 mm);
- G6's settled-window force comparison has its windows.

### 3.3 Required evidence streams

All at native simulation time, all RUN_ID-suffixed: pose CSV (`wrist_3_link` and
`pick_target` from the same world-pose message), Gazebo joint CSV, master joint
CSV, P12.5 controller-law CSV, eight contact streams with start and horizon-end
liveness sidecars, perception logs, unedited m3 log with simulation-timestamped
`LIFT_BEGIN`/`LIFT_DONE`, controller state/interface captures, q_ref samples and
summary, switch and restore transcripts, configuration snapshot, `git
check-ignore -v` record, README and checksums.

---

## 4. Predeclared evaluation

Frozen before execution. Thresholds are reused; **no new physical threshold is
declared**.

### 4.1 G0 quality before LIFT_BEGIN

Reported exactly as `F3_MEASUREMENT_PLAN.md` requires: relative-translation
peak-to-peak, early-to-late trend and velocity proxy, master and follower joint
position ranges, contact stability, coverage.

Predeclared decision rule — a **within-run consistency ratio**, explicitly not a
physical threshold:

> G0 is **UNRELIABLE** (→ INDETERMINATE) if its coverage is inadequate, or if its
> translation peak-to-peak or master position range exceeds **2×** the
> corresponding value measured over `[P0 + 10, P0 + 18]` of the same run.

Derivation of the 2× ratio: within the preserved P12.5 run, window peak-to-peak
varied by at most 1.5346 / 1.3925 = **1.10×** across W1/W2/S1/S2, while the failed
F3 baseline was 3.4116 / 1.4110 = **2.42×** the P12.5 S1 value. A 2× ratio
therefore lies outside observed within-run variation and below the known-bad
case. Absolute values are additionally reported against both references.

### 4.2 Metrics

| Quantity | Definition | Threshold |
|---|---|---|
| `T_wrist3_object` at G0 | `T_go = inverse(T_world_wrist_3_link) @ T_world_pick_target`; coordinate-wise median | reference for all displacements |
| `slip_G0_L1` | ‖p_go(L1) − p_go(G0)‖ | reported |
| `slip_G0_L2` | ‖p_go(L2) − p_go(G0)‖ | **≤ 5.000 mm** (G2) — primary |
| `slip_L1_L2` | ‖p_go(L2) − p_go(L1)‖ | reported |
| **max instantaneous displacement during lift** | `max ‖T_go(t) − p_go(G0)‖` over `LT`, and separately over `[LIFT_BEGIN, LIFT_DONE + 2.000]`, each with the timestamp of the maximum | **≤ 5.000 mm** (G2) — primary transient metric |
| orientation change | `theta = acos(clamp((trace(R_go(G0)ᵀ R_go(X)) − 1)/2))` for X ∈ {L1, L2}, radians and degrees | reported, **not thresholded** |
| object world-Z lift | median `pick_target` world Z, G0→L1 and G0→L2 | reported; a G0→L2 rise materially below G0→L1 is direct sliding evidence |
| master/follower behaviour | position range, mean, stationarity across W1/W2/S1/S2/G0/LT/L1/L2 | G4 |
| controller effort and saturation | commanded effort min/mean/max and saturated-row count over the hold, `LT`, `L1`, `L2` | G3, G4 |
| bilateral contact continuity | fingertip/object contact messages, first/last sim_t, max gap, per interval | G1 (≤ 50 ms) |
| contact-force behaviour | per-fingertip normal and tangential P95/P99 in S1/S2 vs same-run PID + band; maxima with timestamp, contact count, penetration, ratio to PID maximum; force during `LT` reported separately | G6, G7 |
| escape / drop evidence | contact loss, `GRASP_LOST_DURING_LIFT`, object returning toward table height, T_go divergence | FAIL if direct |
| 6–12 Hz | band power on T_go.X, master velocity, summed fingertip normal/tangential force, vs same-run PID baseline | G5, ranking only |

### 4.3 Verdict semantics

Reusing `F3_MEASUREMENT_PLAN.md` §"Verdict semantics", extended by G1–G10:

- **PASS** requires: `retained = yes`; `slip_pass = yes` (`slip_G0_L2 ≤ 5.000 mm`);
  maximum instantaneous displacement over the lift ≤ 5.000 mm; no direct drop
  evidence; adequate G0/LT/L1/L2 coverage (G8); reliable G0 (§4.1); successful
  lift completion and the explicit `lift_only` stop; G1, G3, G4, G6 satisfied; and
  no contradictory contact or joint evidence.
- **FAIL** requires a measured threshold failure or direct loss/drop evidence.
- **INDETERMINATE** for any missing required stream, inadequate physical-state
  coverage, non-native timebase, unreliable G0, ambiguous ownership, or
  contradictory evidence.

**Diagnostic-transport loss is reported separately and does not invalidate
physical evidence** when the missing interval is independently covered by
adequate native ground-truth streams (G9, audit §2–§3). Delivered fraction and
gap distribution are reported for the controller-law stream; the reconstruction
of any unwitnessed instant from the native joint stream must be shown.

---

## 5. Principal risk: softer gain under acceleration

P12.5 showed ≈2× greater static positional wander than H25 (S2 peak-to-peak
1.3925 vs 0.7523 mm; master position range 0.015139 vs 0.008217 rad; maximum
instantaneous displacement 0.972 vs 0.499 mm). A margin that is negligible
against 5 mm at rest is **not** obviously negligible against a 27 mm transient.

**Static-hold success must not be taken to imply lift success.** The protocol is
therefore built so that a transient failure is measurable rather than inferred:

- the **maximum instantaneous** displacement over the lift is a primary metric
  with the existing 5 mm threshold, not a window median — a window median can
  average away exactly the excursion this experiment exists to find;
- the pose stream runs at ≈111 Hz (2166 samples over 20 s in the preserved run),
  giving ≈9 ms resolution; at the expected ≈55 mm/s lift speed the object moves
  ≈0.5 mm between samples, i.e. ≈10 % of the 5 mm threshold. **Stated
  limitation:** this bounds the resolution of the transient maximum. Sample
  count, span and maximum gap over `LT` must be reported, and inadequate `LT`
  coverage makes the transient conclusion INDETERMINATE (G8), not passing;
- Gazebo joint state at 1 ms and the contact streams at ≈1 kHz provide
  independent, denser corroboration of the same interval;
- controller effort and saturation are recorded through the transient: if the
  ±50 N·m clip is reached at any point, that is a first-order finding regardless
  of the displacement outcome.

---

## 6. Predeclared known behaviours (not defects, not changes)

1. **The existing grasp-loss check is less sensitive under an effort
   controller.** `check_grasp_not_lost` compares the actuated joint against a
   0.798 rad bound calibrated for the PID controller's standing 0.8 rad setpoint.
   Under P12.5 the joint is servoed toward the latched `q_ref` (≈0.7896 in the
   preserved run), so whether a total object loss drives it past 0.798 is
   **unvalidated**. Gazebo ground-truth `T_go` and the contact streams are
   authoritative for retention; the joint check is corroboration only.
2. **On grasp loss, transport.cpp issues a release command to an inactive
   controller.** `check_grasp_not_lost` calls `gripper_command(release_position_rad)`
   → `gripper_close_and_hold`, which sends an action goal to `gripper_controller`
   while P12.5 owns the interface. Expected: rejection or a timeout of up to the
   5.0 s command timeout. Its return value is discarded by `transport.cpp` and it
   does not change the returned Result. Predeclared, not repaired.
3. **`gripper_result = TIMED_OUT_HELD` and `within_tolerance = no`** are the
   historical Scene-A close outcome and are not by themselves failures.
4. **The ≈21.6–22.3 mm final-closure seating** occurs before `P0` and is outside
   every window evaluated here.

---

## 7. Implementation state (authorized & validated)

### 7.1 Production — pre-lift barrier in `m3_grasp.cpp` (implemented)

**Why it is unavoidable.** In `lift_only` mode `m3_grasp` runs close → lift with
no external synchronisation point: the measured gap between the close result and
`LIFT_BEGIN` in the original F3 trial was **21 µs**. The P12.5 protocol needs
≈22.8 s of native time between them (2.0 s baseline + q_ref + latch + switch +
20.0 s hold; the switch alone consumed 2.808 s in `20260823_172848_26905`). No
timing-based workaround exists, and the alternative — reimplementing the lift in
an evaluation-only script — abandons the validated `lift_only` path this
experiment is required to use.

**Smallest change.** Two parameters, both defaulting to disabled, and one poll
loop inserted immediately before `lift_attempted = true;`
(`ur5e_pick_place/src/m3_grasp.cpp:1467`):

- `pre_lift_barrier_file` (string, default `""` = disabled);
- `pre_lift_barrier_timeout_s` (double, default e.g. 300.0).

When `pre_lift_barrier_file` is non-empty **and** `lift_only` is true:
write `<marker_file_prefix>.pre_lift_ready`; log a simulation-timestamped
`M3 F3 PRE_LIFT_BARRIER_ARMED t=...`; poll for `pre_lift_barrier_file` every
20 ms (the existing idiom at lines 698, 911, 1382); on appearance log
`M3 F3 PRE_LIFT_BARRIER_RELEASED t=...` and proceed; on timeout return a failure
without lifting.

**Safety properties.** Default `""` leaves the classical and all existing
measurement paths behaviour-identical. It adds no controller, gain, physics,
geometry, perception or threshold change. `m3_grasp` runs a
`SingleThreadedExecutor` on a dedicated spinner thread
(`m3_grasp.cpp:347–349`), so blocking in `main` keeps `/clock`, TF, MoveIt and
all subscriptions live. It reuses the existing marker-file idiom rather than
introducing a new IPC mechanism.

**Status: IMPLEMENTED AND VALIDATED (2026-08-23).** Implemented in
`ur5e_pick_place/src/m3_grasp.cpp`, `ur5e_pick_place/launch/m3_grasp.launch.py`,
and `ur5e_pick_place/include/ur5e_pick_place/failure.hpp`. Object-free barrier
runtime validation passed 17/17 checks (`tools/barrier_runtime_test.sh`).

### 7.2 Evaluation-only — new experiment-local harness and gate

Neither is a modification of existing validated tooling; all are new files
under `evidence/f3_p12_5_lift_scene_A/`:

- `run_scene_A_p12_5_lift.sh` — derived from `run_scene_A_h25.sh`, with:
  `lift_only:=true` / `close_and_hold_only:=false`; the harness launched in the
  **background** (it now returns only after the lift); barrier arm/release
  handling; a wait for `sim_t ≥ P0 + 20.000` before release; `git check-ignore -v`
  on representative intended files, recorded (required by
  `F3_MEASUREMENT_PLAN.md` and absent from every current harness); and G9
  semantics replacing the criterion-10 gate.
- `pre_lift_prerequisite_gate.py` — the current-run grasp proof relocated to the
  barrier. `current_run_prerequisite_gate.py` validates the **run-summary** CSV
  row, which `m3_grasp` writes last, after the lift; it therefore cannot gate a
  pre-lift switch. The new gate validates instead: `.pre_lift_ready` postdating
  the wall-time generation boundary, the current-run sensor/harness artifacts,
  the current-run close evidence in the m3 log, and bilateral current-run
  fingertip/object contact with live recorders. **The existing shared gate is
  left untouched** so the preserved experiments remain reproducible. Unit tests:
  15/15 PASS.
- `f3_lift_p12_5_analysis.py` — offline analyser producing §4. Reuses
  `scripts/perception/f3_retention.py` unchanged for G0/L1/L2, slip, orientation,
  world-Z and contact intervals, and adds the lift-transient maximum, controller
  effort/saturation, banded G6 force comparison, G5 band power, and the G9
  transport-loss report.
- `transport_loss_report.py` — standalone G3/G9 evaluator.

**Status: IMPLEMENTED AND VALIDATED (2026-08-23).** Offline smoke validation
passed 5/5 (`tools/smoke_test_no_manipulation.sh`).

### 7.3 No production change beyond 7.1

No controller, gain, threshold, physics, contact, friction, scene, perception,
grasp-geometry or pre-close change. `config/scene.yaml` is read-only for this
experiment.

---

## 8. Preflight gates

All must pass before launch; any failure stops the experiment rather than being
repaired in flight.

1. `git diff --check` clean; working tree recorded; `git check-ignore -v` on
   representative intended evidence files recorded. **(Validated)**
2. No stale Gazebo / `move_group` / `controller_manager` / detector / recorder
   processes; `gz_assert_clean_slate`. **(Validated)**
3. Frozen-configuration guard: `preclose_margin_rad = 0.4678679450464813`,
   derived pre-close `0.070000` rad, Scene A `(0.45, -0.15)`, 1 ms physics. **(Validated)**
4. P12.5 controller builds and its plugin registers; `test_h25_law` passes. **(Validated)**
5. Controller-only validation of the barrier path and controller-only Gazebo smoke
   validation with **no object, no perception, no MoveIt, no arm command** — barrier
   arms, blocks, releases, and times out correctly (17/17 PASS); controller switch,
   latch, law reconstruction (1068 samples, 0 errors), and restoration PASS. **(Validated)**
6. `f3_retention.py` dry-run against the preserved F3 evidence, reproducing its
   recorded numbers, to prove the analyser is unchanged. **(Validated bit-exact)**
7. Disk headroom ≥ 5 GB (the preserved P12.5 run wrote ≈580 MB for 20 s; this run
   adds the lift and dwell).
8. Fresh simulator start. Per `project_sim_degrades_over_runtime`, no heavy prior
   session in the same simulator instance.

---

## 9. Abort conditions

Any of these writes `INDETERMINATE`, attempts explicit PID restoration, and stops
the stack. **No automatic rerun under any circumstance.**

- unclean preflight or frozen-configuration guard failure;
- F2 path failure, or pre-lift prerequisite gate failure;
- q_ref extraction failure (coverage, spacing, sample count) or latch rejection;
- strict switch rejection, or non-exclusive master-effort ownership at any check
  — a failed switch is terminal, because Jazzy does not roll back the paired PID
  deactivation;
- `P0` not establishable within the wall timeout;
- any recorder process death, or contact liveness/observability failure at start
  or at horizon end;
- barrier timeout, or barrier release before `P0 + 20.000`;
- `LIFT_BEGIN` or `LIFT_DONE` missing a native simulation timestamp;
- physical-state coverage (G8) inadequate over G0, `LT`, `L1` or `L2`;
- transport, place, release, or retreat attempted — a protocol violation, not a
  measurement outcome;
- PID restoration rejected or restored ownership invalid.

Diagnostic-transport loss is **not** an abort condition. It is reported under G9
and escalates only if the native streams cannot cover the missing interval.

---

## 10. Evidence directory layout

```
evidence/f3_p12_5_lift_scene_A/
├── README.md                     frozen before execution
├── PROVENANCE.txt
├── GIT_IGNORE_CHECK.txt
├── MANIFEST.sha256
├── tools/
│   ├── run_scene_A_p12_5_lift.sh
│   ├── pre_lift_prerequisite_gate.py
│   └── f3_lift_p12_5_analysis.py
└── A/
    ├── run_id.txt  protocol_<RUN>.log  VERDICT_<RUN>.txt
    ├── scene_<RUN>.yaml
    ├── pose_<RUN>.csv  gz_joint_<RUN>.csv  master_joint_<RUN>.csv
    ├── p12_5_terms_<RUN>.csv  P0_<RUN>.txt  P0_samples_<RUN>.csv
    ├── q_ref_<RUN>.json  q_ref_samples_<RUN>.csv
    ├── q_ref_publish_<RUN>.log  q_ref_latch_<RUN>.log
    ├── controller_load_<RUN>.log  controller_switch_<RUN>.json
    ├── controllers_pregrasp/postswitch/restored_<RUN>.{txt,json}
    ├── interfaces_pregrasp_<RUN>.txt  interfaces_postswitch_<RUN>.txt
    ├── contact_{fingertip,inner,outer,finger}_{left,right}_<RUN>.log
    ├── contact_manifest_<RUN>.tsv
    ├── contact_liveness_{start,horizon_end}_<RUN>.json
    ├── pre_lift_gate_<RUN>.json  barrier_<RUN>.log
    ├── hold_coverage_<RUN>.json  raw_stream_coverage_<RUN>.json
    ├── transport_loss_<RUN>.json          (G9)
    ├── A_retention_<RUN>.json             (f3_retention.py, unchanged)
    ├── lift_analysis_<RUN>.json           (§4)
    ├── sim/movegroup/object_detector/object_position_world_<RUN>.log
    ├── ros_log_<RUN>/
    └── harness_<RUN>/  A_m3.csv  A_m3.log  A_sensor.json  A_markers/
```

Non-ignored names throughout, per `F3_MEASUREMENT_PLAN.md`. A unique
`harness_<RUN_ID>/` output root remains the run-identity boundary.

---

## 11. Expected comparison against the original F3 Scene-A lift

Reference: `20260823_130143_24582`, `evidence/f3_scene_A_20260823_125623/`.

| Quantity | Original F3 (PID) | This experiment |
|---|---|---|
| pre-lift controller | PID, standing 0.8 rad setpoint | P12.5, latched q_ref |
| close → LIFT_BEGIN | **21 µs** | ≈22.8 s of established hold |
| G0 window content | inside the close settling transient | inside the frozen `S2` of a measured 20 s hold |
| G0 translation p2p | 3.4116 mm (**unreliable**) | predeclared rule §4.1; preserved P12.5 `S2` measured 1.3925 mm |
| G0 master motion | 0.039405 rad | preserved P12.5 `S2` master range 0.015139 rad |
| `slip_G0_L1` | 12.7018 mm | ≤ 5.000 mm required |
| `slip_G0_L2` | **27.2109 mm** | ≤ 5.000 mm required |
| `slip_L1_L2` | 18.5402 mm | reported |
| orientation G0→L2 | 0.35214 rad (20.18°) | reported |
| world-Z G0→L1 / G0→L2 | +115.535 / +101.840 mm — a **13.7 mm fall** while the wrist was static | reported; the same signature is direct sliding evidence |
| `L2` translation p2p | 8.4457 mm — still moving at L2 | reported |
| verdicts | `retained = no`, `slip_pass = no`, `object_drop = indeterminate` | per §4.3 |

Two distinct hypotheses this experiment separates, neither assumed:

- **H-A:** the original slip was caused or dominated by the unestablished G0 —
  the object was still seating when the lift began. P12.5 with a 20 s established
  hold would then retain.
- **H-B:** the slip is caused by the lift transient itself and is independent of
  G0 quality. P12.5 would then also fail, and its softer gain may fail *worse*
  than PID or H25 would.

A PASS supports H-A. A FAIL with a valid G0 supports H-B and is a **more
informative result than the original trial**, because it isolates the transient
from the baseline for the first time. Both outcomes advance F3.

---

## 12. Authorization state

**PRE-EXPERIMENT VALIDATION COMPLETE — READY FOR ONE-RUN AUTHORIZATION (NOT EXECUTED).**
Both §7.1 (production pre-lift barrier) and §7.2 (evaluation harness, prerequisite gate,
transport loss report, and analyzer) are implemented and validated. Preflight gates
§8.1–§8.6 have passed. The scientific lift trial has NOT been executed and requires
separate explicit user authorization.
