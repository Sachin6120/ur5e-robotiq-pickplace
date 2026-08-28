# F3 Measurement Plan — Predeclared Before Execution

Status: measurement plan predeclared; `lift_only` execution support is
implemented but not runtime-validated. F3 has not been run. This document does
not authorize a simulator launch or manipulation.

## Scientific question and validated boundary

Does the perception-derived physical grasp remain mechanically retained during
a vertical lift?

The validated starting boundary is F2 PASS:

```text
RGB-D -> detector -> camera-frame XYZ -> TF2 world position
       -> perception-derived pre-grasp -> Cartesian descent
       -> physical grasp -> STOP
```

F3 extends only through:

```text
perception -> pre-grasp -> descent -> grasp -> vertical lift
           -> settled hold -> STOP
```

Transport, place and release are explicitly excluded.

## Predeclared simulation-time checkpoints

Events are the simulation-timestamped `M3 STAGE 3 LIFT_BEGIN t=...` and
`M3 STAGE 3 LIFT_DONE t=...` emitted by `transport.cpp`.

- G0: `[LIFT_BEGIN - 0.500 s, LIFT_BEGIN]`, the established pre-lift grasp.
- L1: `[LIFT_DONE, LIFT_DONE + 0.250 s]`, immediate post-lift retention.
- L2: `[LIFT_DONE + 1.500 s, LIFT_DONE + 2.000 s]`, settled hold within the
  existing `grasp.slip_sample_dwell_s: 2.0` simulation-time dwell.

For every window the analyzer records the requested and actual intervals,
sample count, maximum sample spacing and coverage adequacy. The instrumentation
coverage rule is: at least two paired samples, at least 90% of the requested
span covered, and no sample gap longer than half that window. This is a data
quality rule, not a physical acceptance threshold. Missing required coverage
makes retention indeterminate.

## Transform and representative poses

The initial, fixed reference choice is Gazebo's `wrist_3_link`; the object is
Gazebo's `pick_target`. The existing recorder captures both from the same
world-pose message at matching simulation timestamps. No TCP, TF, or other
frame may be silently substituted.

```text
T_go(t) = inverse(T_world_wrist_3_link(t)) @ T_world_pick_target(t)
```

Window translation is the coordinate-wise median. Window orientation is the
recorded `T_go` orientation from the sample nearest the requested window's
midpoint. Selecting a real sample avoids naive quaternion-component averaging
and remains directly auditable.

## Metrics

Let `p_go(X)` and `R_go(X)` be a checkpoint's representative translation and
rotation.

```text
slip_G0_L2 = norm(p_go(L2) - p_go(G0))        # primary
slip_G0_L1 = norm(p_go(L1) - p_go(G0))
slip_L1_L2 = norm(p_go(L2) - p_go(L1))

R_delta = transpose(R_go(G0)) @ R_go(L2)
theta = acos(clamp((trace(R_delta) - 1) / 2, -1, 1))
```

`theta` is reported in radians and degrees. Object world-Z displacement is
reported for G0->L1 and G0->L2 as lift evidence, but world lift alone never
decides retention: the question is whether the object stayed fixed relative
to the gripper.

## Closure seating and G0 quiescence

The repeatable approximately 21.6–22.3 mm motion during final closure occurs
before lift and is not F3 lift slip. G0 therefore begins only in the final
0.5 s before `LIFT_BEGIN`. The analyzer reports G0 relative-translation
peak-to-peak variation, a robust early-block to late-block translation trend
and velocity proxy, available master/follower joint position ranges, and
contact stability. These diagnose whether closure seating was still underway.
No new G0-quiescence physics threshold is declared: the repository contains no
validated threshold with those semantics. Material residual motion therefore
produces a warning/engineering review, and unusable sampling produces an
indeterminate verdict, rather than silently folding closure motion into slip.

## Contact evidence

Captured contact streams are summarized separately for G0, the lift interval,
L1 and L2. Required reporting covers left and right fingertip/object contact,
inner-knuckle/object contact when enabled, unexpected robot/table/environment
contact when the captured streams support it, and first/last relevant
simulation timestamps. Contact is corroborating evidence, not the sole
retention metric. Absence of an uncaptured stream is “not available,” not “no
contact.” Geometric retention and contact evidence remain separate.

## Acceptance thresholds and provenance

The primary translation threshold is exactly `0.005 m`, from
`config/scene.yaml: thresholds.post_lift_slip_max_m`. It is the existing M3
Gazebo-ground-truth relative-slip criterion and directly matches the F3 metric,
so it is reused without tuning.

`config/scene.yaml: grasp.grasp_loss_threshold_rad = 0.01 rad` is not a slip
threshold. It is a one-sided early-abort check applied after lift to the
actuated joint relative to expected touch angle; it indicates the gripper has
closed into a gap. The analyzer may report this joint evidence when available,
but it cannot replace the ground-truth relative transform.

No orientation acceptance limit and no independently validated G0-quiescence
limit exist. Orientation is reported but is not thresholded. A new threshold
requires explicit authorization and evidence; none will be fitted to F3 data.

## Verdict semantics

- `slip_pass=yes`: all checkpoints have adequate native simulation-time
  coverage and `slip_G0_L2 <= 0.005 m`; `no` means it exceeds that threshold;
  otherwise `indeterminate`.
- `retained=yes`: geometric relative-slip criterion passes; `no` means it
  fails; missing/invalid data is `indeterminate`. Contact is reported beside
  this result.
- `object_drop`: `yes` requires direct drop evidence, `no` requires adequate
  ground-truth evidence ruling it out, and otherwise is `indeterminate`. A
  slip failure alone does not prove a drop.

Overall F3 PASS requires `retained=yes`, `slip_pass=yes`, no direct drop
evidence, adequate G0/L1/L2 coverage, successful lift completion, and no
contradictory contact/joint evidence. FAIL requires measured threshold failure
or direct loss/drop evidence. Any missing required stream, inadequate coverage,
non-native timebase, unresolved G0 motion, or contradictory evidence is
INDETERMINATE rather than inferred.

## Required raw streams and durable layout

Each future scene evidence root, for example
`evidence/f3_scene_A_<timestamp>/`, must preserve:

- unedited m3 log with simulation-timestamped lift events and run summary;
- pose CSV with native `sim_t`, containing matching `wrist_3_link` and
  `pick_target` samples across G0, lift, L1 and all of L2;
- master and Gazebo follower joint CSVs with native simulation timestamps;
- left/right fingertip and inner-knuckle contact streams, plus available
  unexpected-contact streams;
- sensor/harness record including frozen pre-manipulation object pose;
- analyzer JSON, command transcript, configuration snapshot, README and
  checksums.

Use non-ignored names such as `A_m3.csv`, not nested `m3_grasp_*.csv`, because
the latter matches the repository `.gitignore`. Before any experiment the
harness must run `git check-ignore -v` on representative intended files and
record the result.

## Known warnings carried into F3

- Final closure seats the object approximately 21.6–22.3 mm before lift.
- Final close commonly reports `TIMED_OUT_HELD`; this is historically treated
  as a held result, not proof of retention.
- Master/follower geometry is non-deterministic while coupled joints relax;
  actual follower state is authoritative.
- The perception point is the visible top surface, not the object centre.
- Perception provides no yaw; configured grasp orientation remains in use.
- The frozen 0.070 rad pre-close has thin timeout headroom and must not be
  tuned during F3.

## Opt-in execution boundary (implemented, not runtime-validated)

The implementation adds a boolean `lift_only` parameter, default `false`,
passed through `m3_grasp.launch.py` and `m3_grasp.cpp` into
`TransportParams`. It belongs to the existing family of explicit stop-mode
booleans. Production code rejects any combination of `lift_only` with
`pregrasp_only`, `grasp_only`, or `close_and_hold_only` as `CONFIG_ERROR`; no
precedence silently chooses one mode.

When true, the transport helper completes successful grasp, Stage 3
vertical lift, its existing post-`LIFT_DONE` grasp-loss check, and the full
`slip_sample_dwell_s` hold, then returns before `TRANSPORT_BEGIN`. Success
means the lift Cartesian motion completed, the existing post-lift loss check
did not abort, the measurement dwell completed, and the explicit F3 stop was
reached. It does not mean the offline retention verdict passed. Default false
preserves the current classical lift/transport/place/release/retreat path.
This is static/build-validated execution support only: no F3 run or
perception-derived lift had been performed when the plan was frozen.

## First execution record — Scene A, 2026-08-23

One Scene-A manipulation trial (`20260823_130143_24582`) was run without
tuning. The `lift_only` runtime boundary passed: Stage 3 completed, the full
dwell completed, the explicit stop was reached, and no transport/place/release
occurred. Native-time coverage was adequate and the analyzer measured
27.2109 mm G0->L2 relative slip, above the frozen 5 mm threshold, with later
ground-truth evidence that the object returned to table height.

The experiment verdict is **F3 SCENE A INDETERMINATE**, not PASS: G0 was still
materially dynamic (3.4116 mm relative peak-to-peak, approximately 2.567 mm
trend over the window, and 0.039405 rad master-joint motion), invoking the
predeclared unreliable-baseline rule. Raw evidence and full analysis are in
`evidence/f3_scene_A_20260823_125623/`. Scenes B–D were not run.
