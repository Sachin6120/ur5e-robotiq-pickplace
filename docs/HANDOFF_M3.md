# Handoff — M3, Blockers 1/2 closed, new unresolved reliability finding

Written 2026-08-04. M-1, M0, M1, M2 are closed and committed. Blocker 1 is
fixed and validated live. Blocker 2 is closed — geometric seating, not
friction. Closing it surfaced two items that must land in M3's grasp
procedure (see Blocker 2 section below), one of them severe. Then, building
the grasp-table sweep this same session, a **third, more severe, and
currently unresolved** finding turned up: the exact anchor configuration
Blocker 2 validated 5/5 times stopped reproducing, on two different fresh
sim instances, for reasons not yet understood. Read the new section
immediately after Blocker 2 before trusting anything measurement-shaped
from this environment.

## State

| Milestone | Status | Evidence |
|---|---|---|
| M-1 | closed | merged platform validated + spawns; docs/M-1_reference_report.md |
| M0 | PASS A/B/C | docs/m0_*.log; M0-C reproduced 3x across 2 code paths |
| M1 | PASS | 20/20 planning, executed; docs/evidence/m1_planning.csv |
| M2 | PASS | cartesian_fraction=1.0000 tcp_error_m=0.0000 ground_truth=yes |
| Blocker 1 | closed | docs/geom_run*.log — bit-identical across 3 runs, 0 timeouts |
| Blocker 2 | closed (geometric seating) | docs/probe_zsweep_*.log — see below |

Robot base is at z=0.75 (table height), derived from `robot.base_pose` in
`config/scene.yaml` via `config/scene_xacro_args.py`, which all three launch
files (`ur5e_robotiq_sim_control.launch.py`, `move_group.launch.py`,
`m2_cartesian_approach.launch.py`) import. Verified live this session:
`gz topic -e -t /world/empty/pose/info -n 1` showed `base_link` at
`z: 0.75` on a bare `ros2 launch` with no extra args — the single-source
wiring holds.

Committed so far:
- `1dd32bb` — M2: base elevation propagated through scene_xacro_args.py.
- `0f2ac4f` — Blocker 1 fix: `scripts/lib/gz_settle.{py,sh}`.
- `d64e857` — this doc, first version.
- **Not yet committed**: everything below this line (Blocker 1/2 results,
  the M3-prerequisite updates). Commit after reading this section.

**Remote is configured and pushed**: `origin` ->
`https://github.com/Sachin6120/ur5e-robotiq-pickplace`, branch `main`
(renamed from `master`). Keep pushing after each commit — that was the
whole point of setting it up.

## Blocker 1 — closed, validated live

Protocol: three fixed runs of `scripts/05_measure_gripper_geometry.sh`, no
retry on `[STOP]`, successes and timeouts reported separately.

**Result: 0/3 timeouts. The 0.2 rad sample was bit-identical across all
three runs** (`width_m=0.116784`, `tcp_offset_m=0.115842`, every run) —
not just within tolerance, exactly reproduced. Before the fix: one in five
samples off by 6mm (0.1218 vs 0.1158). Settle times: 0.54–0.60s across all
15 settle calls (3 runs × 5 samples), well under the old fixed 1.5s sleep —
meaning the old budget had margin; the race was in *when* the sample was
taken relative to the goal-accepted callback and Gazebo's physics step, not
in the budget being too tight. Logs: `docs/geom_run{1,2,3}_*.log`.

## Blocker 2 — closed as geometric seating, not friction

Original question: 12–14mm of downward object movement during gripper
closure (measured three times pre-session: 13.7, 12.8, 12.5mm), against
M3's 5mm criterion. Two live-sim tests resolved it, decisively, without a
GUI session.

**Test A — re-run `04_mimic_contact_probe.sh` with settle gating (was
step 2 of the original plan).** Result: 12.42mm drop, settle times 0.31s
(joint) / 0.54s (pose) — both far under the old 3.0s `SETTLE`. Confirms the
drop is real, not a sampling-timing artifact like Blocker 1's 6mm error was:
the system reaches genuine rest in under a second, and the old sleep was
already sampling that rest state, not a mid-motion snapshot.

**Test B — the decisive one. Falsification test on the "geometric
seating" hypothesis**: if the true pad contact surface sits below the
fingertip link origin (which is where `tcp_offset` and the probe's
auto-placement are both anchored), then the ~12mm drop is the box falling
from the link-origin height down to the real pad centre — geometric, not
sliding. This predicts drop should track spawn height 1:1. If instead it's
friction/sliding, drop should be roughly constant regardless of spawn
height. Ran `04_mimic_contact_probe.sh` with explicit `BOX_XYZ` at spawn
heights offset from the auto-placement point (0.49214, 0.13332, 1.11692):

| Spawn offset | Drop (dz) | Stall angle | Outcome |
|---|---|---|---|
| −6mm | −5.87mm | 0.3505 rad | caught, stalled |
| 0 (baseline) | −12.11mm | 0.3407 rad | caught, stalled |
| −12.1mm (predicted ~0) | −2.66mm | 0.3508 rad | caught, stalled — missed prediction by 2.66mm |
| +3mm (bracket) | did not resolve in 2min | n/a | **hung — see below** |
| +6mm | box on floor | 0.0000 rad | full closure, no stall, total ejection |

Predicted-vs-actual for the two clean points: low (−6mm) predicted
12.11−6=6.11mm, measured 5.87mm (off by 0.24mm). That's a hit, not a fit —
sliding/friction would not produce a linear 1:1 relationship between spawn
height and settle distance; geometric seating does, and did.

**PROVEN**: drop is spawn-height dependent, tracking 1:1 over the tested
downward span (−6 to 0mm). Not friction-driven. Blocker 2 closed as
geometric seating.

**MEASURED**: pad contact centre ≈12.1mm below the fingertip link origin
— same frame `tcp_offset` and the probe's auto-placement both use — at
0.45 rad pre-close / 40mm box / 0.8 rad overclose. One calibration point,
same caveat as `tcp_offset`: not a constant across apertures, do not carry
to a different box width or pre-close angle without re-measuring.

**NOT ESTABLISHED — checked, did not confirm**: the −12.1mm point
predicted ~0mm drop and measured 2.66mm, a bigger miss than the −6mm
point's 0.24mm. Hypothesis floated: pads swing through an arc as the
four-bar closes, so the true contact centre is aperture-dependent, not
fixed relative to the link origin — the same phenomenon already known to
make `tcp_offset` vary ~13.6mm across the aperture range, seen from the
other side. This predicts residual should correlate with final stall
angle. Checked against the three data points:

| run | stall angle (rad) | residual vs. linear model |
|---|---|---|
| low | 0.3505 | −0.24mm |
| neg12 | 0.3508 | +2.64mm |

Low and neg12 stalled at essentially the same angle (0.3505 vs 0.3508 rad,
a 0.0003 rad difference — noise-level) but show very different residuals
(−0.24mm vs +2.64mm). If the residual were driven by final stall angle
alone, these two should match closely. They don't. **This data does not
confirm the aperture-dependent-pad-centre hypothesis** — it's not ruled
out either (3 points, ~2-3mm noise floor already established, and the
hypothesis may need spawn-history/contact-approach-path as a second
variable, not just final angle), but don't treat it as settled. Carry the
pad-centre offset as ≈12.1mm ± ~3mm, not ±0.3mm.

**NEW OPEN, severity upgraded**: the capture window's upper bound is
between 0 and +3mm (narrowed from the initial 0–6mm bracket), and crossing
it near the boundary does not fail cleanly. At +6mm the object missed
entirely and fell fast — a bad but finite data point. At +3mm, the master
joint reached full closure (0.8 rad, velocity ≈0) while the box was **still
slowly sliding** — sampled twice, 3 seconds apart, still moving both times
— and the `ros2 action send_goal` call for the overclose command never
returned. It was killed by a 2-minute tool timeout on my end, not a
timeout in the stack itself; nothing in the current probe script or (by
extension) a real grasp procedure would have caught this and it would have
hung indefinitely. Cleaned up manually afterward (removed the leftover
`probe_box`, reopened the gripper — that reopen action completed normally
in under a second, confirming the hang was specific to that near-boundary
state, not a general action-server fault). Logs:
`docs/probe_zsweep_pos3mm_*.log` (truncated by the timeout, has the hang
in progress).

**Competing explanation for the hang, not yet distinguished — worth
checking if this recurs, not worth reproducing deliberately**: my working
theory during the session was that the controller never declared rest
because the box's ongoing motion kept perturbing the mechanism. A second,
equally plausible explanation: the controller's stall/rest decision is
based on ros2_control's state readback, not Gazebo's ground truth, and
this stack has a *documented* readback defect (§3.5 of the M-1 report, the
donor repo's ~15x effort discrepancy). If ros2_control saw a different
master position than Gazebo's actual 0.8 rad, that alone would explain no
result ever being published, independent of whether the box was still
moving. **Next time this hangs**: run
`ros2 topic echo /joint_states --once` against `gz topic -e` ground truth
before killing anything — that one comparison distinguishes the two
explanations directly. Not done this time; the hang was resolved by
cleanup before this alternative was considered.

**ACTION FOR M3 — the actual fix**: grasp composition must target the pad
centre, not the link-origin midpoint that `tcp_offset` and this probe's
auto-placement both currently use. Without it, nominal grasps sit within
~6mm of a boundary whose failure mode is total loss, not degraded
performance.

**ACTION FOR M3 — the containment, not a substitute for the fix above**:
bound every gripper-close action call. The infrastructure for this
already exists and has never been exercised — `Result::GRIPPER_GOAL_REJECTED`
in `ur5e_pick_place/include/ur5e_pick_place/failure.hpp:26`
("gripper action rejected, or not reached in time") and
`gripper.command_timeout_s: 5.0` in `config/scene.yaml:214`. The M3 task is
to wire the timeout that's already specified, not invent one. State this
explicitly wherever it gets implemented: **bounding the call converts a
hang into a failure, it does not make the grasp work.** A near-boundary
grasp with a bounded call gives a clean `GRIPPER_GOAL_REJECTED` and a lost
cycle — correct behaviour, still a lost cycle. The pad-centre correction
is what prevents the lost cycle; the timeout is what stops one bad grasp
from taking the whole 20-cycle run with it. Also fix the probe scripts
themselves (`04_mimic_contact_probe.sh`, `m0_verify.sh`'s M0-C) — they
currently call `ros2 action send_goal` with no cap, which is exactly what
just hung, and they're what you'll keep running during M3 tuning.

**Skipped, correctly**: the GUI closure-with-camera session originally
planned as Blocker 2's step 3. It was going to answer seating-vs-sliding,
and that's now settled by measurement (Test B above), more precisely than
eyeballing would have given.

## NEW, SEVERE, UNRESOLVED: contact resolution stopped reproducing

While building the next artifact (the grasp-table sweep below), the exact
40mm/0.45rad configuration Blocker 2 validated 5/5 times — clean stall,
consistent shortfall, reproducible drop — **stopped working, on two
separate fresh sim instances, for a reason that is not yet understood.**
This is a bigger deal than the sweep it interrupted: it calls into question
whether ANY single measurement taken in this environment can be trusted
without a reproduction check, which directly undercuts the credibility of
running M3's 20-cycle test at all until it's understood.

**What happened, in order:**

1. Built `scripts/06_measure_grasp_table.sh` (see below) with a
   width-dependent `PRECLOSE` formula based on an unverified spec constant
   (`max_opening=0.085`, the 2F-85 spec figure, never checked against
   measurement). First run (25-65mm): 4/5 widths timed out, and the one
   "OK" result was later found to be a false positive — the object fell
   through at spawn and was never caught; the settle check correctly saw
   stillness but never checked *where* the object was resting. Added a
   spawn-validity check (compares post-spawn box height to intended spawn
   height, rejects if >10mm off) — a real, worthwhile fix, keep it.
2. Recalibrated `PRECLOSE` by interpolating `05_measure_gripper_geometry.sh`'s
   actual measured table instead of the spec constant, back-solved to
   reproduce the validated 0.45rad at 40mm almost exactly (0.4523rad, i.e.
   the fix was correct and well-targeted). Re-ran narrowed to 30-50mm: **all
   5 widths timed out, including 40mm** — the exact width/angle pair
   already proven to work.
3. To isolate "my new script has a bug" from "something about the sim
   changed," ran the **original, byte-for-byte unmodified**
   `04_mimic_contact_probe.sh` with its own hardcoded `PRECLOSE=0.45` — no
   formula, no width sweep, the literal script and config that produced
   every Blocker 2 result. **It hung too.** Box placement coordinates
   matched Test A's run essentially exactly. Ruled out my new script as the
   cause.
4. Checked whether this was long-running-session degradation: the `gz sim`
   process had in fact been silently replaced at some point (started
   12:36:48, after every earlier successful Blocker 2 run at 11:22-11:47 —
   nothing in this session explicitly narrated that restart; the parent
   process matched my own background-task launch signature, so it was one
   of mine, just not consciously tracked as a restart at the time). Killed
   it, launched a **completely fresh instance**, confirmed controllers
   active and base at z=0.75 (not a gross misconfiguration), then re-ran
   the unmodified `04_mimic_contact_probe.sh` a second time. **It hung
   again**, on a sim instance that had existed for under two minutes and
   had never run a single prior spawn/close cycle.

**This rules out**: my new script's calibration (step 3 used the original
script entirely), and long-session state accumulation (step 4 used a
freshly-launched instance with zero prior history).

**Does NOT rule out, not yet investigated**: what actually changed. One
loose thread, not yet chased down — the second reproduction attempt's
`§1 Baseline` (gripper open, no object, sampled before any command is
issued) read `robotiq_85_left_knuckle_joint` at **0.7668 rad**, not ~0.
Every probe script's stated precondition is "gripper OPEN" at start. If a
fresh spawn is not reliably starting from the open position, that's a
different and more basic problem than contact-resolution physics, and it
may or may not be related to the hang (the subsequent pre-close command did
successfully reach ~0.45 from wherever it started, so it isn't obviously
the direct cause — but it's an anomaly that showed up in the same window as
the reproducibility failure and hasn't been explained). Check spawn-time
joint state explicitly before doing anything else with this environment.

**What did NOT happen**: I did not keep iterating against a demonstrably
unreliable sim. After the second unmodified-script hang, stopped, cleaned
up (leftover box removed, gripper reopened — both completed normally,
confirming the action server itself isn't broken, just this specific
contact scenario), and wrote this up rather than trying a third instance or
guessing at more fixes.

**Consequence for the grasp-table sweep**: `scripts/06_measure_grasp_table.sh`
exists, has two real fixes in it (spawn-validity check, measurement-derived
PRECLOSE interpolation — see its header comment), and is worth keeping. But
it has never produced a single valid row, and there is no point running it
again until the reproducibility question is resolved — a table built on a
sim that can silently stop reproducing its own validated anchor case is not
a table anyone should trust.

**Consequence for M3 more broadly**: this is the strongest data point yet
for the reframing below. If contact resolution for this mimic-linkage
scenario is only *sometimes* reliable — not "reliable but mistunable
friction," but reliable-then-not for reasons still unknown — that is worse
than a friction problem, and it is exactly the shape of failure that
running 20 unattended cycles without diagnosing this first would surface as
noise: some cycles clean, some hung, no visible pattern, and a very
tempting wrong conclusion ("must be friction, it's inconsistent").

**Recommended next step, not yet done**: before any further measurement,
reproduce this in the GUI (not headless) specifically to watch what the
contact/mimic linkage is doing during a hang — this is a different question
than Blocker 2's seating-vs-sliding one, so "skip the GUI" from that section
does not apply here. Also worth a clean before/after check of joint state
immediately at spawn, across a few fresh launches, to resolve the 0.7668rad
anomaly independently of whether it's the cause.

## Reframing M3: the evidence doesn't point at friction

The spec frames M3 as friction tuning — "treat getting stable friction-grasp
physics working as its own milestone," tune mu and contact parameters,
expect to re-derive because DART isn't ODE. Read cold, that's what someone
would optimize first.

Four distinct failure modes have turned up so far, from Blocker 2's work and
the ejection findings before it. **None of them is friction:**

1. Ejection from full-open closing — a **rate** problem (2.19x closing rate
   is the difference between a grasp and a 363mm ejection).
2. Object dropping ~12mm on closure — a **geometry** problem (seating: the
   TCP frame is anchored at the fingertip link origin, ~12mm above the true
   pad contact centre).
3. Object not caught at all above +3mm of spawn/placement error — also
   **geometry** (capture window has a sharp edge, not a graceful one).
4. The unbounded hang near that edge — **control/plumbing** (an
   already-specified timeout that was never wired up).
5. Contact resolution silently failing to reproduce its own validated
   anchor case, on two independent fresh sim instances, cause unknown —
   **reliability/unknown**, and the most concerning of the five, because
   the others were at least explicable once measured. This one isn't yet.
   See the section immediately above.

Friction may still matter for lift and transport slip — that's genuinely
untested, nothing above touches it. But going into M3 with mu as the main
knob is the same category of mistake as the earlier ones on this project: a
plausible mechanism, adopted before measurement, in a spec that reads
confidently enough to skip the check. The evidence says M3's early work is
grasp-pose geometry and command discipline. Friction is a later, narrower
question, scoped to lift/transport, not grasp closure.

**Practical consequence:** don't let M3 open with a mu sweep because the
spec says to. Open with the pad-centre-corrected grasp target and the
bounded gripper-close call (see Blocker 2 above), run cycles, and see what's
actually left to explain before assuming it's friction.

## Other M3 prerequisites (unchanged from prior session, still open)

**clearance_map vs grip_map.** scene.yaml's `gripper.width_map` is specified
as one function but the code needs two different answers: clearance
(geometric, for the pre-close aperture) and grip angle (empirical, includes
compliance). Conflating them is what caused the ejection bug. Note in
`scripts/05_measure_gripper_geometry.sh`: its width column is
fingertip-link-frame distance, not object clearance — pads sit roughly 28mm
inward per side (inferred, not yet measured; the fingertip links'
`<collision>` origin would give it directly). Cheaper alternative:
`04_mimic_contact_probe.sh` already measures (box width -> stall angle).
Sweep 25/35/45/55/65mm and get the map empirically with pad geometry already
baked in. **Now the same "one table, two outputs" argument applies to the
pad-centre offset and tcp_offset too** — both are shaped by the same
aperture-dependent pad geometry, per the hypothesis above (unconfirmed but
plausible). Worth deriving all three (clearance, grip angle, pad-centre
offset) from one aperture sweep rather than three separate scalars.

**Pre-close belongs in the grasp procedure, not just the test harness.**
Closing from full-open ejects the object — measured, 2.19x closing rate is
the difference between a grasp and a 363mm ejection. scene.yaml carries
`gripper.preclose_heuristic` as a placeholder.

**Friction must be re-derived, not copied.** Harmonic defaults to DART;
ODE-era Gazebo Classic tuning advice does not transfer.

## Facts already established — do not re-derive

- Actuated joint is `robotiq_85_left_knuckle_joint`, range 0.0 (open) ->
  0.8 (closed) rad. NOT `finger_joint` — that is the donor repo's name for a
  different package lineage.
- 5 mimic followers; multipliers in M-1_reference_report.md §3.
- dartsim does not enforce mimic constraints (engine says so at spawn).
  gz_ros2_control writes follower positions in software. But contact DOES
  oppose it: shortfall 0.34 rad range on a 40mm box (0.3338–0.3508 rad
  across all probe runs this session), controller reports `stalled: true`
  when caught, `stalled: false`/`reached_goal: true` when missed entirely
  (the +6mm ejection). See §3.5. Do NOT switch to bullet-featherstone on
  the strength of the mimic feature alone — its ros2_control integration
  has open defects (gz_ros2_control#440, gz-sim#2729).
- Ejection is not a physics-timestep artifact — halving max_step_size
  changed nothing (363 vs 360.1mm). So M3's slip measurement is not
  timestep-confounded. Rate limiting is the only lever.
- `right_knuckle` diverges ~0.027–0.033 rad from its multiplier under load
  only; tracks exactly in free space. Open, low-priority — first place to
  look if grip stability is asymmetric between fingers, and also the
  concrete reason Blocker 1's step 1 protocol refused to auto-retry past a
  settle timeout (confirmed sound this session: 0 timeouts occurred, but
  the caution was correct going in).
- Vertical-tool0 reach ceiling for a ground-mounted UR5e is ~0.85-0.90m.
  This is why the base is elevated (M2, closed). Confirmed live this
  session: base spawns at z=0.75 with no launch args.
- `tcp_offset` varies ~13.6mm across the aperture range — it is NOT
  constant. Current value is a scalar measured at one aperture; changing
  `object.size` invalidates it. **Now understood to be the same underlying
  aperture-dependent-pad-geometry effect as the pad-centre offset above**
  (hypothesis, not yet confirmed — see Blocker 2's NOT ESTABLISHED item).
- Gripper-close action calls can hang indefinitely with no error when the
  spawn/grasp geometry lands near the capture-window boundary (found this
  session, +3mm offset case). Two explanations, undistinguished: contended
  box motion prevents the controller from declaring rest, or the
  ros2_control readback defect (§3.5) means the controller never saw the
  true position. Bound every such call — see Blocker 2 ACTION FOR M3.

## Working methods that earned their keep

- Evidence comes from Gazebo's own state topics, never `/joint_states` and
  never TF. Both report what something believes, which is the thing under
  test.
- Read the generated artifact, not the wizard's summary. Every config bug
  this project has hit came from MSA-generated files that looked correct on
  screen: SRDF end effector, joint limits, action_ns, use_sim_time.
- A suspiciously perfect number gets a falsification test. `tcp_error_m
  =0.0000` was confirmed real by injecting a synthetic 10mm offset into only
  the logged commanded pose and checking it reported 0.0100. Same method
  closed Blocker 2 this session: a prediction sharp enough to come back
  flat (drop should track spawn height 1:1) is worth more than a plausible
  correlation — two points define a line by construction, a third point
  tests it. Applies going forward: prefer a test whose failure mode would
  have looked different from its success, over one more measurement of the
  same kind.
- Claims get partitioned PROVEN / INFERRED / UNKNOWN (or MEASURED / NOT
  ESTABLISHED / NEW OPEN, per Blocker 2 above). Twice a correct observation
  produced a wrong downstream conclusion, and the partition is what caught
  it.
- Never auto-retry a failed measurement/settle/validation step. Record the
  failure, report the failure rate as data alongside whatever variance is
  being measured, and investigate before touching any budget or threshold.
  Validated this session (Blocker 1: 0/3 timeouts, reported as a headline
  number either way, not just checked and discarded).
- When a bounded call times out or hangs, check ground truth from more than
  one source before concluding why. The +3mm hang had two live candidate
  explanations (contended motion vs. ros2_control readback defect) and only
  one was checked in the moment. Next occurrence, sample both
  `/joint_states` and `gz topic -e` ground truth before cleanup, not after.
