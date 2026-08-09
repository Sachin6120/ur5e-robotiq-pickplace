# Handoff — M3, Blockers 1/2 closed, spawn-state/reliability findings resolved

Written 2026-08-04, updated 2026-08-05. M-1, M0, M1, M2 are closed and
committed. Blocker 1 is fixed and validated live. Blocker 2 is closed —
geometric seating, not friction. Closing it surfaced two items that must
land in M3's grasp procedure (see Blocker 2 section below), one of them
severe. Building the grasp-table sweep the same session, a **severe
reliability finding** turned up: the exact anchor configuration Blocker 2
validated stopped reproducing on two fresh sim instances, cause unknown at
the time. A follow-up session ran that down: two separate, now-fixed bugs
(a script-precondition bug and a genuine spawn-time race), plus, while
validating the fixes, a third and probably more consequential bug (a
repo/workspace desync that silently no-opped code edits) and a fourth
(orphaned processes accumulating across repeated launches, a strong
candidate for the *original* reproducibility failure). See "Spawn-state
investigation, closed" below, immediately after Blocker 2, before trusting
anything measurement-shaped from this environment.

## State

| Milestone | Status | Evidence |
|---|---|---|
| M-1 | closed | merged platform validated + spawns; docs/M-1_reference_report.md |
| M0 | PASS A/B/C — C2 displacement 13.4mm vertical / 15.8mm 3D, under `gz_settle_pose_windowed` (post 2026-08-08 fix; see "settle-mechanism history" below for why this number isn't comparable to some earlier ones without naming the mechanism) | docs/m0_20260808_143404.log |
| M1 | PASS | 20/20 planning, executed; docs/evidence/m1_planning.csv |
| M2 | PASS | cartesian_fraction=1.0000 tcp_error_m=0.0000 ground_truth=yes |
| Blocker 1 | closed | docs/geom_run*.log — bit-identical across 3 runs, 0 timeouts |
| Blocker 2 | closed (geometric seating) | docs/probe_zsweep_*.log — see below |
| Spawn-state/reliability | closed (4 bugs found + fixed) | docs/spawn_state_check_*.log — see "Spawn-state investigation, closed" |
| Grasp-table sweep (06) | 5/5 OK, zero timeouts, zero ejections | docs/grasp_table_20260808_135745.log — see "stall_velocity_threshold fix applied and validated" below |
| World-table gap | closed | `config/scene_table_sdf.py`, wired into `ur5e_robotiq_sim_control.launch.py` — see "Table wired into the world; re-run of both grasp tests" below |
| M3 grasp node | First SUCCESS on record (n=1, 90mm object). Pre-close implemented; sim degradation found and fixed (full clean restart, disproved a compliance hypothesis en route). Object-shift and clearance explanations both checked and ruled out. ROOT QUESTION REFRAMED, twice: the same small free-space command succeeds 4/4 from the C++ node but fails 5/5 standalone (CLI), a caller-dependent split — NOT a friction/mechanism finding (retracted after challenge: exact-zero position/velocity for 95s isn't physical). Controller-inactive, competing-MoveIt-client, and topic-staleness all checked live and ruled out. Genuinely unexplained; likely candidate (untested) is a CLI-vs-node action-client timing difference. Highest-priority open item for M3. | docs/m3_grasp_run3_test2_*.log, docs/m3_grasp_probe_*.log, docs/m3_grasp_cube_test_*.log, docs/m3_grasp_traj_test*.log, docs/m3_grasp_extended_timeout_*.log, docs/m3_grasp_watch_test_*.log, docs/m3_grasp_fresh_verify_*.log, docs/m3_grasp_watch2_*.log, docs/m3_velocity_trace_*.txt |

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
- `39632b3`, `c33a07c` — Blocker 1/2 results, grasp-table sweep script,
  reframing, the original severe-unresolved writeup.
- **Not yet committed**: the spawn-state investigation above — precondition
  assertions (`04`/`05`/`06`/`m0_verify.sh`), the deterministic-open launch
  fix, `scripts/07_check_gripper_spawn_state.sh`, the repo/workspace symlink
  restructuring (`~/ur5e_ws/src/*` — not part of this repo, not committed by
  git, but recorded here since it's necessary for the launch fix to have any
  effect), and this section. Commit after reading it.

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

| Spawn offset | Drop (dz) | Shortfall (rad) | Outcome |
|---|---|---|---|
| −6mm | −5.87mm | 0.3505 rad | caught, stalled |
| 0 (baseline) | −12.11mm | 0.3407 rad | caught, stalled |
| −12.1mm (predicted ~0) | −2.66mm | 0.3508 rad | caught, stalled — missed prediction by 2.66mm |
| +3mm (bracket) | did not resolve in 2min | n/a | **hung — see below** |
| +6mm | box on floor | 0.0000 rad | full closure, no stall, total ejection |

Column is `commanded (0.8) − achieved`, i.e. shortfall, not the achieved
angle itself — labeled "Stall angle" in an earlier version of this doc,
which cost a false-alarm investigation later (see "Single-width
validation" below); relabeled here after tracing it back to the raw logs.
Achieved master angle at each of the three clean points, straight from
`docs/probe_zsweep_{low,mid,neg12mm}_*.log`: low 0.4495, baseline 0.4593,
neg12 0.4492 rad.

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

| run | shortfall (rad) | residual vs. linear model |
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

## Spawn-state investigation, closed: two separate bugs, not one

Follow-up session. Answers the question the previous section left open —
"is the gripper open at spawn?" — and along the way found two more
infrastructure bugs that were quietly able to invalidate future measurements
regardless of the answer. All four are now fixed. Read this before trusting
any new measurement from this environment; the earlier sections' *numbers*
still hold (see "Blocker 2's numbers: unaffected" below), but the
*confidence* behind "fresh sim instance" needs updating.

**Was the 11:22 log actually freshly launched?** No. Checked directly:
`docs/geom_run{1,2,3}_*.log` (Blocker 1's `05_measure_gripper_geometry.sh`
sweep, 11:20:18–11:21) ends its last sample at `0.767` rad and has **no
reset/reopen step at all** — the script just stops after its last sweep
sample. Two minutes later, `04_mimic_contact_probe.sh`'s
`probe_settled_20260804_112245.log` — the log this whole investigation
started from — opens with:

```
§ 1. Baseline — gripper open, no object
    robotiq_85_left_knuckle_joint 0.7669999999999888 ...
```

That number is not a fresh spawn state or a controller-activation race. It
is `05`'s last sweep sample, read back almost bit-for-bit, because both
scripts ran back-to-back against the same long-lived sim and nothing ever
reset the gripper between them. The section heading asserted the
precondition it was supposed to verify. That is the whole bug: not bad
physics, a script that printed "gripper open" over a reading of 0.767 rad
without ever comparing the two.

**Blocker 2's numbers: unaffected, checked not assumed.** Two things saved
the actual measurements from this: (1) `04`'s pre-close command is absolute
(`position: ${PRECLOSE}`), not relative to wherever the joint started, so it
reached 0.45 rad correctly regardless of starting from 0.767 or 0.0 — that's
a property of the script's design, not something anyone verified in advance.
(2) Checked all four subsequent z-sweep logs
(`probe_zsweep_{low,mid,high,neg12mm}_*.log`, 11:28–11:46) directly: every
one of them opens with a genuinely clean `~1e-5` rad baseline, because each
inherited a correct reopen from the *previous* run's own `§5 Cleanup` step —
`04` does reopen at the end, unlike `05`. So exactly one run in the whole
Blocker 2 sequence had a dirty precondition, it happened not to matter for
that run's result, and everything downstream of it was clean by inheritance.
Blocker 2's drop measurements stand. The claim "this reproduces because the
precondition holds" was never actually checked before now — it happened to
be true for reasons nobody had designed.

**Separately, and more fundamentally: a genuinely fresh launch does not
spawn open either.** Built `scripts/07_check_gripper_spawn_state.sh` to
settle this independent of any script-residue question: kill everything,
confirm no prior sim instance, launch fresh, sample the master joint at a
fixed delay after controllers report active, repeat N=5. Result — **0/5
open**, clustering at exactly two near-closed values reproducing to 8+
significant figures: `0.7668280971...` (×3) and `0.7928991779...` (×2),
controller-activation wait 5.30–8.20s. Mechanism, confirmed not guessed: the
xacro sets `ros2_control initial_value` for the six arm joints but not for
`robotiq_85_left_knuckle_joint`; nothing holds that joint (and its 5 mimic
followers, software-overridden by a controller that isn't running yet
either) until `gripper_controller` activates, so gravity closes the linkage
in the 5–8s gap. This is a real, independent finding — decoupled from the
11:22 log's residue bug, confirmed on sim instances that had never run a
single prior script.

**Contamination check on this specific result, done retroactively.** The
orphaned-process finding below was discovered *after* this 0/5 run
(`docs/spawn_state_check_20260804_140514.log`, 14:05), in the same overall
session — raising the question of whether this result is a property of the
robot or a property of an already-degraded system, the same shape of bug as
the 11:22 log's unstated precondition above. Checked, not assumed:
controller-activation wait for these 5 launches was 5.30–8.20s, which sits
*inside* the 6.8–13.1s band later established as healthy (post-fix,
orphan-free) and nowhere near the 40+s signature orphaned processes actually
produced (see "orphaned processes" below — the two runs that did hit
degraded/hung timing, 23:54 and 00:05 that session, show activation climbing
to 14–17s before hanging outright). That's evidence against contamination
at 14:05, not proof of a clean system — no process census was taken at the
time, only inferred from a timing proxy established after the fact. Treat
the 0/5 result as provisionally a property of the robot, on secondhand
evidence, not a directly-verified one.

One inference from this data was stated with more confidence than it earns,
and it's unestablished for a logical reason, not a data-quality one — this
matters because the two reasons call for different fixes. The claim: that
the 5.30–8.20s spread in activation time producing a uniformly-closed
outcome refutes a fine-grained timing race. It doesn't, independent of
whether the batch was contaminated. 5/5 closed means gravity won every
single time across that window — a race lost 100% of the time across a
3-second spread is exactly what a race with its open/closed threshold
sitting *outside* that window also looks like. "Fine-grained race, but the
observed window never crossed the threshold" and "no fine-grained race at
all" produce identical data here; nothing in this run distinguishes them.
Re-running the same experiment clean would reproduce the same ambiguity,
not resolve it — the fix is a different experiment (widen or shift the
sampled activation-time window, e.g. by artificially delaying controller
activation, to see whether outcome ever flips), not a cleaner run of this
one. Treat the fine-grained-race question as open, not as refuted.

**The two hangs from the previous section (`probe_reproducibility_check{,2}_*.log`,
13:47/13:51) are NOT explained by either spawn-state finding.** Checked
both baselines directly: check_1 (13:47) opens genuinely clean (~8e-6 rad);
check_2 (13:51) opens dirty (0.76682809714630407 rad — matching the
gravity-race value above, on a sim `kill`ed and relaunched fresh per that
session's own narrative). **Both hung identically at §3 closure regardless
of which baseline they had.** Spawn-state cleanliness is not a variable that
distinguishes hang from no-hang. Whatever causes that hang is still open —
see "orphaned processes" below for a newly-found candidate mechanism, not
yet connected to those specific two hangs.

**Fixes implemented and validated:**

1. **Precondition assertion** (`scripts/lib/gz_settle.py`'s `assert-joint`
   mode / `gz_settle.sh`'s `gz_assert_joint` wrapper): every measurement
   script (`04`, `05`, `06`, `m0_verify.sh`'s M0-C) now reads the master
   joint once, inside its own `§0. Preconditions` block — strictly before
   any `§1` output — and aborts by name if it isn't within `0.05` rad of
   open. This is what actually closes the 11:22 bug: it turns a silently
   wrong section heading into a named, fail-loud `[STOP]`.
2. **Deterministic startup**
   (`ur5e_robotiq_description/launch/ur5e_robotiq_sim_control.launch.py`,
   design note 6): the gripper is commanded open once, chained via
   `OnProcessExit` off `gripper_controller_spawner`, same pattern as the
   existing sequential-spawner chaining (design note 4).

**Validating fix 2 surfaced three more bugs, each independently capable of
making "the fix doesn't work" look true when it wasn't:**

- **Repo/workspace desync (the one that matters beyond this session).**
  `~/ur5e_ws/src/ur5e_robotiq_description` (and `ur5e_pick_place`,
  `ur5e_robotiq_moveit_config`) were plain `cp -r` copies of this repo's
  packages, not symlinks. Editing the repo's launch file had **zero effect**
  on `ros2 launch` until manually `diff`'d and copied over — confirmed by
  grepping the launch log for the new `ExecuteProcess`'s tag and finding it
  never started. Checked how long this had been silently possible: every
  file in all three `ws/src` copies matched its repo counterpart exactly
  (`diff -rq`, both directions) right up until this session's single
  uncommitted launch-file edit, and no logs exist for the ~9.5h window that
  edit sat unsynced — so no prior "trusted" measurement in this project was
  ever taken against a stale copy. But the mechanism was live and would have
  bitten silently the next time it mattered. **Fixed structurally, not just
  for this file**: `rm -rf` each `ws/src` copy and replaced it with a
  symlink to the corresponding repo package
  (`ur5e_pick_place`, `ur5e_robotiq_description`, `ur5e_robotiq_moveit_config`
  — verified byte-identical in both directions before removing anything).
  `colcon build --symlink-install` re-resolved cleanly; the install space's
  launch file now resolves straight through to the repo file with zero
  intermediate copies (`readlink -f` confirmed). This class of bug cannot
  recur for these three packages. If a fourth package is ever added to the
  workspace, symlink it in the same way rather than copying.
- **The validation script's own controller-readiness gate was wrong.**
  `07`'s first version used `ros2 control list_controllers | grep -q
  "active"` — matches the substring inside "**in**active", and doesn't
  check `gripper_controller` specifically. It was satisfied as soon as
  `joint_state_broadcaster` came up, long before `gripper_controller` even
  loaded, so every sample was taken before the deterministic-open command
  had any chance to run. Fixed to match `m0_verify.sh:122`'s existing
  word-boundary pattern: `grep -qE "^gripper_controller\b.*\bactive\b"`.
- **Velocity-based settle cannot distinguish "command finished" from
  "command never started"** — both read as zero joint velocity. Confirmed
  directly: two launches "settled" (via `gz_settle_joint`) in under 0.5s at
  the joint's untouched gravity-rest position, while their open-command
  process was still printing `Waiting for an action server to become
  available` in the launch log — it hadn't even sent the goal yet. This is
  a property of `gz_settle.py` itself, not just the validator: documented
  the precondition explicitly in its header (a caller must have already
  *synchronously* confirmed the command completed — e.g. via a blocking
  `ros2 action send_goal` that already returned — before calling settle).
  **Checked, not assumed, that this doesn't bite the existing measurement
  scripts**: grepped every `gz_settle_joint`/`gz_settle_pose` call site in
  `04`/`05`/`06`/`m0_verify.sh` — every one is immediately preceded by a
  foreground, blocking `ros2 action send_goal` (none backgrounded with
  `&`), so the command is always known-complete before settle starts
  polling in current usage. `07`'s own validator was rewritten to poll for
  actual position convergence to open (±0.05 rad) instead of using
  velocity-settle at all.

**A fourth, unplanned finding, potentially the most consequential of the
four: orphaned child processes accumulating across repeated launches.**
While debugging why a validation run hung for 15+ minutes on
`controller_manager: Waiting for data on 'robot_description' topic to
finish initialization` (not resource exhaustion — 5GB RAM free, load
average 0.65 at the time), found **18 orphaned `parameter_bridge`
processes** (one from nearly every `ros2 launch` this session had ever
run, each still bridging `/clock`), plus a leftover `robot_state_publisher`
and `gz sim`. Root cause: `07`'s `kill_sim()` only targeted the `ros2
launch` parent process and `gz sim` by `pkill -f`. `ros2 launch` spawns
`robot_state_publisher`, every controller spawner, and `parameter_bridge`
as independent child processes (`launch_ros` `Node` actions) that do not
reliably die when the parent is `pkill`'d from outside the launch
framework's own shutdown path. Every "fresh" launch after the first was
sharing a ROS graph with every prior launch's leftover `/clock` publisher.
Fixed: `kill_sim()` now explicitly targets `robot_state_publisher`,
`parameter_bridge`, and `controller_manager/spawner` by name in addition to
the launch process and `gz sim`, then verifies zero survivors and
force-kills any leftover PID directly rather than trusting the pattern
match. **After this fix, controller-activation time dropped back to a
consistent 6.8–13.1s** (vs. 40+ s observed with orphans present) across
every subsequent launch this session — strong circumstantial evidence for
the mechanism, not just a plausible story.

**Moved to `scripts/lib/gz_settle.sh`, not left local to `07`**: `kill_sim`
spawning-children-outlive-parent is a property of `ros2 launch` itself, not
of this one script, so it now lives in the shared bash lib alongside
`gz_settle_joint`/`gz_assert_joint` — any future script that comes to own
sim lifecycle inherits the fix instead of growing its own teardown copy.
Two more functions moved in alongside it, converting system health from
something noticed after a 15-minute hang into something asserted before
every launch:

- `gz_assert_clean_slate` — counts stray `parameter_bridge` /
  `robot_state_publisher` / `gz sim` / spawner processes and **aborts by
  name** if any exist, rather than launching on top of them and calling the
  result fresh. This is the check `07`'s old "`[ok]` no prior sim instance
  running" line should have been doing — the old version only looked for
  the launch.py parent and `gz sim`, exactly the blind spot that let 18
  orphans accumulate unnoticed.
- `gz_wait_controller_active_bounded` — polls for a named controller to
  report active and **`[STOP]`s if it takes longer than `ACTIVATION_BOUND_S`**
  (default 20s, comfortably above the 6.8–13.1s healthy band and well below
  the 40+s degraded one). Controller-activation time is now a known-good
  health signal for this stack, not just something to wait out — a slow
  activation gets recorded as a failure and investigated, same as any other
  measurement precondition, never silently retried or shrugged off as "sim
  was just slow that time."

`07_check_gripper_spawn_state.sh` now sources both from the shared lib
instead of defining its own `kill_sim` and inline activation-poll loop.

**Not confirmed, worth carrying forward**: whether this same leak explains
the *original* Blocker 2/3 "contact resolution stopped reproducing on two
fresh sim instances" finding above. That investigation's own "kill it,
launch a completely fresh instance" step was done by hand, and no record
exists of exactly what kill command was used — so it can't be checked
retroactively. But the shape matches: a "fresh" `gz sim` process launched
on top of an unnoticed leftover `robot_state_publisher`/`parameter_bridge`
from an earlier run in the same session is not actually fresh at the ROS
graph level, even though `gz sim` itself is a new process. **If contact
resolution hangs again**: check `ps -eo pid,cmd | grep -E "gz sim|
robot_state_publisher|parameter_bridge|spawner"` for survivors from a
prior run *before* concluding it's a physics/mimic-linkage problem.

**Final validation, post-fix**: `07` re-run in `SAMPLE_MODE=settle` (polls
for actual convergence to open, not velocity-settle) after the orphan
cleanup and the repo/workspace symlink fix — **4/4 fresh launches settled
open** (0.041, 0.021, 0.025, 0.037 rad, all within the 0.05 rad tolerance;
controller-activation 6.8–13.1s each). Deterministic startup is now
confirmed working, not just implemented.

**Net effect on M3**: the spawn-state question that opened this section is
closed — root cause understood on both halves (script-residue precondition
bug; independent gravity-vs-controller-activation race), both fixed, both
validated live. The contact-resolution hang from the previous section is
still open, but now has a concrete, checkable candidate mechanism
(orphaned processes) it didn't have before. `scripts/06_measure_grasp_table.sh`
is worth retrying now that the two most likely confounds (unverified
starting aperture; unnoticed cross-launch process pollution) are both
fixed — still not done this session, flagged as the natural next step
rather than assumed safe.

## 06 retried post spawn-state fixes: hang reproduces, root cause found, new severe finding

Written 2026-08-06. Direct follow-up to "Spawn-state investigation, closed"
above, which flagged `06_measure_grasp_table.sh` as "worth retrying... not
yet done." Retried it. Confounds checked clean before running: no orphan
processes (`ps -eo pid,cmd | grep -E "gz sim|robot_state_publisher|
parameter_bridge|spawner"` empty before launch), workspace symlinks intact
(`readlink -f` on all three `ws/src` packages resolves into this repo,
verified live), controller-activation 6.58s (healthy band), deterministic
gripper-open confirmed exact (`0.0000` rad via `gz_assert_joint`). All three
previously-fixed confounds ruled out as an explanation for whatever happens
next — this run started from a genuinely clean, verified state.

**Result: 5/5 widths (30–50mm) `TIMEOUT_OVERCLOSE` at the 5.0s bound,
including 40mm** (PRECLOSE=0.4523, the interpolated near-exact reproduction
of the previously-validated 0.45 anchor). Same failure as the earlier
unresolved section, but now with contamination and spawn-state both
positively excluded rather than merely unchecked.

**Live diagnostic, not a retry of the sweep** (per this project's no-retry
rule — investigated the mechanism instead of re-running the same failing
measurement). Manually reproduced the exact Blocker 2 anchor point
(PRECLOSE=0.45 exactly, 40mm box, spawn coords `0.49214 0.13332 1.11692` —
bit-identical to Blocker 2's own baseline row), sent the overclose action
backgrounded with no timeout wrapper, and polled both Gazebo ground truth
and the `/joint_states` ros2_control readback concurrently every 2s — the
comparison the original unresolved section recommended and never did.

| t (s) | gz ground truth (master, rad) | diag_box z (m) |
|---|---|---|
| 2 | 0.4448 | 1.10752 |
| 4 | 0.4414 | 1.10651 |
| 6 | 0.4336 | 1.10534 |
| 8 | 0.4323 | 1.10278 |
| 10 | 0.4322 | 1.10264 |
| 12 | 0.4351 | 1.10249 |

The master joint plateaus by t≈6s, but **the box was still measurably
sinking through the entire 12s window** — genuine multi-second contact
settling, not an instantaneous stall. `/joint_states` at t≈2s read
`position: 0.4442` (matches gz), `velocity: -2.9e-5` rad/s (essentially
zero by ros2_control's own account), `effort: .nan`.

**Effort NaN — checked, ruled out as the hang's cause.** Traced to the
donor package itself, not this repo: `robotiq_gripper.ros2_control.xacro`
(`/opt/ros/jazzy/share/robotiq_description/urdf/`, lines 40–46) declares
only `position` command/state interfaces and a `velocity` state interface
for the master joint — no `effort` state interface exists to read, so NaN
is structural, not a corrupted readback. Confirmed by reading
`GripperActionController::check_for_success` directly
(`/opt/ros/jazzy/include/gripper_action_controller/gripper_controllers/gripper_action_controller_impl.hpp`):
the stall decision is velocity-only (`stall_velocity_threshold`,
`stall_timeout`); `effort` in the result message is populated from
`computed_command_` post-hoc, never read from hardware. So the NaN is a
real, separate gap — anything that ever tries to read gripper effort from
this stack gets garbage — but it is not what causes the 5s timeout.

**INFERRED, not directly instrumented at controller rate**: `controllers.yaml`
sets `stall_velocity_threshold: 0.001`, `stall_timeout: 1.0`. The box's
continued multi-second settling plausibly perturbs velocity readback above
0.001 rad/s intermittently, repeatedly resetting the controller's internal
`last_movement_time_` before a full continuous 1.0s under-threshold window
accumulates — well past `06`'s and `04`'s 5.0s bound
(`gripper.command_timeout_s`). This is a real, physically-grounded contact
duration problem, not contamination or corruption.

**What eventually happened (UNTIMED — several minutes elapsed uninstrumented
while reading controller source; no latency number to report, flagged as a
gap in this diagnostic, not glossed over)**: the action did return —
`stalled: true, reached_goal: false, position: 0.43581`. (Originally flagged
here as differing from Blocker 2's "0.3407 rad" baseline — that comparison
was a units error on this session's part, not a real discrepancy: 0.3407 is
Blocker 2's *shortfall*, not its achieved angle. The corrected comparison,
achieved-vs-achieved, is 0.4358 vs. 0.4593 — within the noise already on
record for this measurement. See "Single-width validation" below.)

**NEW, SEVERE: "stalled: true" is not a stable end state in this stack.**
Re-querying Gazebo ground truth minutes after that SUCCEEDED result (no
further commands issued in between) found the master joint at **exactly
0.8 rad** (full commanded closure) and the box **ejected** — resting on the
floor at `(0.295, 0.231, 0.020)`, tipped on its side, ~1.1m of net
displacement from its spawn point. Mechanism, checked against controller
source, not guessed: `check_for_success`'s stalled branch calls
`active_goal->setSucceeded()` but never calls `set_hold_position()` or
otherwise changes `command_struct_.position_` — the position command
interface keeps being written the ORIGINAL target (0.8) every control cycle
regardless of action status. Sustained physical force continues toward full
closure after the action has already reported success, and given enough
dwell time it ejects the object it just successfully grasped. This directly
contradicts `ur5e_robotiq_description/config/controllers.yaml`'s own comment
("Stalling is the NORMAL end state of a successful grasp," lines 64–66):
true of the ROS action's status, **false of the physical system** if
nothing intervenes after the action returns.

**Consequence for M3**: any grasp procedure that treats `stalled: true` as
"done, safe to move on" is building on a false assumption. This is more
actionable than the earlier "reliability/unknown" framing (see reframing
item 5 below) — it now has a specific code-level cause and a specific fix
shape: re-command hold-position at the stalled joint value immediately after
a stalled result, before doing anything else, in both the probe scripts
(`04`/`06`) and the eventual grasp procedure.

**Fixed and validated live, same session.** `gripper_close_and_hold()` added
to `scripts/lib/gz_settle.sh`: bounds the close call at
`gripper.command_timeout_s`, then unconditionally re-commands a hold goal at
wherever the joint actually ended up — parsed from the action's own `Result:`
block when one arrives (`stalled`/`reached_goal`/`position`), or from a live
Gazebo ground-truth sample when the CLI itself is killed by the bound (which
does NOT cancel the goal server-side — confirmed, not assumed: `timeout`
only SIGTERMs the `ros2 action send_goal` client process). Wired into all
three call sites the handoff had flagged: `06`'s sweep overclose,
`04`'s §3 closure (which also gained the `CMD_TIMEOUT_S` bound it never had),
and `m0_verify.sh`'s M0-C overclose (its `STALLED`/`REACHED` parsing was
switched from re-grepping the action-result file — which would otherwise
pick up the *hold* call's always-`reached_goal:true` result instead of the
actual grasp attempt — to reading `GRIPPER_HOLD_RESULT` directly).

**Validation**: reproduced the exact anchor scenario live (PRECLOSE=0.45,
40mm box, spawn coords `0.49214 0.13332 1.11692`) through the new function.
The close call hit the 5.0s bound as before (`TIMED_OUT_HELD`), but this
time immediately sampled ground truth (0.4576 rad) and issued a hold goal
there, which returned `SUCCEEDED, reached_goal: true` in well under a
second. Watched ground truth for **120s afterward** (vs. the few minutes
that produced full ejection pre-fix): master joint stayed at 0.44–0.45 rad
throughout (no drift toward 0.8), box position stayed at
`(0.485, 0.135, ~1.102)` (z converging, not falling to the floor). No
ejection. Full transcript and readings in this session's tool output; not
re-saved as a separate log file since `grasp_stall_diagnostic_20260806.log`
already documents the failure this fixes and re-running the pre-fix
reproduction case would just be a second copy of the same setup.

**Not done**: `06`'s TIMEOUT_OVERCLOSE outcome is still recorded as a skip
(no silent retry, no reclassification as a valid sample) — the fix stops
the gripper from continuing to squeeze after a call is done, it does not
by itself decide whether a late/timed-out close should count as data.

## Single-width validation (40mm) before trusting a full re-sweep: false alarm, units error

Written 2026-08-06, same session as the fix above. Before running `06`'s
full width sweep post-fix, ran the anchor width (40mm) alone and checked
whether it reproduces Blocker 2's known numbers, on the reasoning that the
hold fix changes what an overclose call now measures (holds at stall
instead of driving to full closure) and that prediction should be checked
before trusting five more rows built on it.

**First pass wrongly concluded it didn't reproduce**, comparing three
achieved angles from this session (M0-C 0.4517, `06` 40mm-alone 0.4528,
manual diagnostic 0.4358) against "Blocker 2's documented anchor: 0.3407
rad" — a ~0.09–0.12 rad gap, reported here as an unexplained discrepancy
and used as the reason to park the sweep.

**That comparison was a units error, not a physics finding.** 0.3407 rad
is Blocker 2's *shortfall* (`commanded − achieved` = `0.8 − 0.4593`), not
its achieved angle — mislabeled "Stall angle" in the table above at the
time, now relabeled "Shortfall (rad)". Confirmed against the raw logs
(`docs/probe_zsweep_mid_20260804_112928.log`, not the summary table):
`achieved master angle : 0.4593 rad`, `shortfall : +0.3407 rad`, printed
as two separate lines. The correct comparison is achieved-vs-achieved:

| source | achieved angle |
|---|---|
| M0-C re-run (this session) | 0.4517 rad |
| `06`, 40mm alone (this session) | 0.4528 rad |
| Manual diagnostic, unbounded (this session) | 0.4358 rad |
| Blocker 2 baseline, `probe_zsweep_mid` | 0.4593 rad |
| Blocker 2 low/neg12, `probe_zsweep_{low,neg12mm}` | 0.4495 / 0.4492 rad |

All five cluster within 0.4358–0.4593, a 0.024 rad spread — the same order
as Blocker 2's own low-vs-neg12 spread (0.4495 vs 0.4492, and separately
the noise floor Blocker 2 itself established at ~2–3mm-equivalent). **This
anchor reproduces.** There is no discrepancy to investigate.

**Consequence**: the earlier framing of this as "NEW, SEVERE, UNRESOLVED...
manifests as a held result at a measurably different angle" (in the section
above) does not hold — struck through in place there rather than deleted,
so the reasoning trail stays visible. `06`'s original reproducibility
finding (widths timing out at the 5.0s bound, root-caused to genuine
multi-second settling in "06 retried" above) is untouched by this
correction — that part was never about the achieved angle's value.

**Not done, deliberately**: did not resume the full width sweep this
session — the units-error correction clears the reason `06` was paused,
but the session is at its limit and a real anchor question deserves room
to work, per the same reasoning that justified pausing in the first place.
Next session: `06` is clear to run the full width sweep — the anchor point
checks out.

## Full width sweep, next session: root cause found and quantified, not yet fixed

Written 2026-08-06, later the same day. Ran `06`'s full default sweep
(30-50mm) against a freshly relaunched, health-checked sim (`docs/grasp_table_20260806_215515.log`):
**5/5 `TIMEOUT_OVERCLOSE` again, zero OK rows** — but no ejections (the
hold fix held). New signal: every held position sat within 0.003 rad of
its own pre-close value, i.e. essentially no measured progress toward the
object in the full 5.0s bound, at every width, not just the 40mm anchor.

**Uncapped diagnostic (40mm anchor, ~90s bound) to see what "no progress
in 5s" actually looks like:** physical position (Gazebo ground truth)
plateaus by t≈5s and stays flat (0.455-0.463 rad, noise-level movement
only) for the rest of the minute. The ROS action itself did not return
`stalled: true` until somewhere between t=60s and t=90s. So the earlier
"contact settling takes longer than 5s" framing was imprecise: **the
physical settle is fast (~5s); it's the controller's own stalled-goal
declaration that is slow (60-90s)** — an order-of-magnitude gap between
what's physically true and what the action reports.

**Root cause, measured not guessed**: `gripper_controller`'s
`stall_velocity_threshold: 0.001` (`controllers.yaml`) requires velocity to
stay under that value continuously for `stall_timeout: 1.0`s. Sampled the
master joint's velocity readback directly (dedicated rclpy subscriber, not
`ros2 topic echo` text parsing, to keep up with the confirmed 500Hz publish
rate) for 30s at a genuine contact-loaded stall (same 40mm anchor). Full
data in `docs/gripper_stall_velocity_noise_20260806.log`. Headline: **75%
of samples exceed 0.001 rad/s even at physical rest**; still 41.68% exceed
0.01 rad/s. Distribution has a "normal jitter" body (median 0.008, p95
0.025) and a rarer spike population (p99 0.245, max 0.264) — roughly 1% of
samples. `stall_velocity_threshold` is set below this sim's actual noise
floor, not just tuned conservatively — the 1.0s continuous window can only
complete by chance, which is exactly what a 60-90s wait looks like.

**Decided, not yet done**: the fix is to raise `stall_velocity_threshold`
from measurement (this data), not to work around it by lengthening
`gripper.command_timeout_s` or by having measurement scripts bypass the
ROS action's result entirely in favor of Gazebo-ground-truth settle-polling.
The latter would fix `06`'s measurements but ship a known-slow completion
signal into the eventual M3 pick-place node unchanged — `stall_velocity_threshold`
only gates when the action *reports* a stall, it does not change the
commanded position or contact physics, so it is a measurement/control-loop
parameter, not a physics-tuning one, and safer to change than it might look.

**Not done, deliberately, flagged for next session** — DONE 2026-08-08, see
"stall_velocity_threshold fix applied and validated" below:
  - Pick an actual threshold value and validate it live (candidate order
    0.05 rad/s clears the noise body with margin; needs checking against
    genuine active-motion velocity too, not just at-rest noise — not yet
    measured, see the noise log's own "NOT DONE" list).
  - Re-measure the noise floor with a faithful 500Hz capture — this
    session's sampler only captured 4374 of ~15000 expected messages over
    30s (caveat documented in the log; the 75%-exceeds-threshold finding is
    stark enough to not be an artifact of that gap, but the exact
    percentiles shouldn't be treated as final).
  - Re-run `06`'s full sweep once the threshold is changed and validated —
    ejection is already fixed, the anchor value is already confirmed; this
    is the last known blocker before `06` can produce real OK rows.

## stall_velocity_threshold fix applied and validated: `06` produces its first real OK rows

Written 2026-08-08. Direct follow-up to "Full width sweep, next session"
above, which quantified the root cause (threshold below the noise floor)
but left picking a value for a session with room to validate live.

**Arithmetic first, no sim.** At the controller's confirmed 500Hz
`update_rate`, `stall_timeout` seconds requires that many consecutive
under-threshold samples (1.0s = 500, 0.2s = 100). Modeled the odds of a
clean window as i.i.d. Bernoulli trials (a simplification — the real signal
is very likely autocorrelated, not white noise, since the observed 60-90s
real-world convergence at the *current* 0.001/1.0 pair is already far
faster than an i.i.d. model predicts at 75% exceedance, which computes to
something on the order of 10^298 seconds — meaning real noise has quieter
stretches than pure-chance modeling assumes, and these estimates should be
read as pessimistic upper bounds, not exact predictions). Computed expected
wait via the standard expected-trials-to-first-run-of-N-successes formula
for three candidate pairs, all landing in the ~1-2.3s expected-wait range:
`0.05 rad/s / 0.2s` (~1.0s, using the directly-measured `>0.05 rad/s:
2.67%` exceedance point), `p99 (0.2453 rad/s) / 0.5s` (~2.3s), `p99.9
(0.2570 rad/s) / 1.0s` (~1.3s). p99.5 required interpolating between the
log's p99 and p99.9 (no raw samples were saved, only the summary), flagged
as softer than the other two. For contrast: leaving `stall_timeout` at the
full 1.0s while raising the threshold to clear p95 (0.0253 rad/s) still
computes to ~3.8x10^11s — confirms threshold alone doesn't fix this, `N`
sits in the exponent, the *pair* is what matters.

**Picked `0.05 rad/s / 0.2s`** — grounded in a directly-measured exceedance
fraction rather than an interpolated percentile, shortest expected wait of
the three. Applied to
`ur5e_robotiq_description/config/controllers.yaml` (comment there
documents the reasoning inline). Workspace symlink confirmed still
resolving straight through (`readlink -f` on the install-space file), so no
`colcon build` was needed for a config-only change.

**Confirmed the running controller actually loaded the new values**, not
just that the file changed — the exact failure mode the repo/workspace
desync bug (see "Spawn-state investigation, closed" above) would produce
silently: `ros2 param get /gripper_controller stall_velocity_threshold` ->
`0.05`, `stall_timeout` -> `0.2`.

**Live validation at the 40mm anchor**
(`docs/probe_stallfix_anchor_20260808_135658.log`, unmodified
`04_mimic_contact_probe.sh`): `gripper_close_and_hold` reported `STALLED in
1.710s` — down from the previously-observed 60-90s, and consistent with
the ~1.0s expected-wait estimate. Achieved angle **0.4501 rad**, inside the
established 0.4358-0.4593 cluster (checked, not assumed — this is what
confirms `stall_velocity_threshold` only changed *when the action reports*,
not the contact physics itself). Box settled ~1.2mm from spawn, no
ejection.

Added `GRIPPER_HOLD_ELAPSED_S` to `gripper_close_and_hold()`
(`scripts/lib/gz_settle.sh`) — wall-clock time from goal send to Result,
wired into all existing call sites automatically. This is the number the
whole threshold exercise is about; previously it was only ever recoverable
by eyeballing log timestamps after the fact.

**`06`'s full width sweep (30-50mm) re-run against the same session's
health-checked sim**: **5/5 OK, zero timeouts, zero ejections**
(`docs/grasp_table_20260808_135745.log`) — the first fully successful run
of this script, after two prior sessions' worth of TIMEOUT_OVERCLOSE. 40mm
row's `grip_angle` (0.45129) matches the anchor validation closely. Table
now also carries `pad_centre_offset` across all 5 widths (0.030-0.050m):
0.000391, 0.001514, 0.001972, 0.000873, 0.001025 — noisy, no obvious trend
with width. Not analyzed further this session; flagged as data toward the
still-open "does pad-centre offset correlate with grip angle / aperture"
question from Blocker 2's NOT ESTABLISHED item, not a resolution of it.

**Not done, carried forward, same caveats as the noise-log's own "NOT
DONE" list:**
- Active-motion noise floor still unmeasured. The applied pair is
  validated against an already-resolved contact stall, not against genuine
  closing motion — a real approach could plausibly dip through 0.05 rad/s
  for a stray 0.2s window before actual contact and false-trigger a
  premature stall report. Worth checking before trusting this pair inside
  the eventual M3 pick-place node's real approach trajectories, not just
  the probe scripts' fixed pre-close-then-overclose pattern.
- Faithful 500Hz re-capture of the at-rest noise floor (original sampler
  only captured 4374/~15000 expected messages) still not done. The applied
  threshold has enough headroom over the measured 97.33rd-percentile point
  (0.05 rad/s at 2.67% exceedance) that this gap is unlikely to change the
  conclusion, but it was never closed.
- The i.i.d.-Bernoulli expected-wait model used to pick the pair is a
  simplification, not a validated model of this sim's actual noise
  autocorrelation — treat the arithmetic as what motivated trying these
  specific numbers, not as a guarantee; the live 1.71s result is the real
  evidence.

**Consequence for M3**: this was the last identified blocker before `06`
could produce trustworthy OK rows (per "Full width sweep, next session"
above). It's now unblocked. The fix lives in `controllers.yaml`, which the
production pick-place node loads the same way the probe scripts do — this
serves M3's grasp procedure directly, not just the measurement harness (the
concern flagged when this session started).

## Box-settle false-quiescence: a second measurement-integrity bug, same shape as the first

Written 2026-08-08, same session, directly after the stall_velocity_threshold
fix above. Before trusting `06`'s new `pad_centre_offset` column (all 5 rows
read 0.4-2mm, in obvious tension with Blocker 2's original ~12.1mm figure at
the same 40mm/0.45rad configuration), checked it with one live re-measurement
rather than assuming either number.

**Confirmed live**: reproduced the 40mm anchor, ran `gripper_close_and_hold`
(STALLED in 2.262s — consistent with the threshold fix above), then watched
Gazebo ground truth on the box continuously, well past the point
`gz_settle_pose` would call it settled. `gz_settle_pose` declared the box
settled at **t=0.58s** (delta 0.14mm over 2 polls). The box was still
measurably sinking **30 seconds later** (another ~9.4mm), and did not level
off until roughly **t=120s**. Total drop from pre-overclose to converged
rest: **1.114003 -> 1.103815 m = 0.010188 m**, consistent with (not
identical to, but the same order as) Blocker 2's original ~12.1mm figure.

**Root cause, same failure shape as `stall_velocity_threshold`, different
mechanism.** `gz_settle_pose`'s consecutive-poll check (2 polls, ~0.3s at the
default 0.15s poll interval) cannot distinguish "at rest" from "creeping at a
small constant rate": a 0.05mm/s creep produces ~0.0075mm per 0.15s poll,
comfortably under the 0.5mm `eps` regardless of how many consecutive polls
are required, because each individual poll-to-poll delta is genuinely small
even while the object is still very much in motion on a longer timescale.
Raising `need_streak` (the stall-threshold fix's playbook) does not fix this
one — there is no noise spike to filter out, the slow tail never violates
`eps` in the first place.

**Why this was invisible until today**: every prior measurement using this
box's settled pose (Blocker 2's drop measurements, `06`'s TIMEOUT_OVERCLOSE
runs) happened to sample AFTER the old `stall_velocity_threshold` bug had
already forced a 60-90s wait for the *joint* stall declaration — which
incidentally gave the box's slow settling more than enough time to finish
before anything downstream read its position. Fixing the joint-stall latency
this session (down to ~1.7-2.3s) removed that accidental buffer and exposed
the box-settle check's own blind spot for the first time. The two bugs were
independent, but the first one had been silently masking the second.

**Fix**: added `gz_settle_pose_windowed` (`scripts/lib/gz_settle.py`,
`gz_settle.sh`) — compares each sample against one taken `window_s` seconds
in the past, not just the previous poll, so a slow sustained creep becomes
visible again (0.05mm/s * 10s = 0.5mm, no longer under `eps`) even though no
single short interval shows it. Default `window_s` equals the poll interval,
reproducing the exact old behavior for every other call site (fingertip
settle, arm-link settle, joint velocity settle) — all of those are genuinely
fast, non-contact processes and were never suspected or shown to have this
problem; only the post-overclose box-pose settle (contact/compliance
settling) was changed to use the windowed variant, in `06` (10s window,
150s timeout — both informed by this measurement) and left as a documented
caveat (not changed) in `04`, which doesn't compute a quantitative metric
from the box pose.

**Validated live**: re-ran the windowed check against the same anchor —
correctly waited **64.26s** and converged at a genuine ~0.035mm/s residual
(0.348mm over the 10s window), instead of falsely declaring done at 0.58s.

**`docs/grasp_table_20260808_135745.log` amended in place** (raw captured
output left untouched, a clearly-marked section appended) marking its
`pad_centre_offset` column invalid for all 5 rows, with a pointer to this
section and the corrected anchor value. `grip_angle_rad` and
`tcp_offset_at_grip_m` in that same log are unaffected (joint/fingertip-pose
driven, not box-pose driven) — do not discard the whole table over this,
only that one column.

**Not done**: the full 5-width re-sweep with the fixed settle check (~150s/
width now that it waits properly — a ~10-15 minute run). Deliberately
deferred; the single re-confirmed anchor value is used as a placeholder
scalar in the meantime (see `config/scene.yaml`'s `grasp.pad_centre_offset`,
same treatment `tcp_offset` got before today).

**Consequence for M3's slip criterion — the actual reason this matters
beyond one config number.** `thresholds.post_lift_slip_max_m` is 5mm. If a
grasped object is still settling into true contact by ~10mm over the better
part of two minutes, and M3's slip check baselines from the pose immediately
after the gripper reports stalled, ongoing seating settling — not friction
failure — will read as slip, and the amount will scale with how long the
cycle takes to reach the post-lift check, not with grasp quality. This would
make M3's pass/fail measurement a clock, not a friction measurement.

Two structurally different explanations for the slow settle were floated,
not yet distinguished: (1) bounded compliant seating into the pads —
exponential approach to an equilibrium, physical and expected to converge
(the observed decaying deltas, ~0.01mm/10s by t=120s, are consistent with
this); or (2) unbounded solver/constraint creep that decays but never truly
stops. The data collected so far favors (1) but does not rule out (2) —
nothing here ran long enough or checked for a truly asymptotic floor versus
a very slow ongoing drift.

**This is the strongest argument yet for actually doing the pad-centre
correction**, not just a data-quality footnote: if the grasp target already
places contact at (approximately) the true pad-centre depth instead of
~10mm short of it, most of this settling has no reason to happen — there is
much less geometric mismatch left to seat into. The correction is not only
about grasp accuracy, it is what keeps the slip measurement itself from
being contaminated by an artifact of bad initial targeting. Whether it
actually shrinks the settle time this much is an open, checkable prediction
once the correction is applied — not yet done.

**Open design item, not yet resolved**: M3's slip-check implementation must
not baseline from the pose immediately after `stalled: true` (or after this
session's `gripper_close_and_hold`). It needs to wait for the OBJECT (not
just the joint) to genuinely settle first, using `gz_settle_pose_windowed`
(the same window-based mechanism, not a new one, and not a blind fixed
sleep — consistent with this project's existing "poll, don't guess a sleep
duration" rule) before capturing the pre-lift baseline pose the slip
computation is measured against. How long that takes post-correction is
unmeasured; recommend checking this empirically once the pad-centre
correction below is applied, before picking a production timeout for it.

## The false-quiescence bug already corrupted a live gate: M0-C's box displacement

Written 2026-08-08, immediately after the box-settle fix above. The
"accidental buffer" framing generalizes: every post-close measurement in
this project got a minute-plus of free settling time it never asked for,
from the old `stall_velocity_threshold` bug's 60-90s wait. Fixing that
threshold removed the buffer everywhere at once, not just in the diagnostic
that happened to expose it.

**Checked by grep, not a re-run** (the number was already in a committed
log): `docs/m0_20260806_214228.log`, M0-C's C2 section —

```
box before close: ... 1.1167676483894935
box after close : ... 1.1151760216363287
displacement: 0.0020 m   (need < 0.03)
```

**Confirmed exactly as suspected**: `gripper_close_and_hold: TIMED_OUT_HELD`
on that run — the 5.0s `gripper.command_timeout_s` bound killed the action
client and fell back to a live ground-truth sample before either the old
slow declaration or a genuine settle could occur, and the immediately
following box-settle call (`gz_settle_pose`, plain consecutive-poll, same
bug as above) then declared the box "settled" almost immediately. Result:
`displacement=0.0020m`, comfortably under `MAX_BOX_DISP=0.030` — a clean
PASS — while measuring the box at a wildly different point on the settling
curve than the 12.5-13.7mm figures `MAX_BOX_DISP` was originally calibrated
against (Blocker 2's pre-`gripper_close_and_hold`, unbounded
`ros2 action send_goal` runs, which genuinely waited for the old slow
declaration and therefore sampled well past the point of full settling).
**The gate was passing on a number that no longer meant what the threshold
was set against — the dangerous case, not the safe one: a gate that changes
what it measures while staying green.**

This was live independent of today's `stall_velocity_threshold` fix — the
5.0s `command_timeout_s` bound in `gripper_close_and_hold` was always going
to cut the wait short one way or another, so this has been silently true
since `gripper_close_and_hold` was introduced (2026-08-06), not something
today's session newly broke.

**Fixed**: `m0_verify.sh`'s C2 box-after-overclose settle
(`scripts/m0_verify.sh`, the call feeding `BOX_AFTER` into `m0c_eval.py`)
switched from `gz_settle_pose` to `gz_settle_pose_windowed`
(`BOX_SETTLE_WINDOW_S=10.0`, `BOX_SETTLE_TIMEOUT_S=150.0`, same values as
`06`'s fix). `MAX_BOX_DISP=0.030` was NOT changed — the historical
12.5-13.7mm figures it was calibrated against remain valid (they were taken
under the accidental long buffer, which happened to be long enough for
genuine settling), so once the settle check is fixed to genuinely wait
again, the gate is measuring the same thing it always was and the existing
threshold still applies.

**Scanned every other `gz_settle_pose` call site in `scripts/`** for the
same pattern (settling AFTER a gripper close, on an object, vs. settling
before any contact force is applied):

| site | what it settles | contact-loaded? | action |
|---|---|---|---|
| `04`:190, `06`:358, `m0_verify.sh`:413 | fingertips, pre-close | no | unchanged |
| `04`:270, `06`:407, `m0_verify.sh`:465 | box, post-spawn (pre-overclose) | no (gravity only) | unchanged |
| `04`:303 (§3 closure) | box, post-overclose | **yes** | **fixed** — same windowed swap, this session |
| `06` (post-overclose) | box, post-overclose | **yes** | **fixed earlier this session** |
| `m0_verify.sh`:495 (C2) | box, post-overclose | **yes** | **fixed this section** — the one that was confirmed corrupted |
| `05_measure_gripper_geometry.sh`:117 | fingertips, free-space sweep (no object at all) | no | unchanged |

Three sites shared the exact same bug shape; all three now use
`gz_settle_pose_windowed`. Every other site settles either a non-object link
or an object under gravity alone (no sustained contact force driving a slow
compliance creep) — no evidence or mechanism for the same failure there.

**Re-run, same session, prediction written down first.** Two candidate
outcomes were laid out before running: land near 12-13mm (historical figures
were converged, this session's ~10.2mm anchor value differed by
configuration) or land near ~10.2mm (60-90s was never fully converged
either, historical figures were themselves a slight undercount). A third
possibility was also raised at the time — that `gripper_close_and_hold`'s
early hold (engaging in ~2s) versus the old protocol's sustained driving
force for the full 60-90s changes the actual physical amount of seating, not
just the sampling delay — predicting ~10mm specifically for that reason.

**Result: displacement 15.8mm (3D), 13.4mm vertical** (`box before close`
1.116769, `box after close` 1.103336) — landing in the historical
12.5-13.7mm range, not near the ~10.2mm anchor value from earlier this
session. The sustained-force-duration hypothesis is not supported by this
result.

**Chased down why, rather than accepting the miss.** Compared M0-C's
`box before close` baseline (1.116769, sampled at the fingertip-midpoint
spawn height) against this session's own earlier anchor diagnostic's
baseline (`Z_BEFORE_OVERCLOSE=1.114003`, nominally the same point) — a
2.9mm gap between two readings that should have matched. Root cause: the
anchor diagnostic's "before" sample was read in a SEPARATE tool call, one
full round-trip after a `gz_settle_pose` "settled" declaration for the
spawn — a real-world gap (ordinary interactive latency, not a script
sleep) during which the box apparently drifted further before the baseline
was actually captured. That pre-overclose gravity-settle call was never
covered by this session's `gz_settle_pose_windowed` fix (only the
POST-overclose, contact-loaded calls were judged to need it) — reasonably
so for a script's fixed, tight command sequence, but an ad-hoc multi-turn
interactive diagnostic doesn't have that guarantee. Computed from the SAME
true baseline M0-C uses (spawn height 1.11689, not the drifted
`Z_BEFORE_OVERCLOSE`), the anchor diagnostic's own data gives **13.075mm**
— consistent with M0-C's 13.4mm and the original ~12.1-13.7mm figures.

**Conclusion**: the historical 12.5-13.7mm figures were essentially
converged and valid all along; `MAX_BOX_DISP=0.030`'s calibration basis
needs no footnote. The ~0.1-0.2mm figures earlier the same day were the
false-quiescence bug (fixed, real). The ~10.2mm figure that briefly replaced
it in `config/scene.yaml`'s `grasp.pad_centre_offset` was itself a narrower,
different artifact — a baseline-timing gap specific to interactive
measurement, not a flaw in the settle-check fix. **Corrected**
`grasp.pad_centre_offset` to `0.013433` (M0-C's clean, single-script,
no-cross-tool-call-gap value) in the same commit as this section.

None of this affects `MAX_BOX_DISP` itself, which was never changed and
remains correctly calibrated throughout.

**Settle-mechanism history for M0-C's C2 displacement — record the
mechanism alongside the number, not just the number.** Same gate, same
`MAX_BOX_DISP=0.030` threshold, three genuinely different measurement
regimes across this project's history:

| when | value | settle mechanism | comparable to today? |
|---|---|---|---|
| pre-session (Blocker 2 era) | 12.5-13.7mm | unbounded `ros2 action send_goal`, no `gripper_close_and_hold`, no explicit settle-poll — the wait was 60-90s of the OLD `stall_velocity_threshold` bug's action-call latency itself, which happened to fully settle the box as a side effect | yes — converged, confirmed by today's re-run |
| 2026-08-06 | 2.0mm (`docs/m0_20260806_214228.log`) | `gripper_close_and_hold` (5s bound, `TIMED_OUT_HELD`) + plain `gz_settle_pose` (consecutive-poll, ~0.5s) — the false-quiescence bug, sampled the box mid-settle | **no** — this number was never a real measurement of the physical quantity, only of how fast the broken check declared done |
| 2026-08-08 | 13.4mm (`docs/m0_20260808_143404.log`) | `gripper_close_and_hold` (fast `STALLED`, ~2s) + `gz_settle_pose_windowed` (10s window, 150s timeout, converged at 51.41s) | yes — this is the current, trusted mechanism going forward |

A future session comparing a new M0-C number against history should check
which of these it's landing near, and treat a bare number without the
settle mechanism attached as unverifiable.

**The baseline-artifact hazard is a category of mistake, not a one-off,
and deserves naming on its own — not folded into "the value was wrong."**
The failure: a before/after pair was sampled across two separate tool
calls (spawn the box + declare "settled" in one call, read the "before"
value in the next), and the ordinary latency between turns — not a script
sleep, not simulated time, just the real-world gap in an interactive
session — let the box drift ~2.9mm further before the baseline was
actually captured. Nothing in the number itself flags this; `1.114003`
looks exactly as precise and trustworthy as `1.116769` until you know one
of them was read a full round-trip later than intended. This is a
structurally different hazard from the false-quiescence bug above (that
was a broken *algorithm*; this is a broken *protocol* — correct settle
logic, sampled across a gap the logic was never asked to cover) and will
recur any time a measurement's before/after pair is split across turns
instead of captured inside one script run. **This is exactly why M0-C's
value is the one to trust**: it is a single script's execution, spawn to
sample, with no interactive gap anywhere in the middle. Any future
before/after comparison should be captured the same way — one script run,
not stitched together from separate interactive steps — even when the
individual steps look identical to a single-script equivalent.

## Pad-centre correction and grasp-success verification: design, not yet implemented

Written 2026-08-08. `ur5e_pick_place` has no M3 grasp node yet (only
`m1_joint_goal.cpp` and `m2_cartesian_approach.cpp` exist) — this is fresh
design, not a modification of existing grasp code. Two related pieces:

**1. Pad-centre-corrected grasp target.** `config/scene.yaml`'s
`grasp.tcp_offset` and new `grasp.pad_centre_offset` (added this session,
see above) are now measured, not guessed: `tcp_offset_at_grip=0.120405` at
an exact width match to `object.size[grasp_width_axis]` (0.045m, no
interpolation needed), plus a separate `pad_centre_offset=0.010188`. Kept as
two named scalars, not folded into one, so a future re-measurement of either
doesn't require unpicking a combined number.

Grasp composition should target `tool0_offset = tcp_offset + pad_centre_offset`
along tool0's local Z (not `tcp_offset` alone) — the true pad contact surface
sits `pad_centre_offset` farther from tool0 than the fingertip-link-origin
point `tcp_offset` is anchored to, per Blocker 2's falsification test.
**Flagged, not asserted**: the direction (add, not subtract) is reasoned from
geometry — vertical approach, tool0 above the object, true contact deeper
than the naive anchor implies a larger standoff is needed to still meet it at
the right depth — but has not been confirmed by a live grasp using the
corrected target. This project has already been burned twice by exactly this
kind of sign/frame mixup (the original `tcp_offset` anchor-frame confusion;
the shortfall-vs-achieved-angle units error). **Verify with a live
approach-and-grasp cycle before trusting this unattended**: command the
corrected target, close, and confirm the achieved contact geometry (fingertip
pose relative to the object) actually improves versus the uncorrected
baseline, not just that the run completes.

**2. Grasp-success verification, not the action result.** The ROS action
result is not evidence of a successful grasp — `stalled: true` only means the
controller's own noisy heuristic fired, and this session already showed that
heuristic can misfire in both directions on a false-quiescence-style bug
(the stall-declaration bug from earlier this session was the same category:
a signal reported "done" without the physical state matching). The spec's own
warning applies here directly: never treat a successful `attachObject` as
evidence of a successful grasp.

Design: after `gripper_close_and_hold` returns, compare the achieved joint
angle (`GRIPPER_HOLD_POSITION`, already captured by that function) against
the `grip_angle_rad` this session's grasp table predicts for
`object.size[grasp_width_axis]` (interpolated between the nearest two rows in
`docs/grasp_table_20260808_135745.log` if the object width doesn't land on an
exact sample, as it does for the current 0.045m object). Outside some
tolerance (not yet picked — should be informed by the same-width spread
already on record, e.g. the 0.4358-0.4593 rad cluster at 40mm, before
guessing a number) means the gripper did not actually catch the object at the
expected geometry: return `Result::GRIPPER_GOAL_REJECTED` (reusing the
existing enum value — this is still "gripper action rejected, or not reached
in time" in spirit, a goal that nominally succeeded but not at a position
consistent with holding the object) and abort the cycle, rather than
proceeding to `attachObject` and transporting nothing.

This turns the grasp table from a targeting input into a verification input
too — the same measured data serves both roles, and the second role is what
actually closes the active-motion / false-early-stall risk that was flagged
(and mostly ruled out empirically, see below) for the
`stall_velocity_threshold` change: even if a future noise regime did cause an
early false stall, the achieved-angle check would catch it as a mismatch
against the table and reject the cycle, rather than silently transporting an
ungrasped object.

**Free check performed on existing sweep data, not a new sim session**:
plotted `grip_angle_rad` and `width_at_grip_m` against `box_width_m` across
`06`'s 5 rows. Both strictly monotonic (larger object -> smaller stall
angle, larger measured width-at-grip), no reversals. Deltas cluster tightly
(angle: mean -0.0447 rad/5mm step, stdev 0.0043, CV ~9.5%; the single
largest step, 30->35mm, is consistent with the already-documented four-bar
linkage nonlinearity, not a discontinuity). A false-early-stall on some
widths and not others would show up as a ragged or non-monotonic relationship
— it does not. This does not fully retire the active-motion noise-floor gap
(still genuinely unmeasured — see the stall_velocity_threshold section
above), but it is direct evidence against the failure mode actually occurring
in data already collected this session, and the grasp-success check above is
a stronger, permanent mitigation than further noise characterization would
have been regardless.

**Not yet done**: no code written — `ur5e_pick_place` has no grasp/M3 node to
put this in yet. This section is the design for when that node is built:
pad-centre-corrected target (direction pending live verification), grasp
table lookup/interpolation by object width, achieved-angle tolerance check
feeding `Result::GRIPPER_GOAL_REJECTED`, and (per the section above) a
windowed object-settle wait before any slip baseline is captured.

## M3 grasp node: implemented, first live tests run, one anomaly open

Written 2026-08-08. First actual M3 code — `ur5e_pick_place/src/m3_grasp.cpp`
(new node), `config/grasp_table.yaml` (new, structured extraction of the
`06` OK rows), `config/scene.yaml`'s new `grasp.grasp_tolerance_rad`,
`ur5e_pick_place/launch/m3_grasp.launch.py`. Scope deliberately narrow, per
the design above: pad-centre-corrected approach (reusing M2's proven
two-stage joint+Cartesian pattern with `tcp_offset + pad_centre_offset` in
place of `tcp_offset` alone), bounded gripper close+hold (C++ port of
`gripper_close_and_hold`, same semantics: primary result from the action's
own Result, TIMED_OUT_HELD ground-truth fallback if it doesn't arrive in
time, unconditional hold at whatever was achieved), and grasp-success
verification against `grasp_table.yaml` (`GRIPPER_GOAL_REJECTED` on a
tolerance miss, reusing the existing enum value per the design decision
above). No lift, no attachObject, no slip check — still out of scope,
unchanged from the design section above.

Compiles clean (`colcon build --symlink-install --packages-select
ur5e_pick_place`), new deps (`rclcpp_action`) wired into `CMakeLists.txt`/
`package.xml`. `control_msgs` was already a declared dependency, unused
until now — this is the node it was waiting for.

**Test 1 — no object in the scene.** Ran against the live sim with nothing
spawned at `object.pick_pose` (an oversight, not a plan — the launch file
correctly doesn't spawn a test object itself, that's a harness concern).
Approach succeeded (`cartesian_fraction=1.0`), gripper closed to
`0.7901 rad` (near-full, `reached_goal: true` — nothing stopped it),
compared against the table's `0.4055` expected for a 45mm object:
**correctly rejected**, `GRIPPER_GOAL_REJECTED`, `|err|=0.3846 >>
tolerance`. This is the check doing exactly its job — a miss that would
have looked like a normal successful `GripperCommand` result was caught.

**Test 2 — object spawned, but found a second latent gap first.** Spawning
`pick_target` at `object.pick_pose` (0.450, -0.150, 0.795) dropped it
straight through to the floor (z settled ~0.02m) — **`empty.sdf` has no
table collision surface at z=0.795.** Every prior probe script
(`04`/`06`/`m0_verify.sh`) sidesteps this by spawning its test object
relative to the gripper's OWN current position, never at an absolute
world-frame height, so this gap was never hit before now. `object.pick_pose`
assumes a physical surface that doesn't exist in the test world. **Not
fixed** — worked around for this test with a temporary static support
platform (`temp_test_table`, removed after), not a permanent world change.
**Flagged as a real, separate blocker for the actual M3 milestone**: the
world needs a table (or `object.pick_pose` needs reconciling with whatever
the real surface is), independent of anything grasp-composition-related.

**Test 2, re-run with the object properly supported**: approach succeeded
again. Gripper result: `TIMED_OUT_HELD` at `achieved=0.4291 rad` (5.0s
bound expired with no Result, fell back to ground-truth sampling) — NOT the
fast `STALLED` this session's threshold fix produced at the 40mm anchor.
Compared against the table's `0.4055` expected: `|err|=0.0235`, landing
**exactly at** `grasp_tolerance_rad` (also `0.0235`, not a coincidence of
values matching, just where this particular trial happened to fall) —
**rejected**, outside tolerance (boundary is exclusive as coded).

**Two things about Test 2 not yet explained, flagged rather than
glossed over:**

1. **Why did the fast stall threshold not produce a `STALLED` result
   here?** Confirmed live the running controller still has the
   session's fix (`stall_velocity_threshold=0.05`, `stall_timeout=0.2` —
   `ros2 param get /gripper_controller ...` checked directly, not
   assumed). So the threshold value isn't the issue. Working hypothesis,
   not confirmed: this object (45mm square cross-section, grasped at the
   pad-centre-corrected depth) produced messier, more sustained contact
   noise than the well-characterized 40mm anchor the fix was validated
   against — see point 2. This would be the first live manifestation of
   the still-open "active-motion noise floor... not yet measured" gap
   flagged when the threshold was picked (see the
   `stall_velocity_threshold` section above) — a different, noisier
   contact regime than the one the fix was tuned on.
2. **The object moved laterally during closing** (~13-20mm in XY, checked
   directly via `gz topic -e` on `pick_target` before cleanup), and
   `wrist_3_link` ground truth showed a correlated-direction positional
   offset from its commanded target (ground-truth TCP error 53mm total —
   M2 established ~0.0000m tracking error with the same reconstruction
   method on an unloaded approach, so this is new, not a measurement
   artifact carried over). Read together with point 1: an asymmetric,
   sliding contact (not the clean, centred stall the 40mm anchor
   consistently produced) is a coherent single explanation for both —
   sustained sliding could plausibly generate the kind of ongoing velocity
   noise that prevents the stall check from converging, and could also
   explain a real (not measurement-artifact) small arm perturbation via
   sustained off-axis contact force. **Not confirmed** — a coherent
   hypothesis, not a demonstrated mechanism. Worth a GUI look or an
   extended-timeout diagnostic (same style as the box-settle
   investigation above) before trusting any tuning decision made on top
   of it.

**Net assessment**: the verification logic is doing real work — both
trials it saw were genuinely bad grasps (no object; a borderline/uncertain
one) and it rejected both rather than rubber-stamping either. That's the
safety property the design was for, and it held. What's NOT yet
established is whether the pad-centre correction's direction/magnitude is
actually right — neither trial produced a clean accept to check the
achieved contact geometry against, and Test 2's near-miss is confounded by
the unexplained sliding/timeout behavior rather than being a clean
data point on correction accuracy. **Do not tune `grasp_tolerance_rad` or
`pad_centre_offset` from this one trial** — investigate the sliding/timeout
anomaly first, per this project's own no-silent-acceptance-of-a-miss
discipline.

**Not done**: the table-collision-surface gap (world needs a table);
investigating the sliding-contact/timeout anomaly; any trial that produced
a clean WITHIN-tolerance result to check the correction's sign against
(everything so far has been a reject, for two different and only partially
understood reasons); lift/attachObject/slip-check (always out of scope for
this pass).

## Where this left off — the missing table is one cause, not two anomalies

Written 2026-08-08, end of session. Before treating the two Test 2 anomalies
above (TIMED_OUT_HELD instead of fast STALLED; 13-20mm lateral object
drift + 53mm arm-position error) as two separate things to investigate:
they very likely both come from the SAME cause as the table-collision gap
already flagged, not from anything new.

`config/scene.yaml`'s `table:` block (`size: [1.20, 0.80, 0.75]`,
`pose: (0.55, 0.00, 0.375, ...)`, `surface_z: 0.75`) has existed since this
file's earliest version — the contract implied by its presence is that
whatever spawns the world reads it and puts a matching collision surface
there. **That wiring was never built.** `empty.sdf` has no table. Test 2's
temporary static platform covered `object.pick_pose`'s footprint just
well enough for the box to rest there at spawn time, but the object was
never resting on a REAL table during the grasp attempt in any sense that
matches production geometry — and, more importantly, nothing about that
patch guarantees the surface was doing what a real table would during
contact loading.

**An object with no genuine table under it, being squeezed by closing
fingers, is not stationary — it's free to slide, tip, or partially fall
during exactly the window the stall check is trying to converge over.**
That single mechanism accounts for all three observations without
requiring three explanations:

- **Sliding contact instead of a clean stall** — a wobbling/underspecified
  support means the box can shift under finger pressure instead of being
  pinned against a rigid surface the way `04`/`06`/`m0_verify.sh`'s tests
  (which spawn directly at the gripper's own position, no floor involved)
  never had to contend with.
- **A close that never reaches quiescence within 5s** — sustained object
  motion is sustained velocity-domain noise at the joint, which is exactly
  what prevents `stall_timeout`'s continuous under-threshold window from
  completing (the same mechanism the `stall_velocity_threshold` fix
  targeted, just re-triggered by a different noise source than the one it
  was tuned against).
- **The arm holding position while the object (and apparent TCP ground
  truth) departs** — not a MoveIt/arm positioning error (M2 already proved
  ~0mm tracking with the identical reconstruction method); the geometry
  reads as displaced because the OBJECT moved during the still-open
  gripper action window, not because the arm did.

**Consequence**: the working hypothesis in the section above ("messier
contact than the 40mm anchor produced") had the right shape but the wrong
attributed cause — not a property of this object's geometry or the
pad-centre correction, but a property of testing without the table this
project's own config file has specified from the start. Investigating the
sliding/timeout anomaly as a physics or tuning question, on a world that is
missing its floor, would produce a finding that means nothing — it would be
characterizing an artifact of the test setup, not the grasp.

**Tomorrow's first job, in order, before anything else**:
1. Generate the table's collision geometry from `config/scene.yaml`'s
   `table.size`/`table.pose` and spawn it into the world the same
   principled way `scene_xacro_args.py` already does for the robot's base
   pose (single source of truth, no hand-copied constant) — not another
   one-off temporary platform like Test 2's workaround.
2. Re-run both M3 grasp tests (no-object case should still correctly
   reject; the with-object case is the real test) against a world that
   actually has the table scene.yaml has been describing all along.
3. Only THEN chase whatever anomaly, if any, survives a properly-supported
   object. It may vanish entirely (most likely, per the mechanism above),
   or persist in a smaller/cleaner form worth its own investigation.

**Confirmed right in retrospect**: not tuning `grasp_tolerance_rad` or
`pad_centre_offset` off Test 2's trial. Either constant would have quietly
absorbed the missing table's effect — a tolerance widened or an offset
adjusted to accommodate a sliding, unsupported object would have been
calibrated against an artifact, and would then silently mismeasure the
correction once the table gap is actually fixed.

## Table wired into the world; re-run of both grasp tests

Written 2026-08-09. `config/scene_table_sdf.py` added (single source
deriving a static-box SDF from scene.yaml's `table:` block, same treatment
`scene_xacro_args.py` gives `robot.base_pose`) and wired into
`ur5e_robotiq_sim_control.launch.py` as a second `ros_gz_sim create` spawn
alongside the robot's own — not another one-off `temp_test_table` like
Test 2's workaround. `scripts/08_spawn_pick_object.sh` added as the
object-side counterpart: spawns `object:` at `pick_pose` from scene.yaml
instead of a hand-typed `gz service` call.

**Prediction, written before re-running anything**: if the missing table
was the single cause behind Test 2's anomalies (per "the missing table is
one cause, not two anomalies" above), then with a genuine table present:

- The gripper close should reach `STALLED` within ~1-2s (matching the 40mm
  anchor's fast-stall behavior), not `TIMED_OUT_HELD` at the 5.0s bound.
- Lateral object drift during closing should be at or near zero — not the
  previously observed 13-20mm.
- Ground-truth TCP error should return to ~0mm, matching M2's established
  tracking accuracy — not the previously observed 53mm.

One fix, three symptoms: all three should disappear together, not
independently. Test 1 (no object spawned) should still correctly reject,
unchanged — that trial never depended on the table.

## Both tests re-run against the table world: prediction confirmed, first-ever grasp SUCCESS

Written 2026-08-09. Fresh sim + move_group, table verified spawned at the
correct pose live (`gz topic -e -t /world/empty/pose/info` showed
`table` at `x=0.55 z=0.375`, not the earlier bug's `(0,0,0)` — see below),
gripper open confirmed, controller-activation 2.4-3.8s across three
restarts (healthy band). `scripts/08_spawn_pick_object.sh` added
(single-source object spawn from scene.yaml's `object:` block, mirroring
`scene_table_sdf.py`'s treatment of `table:`) to replace Test 2's original
hand-typed `gz service` call.

**Real bug found and fixed before any test could be trusted**: the first
table-spawn attempt put the table at world origin `(0,0,0)`, not
`(0.55, 0.00, 0.375)`. Root cause: `ros_gz_sim create -string` does NOT
honor a `<pose>` embedded in the SDF string — it always spawns at
`-x/-y/-z/-R/-P/-Y`, defaulting to 0 if those flags aren't also passed.
This is specific to the `create` executable's own argument handling, not a
property of SDF or of the `gz service .../create` EntityFactory path
04/06/m0_verify.sh's scripts use (that one does honor an embedded pose,
confirmed by years of correct box placements). The robot spawn never
exposed this because its pose is baked into the URDF via xacro args
(`base_xyz`/`base_rpy`), not an SDF-level model pose — a genuinely
different mechanism that happened to look the same. Fixed:
`config/scene_table_sdf.py` gained `table_pose_args()`, returning explicit
`-x ... -Y ...` flags from `scene.yaml`'s `table.pose`, now passed
alongside `-string` in the launch file's spawn `Node`. Verified live after
the fix: `table` at `x: 0.55 z: 0.375` (y omitted = 0, matching
scene.yaml).

**Test 1 (no object) — confirmed still correctly rejects**, unchanged from
2026-08-08: gripper closed to `0.7901 rad` (near-full, nothing to catch),
compared against the table's `0.4055` expected for 45mm, `GRIPPER_GOAL_REJECTED`,
`tcp_error_m=0.0000` (perfect tracking, matching M2). Took 3 attempts to get
a clean run — see "New, unrelated flakiness surfaced" below; none of the
intervening failures involved the gripper/grasp logic at all.

**Test 2 (object spawned on the real table) — SUCCESS. First one on
record for M3.** `object.pick_pose` object settle-checked before the grasp
attempt: spawned via `08_spawn_pick_object.sh`, settled in 0.64s at
`z=0.79500` — exactly `table.surface_z + size.z/2`, i.e. genuinely resting
on the table, not falling through. Grasp result:

| metric | before (missing table, 2026-08-08) | after (table present, 2026-08-09) | M2 baseline | prediction |
|---|---|---|---|---|
| gripper result | `TIMED_OUT_HELD` at 5.0s bound | `STALLED` in ~2.0s | n/a | fast STALLED — **confirmed** |
| lateral object drift | 13-20mm | 8.7mm (measured via `gz topic -e`, spawn vs. post-grasp `pick_target` pose) | n/a | near zero — **mostly confirmed**, not fully zero |
| ground-truth TCP error | 0.0534m (53mm) | 0.0330m (33mm) | ~0.0000m | ~0mm — **partially confirmed**, improved but not zero |
| grasp-success check | N/A (rejected) | `achieved=0.3854` vs `expected=0.4055`, `|err|=0.0201 < tolerance=0.0235` — **WITHIN TOLERANCE** | n/a | n/a |

**One fix, three symptoms — direction and rough magnitude all confirmed,
not just the first one.** All three moved in the predicted direction and
by a similar large fraction (TIMED_OUT_HELD→STALLED: qualitative fix;
drift 13-20mm→8.7mm, ~50-60% reduction; TCP error 53mm→33mm, ~40%
reduction) — consistent with one shared root cause (the missing table)
dominating all three, not three independent effects that happened to
improve together by coincidence.

**Residual, NOT the same failure mode as before — a new, smaller, separate
finding.** 33mm TCP error is NOT random: `docs/m3_grasp_run3_test2_*.log`
+ CSV show it's almost entirely a z-axis gap — `achieved_z=0.827177` vs
`commanded_z=0.795000`, a 32mm vertical shortfall, with x/y off by only
~4-6mm each. Reading this together with grasp composition's own
`corrected_offset=0.133838` (`tcp_offset=0.120405 + pad_centre_offset=0.013433`,
flagged in this doc's design section as "NOT YET LIVE-VERIFIED" direction/
magnitude): a coherent explanation is that the gripper stalled on contact
~32mm before reaching the naive target depth, i.e. the correction is
standing tool0 off farther than the true pad-centre depth actually
requires. This is now the first LIVE data point bearing on that
not-yet-verified correction, not a confounded trial like the pre-table
Test 2 was — worth investigating before trusting `pad_centre_offset`'s
current 0.013433m value or the "add, not subtract" direction unattended.
**Not yet investigated further this session** — flagging per "chase only
what survives," and this genuinely is what survives: small, consistent
with a known open question (pad-centre correction direction/magnitude),
not the sliding/unsupported-object failure mode the table fix targeted.

**New, unrelated flakiness surfaced while re-running — neither is the
missing-table question, both are new open items:**

1. **OMPL/RRTConnect self-collision, reproduced 1/3 attempts.** Test 1's
   first genuine attempt (after fixing the move_group precondition below)
   failed `PLAN_FAILURE`: a found RRTConnect solution passed initial
   planning but failed `ValidateSolution`'s stricter recheck — self-collision
   between `forearm_link` and `robotiq_85_right_finger_link` at some
   intermediate waypoint. Same pre-grasp target coordinates as the
   successful 2026-08-08 run (`[0.4500 -0.1500 1.0288]`), so this is
   planner-search randomness (no fixed OMPL seed), not a regression from
   today's table change. **Open question, not yet investigated**: only one
   "Calling Planner 'OMPL'" line appeared in the move_group log despite
   `plan_attempts=10` being set via `setNumPlanningAttempts` — suggests
   MoveIt's attempt-count may govern re-search only when OMPL itself fails
   to find *any* solution within budget, not when a found solution later
   fails the separate `ValidateSolution` response-adapter recheck. If so,
   `plan_attempts=10` is not actually providing the retry margin its value
   implies for this specific failure mode. Worth checking against MoveIt's
   source before relying on it for the 20-cycle run.
2. **`rclcpp_action`: "unknown goal response/result response, ignoring"
   + `EXECUTE_FAILURE` code -7 on Test 2's first attempt. ROOT CAUSE FOUND,
   not a mystery: operator error this session, not an environment bug.**
   Before Test 2, `kill_sim` + `pkill -9 -f "move_group.launch.py"` was run
   to get a clean baseline, then a fresh `move_group` was launched. Checked
   afterward, not assumed: the `pkill` silently failed to match — `ps -eo
   pid,lstart,cmd` showed the ORIGINAL move_group (from Test 1, started
   12:24:59) still alive alongside the new one (started 12:29:48), **two
   move_group instances simultaneously claiming the same action server
   names** for the entire first Test 2 attempt. A planning request handled
   by one instance and an execute request answered by (or routed to) the
   other is a direct, mechanical explanation for "unknown goal response,
   ignoring" — no DDS-crosstalk speculation needed. The second attempt
   "succeeding" was luck (which of the two duplicate servers happened to
   answer which call), not a fix — the duplicate was still running
   underneath it. Killed the stray (`kill -9` on the actual PIDs, not
   `pkill -f`, then verified via `ps` — the same "checked, not assumed"
   discipline `gz_assert_clean_slate` already applies to `gz sim`/
   `robot_state_publisher`/`parameter_bridge`, which do NOT currently
   include `move_group` in their pattern match). **Action item, not yet
   done**: `gz_assert_clean_slate` (`scripts/lib/gz_settle.sh`) should
   also check for stray `move_group` processes — it currently only
   greps `gz sim|robot_state_publisher|parameter_bridge|controller_manager|
   spawner`, which is exactly the blind spot that let this go undetected
   through two full test attempts.

**Also newly required, not previously documented as a precondition
anywhere obvious**: `m3_grasp.launch.py`'s own header says move_group must
already be running, same as M1/M2 — true, but easy to miss (the file's
`PRECONDITION` comment says it, nothing enforces or checks it). First Test
1 attempt today hung past "Loaded robot model" for 30s+ with no error
before this was diagnosed as move_group simply not running. Worth a
startup guard (parallel to the existing base-pose guard) that fails fast
and names the missing precondition, rather than a silent hang — not done
this session, flagged as a real gap.

## pad_centre_offset=0 probe: correction direction confirmed right, residual is separate

Written 2026-08-09, same session, immediately after the table fix commit.
The 32mm z-shortfall above is roughly 2.4x `pad_centre_offset` (13.433mm)
— specific enough a signature to be worth checking before concluding
"the correction is slightly off": could instead mean the correction is
applied backwards, or double-counted against `tcp_offset`. One cheap probe
settles it while the sim is warm: re-run the identical grasp with
`pad_centre_offset` forced to 0.0 (via a scratch copy of scene.yaml passed
as `scene_file:=...`, not editing the real config) and check which way the
z-error moves. Object respawned fresh via `08_spawn_pick_object.sh`,
gripper reopened first, same pick_pose, same sim instance.

**Result: the error GREW, not shrank.**

| | with correction (0.013433) | without correction (0.0) |
|---|---|---|
| commanded grasp tool0 target z | 0.9288 | 0.9154 |
| ground-truth `grasp_tcp` z | 0.827177 | 0.8376 |
| z-shortfall vs. object (0.795) | 32.2mm | 42.6mm |
| total `tcp_error_m` | 0.032955 | 0.0436 |
| grasp-success check | outside tolerance (0.3854 vs 0.4055 expected) | outside tolerance (0.3631 vs 0.4055 expected) |

Growth = 10.4mm, in the same direction and roughly the same order as
`pad_centre_offset` itself (13.4mm) — not ~2x it, which rules out
double-counting (that would predict the *opposite* trial, the
WITH-correction one, showing roughly double the single-offset error, not
observed). **This settles the question the design doc had flagged as
"NOT YET LIVE-VERIFIED": the correction's direction (add, not subtract) is
right.** Removing it makes the grasp worse, not better.

**Sanity-checked the two numbers against each other, not taken at face
value**: `ground-truth grasp_tcp` is computed in `m3_grasp.cpp` as
`tool0_actual + tool0_basis * (0, 0, corrected_offset)` — the SAME
`corrected_offset` used both to command the approach depth and to
reconstruct the reported TCP position afterward. That means a perfectly-
tracking arm would report ~0 error regardless of the offset's correctness
(the offset cancels itself out of the reconstruction) — UNLESS the arm's
actual descent gets physically stopped short of its open-loop commanded
target by genuine contact resistance, which is exactly what should happen
here (the Cartesian descent doesn't know about the object; it's a kinematic
plan, and the object is physically in the way). Backed out the implied
actual tool0 height from both trials
(`tool0_actual_z = achieved_tcp_z + offset_used`): **0.9610m (with
correction) vs. 0.9580m (without), a 3.0mm difference** — i.e. the arm
stalled at essentially the SAME real-world height both times, independent
of which offset was commanded, consistent with contact (not the open-loop
target) determining where the descent actually stops. This means the
10.4mm growth in reported error is very close to a pure arithmetic
consequence of using a smaller offset in the reconstruction, not a
different physical outcome — exactly the kind of falsification check this
project's "a suspiciously good/specific number gets a check" discipline
calls for before trusting a two-point comparison.

**What this does NOT settle, flagged as the next open question**: the
32mm residual with the correction applied. Backing out the implied true
offset from `tool0_actual_z ≈ 0.961` and the object surface at `0.795`:
`0.961 - 0.795 ≈ 0.166m` needed vs. `0.133838m` currently used — suggesting
`tcp_offset + pad_centre_offset` may be undershooting the TRUE tool0-to-
pad-centre distance by roughly another 32mm at this trial's achieved stall
angle (0.3854 rad), not that the sign is wrong. One candidate thread, not
yet checked: this trial's achieved angle (0.3854 rad) differs from
`grasp_table.yaml`'s 45mm row (0.4055 rad, itself measured at 0.40553 rad
in the original sweep) — and `tcp_offset` is known to vary ~13.6mm across
the aperture range (scene.yaml's own tcp_offset table), so a stall at a
different angle than the table's reference row could mean the *tcp_offset*
component (not just pad_centre_offset) is off for this specific trial, not
only the pad-centre term. **n=2 on the direction question (settled); n=1
each on any magnitude number — do not tune `pad_centre_offset`'s value from
this alone.**

## Stall-angle/tcp_offset-interpolation check: ruled out, not partial

Written 2026-08-09, same session. The probe above left one candidate
thread for the 32mm residual: the successful trial's achieved stall angle
(0.3854 rad) differs from `grasp_table.yaml`'s 45mm reference row (0.4055
rad), and `tcp_offset` is known to vary ~13.6mm across the full aperture
range — so `tcp_offset` itself, evaluated at the trial's ACTUAL achieved
angle rather than the reference row's angle, might account for some of the
32mm. Testable with arithmetic already on hand, no new sim run: interpolate
`tcp_offset_at_grip_m` between the two `docs/grasp_table_20260808_135745.log`
rows bracketing 0.3854 rad (45mm row: angle=0.40553, tcp_offset=0.120405;
50mm row: angle=0.35892, tcp_offset=0.119550 — 0.3854 falls 43.2% of the
way toward the 50mm row) and compare against the 0.120405 value actually
used (the 45mm exact-width row, not angle-adjusted).

**Result: interpolated tcp_offset = 0.120036m, a delta of only 0.37mm from
what was used — 1.1% of the 32mm residual.** Cross-checked against a
second, independent source (the wider free-space joint-angle-vs-tcp_offset
table in `scene.yaml`'s own comments, `05_measure_gripper_geometry.sh`'s
output, interpolated between its 0.2 and 0.4 rad rows): 0.120064m, only
0.03mm apart from the grasp-table interpolation — the two methods agree,
so this isn't an artifact of which table was used.

**This rules the thread out, rather than confirming a partial explanation.**
If it had accounted for even ~5mm, the stall-angle/aperture-coupling story
would be a real partial answer with ~27mm left over. At 0.37mm it's
essentially nothing — `tcp_offset`'s own known aperture-dependence is NOT
where the 32mm comes from. The correction's SIGN is confirmed right (per
the zero-offset probe above); this arithmetic now also rules out one
specific, plausible-sounding candidate for the MAGNITUDE gap.

**Reframes, doesn't resolve, the "one phenomenon or two" question.** The
achieved angle being lower than the table's reference (0.3854 vs 0.4055 —
fingers stopped less closed than expected for a true 45mm object) could
still mean the gripper contacted something effectively wider than 45mm at
whatever depth it actually reached — i.e. the early stall and the z-shortfall
could be the SAME event seen two ways (off-target contact producing both a
smaller angle and a shallower position), not two independent effects. This
arithmetic doesn't test that hypothesis directly, but by eliminating the
"boring" explanation (tcp_offset's documented aperture-curve, applied
mechanically) it makes the same-underlying-cause reading more likely by
elimination, not because anything new confirms it. **Not yet checked**:
a live trial specifically designed to separate the two (e.g. does the
z-shortfall track spawn/target depth the way Blocker 2's falsification test
tracked spawn height 1:1, isolating position from angle) would be the
analogous decisive check, same method that closed Blocker 2 — not run this
session.

## Depth-tracking trial: two points confirm "clamped descent," a third exposes a different failure mode entirely

Written 2026-08-09, same session. The stall-angle arithmetic left "one
phenomenon or two" open — resolved (for the tested range) with the
Blocker-2-style falsification test that question actually calls for:
command the grasp at multiple depths (via `pad_centre_offset`, same
scratch-scene.yaml method as the zero-offset probe) and check whether the
achieved height tracks the command 1:1 (two effects: accurate tracking +
a separate constant offset error) or stays fixed regardless of command
(one phenomenon: something clamps the descent at a fixed physical height).
Two points already existed from the probes above (`offset=0.0` and
`offset=0.013433`); added a third at `offset=0.030` (commanded target
~30mm shallower than the first point) to get a real spread.

| trial | commanded tool0 target z | achieved z | shortfall |
|---|---|---|---|
| `offset=0.000` | 0.915405 | 0.9580 (back-calculated) | 42.6mm |
| `offset=0.013433` | 0.928838 | 0.961015 (back-calculated) | 32.2mm |
| `offset=0.030` | 0.945405 | **0.945405 (measured directly via `gz topic -e`, `wrist_3_link`)** | **0.0mm** |

**First two points: a clean, strong confirmation of "clamped descent."**
Commanded target moved 13.4mm between them; achieved height moved only
3.0mm (0.958 -> 0.961). That's the Blocker-2 falsification signature for
"something else has a fixed effect independent of what you asked for" —
matches the "one phenomenon" prediction closely.

**Third point does NOT extend that line — it's a qualitatively different
outcome, not more of the same trend.** Reached its FULL commanded target
with zero error (confirmed by direct `wrist_3_link` ground truth query,
not just the log's own claim) — no clamping at all this time. But the
object was found afterward displaced **180mm laterally and 23mm down**
from its settled spawn pose, and the gripper closed to `REACHED_GOAL`
0.7902 rad — the same signature as Test 1's original no-object trial
(0.7901 rad). **The object got knocked away during descent/approach
rather than the arm being stopped by it**, and by the time the close
action ran, there was nothing there. A light 150g object can be shoved
aside by a glancing, off-centre contact without generating enough reaction
force to visibly deflect the arm's own trajectory — different from the
first two trials, where contact was apparently square/centred enough
(object backed by the rigid table, not pushed aside) to actually stop the
arm's descent.

**Reading this honestly, not force-fit into the two predicted buckets**:
within the tested near-current-value range (`offset` 0 to 0.013433), the
data cleanly supports "one phenomenon — a fixed clamp near z=0.96, most
likely the fingers (at whatever aperture they're at during descent)
contacting the object's top edge before tool0 reaches its open-loop
commanded depth." Pushing `pad_centre_offset` substantially further
(`0.030`, ~17mm past the current value) does not approach zero shortfall
smoothly — it jumps into a THIRD regime: a glancing miss that ejects the
object rather than a controlled grasp. This is the same shape of finding
as Blocker 2's "capture window has a sharp edge, not a graceful one" — the
useful correction range likely has a boundary not far past the current
value, past which the geometry stops producing a clean square contact at
all.

**Practical consequence, not yet acted on**: the fix for the 32mm residual
is NOT "increase `pad_centre_offset` until shortfall reaches zero" — the
two-point data shows the clamp barely responds to a modest increase (3mm
of movement for 13.4mm of command), and a larger increase found the edge
of the capture window instead of approaching zero. The 32mm gap likely has
a different root cause than `pad_centre_offset`'s magnitude alone —
plausibly the finger/pre-close aperture geometry during descent, not
purely a standoff-distance calibration question. **Not yet investigated**: what specifically stops the arm at z≈0.96.
Checked one candidate mechanism immediately, before leaving it on record
unverified: does the gripper pre-close to `gripper.preclose_heuristic`
before or during the Cartesian descent, such that the fingers' own
geometry at that narrower aperture could clip the object before tool0
reaches full depth? **No** — grepped `m3_grasp.cpp` directly:
`preclose`/`PRECLOSE` do not appear anywhere in it. The gripper stays
fully open (near-max aperture, ~85mm, well wider than the 45mm object)
through the entire Cartesian descent; closing only happens afterward via
`gripper_close_and_hold`. This is itself a known, separately-tracked gap
(`config/scene.yaml`'s `gripper.preclose_heuristic` is "still the old
single-scalar placeholder," per "Other M3 prerequisites" below — "M3 will
fail and look like a friction problem otherwise") but it rules OUT
fingers-at-preclose-aperture as the mechanism behind THIS clamp, since
there's no preclose happening at all yet. With the gripper fully open
throughout descent, the fingers are spread wider than the object and
shouldn't intersect its width — leaving the actual clamp mechanism
genuinely unexplained: a GUI look at the descent in progress is the
natural next step, not a code-reading guess.

## What touches first: inner knuckle links, not the pads — geometry confirmed live

Written 2026-08-09, same session, direct follow-up to a sharper read of the
depth-tracking data than the write-up above gave credit for: point 3
(offset=0.030) reaching its FULL commanded depth with zero shortfall
disproves a fixed-height clamp outright — if something were limiting the
descent at a fixed z regardless of command, point 3 (commanded to go
DEEPER than points 1-2) would have hit it too. It didn't; the object got
displaced instead. That means the OBJECT is the obstruction, not a fixed
mechanical limit independent of it. Sharper still: reconstructing
`tool0 - tcp_offset` (NOT the full corrected offset — the naive,
fingertip-link-origin anchor point) for the two stalled trials lands
within 0.6-2.4mm of the object's own top face (`pick_pose.z=0.795 +
half-height 0.045 = 0.840`) — checked, not assumed:

```
offset=0.000:    tool0-tcp_offset = 0.8376   gap to top = -2.4mm
offset=0.013433: tool0-tcp_offset = 0.8406   gap to top = +0.6mm
```

That precision (sub-3mm on two independent trials) is not a coincidence.
**The descent is terminating on contact with the object's TOP FACE, not
settling around its sides at the intended pad-centre depth.**

**Live-instrumented to find out which part, rather than guessed.** Two
open questions this raised: (1) the open aperture (~85mm nominal for a
2F-85) should give ~20mm clearance per side around a 45mm object — so
either the approach is laterally misaligned by more than that, or some
part OTHER than the pads is what's contacting; (2) is the commanded grasp
z geometrically sound (targeting the object's centre, requiring the
descent to pass the top face without contact)? Answered (1) directly:
re-ran the grasp with a tight poll loop watching the log for "execution
reported SUCCESS" (Cartesian descent complete, gripper still fully open,
closing not yet started) and fired an immediate full `gz topic -e -t
.../pose/info` snapshot at that instant — catching the geometry BEFORE any
finger motion could confound it. Evidence:
`docs/m3_grasp_probe_instrumented_20260809_145648.log` (run log) +
`docs/m3_pose_snapshot_at_stall_20260809_145648.txt` (raw pose dump).

**Result — fingertip pads have generous clearance; inner knuckle links do
not, at all:**

| part | y-separation | half-width | clearance vs. object half-width (22.5mm) | z vs. object top (0.840) |
|---|---|---|---|---|
| fingertip pads | 129.3mm | 64.6mm | **+42.1mm** (well clear) | −11.5 / −9.4mm (already below top — correct, intended depth) |
| outer knuckle links | 61.2mm | 30.6mm | **+8.1mm** (clear, but tight) | +34.9 / +35.9mm (well above top at this instant) |
| inner knuckle links | 25.4mm | 12.7mm | **−9.8mm** — **inside the object's own footprint** | +28.7 / +29.1mm (above top, but laterally already overlapping) |

**This is decisive on its own, independent of any dynamics.** The
fingertip pads are correctly positioned — already below the object's top,
generously clear laterally, exactly where a proper grasp needs them. The
inner knuckle links, by contrast, sit laterally INSIDE the object's own
45mm-wide footprint (both left and right) at this fully-open aperture —
meaning any further descent into the object's height band (0.75-0.840)
guarantees a collision there, structurally, independent of the fingertip
pads' own generous clearance. **It is not "the fingers failing to
straddle" — the pads straddle fine. It's the knuckle assembly, closer to
the base, whose lateral spacing does not scale with aperture the way the
pad spacing does, and which cannot clear this object's width at all.**

**Caveat, stated precisely rather than overclaimed**: this geometry was
measured at ONE joint state (fully open, aperture=0) — whether the inner
knuckle links' lateral position stays fixed across the full aperture range
(plausible, since a rotating link's reported origin is typically at or
near its fixed base pivot, not its swept extent) or changes with closing
was NOT checked across multiple apertures this session. Treat "does not
scale with aperture" as a reasoned inference from one measurement, not an
established fact across the range.

**Secondary, softer confirmation attempted, weaker than intended.** Tried
a second instrumented capture ~1.3s into the closing window (mid-motion)
to catch the actual contact moment. That particular re-run turned out
noisier than the clean trials — `TIMED_OUT_HELD` rather than `STALLED`,
`tcp_error_m=0.0517` (larger than the clean-trial baseline) — matching the
still-occasional variability already documented in this contact regime.
By 1.3s in, the object had already been shoved substantially from its
spawn pose (~30mm/26mm lateral), making a clean "linkage vs. original
footprint" comparison impossible; qualitatively still consistent with the
inner knuckle links remaining very close to the object's (now-shifted)
footprint, but this run is a supporting data point, not a second clean
proof — don't lean on it the way the static full-open snapshot can be
leaned on. Evidence: `docs/m3_grasp_probe_instrumented2_20260809_145648.log`
+ `docs/m3_pose_snapshot_mid_close_20260809_145648.txt`.

**Consequence for the 32mm residual and for `pad_centre_offset` more
generally**: this reframes the whole open question. It was never really a
`pad_centre_offset` calibration error — the correction's DIRECTION is
still confirmed right (per the zero-offset probe), but the MAGNITUDE
question is moot for this object width: no value of `pad_centre_offset`
fixes a gripper whose inner-knuckle assembly cannot geometrically clear a
45mm-wide object. This is a **gripper/object width compatibility limit**,
not a targeting-calibration gap — consistent with (and now mechanistically
explaining) point 3's capture-window-edge finding above: push the standoff
far enough and the interaction changes from "knuckle clips the top" to "the
whole approach misses/glances," never passing through a clean zero-shortfall
regime in between for this object width. **SUPERSEDED by the section immediately below** — the "narrower object
width" framing here was wrong; the real constraint turned out to be object
HEIGHT, not width, and is now rigorously confirmed via mesh geometry, not
inferred from one link-origin snapshot. Left here struck through in
spirit (not deleted) so the reasoning trail stays visible, same practice
as this doc's other corrections.

## "Positioned wrong or built wrong?" — resolved via URDF geometry, not a GUI look

Written 2026-08-09, same session, direct response to a legitimate
methodological challenge: does inner-knuckle lateral spacing actually
change with aperture (25.3mm fixed at ALL apertures would be geometrically
suspicious for a 0-85mm-stroke four-bar), and are the knuckles positioned
wrong (descent too deep) or is the gripper built wrong (structural
incompatibility)? Both answered with data, not a GUI look — the GUI isn't
available to this agent for direct visual inspection, but URDF/mesh
geometry is, and turned out to be decisive.

**Check 1 — swept the master joint 0.0 to 0.767 rad (10 points), same
method as `05_measure_gripper_geometry.sh`, tracking inner-knuckle,
outer-knuckle, and fingertip Y-separation and Z-position relative to
`wrist_3_link` at each step** (no object in the scene — pure free-space
gripper-geometry characterization). Result: inner-knuckle Y-separation is
**dead flat at 25.3mm across the entire range** (25.3mm at 0.0 rad,
25.3mm at 0.767 rad, no measurable variation in between). Outer knuckle
similarly flat (61.0mm -> 60.9mm). Fingertip pads DO vary substantially
with aperture (135.2mm open -> 102.5mm at the achieved stall angle 0.385
rad -> 102.6mm near-closed) — confirming the sweep methodology is
sound and the flatness of the knuckle links is real, not a sampling
artifact. Full sweep evidence: `docs/m3_knuckle_sweep_20260809_151156.log`.

**Why the knuckle links are aperture-invariant — checked in the URDF, not
assumed.** `robotiq_2f_85_macro.urdf.xacro`: both
`robotiq_85_left_inner_knuckle_joint` and `robotiq_85_left_knuckle_joint`
are declared with `<axis xyz="0 -1 0" />` — rotation exactly about the Y
axis, the SAME axis the fingers separate along. Rotation about an axis
cannot change any point's coordinate ALONG that axis — so not just the
link's reported origin (which sits on the rotation axis by construction)
but the link's ENTIRE body, wherever its mesh actually extends, has a
Y-coordinate that is mathematically invariant under the joint's rotation.
This is a real, deliberate property of this gripper's kinematics (the
proximal linkage stays close to the mounting plane; only the distal pads
swing outward as the four-bar opens), not a URDF/mimic bug and not a
simulation artifact.

**Correcting a real gap in the earlier "what touches first" analysis,
directly, before it went further**: the earlier −9.8mm clearance number
used each link's reported GROUND-TRUTH ORIGIN, which for a link on a
fixed-axis pivot is the pivot location, not necessarily where its physical
COLLISION MESH extends to. A body can pivot at one point while its mass
sits elsewhere entirely. This needed checking, not defending. Read the
actual `<collision>` mesh geometry from the URDF and computed real
bounding boxes directly from the STL files
(`left_inner_knuckle.stl`, `left_knuckle.stl`, in
`/opt/ros/jazzy/share/robotiq_description/meshes/collision/`):

| link | local Y bbox (mm) | world Y-span (pivot + bbox) | object Y-span | overlap |
|---|---|---|---|---|
| inner knuckle (left) | −17.35 to +17.65 | [−0.1555, −0.1205] | [−0.1731, −0.1281] | **27.4mm** |
| outer knuckle (left) | −8.08 to +8.48 | [−0.1282, −0.1116] | [−0.1731, −0.1281] | 0.1mm (noise floor) |
| fingertip pad | (varies with aperture — see sweep table) | clear at every tested aperture | — | none |

**The inner knuckle's physical mesh overlaps the object's lateral
footprint by 27.4mm — not the link-origin's -9.8mm approximation, the
actual collision geometry, confirmed by three independent lines of
evidence converging (empirical sweep flatness, the joint-axis rotation
argument, and the STL mesh bounding box itself).** This is now
established, not inferred from one ambiguous snapshot. The outer knuckle
is genuinely borderline (0.1mm, within measurement noise) — not the
culprit either way. Fingertip pads remain clear at every aperture tested,
including the achieved stall angle, not just full-open.

**"Positioned wrong or built wrong" — answered directly, with a margin
number, not a guess.** Computed where the inner knuckle would sit if the
pads were positioned EXACTLY at the object's true grasp depth (mid-height,
0.795 — the intended, correct target, not the ~32mm-short position this
session's trials actually reached). Fingertip-to-inner-knuckle Z offset
(measured in the same sweep, at the achieved stall angle): 49.7mm (knuckle
sits that much closer to tool0 than the pads). If pads sat exactly at
0.795: knuckle would be at `0.795 + 0.0497 = 0.8447`, versus the object's
top at `0.840` — **a margin of only 4.7mm, even under perfect
positioning.** Combined with the knuckle's mesh having substantial local
Z-extent (up to ~49mm) that sweeps through world space as the linkage
closes, and zero-to-negative lateral clearance throughout, a 4.7mm margin
is not survivable — contact is essentially inevitable regardless of
getting the depth exactly right.

**Answer: BUILT, not positioned.** This is a genuine structural
incompatibility between this gripper's proximal-linkage geometry and this
object's HEIGHT (90mm) — not a targeting/depth bug in `m3_grasp.cpp`'s
composition, and not fixable by any `pad_centre_offset` value. The
2F-85's 85mm figure describes fingertip STROKE (open-to-closed pad
travel), which is a different quantity entirely from proximal-linkage
ground clearance over a tall object — a real product can have a full
85mm stroke and still have its knuckle assembly foul a sufficiently TALL
object approached from directly above, exactly as this data shows.
Whether the physical 2F-85 has this same limitation is unverified (this
URDF is the best available model of it, per this project's own recon
process, but recon confirmed joint names/limits, not this specific
geometric interaction) — flagged, not asserted as a general 2F-85 fact.

**Correction to the earlier (wrong) recommendation**: this is a HEIGHT
constraint, not a WIDTH constraint. The earlier suggestion to check
whether a narrower `object.size` width would clear was based on the
flawed link-origin analysis and pointed at the wrong dimension entirely —
width was never the limiting factor (fingertip pads clear at every
aperture regardless of object width, up to their own stroke limit). The
actual lever, if this needs a design-level fix rather than a different
object, is `object.size`'s HEIGHT (currently 0.090m) relative to the
~50mm fingertip-to-knuckle Z offset — an object with less than roughly
50mm of half-height (i.e. under ~100mm tall, with real margin needed
comfortably under that) would let the knuckle clear the top even at the
correct mid-height grasp depth. **Not yet decided or acted on** — this is
a finding to inform that decision, not a decision itself.

## Object height changed to 45mm cube — depth fix confirmed, a DIFFERENT problem exposed

Written 2026-08-09, same session. Per the formula above
(`knuckle_clearance_mm ≈ 50 - height_mm/2`, validated against the live
4.7mm measurement at height=90mm), changed `config/scene.yaml`'s
`object.size` from `[0.045, 0.045, 0.090]` to `[0.045, 0.045, 0.045]` (a
cube — width unchanged, so the already-measured grasp table stays valid)
and updated `pick_pose.z`/`place_pose.z` from 0.795 to 0.7725
(`table.surface_z + new_size.z/2`). Predicted ~27.5mm knuckle clearance at
this height, comfortable margin over the 90mm object's marginal 4.7mm.

**Depth tracking: confirmed, dramatically.** `tcp_error_m` dropped from
32mm (90mm-tall object) to **4.1mm** — reproduced identically on two runs
(`docs/m3_grasp_cube_test_*.log`, `m3_grasp_cube_test.csv`/`cube_test2.csv`).
This validates the underlying reasoning: with the knuckle interference
removed, the arm actually reaches its intended target depth for the first
time this session, matching M2's own established tracking accuracy far
more closely than any prior M3 trial. The formula's core claim — that the
32mm residual was a knuckle/height problem, not a `pad_centre_offset`
calibration gap — is now confirmed by more than just the static geometry
argument: fixing the geometry fixed the depth-tracking number directly.

**But the grasp itself did NOT succeed — a different, newly-exposed
mechanism, not a leftover of the old one.** Both cube-height runs reported
`STALLED` at `achieved=-0.0000 rad` (essentially zero — the gripper never
meaningfully closed at all) in ~0.3s, far faster than any genuine contact
stall this session (all previous stalls, clean or confounded, took
1.7-2.9s). Reproduced identically (same `tcp_error_m=0.0041` to 4 decimal
places on both runs) — not noise, a deterministic consequence of the new
geometry. Checked, not assumed: the object was confirmed UNMOVED
(ground-truth `pick_target` position exactly matches its settled spawn
pose after the "stall") — this is not another knock-away.

**Root cause, checked against the same fingertip sweep data used above,
not guessed.** The knuckle-clearance formula only modeled the KNUCKLE's
geometry (aperture-invariant, per the Y-axis rotation argument). It did
NOT account for the FINGERTIP PADS' own vertical travel as they close —
which the earlier sweep data already contained but wasn't applied here:
pads descend ~8.5mm just going from 0.0 to 0.3 rad of closing. At the new,
shorter object's top (0.795), the pads sit only **6.0mm above** it at
full-open (measured this run: pad z=0.801) — meaning the first ~0.15-0.2
rad of closing motion (a small fraction of the ~0.4055 rad needed for a
real grip) is enough for the pads themselves to descend into contact with
the object's top face, before any real closure happens. **Shrinking the
object fixed the knuckle's problem and shifted the binding constraint to
the pads' own descent-while-closing — a mechanism the "positioned wrong or
built wrong" analysis didn't model, because it only asked whether the
KNUCKLE clears at a FIXED grasp depth, not whether the PADS clear while
actively closing.**

## Pre-close implemented — validated in free air, but a self-found bug blocks trusting the deeper analysis

Written 2026-08-09, same session. Implemented pre-close in `m3_grasp.cpp`
per the reasoning above: a new Stage 1.5 between pre-grasp arrival and the
Cartesian descent, closing to `interpolate_grip_angle(width) -
preclose_margin_rad` (new `grasp.preclose_margin_rad` scalar in
scene.yaml, default 0.05 rad, unvalidated) while still at pre-grasp
height — free air, standoff metres above the object. Reuses the SAME
`gripper_close_and_hold` bounded-close mechanism, just with a different
target and a different expected outcome (`REACHED_GOAL`, not `STALLED` —
nothing should be there to stall against). Grasp-table interpolation moved
earlier in the function so this target is available before the descent;
the final close-and-verify call downstream is otherwise unchanged, just
starting from a smaller residual. Compiled clean
(`colcon build --packages-select ur5e_pick_place`).

**Pre-close itself works exactly as designed.** Live run:
`pre-close: REACHED_GOAL in achieved=0.3460 rad (target was 0.3555, free
air)` — clean, no anomaly, confirms nothing is in the way at pre-grasp
height and the mechanism does what it's supposed to.

**The grasp still fails — with a DIFFERENT, not obviously related,
symptom.** Final result: `gripper_close_and_hold: REACHED_GOAL in
achieved=0.7906 rad` (essentially fully closed — the same signature as
Test 1's original no-object trial) and `tcp_error_m=0.0651` (65mm, worse
than either the pre-preclose 32mm or the non-preclose cube test's 4.1mm).
Checked ground truth: the object moved from its settled spawn pose by
~17mm/12mm laterally, Z UNCHANGED (still exactly at table height) — shoved
sideways along the table, not knocked into the air. `docs/m3_grasp_preclose_test_*.log`.

**Attempted to explain this with the same STL-mesh method that resolved
the knuckle question — found a bug in my own analysis before trusting it,
not after.** Extended the mesh-bounding-box approach to estimate the
fingertip pad's Z-extent (not just knuckle Y-extent) and to model the
KNUCKLE's Z-sweep during the whole descent trajectory (not just its final
resting position) — reasoning that pre-closing before descent means the
already-lowered knuckle sweeps through the object's height band for the
ENTIRE Cartesian descent, not just at the end, which the earlier "final
position only" margin check never modeled. This produced numbers
suggesting collision was likely (and worse with pre-close than without).
**Before reporting that as a finding, sanity-checked it against a case
where the physical answer is already known**: applied the same mesh-Z
method to the earlier NON-preclose h=45mm test (fully open aperture, no
contact yet, wrist_3 ground truth already on record) and asked where it
predicts the pad's lowest point to be. **Answer: 0.7447m — below the
table surface (0.750m), which is physically impossible** (the gripper
never touched the table in that trial). The origin-based reading from the
SAME moment (0.801m, using this project's established `tcp_offset`
methodology) is physically sensible and consistent with everything
observed. **This means my mesh-based Z-axis reasoning has an unresolved
bug or methodological error — most likely a mismatch between the STL
mesh's authored local frame and the rotation I applied, or the wrong mesh
file/orientation source** — not yet found. The Y-axis knuckle-overlap
finding (27.4mm) is NOT necessarily affected (it was independently
cross-validated by the aperture-invariance argument, a separate,
purely-mathematical check unrelated to any Z-axis reasoning), but
everything built on the mesh-based Z-extent — the "h<75.7mm" formula, the
"15.4mm margin at h=45mm," and this session's attempted explanation for
why pre-close made things worse — **is retracted as unverified, not
confirmed**. Better to find this by checking against a known-physical case
than to hand the user a formula built on a silent bug.

**Where this leaves things, honestly**: pre-close's mechanism is
validated (clean in free air) and worth keeping — it's still a documented
spec requirement independent of whether it alone solves this object's
grasp. But it did not fix the h=45mm cube grasp, and turned a
partially-understood failure (pad stalls at ~0 rad) into a
less-understood one (full closure, object shoved sideways) that this
session's tools couldn't reliably explain. **Not recommended: continuing
to guess at further targeting/geometry tweaks without a reliable way to
verify them** — this is exactly the "trial number five" scenario worth
avoiding. **Two paths forward, neither taken yet**: (1) a live GUI look
(`gazebo_gui:=true`) — this agent cannot see it directly, but the user
can; (2) a position-only (no mesh/orientation math) multi-sample capture
of the object's ground-truth pose throughout the descent trajectory, to
at least pin down WHEN contact starts without relying on the buggy
mesh-Z method. Both are more reliable than another formula built on the
same unverified geometric reasoning.

## Trajectory capture: lateral-capture hypothesis confirmed directly, clearance math corrected, real progress

Written 2026-08-09, same session. Ran the position-only trajectory
capture instead of continuing to guess: a background poller sampled
`pick_target`'s ground-truth pose every 0.2s throughout an entire grasp
attempt (pre-close through final close), correlated against the run log's
own stage timestamps. No mesh, no orientation math — pure position, so it
can't inherit the Z-axis bug found above.

**Result: the object starts moving ~1.1s into the ~2.0s Cartesian
descent** — well before reaching target depth, well before the final
close phase even begins. This is unambiguous: a pad (or the gripper more
generally) is contacting the object DURING descent, not during closing.
Confirms the first of the three predicted outcomes — lateral misalignment,
not a vertical/knuckle mechanism. Evidence:
`docs/m3_grasp_traj_test1_20260809_172336.log` (run log) +
`docs/m3_object_trajectory1_20260809_172336.txt` (raw position trace).

**Corrected a second calculation error, this one in clearance, not
Z-axis.** The earlier "~29mm of pad clearance" estimate used the
fingertip LINK's raw Y-separation directly against the object's
half-width — wrong, because the actual rubber pad sits inset from that
link's reference point (an existing, already-documented project note:
"pads sit roughly 28mm inward per side," inferred, not directly measured
before now either). Correcting for that inset: true pad-to-pad clearance
at the original `preclose_margin_rad=0.05` pre-close angle (0.346 rad) is
only **~1.4mm per side** — not 29mm — against a measured ~8.7mm lateral
error. Capture failure is exactly what this predicts. At full open,
corrected clearance is ~17mm/side, comfortably absorbing that same error —
consistent with why the ORIGINAL (no pre-close) descent never had this
problem.

| angle (rad) | fingertip y-sep (mm) | corrected clearance/side (mm) |
|---|---|---|
| 0.0 (full open) | 135.2 | 17.1 |
| 0.1 | 126.2 | 12.6 |
| 0.2 | 116.5 | 7.8 |
| 0.346 (old preclose target) | 103.7 | 1.4 |
| 0.385+ (near/at grip angle) | ~102.5 | 0.8 |

**Increased `preclose_margin_rad` from 0.05 to 0.30** (targeting ~0.1 rad
pre-close, ~12.6mm/side clearance, comfortably over the 8.7mm error) and
re-ran the SAME trajectory capture. **Real, substantial improvement**:
`tcp_error_m` dropped to **8.1mm** (vs. 65mm before), and the gripper
result changed from a clean miss to genuine, sustained contact —
`TIMED_OUT_HELD` at `achieved=0.0910 rad`, holding stably. Trajectory
data: object stationary through pre-close, a brief ~5mm lift and small
lateral shift (~7mm/2mm) about 0.9-1.3s after pre-close completes, then
**settles and holds rock-steady for the remaining ~10s of capture** — not
knocked away this time, genuinely caught and held. Evidence:
`docs/m3_grasp_traj_test3_20260809_172336.log` +
`docs/m3_object_trajectory3_20260809_172336.txt`.

**Not yet a passing grasp**: `achieved=0.0910 rad` is far short of the
table's `0.4055 rad` expected angle for this object. Checked before
concluding anything about why: was this a genuine contact stall, or a
false-positive from the still-unmeasured active-motion noise floor risk
flagged when `stall_velocity_threshold` was picked ("a stray 0.2s quiet
window mid-motion could in principle false-trigger early")? Two cheap
discriminators exist (re-issue the close and see if it moves further; or
check the elapsed time — genuine stalls this session ran 1.7-2.9s, the
earlier no-object misses ran ~0.3s). `GRIPPER_HOLD_ELAPSED_S` was never
ported from `scripts/lib/gz_settle.sh` to `m3_grasp.cpp`'s
`gripper_close_and_hold` — a real, separate gap — so it isn't in the log
verbatim, but computable from the log's own timestamps.

**Result: neither predicted pattern. `TIMED_OUT_HELD`, elapsed 5.209s —
the FULL `gripper.command_timeout_s` bound (5.0s), not the fast ~0.3s a
false stall would produce.** `TIMED_OUT_HELD` specifically means the
controller's own `stall_velocity_threshold`/`stall_timeout` mechanism
NEVER cleanly declared `stalled: true` within the whole 5-second window —
velocity stayed above threshold, or kept crossing it, often enough that a
clean 0.2s quiet window never completed, for the entire bound. This rules
out the false-stall hypothesis specifically (a false stall would resolve
FAST, not time out) — but it's also not a clean genuine stall either.
Read together with the trajectory data above (object lifts ~5mm, shifts
laterally, then settles into a stable hold over roughly this same window):
the picture is sustained, MESSY contact — the object moving/settling
against the pad kept generating enough velocity noise to repeatedly reset
the stall-timer's quiet-window requirement, so the controller never got a
clean 0.2s to declare done, and the 5s client-side bound is what actually
ended the call. **Genuine contact, not a noise-floor false trigger** — the
open question is why it resolves at such a wide aperture (off-centre
contact, tipping, or wedging against a single pad), not whether contact
happened at all. Not investigated further this session, per plan.

**Reframe, recorded before it's lost — the question above may be the
wrong one.** `TIMED_OUT_HELD` means the close call was CUT OFF at the 5s
bound, not resolved by a controller-declared stall. `achieved=0.091 rad`
is wherever the joint WAS when the client gave up waiting, not
necessarily where it would have settled. With velocity noise present
throughout (that's what prevented the controller's own stall check from
ever completing), the joint may simply have still been CLOSING, slowly,
when time ran out — not sitting wedged at an equilibrium. This is a
different question than "why does it resolve at 0.091" — it may instead
be "why is it closing so slowly" or "it just hadn't finished yet."

**Tomorrow's first job, cheap**: re-run with `gripper.command_timeout_s`
extended well past 5s (e.g. 60-90s, matching the scale of the original
pre-fix stall-latency finding) and watch the achieved angle over time
(same style as the earlier mid-close instrumented captures, or a direct
extended, unbounded diagnostic).
- **Continues closing toward ~0.4055** → not an equilibrium at all, just
  a bound too short for whatever's damping the motion this time. Same
  shape as the original 60-90s stall-latency finding this project already
  solved once (`stall_velocity_threshold` fix) — worth checking whether
  this is a recurrence of that same class of problem in a new context
  (contact-loaded, off-centre, or otherwise messier than the anchor case
  the fix was tuned against), or something new.
- **Sits at ~0.091 with velocity noise but no net travel over a much
  longer window** → genuinely wedged; off-centre/single-pad contact
  becomes the live question, investigated for real rather than assumed.

A single extended-bound run separates these two completely different
next investigations before either gets chased on a guess.

## Extended-timeout diagnostic: neither predicted branch — a third, more surprising result

Written 2026-08-09, next session (same day). Ran the diagnostic with
`gripper.command_timeout_s` raised to 30s (scratch scene.yaml, not the
real config) and, this time, polled the MASTER JOINT ANGLE itself
(ground truth, `/world/empty/model/ur5e_robotiq/joint_state`) every 0.5s
throughout, rather than the object's pose.

**Prediction, written before running.** Leaning toward "keeps closing
toward ~0.4055" (a too-short bound, same shape as the original 60-90s
stall-latency finding), for two reasons already on record: (1) the
trajectory data showed the object still actively settling — lifting
~5mm, shifting laterally — in the same window the joint was timing out,
consistent with an ongoing, not-yet-resolved contact process rather than
a static wedge; (2) `TIMED_OUT_HELD`'s defining property is that velocity
never stayed under threshold for a clean 0.2s, which is easier to explain
by continued (if slow/damped) motion than by a truly static jam, though
a genuine wedge with persistent micro-vibration could also produce that
signature — this prediction could be wrong. Extending the bound to 30s
and logging the master joint over time settles it either way.

**Result: the prediction was wrong, but not in favour of the other
branch either — a third, more surprising thing happened.** Neither "kept
closing toward 0.4055" nor "sat flat at ~0.091 with noise." Instead, the
master joint (ground truth, no explicit gripper command issued during
this window — pre-close had already completed and held at 0.096 rad,
final close hadn't started yet) **swung on its own from 0.096 up to
0.497 rad and back down to fully open (~0), entirely DURING the arm's
Cartesian descent**:

| t (offset from descent start) | joint angle (rad) | note |
|---|---|---|
| −0.38s | ~0.000 | just before pre-close lands |
| +0.26s | 0.303 | jumps — no command issued here |
| +0.91s | **0.497** | peaks — past the FULL grip angle (0.4055) |
| +2.32s | 0.003 | back near zero |
| +5.10s onward | ~0.000 | fully open, stays here for the remaining ~30s |

The final close call (issued only after this had already happened, once
the arm's Cartesian descent completed) found nothing: `STALLED` in 0.75s
at `achieved=-0.0000` — a clean, fast stall at essentially zero closure,
the same signature as a genuine no-object miss. Ground truth on the
object afterward: moved ~11mm/1mm laterally, Z unchanged — a real but
modest disturbance, smaller than the earlier 17mm/12mm shove.
`docs/m3_grasp_extended_timeout_20260809_173827.log`,
`docs/m3_master_joint_trace_20260809_173827.txt`.

**Reading this, not just reporting it**: the joint moving substantially
with NO explicit command issued is only possible because contact force
can move the mimic linkage independently of what's commanded — an
already-established fact of this stack (`dartsim does not enforce mimic
constraints... contact DOES oppose it`, from the M-1 recon). The most
coherent story: the pre-closed gripper (0.096 rad, ~12.6mm/side
clearance) caught the object during descent — consistent with the
lateral-capture mechanism already confirmed — swinging the follower
joint up toward (and past) the full grip angle as contact resisted the
mimic linkage, then the object SLIPPED FREE partway through, letting the
joint relax back open. This is NOT a controller-timing question at all
(the extended 30s bound was never even exercised — the eventual close
stalled cleanly in well under a second) — it's a **contact-dynamics**
question, and it means the SAME configuration
(`preclose_margin_rad=0.30`, same object) that produced a stable catch
one run (traj_run3: held 10+s, `tcp_error_m=0.0081`) produced a
catch-then-release the very next run. **This configuration does not yet
reproduce reliably — that's the headline finding, more than either
predicted branch was.**

**Consequence**: the "keeps closing vs. sits flat" question this
diagnostic was built to answer turned out not to be the live one this
run — a third failure mode (transient capture, then release, during
descent) preempted it. Worth repeating this diagnostic a few more times
before concluding anything about run-to-run consistency at
`preclose_margin_rad=0.30` — n=2 (one clean catch, one catch-then-release)
is not enough to characterize a contact process already known from this
project's own history to be sensitive near a marginal clearance boundary
("capture window has a sharp edge, not a graceful one," Blocker 2). Not
run further this session — flagging the variability itself as the
finding, not chasing a fix for it yet.

**Also queued for next session**: port `GRIPPER_HOLD_ELAPSED_S` from
`scripts/lib/gz_settle.sh` into `m3_grasp.cpp`'s `gripper_close_and_hold`
— reconstructing elapsed time from log timestamps after the fact is
exactly the shape of hazard that produced the M0-C baseline-artifact
mistake earlier in this project; the C++ node should carry the same
instrumentation the bash version already does, not require re-deriving it
each time.

**One unexplained anomaly, flagged not chased**: an intermediate re-run at
`preclose_margin_rad=0.30` (before the one reported above) logged a
pre-grasp/grasp target matching the OLD 90mm object's height
(`0.9288`/`1.0288`) despite `scene.yaml` verified, both before and after,
to correctly hold the 45mm cube's values. Did not reproduce on immediate
retry with the identical procedure (confirmed via a direct Python read of
`scene.yaml` immediately before that retry's launch). No second copy of
`scene.yaml` found on this filesystem (checked via `find /`) that could
explain a stale read. Not chased further given it didn't reproduce and
time was better spent on the confirmed finding above — but worth
remembering if a similarly stale-looking target value shows up again in a
future session.

## Why it consistently stops near 0.09-0.10 rad: two checks, one ruled out, one real lead

Written 2026-08-09, same day, watching live with the user via
`gazebo_gui:=true`. Across several runs at `preclose_margin_rad=0.30`,
the final close consistently lands within ~0.005 rad of pre-close's own
target (0.096-0.1055) regardless of outcome (stable hold, catch-then-
release, or hold-with-zero-disturbance). That tight clustering, right at
the pre-close boundary, is the thing to explain.

**Check 1 — does the controller's stall-timer carry over from the
pre-close hold into the final close, making it look stalled before it
even starts moving? Ruled out, by reading the actual controller source,
not guessing.** `gripper_close_and_hold()` unconditionally re-issues a
hold command after every call (line ~306) — including pre-close — so the
final close's goal is sent right after the joint was just being actively
held stationary. If the stall-detection clock didn't reset on the new
goal, a stall could fire almost instantly regardless of real motion.
Checked directly:
`/opt/ros/jazzy/include/gripper_action_controller/gripper_controllers/gripper_action_controller_impl.hpp`,
`accepted_callback()`: `last_movement_time_ = get_node()->now();` runs
unconditionally on every accepted goal (line 109), before the new goal
starts executing. **This is not a stall-timer carryover bug** — the
clock genuinely resets each time.

**Check 2 — compare this project's own free-space sweep (used to build
every clearance estimate this session) against its own established
real-contact measurement. Found a real, directly-measured gap.** At a
DIRECTLY comparable angle (0.4 rad, essentially the same as the grasp
table's 0.40553 rad stall angle for a 45mm object): my free-space sweep
(no object, gripper closing in open air) reads fingertip y-separation
**102.7mm**. `grasp_table.yaml`/`06`'s sweep (a REAL 45mm object, contact-
loaded, measured at the moment of genuine stall) reads **95.6mm** — 7.1mm
LESS. **The contact-loaded gripper closes further than free-space
kinematics predict at the same commanded angle.** More strikingly: the
free-space sweep's fingertip separation never drops below ~102.5mm
anywhere across the full tested range (0 to 0.767 rad) — it never reaches
95.6mm AT ALL, at any angle, in free space.

**Reading this**: dartsim doesn't rigidly enforce the mimic constraint
(already established — `gz_ros2_control` writes follower positions in
software, but genuine contact force CAN move them independently). Under
real contact load, the follower/pad geometry compresses further than
free-space software-mimicry alone predicts at the same master angle — a
real compliance effect this session's free-space-only sweep never had a
chance to see. **Every clearance number computed this session — the
original ~29mm miscalculation, the corrected ~1.4mm/~12.6mm-per-side
figures, all of it — was built entirely from the free-space sweep,
and is therefore probably still systematically optimistic.** The true
contact-loaded clearance at pre-close's aperture is unmeasured, but this
comparison gives a directional, concrete reason to expect it's smaller
than calculated — consistent with genuine contact starting almost as soon
as the final close begins, which is exactly the observed clustering.

**RETRACTED, same session, minutes later — the compliance mechanism above
is wrong. Full clean restart + re-verification disproved it directly, not
just left it unmeasured.** Challenged before building on it: does `05`'s
own historical free-space width and this session's ad-hoc sweep actually
measure the same thing? Read the code directly — `05`, `06`, and
`grasp_table.yaml` all compute width identically
(`norm(sub(left_pos, right_pos))` between the two fingertip LINK
origins, no pad-inset anywhere). Confirmed: not a units/definition
mismatch. But verifying this LIVE exposed something more important:
commanding the gripper to 0.4 rad in a genuinely empty scene (object
removed, confirmed via ground truth) **stalled instantly at ~0 rad** —
impossible in true free space, and not what this session's own earlier
sweep or `05`'s historical baseline ever showed.

**Did a full clean restart** (`kill_sim`, verified `gz_assert_clean_slate`
passes, fresh `ros2 launch` with `gazebo_gui:=true`, controller-activation
1.43s — healthy) and re-ran the identical free-space test: gripper closes
smoothly to 0.390 rad, `reached_goal: true`, no stall. Measured width:
**95.834mm — bit-identical to `05`'s historical 2026-08-04 baseline
(`docs/geom_run1_20260804_112018.log`: `0.4  0.095834`)**, and within
0.24mm of the contact-loaded `grasp_table` value (95.597mm at 0.40553
rad). **There is no free-space-vs-contact-loaded compliance gap.** The
102.7mm this session's own mid-session sweep reported, and the instant
spurious stall just observed, were both artifacts of **the long-running
sim instance having degraded** — not orphaned processes (checked, none
present both times), some other accumulated state (physics-engine
internal, not process-level) after several hours of continuous heavy use
across this session. A process census alone is not sufficient to
guarantee sim health, extending this project's own "system health is an
assertion" discipline — it needs a behavioral check (a known-answer
sanity test), not just a clean `ps` listing.

**This has a real consequence beyond just this one measurement**: if a
simple, well-established free-space reading degraded silently mid-session,
**the run-to-run variability observed in the 45mm-cube grasp trials this
same session** (stable 10s+ hold, catch-then-release, hold-with-zero-
disturbance — all logged as "genuine contact-dynamics sensitivity") needs
re-checking on this fresh instance before trusting that framing. Some or
all of that variability may have been sim degradation, not a real
property of the grasp near a marginal clearance boundary. Re-verifying
next, same session, while the fresh instance is up.

**Re-verification: the grasp behaviour is real, not a degradation
artifact — full clean restart, ran the same trial again.** Fresh sim,
fresh move_group, fresh object spawn, `preclose_margin_rad=0.30`
unchanged. Result: `TIMED_OUT_HELD` at `achieved=0.0914 rad`,
`tcp_error_m=0.0078` — matching the DEGRADED-sim `watch_test` run
(`0.0912`, `0.0078`) to within 0.0002 rad. **This specific behaviour
(stops near pre-close's own aperture) reproduces identically on a
verified-healthy sim.** It was never a degradation artifact — only the
free-space WIDTH measurement (and the compliance story built on it) was.

**Clean clearance recheck at the actual pre-close aperture, on the fresh
sim, using the correct (no ad-hoc inset) methodology.** Measured width
at exactly `0.096 rad` (pre-close's own achieved value): **127.781mm**.
Derived a proper pad-inset from the project's own contact data instead of
reusing the old inferred-not-measured 28mm figure:
`grasp_table`'s 95.597mm width at genuine 45mm-object contact implies
`(95.597-45)/2 = 25.3mm` inset per side — empirically grounded, not
guessed. Applied to the pre-close reading: true pad-to-pad gap ≈ 77.2mm,
**clearance ≈ 16.1mm per side — comfortable, not marginal.**

**Where this leaves the open question.** Both of this session's static
explanations for the 0.09-0.10 rad stopping point are now ruled out:
neither a free-space/contact-loaded compliance gap (disproven — the
fresh width measurement matches the historical baseline and the
contact-loaded reference almost exactly) nor insufficient static
clearance at the pre-close aperture (disproven — ~16mm is generous).
What remains standing, unaffected by any of this width-methodology
confusion because it never depended on it: the **trajectory capture**
from earlier this session, which used position-only ground truth (no
mesh math, no width formula) and directly showed the object MOVING during
the descent itself — a dynamic event, not a static clearance shortfall.
The most likely explanation is still that the object shifts during
descent (observed directly, multiple times, by amounts from ~5mm to
~17mm depending on the run) enough to close the nominally-comfortable
16mm margin at some point during the approach — not that the margin was
never there. Not yet directly confirmed (would need the trajectory
capture repeated with simultaneous fingertip-position tracking to see
the margin actually closing), but this is now the best-supported
remaining hypothesis, having survived two rounds of the other candidates
being checked and eliminated.

## A third run, watched live, reveals the object was never necessary at all

Written 2026-08-09, still the same session, watching live with the user.
Ran the identical trial a third time on the fresh sim: `TIMED_OUT_HELD`
at `achieved=0.0909 rad`, `tcp_error_m=0.0077` — the THIRD near-identical
reproduction (0.0914, 0.0912, 0.0909; 0.0078, 0.0078, 0.0077). Checked the
object afterward: **zero displacement**, exactly at spawn — the second
run (of three) with no disturbance at all, alongside two with visible
shifts. Two sub-patterns at the identical config, not one.

**That split motivated a cheaper, sharper test: does resistance require
the object at all?** Removed the object entirely, reopened the gripper,
commanded the SAME pre-close aperture (0.096 rad) with nothing anywhere
near the gripper. **The command hung — a plain `ros2 action send_goal`
with a 10s wrapper never returned.** Polled ground truth directly:
the master joint was genuinely, slowly creeping (0.048 -> 0.056 -> 0.071
rad over ~30s, not stuck at zero, not moving at normal speed either) —
eventually reaching 0.096 and settling cleanly (`velocity≈0`), but only
after roughly 60-90 SECONDS for a trivial, completely unobstructed
0.096 rad closure. Every other pre-close call this entire session
completed in under 1 second.

**Checked, not assumed: is the stall-threshold fix actually still
loaded?** `ros2 param get /gripper_controller stall_velocity_threshold`
-> `0.05`; `stall_timeout` -> `0.2`. Correctly loaded, not reverted. This
is NOT the fix regressing.

**This is a live, first-time-observed manifestation of a risk this
project flagged and never closed out**: when `stall_velocity_threshold`
was picked (`docs/HANDOFF_M3.md`, "stall_velocity_threshold fix applied
and validated"), the validation was explicitly against an
ALREADY-RESOLVED CONTACT STALL — "Active-motion noise floor still
unmeasured... a real approach could plausibly dip through 0.05 rad/s for
a stray 0.2s window before actual contact and false-trigger a premature
stall report." The measured noise floor itself (same doc,
`gripper_stall_velocity_noise_20260806.log`) has a "rarer spike
population (p99 0.245, max 0.264)" — i.e. even genuine rest occasionally
exceeds the 0.05 rad/s threshold by 1% of samples. A SLOW, GENTLE
free-space approach to a small target (not a fast open-to-closed sweep)
spends a long time in a low-velocity regime where this same noise can
repeatedly reset the stall timer's 0.2s quiet-window requirement, without
ever being fast enough to look like "real" motion by the same threshold —
plausibly explaining 60-90s of genuine, physical, if very slow, closing.

**Why this matters for everything read as `TIMED_OUT_HELD` today**: the
final close call in every grasp trial is bounded at 5.0s
(`gripper.command_timeout_s`). If the SAME noise-driven latency affects
that call — closing from ~0.096 toward 0.8 — then achieving only ~0.091
within a 5-second bound may mean "hadn't gotten far into a genuinely slow
approach yet," not "hit real resistance right at the pre-close boundary."
This is a DIFFERENT, more fundamental candidate explanation than either
the (already-ruled-out) compliance/clearance stories OR the
already-queued "extend the bound and watch" diagnostic — except that
diagnostic (run last session, `preclose_margin_rad=0.30`, 30s bound) saw
a dramatic joint SWING (0.096 -> 0.497 -> 0), not a slow creep, so it does
not cleanly fit this mechanism either. **Two different anomalous
behaviors have now been observed at the same configuration
(slow-creep-to-target in pure free space; dramatic swing-and-release
during an actual grasp attempt) — not yet reconciled into one story.**

**Not yet done, and higher priority than continuing to tune
`preclose_margin_rad`**: reproduce the free-space slow-creep with direct
velocity instrumentation (same method as the original noise-floor
measurement — a dedicated rclcpp subscriber, not `gz topic -e` polling
which is too coarse to see this) to confirm the noise-driven-reset
mechanism directly rather than infer it. Until this is understood, treat
EVERY `TIMED_OUT_HELD` result from today's session (and the
`preclose_margin_rad=0.30` value itself) as measured against clocks whose
meaning is now in question — not necessarily wrong, but not confidently
interpretable either.

## Direct velocity instrumentation: three distinct, reproducible behaviors for the identical command — not reconciled

Written 2026-08-09, still the same session. Built a dedicated `rclpy`
subscriber to `/joint_states` (same method as the project's original
2026-08-06 noise-floor measurement — direct subscription, not polling)
and ran the SAME command (0.096 rad, from a genuinely open/settled 0.0
rad, nothing anywhere near the gripper) repeatedly, in isolation.

**Result 1 — 5/5 standalone trials, all identical: clean, fast FAILURE.**
Every one of 5 repeated trials returned `stalled: true` at
`position≈0` (literally no measurable movement — the velocity trace
confirms exactly zero velocity for the entire capture window, not just a
small drift) in 1.8-4.4 seconds. Not the 60-90s creep from the section
above. Not a slow approach. Genuinely never started moving, and the
controller correctly (from its own perspective) declared a stall because
velocity truly never exceeded the noise floor.

**This directly contradicts what happens inside every actual grasp
trial**, where this SAME command (pre-close, target 0.096-0.1055 rad,
also starting from a genuinely open state) has succeeded with
`REACHED_GOAL` in every single logged run today (`traj_run3`,
`watch_test`, `fresh_verify`, `watch2` — 4/4), taking 3.6-6.0s each time
— slower than instant, but always genuine, measurable progress to the
target, never a stall.

**A third pattern, also reproduced**: one further isolated trial (this
time preceded by 20 rapid `gz topic` queries, testing whether recent sim
activity changes the outcome) hung well past 95 seconds with the joint
still reading essentially exactly 0 position and 0 velocity — worse than
the original 60-90s creep (that one at least showed measurable
intermediate progress: 0.048, 0.056, 0.071 rad at successive checks; this
one showed none at all within the observed window).

**Three reproducible outcomes for the identical command, and no theory
tested so far reconciles all three**: clean fast success (inside grasp
trials), clean fast failure (standalone, 5/5), and slow-or-absent creep
(standalone, 2 occurrences). Considered and rejected as a full
explanation: "recent activity helps initiate motion" — the FINAL close
call inside a grasp trial (the one that actually shows the problematic
`TIMED_OUT_HELD`-near-0.09 pattern) is issued immediately after (a)
pre-close's own motion, (b) pre-close's own explicit hold-to-stationary
command, and (c) the arm's ~2s Cartesian descent — i.e. immediately after
recent activity — yet IT is the unreliable one, while pre-close (also
preceded by recent activity, the arm's joint-space move to pre-grasp) is
consistently reliable. Recent activity alone does not cleanly predict
which calls succeed.

**Not resolved tonight — and NOT a friction/mechanism finding, corrected
before it went in as one.** Challenged directly: exactly-zero position
AND exactly-zero velocity, held for up to 95 seconds, does not look like
stiction — genuine friction/creep produces small nonzero wander (like the
60-90s case's measurable 0.048/0.056/0.071 rad progression), not a value
identical to the noise floor for a minute and a half. And the split is by
CALLER, not by physical condition: standalone CLI 5/5 failure, in-node
C++ client 4/4 success, same joint, same sim, same command in substance —
physics can't distinguish which process sent the goal, so something about
the software CONTEXT differs, not the mechanism.

**Three cheap, direct checks, all done live, all ruled out — not left as
untested candidates:**
1. **Is the controller active at the moment of the call?** `ros2 control
   list_controllers` immediately before sending: `gripper_controller
   ... active`. Confirmed clean.
2. **Is a competing client interfering?** `ros2 action info
   /gripper_controller/gripper_cmd` showed `moveit_simple_controller_manager`
   registered. Killed `move_group` entirely (verified: no `moveit`/
   `move_group` process survives, no orphans). Re-ran the identical
   standalone test: **still fails identically** (`stalled: true` at
   `position≈0`, `2.34s`). Not MoveIt. (Note: the client listing itself
   turned out to be STALE — it still showed `moveit_simple_controller_manager`
   even after that process was confirmed dead, meaning `ros2 action info`'s
   client list reflects DDS discovery history, not necessarily live
   connections — a caveat for trusting that command's output at face value
   in future checks.)
3. **Is `/joint_states` stale?** Two samples ~2s apart, header timestamps
   advanced by ~2.7s (sim time tracking wall-clock normally). Not a frozen
   read.

**Genuinely unexplained, not "previously-unknown mechanism property."**
That framing was too easy to reach and too hard to falsify — retracted.
What's actually established: the failure is real, reproducible (6+ times
tonight), caller-dependent (CLI vs. in-node client), and NOT explained by
controller state, a competing client, or topic staleness. What's not yet
checked: whether the CLI tool's own action-client implementation has some
timing/subscription difference from a persistent `rclcpp_action::Client`
in a spinning node (e.g. a race between goal acceptance and the
realtime control loop's next cycle actually picking up the new command
before the stall-clock starts) — this is the most likely remaining
candidate but is NOT yet confirmed, just not yet ruled out. Next session
should start here, not with a multi-trial physical experiment design.

**Consequence, not yet resolved**: there may be a genuine trade-off here,
not a simple monotonic "shorter is better." A taller object gave the pads
more downward travel budget before their own closing-descent reached the
top face (previously 9-11mm of headroom AT FULL OPEN, i.e. pads already
past the top before closing even started) but put the KNUCKLES in
contact instead. A shorter object clears the knuckles but leaves LESS
headroom for the pads' own descent. **Not yet checked**: whether some
intermediate height, or a small adjustment to `pad_centre_offset` /
`tcp_offset` (targeting slightly ABOVE the object's exact mid-height,
trading some pad-centre accuracy for descent headroom), resolves both
constraints simultaneously — this needs the SAME kind of rigorous,
measured treatment the knuckle question got, not a guess. Object geometry
choice for M3 is still open, not settled by this session.

**Net assessment**: the missing-table hypothesis from 2026-08-08 is
confirmed as the dominant cause of Test 2's original three anomalies — not
speculatively, by prediction-then-measurement. M3 has its first passing
grasp trial. What's left before trusting `grasp_tolerance_rad` or
`pad_centre_offset` as tuned: more trials (this is n=1, and the same
"suspiciously good number gets a falsification test" discipline that
caught the shortfall-vs-achieved-angle and baseline-artifact mistakes
earlier in this project applies here too), and a look at whether the 32mm
z-shortfall is systematic (repeats every trial) or this trial's noise.

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
5. Contact resolution timing out past the 5s bound even on a verified-clean
   system — **contact settling genuinely takes longer than 5s in this
   scenario**, not contamination (see "06 retried" above). Related but
   distinct and more actionable: once a grasp *does* resolve to
   `stalled: true`, the controller does not hold that position — the raw
   position command keeps driving toward full closure and can eject the
   object minutes later. **control/plumbing**, same category as #4, with a
   known fix shape (hold-position after stall).

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
- System health is an assertion, not a discovery made after a 15-minute
  hang. Controller-activation time is now a known-good signal for this
  stack (6.8–13.1s healthy, 40+s means contamination, not "just slow") —
  `gz_assert_clean_slate` and `gz_wait_controller_active_bounded`
  (`scripts/lib/gz_settle.sh`) turn that into a standard preamble every
  sim-launching script runs, converting "unexplained 15-minute hang" into
  "refusing to measure on a contaminated system." Same principle as the
  settle-condition gating and precondition assertions above, applied one
  level up: to the sim process tree itself, not just to individual joint
  readings. A finding measured without this check (e.g. the 0/5-open result
  above, taken before these existed) needs its timing retroactively checked
  against the healthy/degraded bands before being trusted at face value —
  don't assume a result is a property of the robot when it might be a
  property of an uncontrolled system state.
