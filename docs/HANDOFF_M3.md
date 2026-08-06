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
| M0 | PASS A/B/C | docs/m0_*.log; M0-C reproduced 3x across 2 code paths |
| M1 | PASS | 20/20 planning, executed; docs/evidence/m1_planning.csv |
| M2 | PASS | cartesian_fraction=1.0000 tcp_error_m=0.0000 ground_truth=yes |
| Blocker 1 | closed | docs/geom_run*.log — bit-identical across 3 runs, 0 timeouts |
| Blocker 2 | closed (geometric seating) | docs/probe_zsweep_*.log — see below |
| Spawn-state/reliability | closed (4 bugs found + fixed) | docs/spawn_state_check_*.log — see "Spawn-state investigation, closed" |
| Grasp-table sweep (06) | 0 OK rows so far; ejection bug fixed; anchor re-checked and reproduces (units error, not drift) — clear to run full sweep | docs/grasp_table_20260806_210710.log, docs/grasp_stall_diagnostic_20260806.log — see "06 retried" and "Single-width validation" below |

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
