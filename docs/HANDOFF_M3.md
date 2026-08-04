# Handoff — M3, Blocker 1 validation in progress

Written 2026-08-04, end of session. M-1, M0, M1, M2 are closed and committed.
Blocker 1's fix is written and committed but NOT yet validated against a live
sim. Blocker 2 is open and its resolution now depends on Blocker 1's step 2.

## State

| Milestone | Status | Evidence |
|---|---|---|
| M-1 | closed | merged platform validated + spawns; docs/M-1_reference_report.md |
| M0 | PASS A/B/C | docs/m0_*.log; M0-C reproduced 3x across 2 code paths |
| M1 | PASS | 20/20 planning, executed; docs/evidence/m1_planning.csv |
| M2 | PASS | cartesian_fraction=1.0000 tcp_error_m=0.0000 ground_truth=yes |

Robot base is at z=0.75 (table height), derived from `robot.base_pose` in
`config/scene.yaml` via `config/scene_xacro_args.py`, which all three launch
files (`ur5e_robotiq_sim_control.launch.py`, `move_group.launch.py`,
`m2_cartesian_approach.launch.py`) import. The M2 node has a runtime guard
comparing Gazebo's actual base_link pose against scene.yaml and aborting with
CONFIG_ERROR on mismatch.

Committed this session:
- `1dd32bb` — M2: base elevation propagated through scene_xacro_args.py to
  both the sim spawn launch and move_group's robot_description mapping
  (previously only the spawn launch read base_xyz/base_rpy — move_group's
  kinematic model silently disagreed with Gazebo's about base_link).
  Adds m2_cartesian_approach, static_scene_tf, moveit_compat.hpp.
- `0f2ac4f` — Blocker 1 fix: scripts/lib/gz_settle.{py,sh}, wired into
  05_measure_gripper_geometry.sh, 04_mimic_contact_probe.sh, m0_verify.sh.

**No git remote is configured.** Eight commits of measured findings exist on
exactly one machine. `git remote add origin <url> && git push -u origin main`
is still outstanding — do it early, before generating anything expensive.

## Where we are right now: Blocker 1 validation, step 1

Blocker 1 was: fixed sleeps before ground-truth samples produce race
conditions. 05_measure_gripper_geometry.sh's sweep once sampled fingertip
pose mid-settle and got 0.1218 at the 0.2 rad point against a true ~0.1158 —
6mm of noise, caught only because it looked wrong and was hand-retested. M3
runs 20 unattended cycles; nobody retests individual samples there.

**Fix (committed, unvalidated):** `scripts/lib/gz_settle.py` polls Gazebo's
own `joint_state`/`pose/info` topics (never `/joint_states`) until velocity
or successive pose deltas stay under threshold for two consecutive polls,
guarding against a sample landing on a velocity zero-crossing. Timeouts fail
loudly — nonzero exit, `[STOP]` on stderr — instead of sampling a moving
target. Wired in with severity matched to what each script is for:
- `05_measure_gripper_geometry.sh` — aborts the sweep on timeout.
- `04_mimic_contact_probe.sh` — warns loudly but keeps sampling on the
  post-overclose settle (this script is deliberately no-judgment).
- `m0_verify.sh` M0-C — the two settle points feeding the C2 pass/fail
  criteria (pre-close placement, post-overclose) set `C_FAIL=1` on timeout;
  the subordinate C1 free-space sample only warns.

**Validated so far:** syntax-checked, and the timeout/dead-topic path
verified against the real `gz` binary (no sim needed for that — `gz topic -e`
was found to block indefinitely against a topic with no publisher; bounded
it, confirmed clean `[STOP]` + exit 1, no uncaught exception). **Not yet run
against a live sim at all.**

### Step 1 (next action): regression test on the known failure

Three fixed runs of `scripts/05_measure_gripper_geometry.sh` against a live
sim. Watch the 0.2 rad sample for variance collapse toward ~0.1158.

**Protocol — decided and non-negotiable, do not relitigate mid-session:**
no retry on `[STOP]`. Run exactly three attempts. Report successes and
timeouts *separately*. Timeout rate is a headline result, not a nuisance to
re-roll past.

**Why no auto-retry:** this run's whole purpose is measuring variance. If a
timeout triggers a silent re-run, you're conditioning on success and
discarding exactly the outcomes that would show the fix didn't work — the
test becomes unfalsifiable. Worse, a timeout has two structurally different
causes that need different responses, and auto-retry collapses them into
"flaky, bump the timeout," silently defaulting to the first and hiding the
second:
1. **Budget too tight** — the gripper settles, just slower than allowed.
   Tune the budget, knowingly, and note the old fixed sleeps were marginal.
2. **It doesn't settle** — oscillation or a limit cycle, not convergence.
   This is a physics finding: if the gripper never reaches steady state
   after closing, every M3 slip number is sampled off a moving target and
   the 5mm criterion is meaningless regardless of friction. §3.5 already
   flags `right_knuckle` diverging ~0.027 rad from its mimic multiplier
   under contact load only (tracks exactly in free space) — a
   kinematic-override linkage hunting under load instead of settling is not
   a hypothetical here.

If any timeout appears in the three runs, that is the finding —
investigate before touching the budget. This "record and report, never
auto-retry past a failure" rule applies to any future settle/validation
polling in this project, not just this one script.

Bring the stack up first, in its own terminal:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ur5e_ws/install/setup.bash
ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py
```

No extra args — the base should land at z=0.75 on its own now, from
scene.yaml through scene_xacro_args.py. If it spawns at z=0, STOP: the
single-source wiring regressed and nothing downstream is trustworthy.

### Step 2 (immediately after step 1, same session): Blocker 2 may dissolve

Blocker 2 was: 12–14mm of downward object movement during gripper closure,
measured three times (13.7, 12.8, 12.5mm), against M3's 5mm criterion — with
the closure-settle box-pose sample taken on the same class of fixed-sleep
timing that just produced the 6mm error in step 1.

Re-run `scripts/04_mimic_contact_probe.sh` with settle gating now in place.
If box displacement drops under 5mm, Blocker 2 is closed without a GUI
session — which is why this step comes before step 3, not after.

### Step 3 (only if step 2 still shows >5mm)

One GUI closure run with the camera on the fingertips, to distinguish the
object being seated by the pads from it sliding between them. This was the
original plan for Blocker 2 before step 1's regression test suggested part
of the 12–14mm might be measurement timing rather than physics.

### Also worth doing, cheap, while runs are in progress

Log the observed settle time itself (already free — every successful
`gz_settle_*` call prints `[settle] ... settled after Xs (...)` to stderr,
which lands in whatever `2>&1 | tee docs/....log` the script is invoked
with). After the runs: `grep '\[settle\]' docs/<run>.log` to compare
observed settle durations against the old fixed sleeps (1.5s, 1.0s, 2.0s,
3.0s this project used before today). If settling routinely took longer than
those old sleeps, every historical measurement taken under them — the
tcp_offset curve, the M0-C numbers, the 363mm ejection figure — carries the
same race and may need a second look. Probably fine, since most of those
reproduced across runs, but cheap to check. Skip building a summarizer for
this — three runs × ~10 settle calls is ~30 lines, grep handles it. Revisit
if tomorrow's data is messier than expected.

## Other M3 prerequisites (unchanged from prior session)

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
baked in.

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
  oppose it: shortfall 0.342 rad on a 40mm box, controller reports
  `stalled: true`. See §3.5. Do NOT switch to bullet-featherstone on the
  strength of the mimic feature alone — its ros2_control integration has
  open defects (gz_ros2_control#440, gz-sim#2729).
- Ejection is not a physics-timestep artifact — halving max_step_size
  changed nothing (363 vs 360.1mm). So M3's slip measurement is not
  timestep-confounded. Rate limiting is the only lever.
- `right_knuckle` diverges ~0.027 rad from its multiplier under load only;
  tracks exactly in free space. Open, low-priority — first place to look if
  grip stability is asymmetric between fingers, and also the concrete reason
  Blocker 1's step 1 protocol refuses to auto-retry past a settle timeout.
- Vertical-tool0 reach ceiling for a ground-mounted UR5e is ~0.85-0.90m.
  This is why the base is elevated (M2, closed).
- `tcp_offset` varies ~13.6mm across the aperture range — it is NOT
  constant. Current value is a scalar measured at one aperture; changing
  `object.size` invalidates it.

## Working methods that earned their keep

- Evidence comes from Gazebo's own state topics, never `/joint_states` and
  never TF. Both report what something believes, which is the thing under
  test.
- Read the generated artifact, not the wizard's summary. Every config bug
  this project has hit came from MSA-generated files that looked correct on
  screen: SRDF end effector, joint limits, action_ns, use_sim_time.
- A suspiciously perfect number gets a falsification test. `tcp_error_m
  =0.0000` was confirmed real by injecting a synthetic 10mm offset into only
  the logged commanded pose and checking it reported 0.0100.
- Claims get partitioned PROVEN / INFERRED / UNKNOWN. Twice a correct
  observation produced a wrong downstream conclusion, and the partition is
  what caught it.
- Never auto-retry a failed measurement/settle/validation step. Record the
  failure, report the failure rate as data alongside whatever variance is
  being measured, and investigate before touching any budget or threshold.
  See Blocker 1 step 1 above for the concrete reasoning.
