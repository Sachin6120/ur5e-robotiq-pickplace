# UR5e + Robotiq 2F-85 Pick-and-Place (MoveIt 2 / Gazebo Harmonic)

A ROS 2 Jazzy + Gazebo Harmonic pick-and-place simulation: a UR5e with a
Robotiq 2F-85 gripper picks a known object from a fixed pose on a table and
places it at a second fixed pose. All motion goes through MoveIt 2 —
no custom IK, no MoveIt Task Constructor, no perception. That scope is
deliberate, not deferred: see `UR5E_PROJECT_START_PROMPT.md`, the spec this
whole project was built against and never departed from.

**Status: v1 complete.** Milestones M-1 through M5 are all closed. The full
loop — approach, friction grasp, lift, transport, place, release, retreat —
runs end to end, with every claim in this document backed by a Gazebo
ground-truth measurement, not a status message. See "Known limitations"
below for what "complete" does and doesn't mean here.

## What this is not

- **No perception.** Object pose is read from `config/scene.yaml`, not a
  camera. Out of scope from the spec's first paragraph.
- **No MoveIt Task Constructor.** Deferred deliberately (see
  `docs/M-1_reference_report.md` §2) so it never enters the tree unexamined.
- **No custom grasp planner.** One object, one pick pose, one place pose,
  by design — this is a pick-and-place demonstration, not a grasp-planning
  system.

## Stack

- ROS 2 Jazzy Jalisco, Ubuntu 24.04 Noble
- Gazebo Harmonic (`dartsim` physics — see "Known limitations", item 1)
- UR5e via `Universal_Robots_ROS2_GZ_Simulation` / `..._Description`
- Robotiq 2F-85 via `ros-jazzy-robotiq-description`, merged in (M-1)
- MoveIt 2, two planning groups (`arm`, `gripper`), config built fresh via
  Setup Assistant against the merged URDF

## Repo layout

```
config/scene.yaml            single source of truth: object pose, place pose,
                              object dimensions, gripper/grasp parameters —
                              read by both the Gazebo world spawn and the TF
                              publisher, so they cannot silently disagree
ur5e_robotiq_description/    merged URDF/xacro, sim launch, ros2_control config
ur5e_robotiq_moveit_config/  MoveIt config (arm + gripper planning groups)
ur5e_pick_place/             application code: m3_grasp.cpp (the pick-place
                              node), transport.cpp (lift/transport/place/
                              release/retreat), failure.hpp (typed failure enum)
scripts/                     numbered setup/verification scripts (00-11) plus
                              scripts/lib/ (gz_settle, sample_pose, slip —
                              the shared ground-truth measurement tooling)
docs/HANDOFF_M3.md           the full session-by-session narrative: every
                              measurement, every dead end, every fix, in the
                              order it happened. This README is the map;
                              that file is the territory.
UR5E_PROJECT_START_PROMPT.md the original spec (repo root) — milestone
                              definitions and pass criteria quoted below
                              come from here
runs/                        milestone-level CSV evidence (M1 planning sweep,
                              M3/M5 20-cycle sweeps)
```

**Workspace note:** this repo's three ROS packages are symlinked into
`~/ur5e_ws/src/` (not copied — a copy silently desynced from the repo once
already, see `docs/HANDOFF_M3.md`'s "repo/workspace desync" entry). Build
with `colcon build --symlink-install` from `~/ur5e_ws`.

## Running it

Both commands below must be run **from the repo root** — `--trial-cmd` and
`--out` are relative paths, and `scripts/11_m3_cycles.sh` resolves them
against the current working directory, not the script's own location.
Nothing needs to be pre-sourced in your shell first: `docs/m3_run_full_cycle_trial_live.sh`
(what `--trial-cmd` below invokes) sources `/opt/ros/jazzy/setup.bash` and
`~/ur5e_ws/install/setup.bash` itself, as its first action, and
`scripts/11_m3_cycles.sh` itself makes no `ros2`/`gz` calls directly — only
`python3` and the `gz` CLI need to already be on `PATH`. `--trial-cmd` is a
required argument, not defaulted — the harness has no built-in notion of
what one cycle is; it only knows how to watch for stage markers and retry
on a pre-grasp gate failure.

**One full cycle, annotated:**

```bash
cd ~/ur5e_pickplace   # repo root — see note above
bash scripts/11_m3_cycles.sh --cycles 1 \
  --trial-cmd "bash docs/m3_run_full_cycle_trial_live.sh" \
  --out runs/single_cycle_$(date +%Y%m%d_%H%M%S).csv
```

This launches the sim, move_group, spawns the object, and runs the full
pick → lift → transport → place → release → retreat sequence, streaming
`m3_grasp`'s own stage markers (`M3 STAGE 3 LIFT_BEGIN`, …) live. (Run
through the harness even for one cycle — the trial script needs
`M3_MARKER_PREFIX` set in its environment, which the harness supplies; see
`docs/HANDOFF_M3.md`'s M4 entry for what happens if you skip this.)

**20-cycle repeatability sweep (M5's own check):**

```bash
cd ~/ur5e_pickplace   # repo root — see note above
bash scripts/11_m3_cycles.sh --cycles 20 \
  --trial-cmd "bash docs/m3_run_full_cycle_trial_live.sh" \
  --out runs/m3_cycles_$(date +%Y%m%d_%H%M%S).csv
```

Produces one CSV row per cycle: slip (Gazebo ground truth, relative to the
flange), ejection flag, gripper result, achieved grip angle. Never silently
retries a measurement — see `scripts/11_m3_cycles.sh`'s own header for the
retry policy (pre-grasp gate failures are retried because they can't have
selected on an outcome; anything after the grasp starts counts as-is).

**Not verified against a literal clean shell this session** — the claims
above are read from the scripts' own sourcing/path-resolution logic, not
confirmed by tearing down every piece of accumulated session state
(workspace symlinks, sourced profiles, running processes) and starting
over. Worth a real cold-shell run before trusting this section fully;
flagged rather than silently assumed.

Every run logs its grasp mode at startup — `GRASP MODE: friction (physics)`
— per the spec's requirement that this never be silently ambiguous.

## Milestones

Criteria are quoted from `UR5E_PROJECT_START_PROMPT.md`, not restated from
memory. Full evidence and the reasoning behind each result is in
`docs/HANDOFF_M3.md`; this table is the index.

| Milestone | Criterion | Result | Evidence |
|---|---|---|---|
| M-1 | combined URDF+MoveIt config assembled, spawns cleanly | closed | `docs/M-1_reference_report.md` |
| M0 | stack verification A/B/C — pass/fail note with log lines | PASS | `docs/m0_20260808_143404.log` |
| M1 | MoveIt executes a joint goal — 20/20 planning success, logged | PASS | `docs/evidence/m1_planning.csv` |
| M2 | TF-derived pre-grasp/grasp reached, no gripper — TCP pose vs commanded, ground truth | PASS — `tcp_error_m=0.0000` | `docs/HANDOFF_M3.md` M2 row |
| M3 | friction grasp tuning — 20 cycles, ≥18 with slip <5mm, zero ejection/penetration | **PASS** — 20/20, slip 0.227–0.442mm | `runs/m3_cycles_retry20_20260812_034544.csv` |
| M4 | full loop incl. place and retreat — one annotated run log | **PASS** — placement measured, 0.162mm from commanded place pose | `docs/m3_cyclelive_grasp_20260812_113952_14404.log`, `docs/m4_placement_20260812_113952_14404.txt` |
| M5 | repeatability — 20 cycles, CSV | **PASS** — same sweep as M3 | `runs/m3_cycles_retry20_20260812_034544.csv` |

**M3, M4, and M5 are not three independently-earned checkmarks.** M3 and M5
are the same 20-cycle sweep read two ways; M4 is one additional
ground-truth measurement (object placement) taken on top of the same
stack. `docs/HANDOFF_M3.md`'s M4/M5 entries say this explicitly — stated
here too because a table like the one above is exactly the format that
would otherwise imply three separate proofs.

## Known limitations

Not a TODO list — these are the actual boundary of what this v1
demonstrates, each one a finding earned through live measurement, not a
gap left by lack of time.

**1. Grasp rigidity is a modelling choice, not a physics constraint.**
Gazebo Harmonic's default engine, `dartsim`, does not implement mimic
joint constraints at all — confirmed at every spawn (`[Err] ... the chosen
physics engine does not support mimic constraints, so no constraint will
be created`) and corroborated by the open upstream issue, `gz-physics#432`.
The gripper's five follower joints track the actuated master
(`robotiq_85_left_knuckle_joint`) because `gz_ros2_control` writes each
follower's velocity command from the mimic formula every control cycle —
a software override standing in for a linkage the physics engine doesn't
enforce. In free space this tracks to float precision. Under contact load
it can be perturbed or overridden by the physics solver (see item 2). Every
"the fingers closed rigidly" claim in this project rests on that software
override holding, not on a real four-bar mechanism.

**2. The fingertip degrees of freedom were removed, not fixed.** Both
fingertip joints were, for most of this project, `continuous` joints
mimicking the master. Under load, one of them (confirmed via a
purpose-built `gz-sim` plugin reading `JointVelocityCmd` directly from the
simulator's own entity-component data, not inferred from `/joint_states`)
received a correctly-computed command and moved in the *opposite* sign
anyway — 97.5% of samples during a runaway window had commanded and
measured velocity of opposite sign, sustained, not a transient. That is a
`dartsim` contact-resolution behavior this project measured precisely and
did not explain at the engine-internals level. The fix in place —
`robotiq_2f_85_macro.urdf.xacro`'s "TENTH OVERRIDE" — changes both
fingertip joints from `continuous`+mimic to `fixed`, at an angle
(`fingertip_grasp_theta`) chosen so the pad sits parallel *at this
project's object width*. It removes the degree of freedom the mechanism
was acting on rather than resolving the mechanism itself, and that fixed
angle is derived per object width by `scripts/lib/gripper_geometry.py`'s
`theta_for_width()` — a different object size needs a re-derived angle,
not a reused constant.

**3. A freshly-launched sim occasionally fails its own preflight for an
unknown reason.** `gz_assert_gripper_responsive`, run before every cycle,
occasionally reports the gripper unresponsive on a sim instance with no
prior heavy use, no orphaned processes, and normal (7–12s) controller
activation timing — ruling out every previously-identified degradation
cause. One 20-cycle sweep hit this on 3/20 attempts; the very next hit it
on 0/20. `scripts/11_m3_cycles.sh`'s retry harness treats this class of
failure as a precondition failure and retries it — principled, since it
fires before the grasp attempt and so can't be selecting on an outcome —
but that is a policy for surviving the failure, not a diagnosis of it.
Anyone running this project's tooling at scale should expect an occasional
fresh launch to fail its own health check and get silently retried by the
harness; the underlying cause is still open.
