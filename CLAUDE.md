# CLAUDE.md

Guidance for Claude Code working in this repository.

## What this is

A ROS 2 Jazzy + Gazebo Harmonic pick-and-place simulation: a UR5e with a
Robotiq 2F-85 gripper picks a known object from a fixed pose and places it
at a second fixed pose, entirely through MoveIt 2. Read `README.md` first —
it is the current, correct entry point (status, milestone results, known
limitations). `UR5E_PROJECT_START_PROMPT.md` is the original spec; the
scope boundaries below come from there and are load-bearing, not
incidental:

- No perception. Object pose comes from `config/scene.yaml`, not a camera.
- No MoveIt Task Constructor.
- No custom, hand-written inverse kinematics. All motion goes through
  MoveIt 2.

For the full session-by-session history of dead ends, measurements, and
fixes, see `docs/HANDOFF_M3.md`. Prefer it over guessing when a constant's
provenance matters.

## Secrets and sensitive data

Never commit secrets, API keys, passwords, tokens, private credentials, or
sensitive personal data. This applies regardless of whether the repository
or a given branch is assumed to be private — a private remote is not a
substitute for keeping credentials out of version control, since
visibility and history can change later. If a file that might carry
credentials is about to be staged (`.env` files, service-account JSON,
anything with "key," "token," or "secret" in the name), stop and check its
contents before committing, even if it was already untracked or covered by
`.gitignore` elsewhere in the tree.

## Stack

- ROS 2 Jazzy Jalisco, Ubuntu 24.04 Noble (WSL2-friendly; see
  `docs/SETUP_WSL.md`)
- Gazebo Harmonic, `dartsim` physics engine
- MoveIt 2, two planning groups: `arm`, `gripper`
- This repo's three packages are symlinked into `~/ur5e_ws/src/`, not
  copied. A copy has silently desynced from the repo before. Build with
  `colcon build --symlink-install` from `~/ur5e_ws`.

## Packages

- `ur5e_robotiq_description/` — merged UR5e + Robotiq 2F-85 URDF/xacro,
  the `gz_ros2_control` sim launch, and `config/controllers.yaml`. The
  vendored `urdf/vendor/robotiq_2f_85_macro.urdf.xacro` carries several
  numbered "OVERRIDE" patches (search for `OVERRIDE` in that file) — each
  one exists because of a specific measured failure; don't revert one
  without reading why it's there.
- `ur5e_robotiq_moveit_config/` — Setup-Assistant-generated MoveIt config
  (SRDF, kinematics, controllers, joint limits). Treat these files as
  generated, not as application code: a bug in `ur5e_pick_place/`'s
  application logic should be fixed there, not worked around by editing
  generated MoveIt config. Edit files in this package only when the change
  is actually about MoveIt configuration itself — e.g. planning groups,
  kinematics solver settings, controller wiring, or joint limits.
- `ur5e_pick_place/` — the application nodes, one per milestone:
  `m1_joint_goal.cpp` (planning reliability), `m2_cartesian_approach.cpp`
  plus `static_scene_tf.cpp` (TF-derived approach), and `m3_grasp.cpp`
  plus `transport.cpp` (grasp, lift, transport, place, release, retreat —
  the full loop). `include/ur5e_pick_place/failure.hpp` defines a single,
  deliberately exhaustive `Result` enum shared by every node — adding a
  failure mode without a matching `to_string()` case is a compiler
  warning, not a silent gap. Preserve that property when touching it.

## Single source of truth: config/scene.yaml

Object pose, table pose, place pose, object dimensions, grasp geometry
(`tcp_offset`, `pad_centre_offset`, standoff/retreat distances), gripper
squeeze/stall parameters, and thresholds all live in `config/scene.yaml`.
It is read by the Gazebo world spawner, the static TF publisher, and the
MoveIt planning-scene loader, so those three cannot silently disagree.

Never hardcode a pose, offset, or threshold in a `.cpp` or `.launch.py`
file. If a coordinate or tuning constant is needed, it belongs in
`scene.yaml` (or `config/grasp_table.yaml` for width-indexed grip data),
read once and passed down as parameters.

Every non-obvious constant in `scene.yaml` is documented in place with how
it was measured and its confidence level. When editing a value, update the
comment with the same rigor: what was measured, how, and what's still
unverified. Look for markers like `NOT YET LIVE-VERIFIED`, `SUPERSEDED`,
and `REASONED, NOT DERIVED` before trusting a number at face value — they
mean exactly what they say.

## Ground truth over self-reported status

This project has a hard-won rule: don't trust a component's own
success/failure report; measure the physical outcome in Gazebo directly.
Examples already in the codebase:

- The gripper controller's `stalled:true` does not mean the fingers are
  holding position — grasp success is verified by ground-truth slip
  (`scripts/lib/slip.py`), not by the action result. See
  `docs/HANDOFF_M3.md` for the history behind this.
- Object "settled" checks must use a windowed quiescence sampler
  (`scripts/lib/gz_settle.py`'s `gz_settle_pose_windowed`), not a single
  instantaneous sample — a naive check previously declared a
  still-sinking box "settled" at 0.58s.
- `scripts/lib/gripper_geometry.py` measures pad geometry from the actual
  mesh the URDF's `<collision>` points at, not from a hand-derived
  estimate.

Apply the same standard to new work: if you add a check, prefer reading
Gazebo state (`gz topic`, `gz service`, or a purpose-built plugin that
reads simulator state directly) over trusting a ROS action's own status
field.

## Never silently retry a failed measurement

If a settle check, pose verification, or timing measurement fails or
times out, that is a result to record and investigate, not something to
quietly re-run until it passes. The one sanctioned exception is
`scripts/11_m3_cycles.sh`'s retry of pre-grasp gate failures specifically,
because that gate fires before the grasp attempt and so cannot be
selecting on outcome; that reasoning is documented in the script's own
header and should not be generalized without the same justification.

## Sample "before" and "after" values in one script, not across turns

When measuring a before/after quantity — for example, object pose before
versus after closing the gripper — do it in a single script or shell
invocation. Splitting it across two separate tool calls introduces a real
wall-clock gap that a slow process, such as a settling object, can drift
through invisibly. This has already produced a false measurement once in
this project's history.

## Build & run

Build:

```bash
cd ~/ur5e_ws
# Under WSL2's default memory cap, a full parallel build can get OOM-killed.
# Add --parallel-workers 2 to colcon build if that happens, or raise the
# cap in .wslconfig (see docs/SETUP_WSL.md).
colcon build --symlink-install
source install/setup.bash
```

Typical manual flow, with the sim and move_group each in their own
terminal, per the milestone launch files' own headers:

```bash
# terminal 1
ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py
# terminal 2
ros2 launch ur5e_robotiq_moveit_config move_group.launch.py
# terminal 3, e.g. M3
ros2 launch ur5e_pick_place m3_grasp.launch.py
```

Full-cycle trial harness (must be run from the repo root — `--trial-cmd`
and `--out` resolve relative to the current working directory, not the
script's own location):

```bash
cd ~/ur5e_pickplace
bash scripts/11_m3_cycles.sh --cycles 1 \
  --trial-cmd "bash docs/m3_run_full_cycle_trial_live.sh" \
  --out runs/single_cycle_$(date +%Y%m%d_%H%M%S).csv
```

Nothing needs sourcing first — the trial script sources ROS and workspace
setup itself. Only `python3` and the `gz` CLI need to already be on
`PATH`. See `README.md`'s "Running it" section for the annotated 20-cycle
repeatability sweep and what each CSV column means.

Run WSL2 headless (`gz sim -s <world>`) for data collection; use the GUI
only for spot checks and screenshots — see `docs/SETUP_WSL.md` for why and
for GPU/rendering troubleshooting.

## Known environment quirks worth remembering

- Sim degrades over long sessions. Roughly 30-40 minutes of heavy use has
  flipped a 4/4-success configuration to 21/21 failure with no change
  visible in a process census. Budget mid-session sim restarts for long
  sweeps; a `probe_gripper_cmd` run is a cheap behavioral sanity check.
- A fresh sim occasionally fails its own preflight gate
  (`gz_assert_gripper_responsive`) for no yet-known reason, unrelated to
  prior heavy use. Currently retried by the harness as a precondition
  failure, not treated as diagnosed.
- `dartsim` does not implement mimic joint constraints. Follower joints
  are tracked in software by `gz_ros2_control` every control cycle, not
  enforced by the physics engine. This is a real limitation, not a bug to
  fix, and several URDF overrides exist because of it.

## Repo hygiene

The repo root and `docs/` regularly accumulate ad-hoc trial scripts and
CSV trace dumps from interactive debugging sessions — check `git status`;
there are usually untracked `docs/m3_*` scripts and stray root-level
`*.csv` files at any given time. `.gitignore` already excludes `runs/`,
`docs/*.log`, `docs/*.csv`, `docs/*.txt`, and `m3_grasp_*.csv`, but not
every generated file matches those patterns. Before committing, check
`git status` for stragglers rather than assuming `.gitignore` caught
everything. Don't add generated trial output to version control, and ask
before deleting untracked files that might represent in-progress
debugging work rather than pure scratch output.

## Milestone status

M-1 through M-5 are marked complete in `README.md`'s milestone table
(v1 complete). If asked to work on "the next milestone," there isn't one
defined yet — check with the user rather than assuming scope. Reread
`README.md`'s "Known limitations" section first: several open items
(grasp rigidity being a modelling choice rather than a physics guarantee,
the unexplained fingertip-runaway root cause, and the intermittent
preflight failure) are explicitly not TODOs to silently pick up, but the
documented boundary of what v1 demonstrates.
