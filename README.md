# UR5e + Robotiq 2F-85 Pick-and-Place — M-1 / M0 kit

Scaffolding for the gated milestones. **No application code yet** — the spec gates
M1–M5 behind M-1 and M0, and nothing here jumps that gate.

## What's in here

**Target host: Ubuntu 24.04 LTS (Noble).** ROS 2 Jazzy has no binaries for any
other release, and building it from source is not worth it here.

| File | Milestone | Purpose |
|---|---|---|
| `scripts/02_bootstrap_noble.sh` | — | Fresh Noble box → full stack installed and verified |
| `docs/M-1_reference_report.md` | M-1 | Reference repo decision, what must change, open questions |
| `scripts/00_recon.sh` | M-1 | Collects ground truth from your machine so the URDF merge is written from fact |
| `config/scene.yaml` | all | Single source of truth. `# RECON` markers = not yet known |
| `scripts/m0_verify.sh` | M0 | A/B/C checks, emits pass/fail note with log lines |
| `scripts/01_git_hygiene.sh` | — | Writes `.claude/settings.json`, run before first commit |

## Order of operations

```bash
# 0. git hygiene, before the first commit
bash scripts/01_git_hygiene.sh

# 1. install the stack on the fresh Noble machine  (~20-40 min, prompts for sudo)
bash scripts/02_bootstrap_noble.sh 2>&1 | tee bootstrap_$(date +%Y%m%d_%H%M%S).log
#    exit 0 = stack verified.  exit 1 = read the [FAIL] lines, stop.

# 2. optional but recommended — does UR's own arm-only path work, no gripper?
ros2 launch ur_simulation_gz ur_sim_moveit.launch.py ur_type:=ur5e
#    if this doesn't come up, the problem is upstream of anything we add.

# 3. recon — no sim needed, read-only, installs nothing
source /opt/ros/jazzy/setup.bash
bash scripts/00_recon.sh 2>&1 | tee docs/recon_$(date +%Y%m%d_%H%M%S).log
#    -> send me the log. I write the URDF merge from it.

# 4. (M-1 assembly happens here — merge xacro, Setup Assistant, clean spawn)

# 5. M0 — sim must already be up in another terminal
bash scripts/m0_verify.sh 2>&1 | tee docs/m0_$(date +%Y%m%d_%H%M%S).log
#    exit 0 = cleared for M1.  exit 1 = fix it first.
```

## Why bootstrap is a script, not a checklist

M0-A exists to catch a stray Fortress or Gazebo Classic dependency. The cheapest
way to pass it is to never create the condition. Jazzy's default Gazebo *is*
Harmonic, so installing `ros-jazzy-ros-gz` from the ROS 2 repo and letting it
pull its own Gazebo is the low-risk path. Hand-typed installs are where a stray
`gazebo11` or `ignition-*` gets in — usually from a blog post written for
Fortress. The script also refuses to add `packages.osrfoundation.org` as a
second apt source, which is the other common way to end up mismatched.

It checks for pre-existing contamination *before* installing anything, so on a
genuinely fresh image you get a clean baseline on record.

## Two things worth reading before you start

**`/joint_states` is not trustworthy in this stack.** The donor repo documents an
open `gz_ros2_control` state-readback defect — joint effort off by up to ~15x with no
consistent scale factor. `m0_verify.sh` therefore sources mimic-tracking ground truth
from Gazebo's own `JointStatePublisher` topic and only uses `/joint_states` as a
cross-check. If the two disagree, Gazebo wins. This also means the donor's
effort-threshold contact detection is built on a suspect signal — relevant if the
contact-triggered-attach fallback is ever authorised.

**M0-C failure looks like a friction problem.** A partially-tracking finger linkage
produces exactly the symptoms of bad grip: object slips, grasp is unstable, tuning
`mu` does nothing. If M0-C fails, do not spend a single hour on M3 friction tuning.
The script says this out loud when it fails, on purpose.

## What's deliberately absent

- MoveIt Task Constructor — deferred by the spec until perception lands
- Any perception / object detection — out of scope for v1
- Hardcoded poses anywhere outside `scene.yaml`
- The URDF merge itself — blocked on recon output, by design
