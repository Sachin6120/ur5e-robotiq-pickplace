# RGB-D Perception Handoff

Local checkpoint originally written 2026-08-22 on branch `rgbd-perception`,
extended through 2026-08-23. This file is documentation only.

**Current state: Milestones A, B, C, D, E, F1 and F2 are accepted complete and
PASS. F3 (perception-derived lift) HAS started and its Scene A trial FAILED**
(`RUN_ID=20260823_190610_13717`, G0->L2 slip 26.054 mm against a 5.000 mm
criterion; Scenes B-D not run). The validated boundary still ends at physical
grasp establishment; see §11.3. For F3 status, its accepted root-cause analysis
and the current next task, read `HANDOFF.md` and `PROJECT_STATE.md` — this file
is not maintained for F3.

Sections are appended in chronological order, so any forward-looking statement
inside an earlier section describes what was true when that section was
written. The line immediately below is preserved verbatim for that reason.

> Milestones A, B, and C are accepted complete. Milestone D has not started.

## 1. Project and source-of-truth order

This is a ROS 2 Jazzy / Gazebo Harmonic UR5e + Robotiq 2F-85 simulated
pick-and-place project. It uses `ros2_control`, MoveIt 2, URDF/XACRO, Gazebo
physics/contact, TF, and C++ ROS nodes. The repository root is
`~/ur5e_pickplace`.

Before making changes, inspect this file, `git status`, and the relevant
diffs. Repository/runtime evidence is authoritative. The accepted
measurements in this handoff supplement the repository because most experimental
captures intentionally live in `/tmp` and are not committed.

## 2. Validated classical manipulation baseline

The non-perception pipeline predates this branch and is preserved at tag
`m6-30mm-stable` / commit `7b875a4`:

- M-1 through M2 established the merged model, controllers, planning, TF, and
  Cartesian approach.
- M3-M5 established physical grasp, full pick/transport/place/release/retreat,
  and repeatability; the detailed history is in `docs/HANDOFF_M3.md`.
- M6 established the 30 mm object configuration represented by the stable tag.
- The Robotiq production solution uses
  `effort_controllers/GripperActionController` on the one actuated master joint,
  `robotiq_85_left_knuckle_joint`, with PID `P=50`, `D=2`, `I=0`. Direct effort
  bypasses DART's velocity-servo equality constraint and eliminated the
  destructive 18-43 Hz position-servo/contact limit cycle. The four production
  regression configurations completed the full seven-stage lifecycle.
- Frozen gripper/controller details are in
  `ur5e_robotiq_description/config/controllers.yaml`, commit `f33879d`, and the
  M10 section of `docs/HANDOFF_M3.md`. Do not revert to the former position
  controller or casually change the master/mimic interface set.

The intended initial perception/manipulation sequencing is deliberately
stop-and-capture, not visual servoing:

```
HOME -> M1 OBSERVATION POSE -> STOP -> CAPTURE -> RETURN HOME
     -> EXISTING MANIPULATION PIPELINE
```

Only the observation side through a 2D mask is complete. Its integration into
manipulation has not started.

## 3. RGB-D chronology

### Milestone A — observation geometry and sensor performance: COMPLETE

Architecture: fixed eye-to-hand Gazebo `rgbd_camera`, mounted to world and
enabled explicitly with `enable_camera:=true`. RGB, depth, and CameraInfo are
bridged Gazebo-to-ROS; the point cloud is intentionally not bridged.

Final accepted configuration:

| Item | Frozen value |
|---|---|
| Camera body XYZ | `[0.450, 0.025, 2.400]` m |
| Camera body RPY | `[0, pi/2, 0]` rad (straight down) |
| Optical joint RPY relative to body | `[-pi/2, 0, -pi/2]` rad |
| Optical convention | +Z world -Z/down; +X world -Y/image-right; +Y world -X/image-down |
| Resolution | `960 x 720` |
| Horizontal FOV | `1.047` rad, approximately 60 degrees |
| Update rate | 5 Hz simulation time |
| Clipping | near 0.10 m, far 3.0 m |
| RGB | `rgb8` (`R8G8B8`) |
| Depth | `32FC1`, metres |
| Optical frame | `camera_optical_frame` |
| M1 joint pose | `[0.5, -1.2, 1.0, -1.4, -1.5708, 0.0]` |

Accepted results at M1:

- Object footprint: `23 x 16 = 368` pixels.
- Object visibility: 100% clear.
- Placement-target visibility: 100% clear.
- Workspace clear: 72.930%.
- Frame occlusion: 8.357%.
- All four table corners visible.
- Effective RTF after the 5 Hz change: 0.957.
- Fresh corresponding RGB-D pair after the arm became stationary: 0.186 s.
- Representative object depth: approximately 1.605 m; table/target: 1.650 m.

Why M1 is mandatory: at the same 2.4 m camera pose, HOME physically hid the
placement target. Moving only the arm from HOME to the already-validated M1
joint goal changed target visibility from 0% to 100%, object visibility stayed
100%, and workspace clear changed from 72.6% to about 73.05%. M1 therefore
solved physical occlusion without changing camera calibration.

Rejected precursor configurations:

- Straight-down z=1.600 m at HOME: object visible at `33 x 20 = 660` pixels,
  but the arm/wrist occluded 46.1% of the workspace and 43.9% of the frame.
- Straight-down z=2.400 m at HOME: workspace occlusion improved to 27.4% and
  frame occlusion to 7.0%, with object `16 x 10 = 160` pixels and fully clear,
  but the placement target was completely occluded. Depth clipping had ample
  headroom. A +X-only camera translation was analytically ruled out because the
  offset needed to clear the arm would move required workspace outside the FOV.
- At z=2.400 m and M1, 640x480 solved visibility but measured only
  `15 x 10 = 150` pixels, so that experiment correctly retained its formal
  `M1 PERCEPTION POSE — FAIL` verdict against the predefined 160-pixel minimum.
  Architecturally, it proved M1 solved occlusion; remaining failure was sampling.
- 1280x960 at M1 passed sampling (`31 x 20 = 620`) but caused severe performance
  regression: RGB about 5.37 Hz wall-clock, depth about 2.51 Hz, RTF
  0.0116-0.0137 during the original render-bound measurement.
- 960x720 passed (`23 x 16 = 368`) and was frozen as the lowest tested practical
  sampling resolution. At 30 simulation-time Hz it was still performance-poor:
  RGB about 7.15 Hz, depth about 3.79 Hz, RTF 0.0188-0.0212 in the original
  render-bound experiment.

The performance audit separated simulation-time from wall-clock rates. With
960x720/30 Hz, camera-disabled Gazebo was about 1.0 RTF, while RGB-D rendering
gave about 0.37 effective RTF; native Gazebo RGB/depth was about 11.1 Hz,
matching `30 Hz * 0.37`. Thus Gazebo RGB-D rendering/sensor generation was the
dominant cost, not DDS loss; the bridge added secondary throughput loss.
Changing exactly one variable, sensor update rate 30 -> 5 Hz, recovered 0.957
RTF while retaining perception and giving a fresh stopped pair in 0.186 s.
That is why 5 Hz is frozen. Do not optimize Milestone A further.

### Milestone B — quantitative metric depth: COMPLETE / PASS

Method:

1. Verified runtime CameraInfo, `32FC1`, optical frame, and world-to-camera TF.
2. Verified the straight-down optical transform before using world-Z
   differences as optical depth.
3. Spawned the same object, executed M1, waited for all six arm joints to stop,
   discarded stale frames, and captured fresh sensor frames.
4. Used small interior ROIs away from object/table/robot boundaries.
5. Compared analytical geometry against measured mean/median/min/max, validity,
   and five-frame repeatability. Acceptance was fixed at <=2 mm error, 100% ROI
   validity, correct metric units, and stable repeatability.

Analytical and accepted results:

| Surface | Geometry | Expected | Accepted measured result | Error |
|---|---|---:|---:|---:|
| Object top | camera z 2.400 minus settled top z 0.795 | 1.605 m | mean 1.605000084 m | 0.000084 mm |
| Table top | camera z 2.400 minus surface z 0.750 | 1.650 m | mean 1.650000215 m | 0.000215 mm |
| Floor | camera z 2.400 minus world z 0 | 2.400 m | approximately 2.400000 m | float-quantization scale, far below 2 mm |

The settled object centre was z=0.772500 m for a 45 mm-tall object, hence top
z=0.795000 m. Tested ROIs were 100% finite with no NaN, positive Inf, negative
Inf, or zero values. The saved five-frame capture contains 691,200/691,200
finite pixels per frame. Fixed ROI values were effectively unchanged across all
five frames; frame-to-frame standard deviation and maximum deviation were at
floating-point noise/zero scale. Units are metres. Final accepted verdict:
`MILESTONE B — DEPTH VALIDATION PASS`.

Temporary evidence present at handoff time (not repository-tracked):
`/tmp/milestone_b_capture.npz` and `/tmp/milestone_b_m1.csv`. Do not assume
`/tmp` survives a reboot or a new environment; the accepted numbers above are
the checkpoint.

### Milestone C — deterministic object mask: COMPLETE / PASS

Implementation location:

- `ur5e_pick_place/src/object_detector.cpp` (new, currently untracked)
- `ur5e_pick_place/CMakeLists.txt` (modified)
- `ur5e_pick_place/package.xml` (modified)

Architecture uses sensor data only:

1. Build a 1 mm histogram over a broad 1.20-2.00 m depth interval and infer the
   dominant tabletop plane independently for every observation.
2. Keep finite pixels whose minimum RGB channel is at least 200, RGB channel
   spread is at most 30, and whose depth is 10-100 mm above that inferred plane.
3. Run 8-connected component extraction and select the largest component
   satisfying the fixed, position-independent size rules.

No object world XYZ, image coordinate, bounding box, manual ROI, known centre,
or Gazebo state enters detection. Gazebo truth was queried only after each
detection for evaluation.

Synchronization and interfaces:

- Approximate-time `message_filters` synchronization for RGB, depth, and
  CameraInfo; queue 10; maximum interval 50 ms.
- Subscribes `/overhead_camera/image`, `/overhead_camera/depth_image`, and
  `/overhead_camera/camera_info`.
- Publishes `object_detector/detected`, `object_detector/mask`,
  `object_detector/debug_image`, `object_detector/bounding_box`,
  `object_detector/centroid`, and `object_detector/component_area`.
- Debug output is a headless ROS image with bounding box and centroid; no GUI is
  required.

Frozen detector parameters:

| Parameter | Value |
|---|---:|
| `brightness_min` | 200 |
| `chroma_max` | 30 |
| `min_height_m` / `max_height_m` | 0.010 / 0.100 |
| `plane_min_depth_m` / `plane_max_depth_m` | 1.20 / 2.00 |
| `plane_bin_m` | 0.001 |
| `min_component_area` / `max_component_area` | 160 / 5000 |
| `min_component_width` / `min_component_height` | 16 / 10 |

Acceptance was declared before the live matrix: detect 4/4 present scenes,
zero false positives in the absent scene, centroid error <=3 px, mask area
>=160 px, projected top-surface bounding-box IoU >=0.70, and selected per-scene
processing observation <50 ms.

| Scene, world XY | Detected bbox | Detected centroid | Truth projection | Error | Area | IoU | Latency |
|---|---|---|---|---:|---:|---:|---:|
| Original `(0.45,-0.15)` | `[559,352,23,16]` | `(570.0,359.5)` | `(570.670,360.000)` | 0.836 px | 368 | 0.958 | 33.076 ms |
| Additional 1 `(0.80,-0.25)` | `[611,171,23,15]` | `(622.0,178.0)` | `(622.482,178.660)` | 0.817 px | 345 | 0.952 | 33.874 ms |
| Additional 2 `(0.80,0.25)` | `[352,171,23,15]` | `(363.0,178.0)` | `(363.424,178.660)` | 0.784 px | 345 | 0.952 | 32.492 ms |
| Additional 3 `(0.18,-0.22)` | `[595,492,24,16]` | `(606.5,499.5)` | `(606.938,499.891)` | 0.587 px | 384 | 0.944 | 32.187 ms |

All detected masks had 100% finite depth and mean depth 1.605000 m. With the
object removed using the existing project mechanism, the unchanged detector
consistently published `NO_OBJECT`; accepted false-positive components: zero.
Final verdict: `MILESTONE C — OBJECT DETECTION PASS`.

`colcon build --packages-select ur5e_pick_place --symlink-install` passed.
The only build stderr was the pre-existing MoveIt `tl_expected` deprecation.

## 4. Frozen configuration — do not change casually

Milestone D must start from all of the following unchanged:

- Camera body pose `[0.450,0.025,2.400]`, RPY `[0,pi/2,0]`.
- Optical-frame joint and REP-103/145 axis mapping documented above.
- 960x720, 60-degree HFOV, 5 Hz simulation time, near/far 0.10/3.0 m.
- RGB `rgb8`, depth `32FC1`, `camera_optical_frame`.
- M1 `[0.5,-1.2,1.0,-1.4,-1.5708,0.0]` and stopped-capture architecture.
- Every Milestone C parameter and synchronization setting in the table above.
- Classical arm/controller interfaces and the direct-effort Robotiq controller:
  one master joint, effort GripperActionController, P=50/D=2/I=0, fixed
  fingertips and existing mimic configuration.
- Object dimensions/appearance, table, target, lighting, physics, bridge, QoS,
  and scene geometry used by the accepted baseline.

Milestones A and B must not be reopened. Milestone C must not be retuned during
Milestone D merely because a later calculation is inconvenient. Any future
change to a frozen value requires a separately authorized controlled experiment.

## 5. Git and local state at checkpoint

Branch: `rgbd-perception`.

Stable tag/hash:

```
m6-30mm-stable -> 7b875a4
HEAD            -> 7b875a4
```

`origin/m6-width-30mm` also points to `7b875a4`. Therefore the current commit is
not ahead of or behind its upstream; all RGB-D work exists only as uncommitted
working-tree content. `origin/main` is at `9c26214`, but comparing this feature
branch directly to `origin/main` is not evidence that the configured upstream is
behind.

Tracked modified files:

- `ur5e_pick_place/CMakeLists.txt` — intentional Milestone C build target and
  dependencies.
- `ur5e_pick_place/package.xml` — intentional Milestone C dependencies.
- `ur5e_robotiq_description/CMakeLists.txt` — intentional RGB-D world/install
  support from Milestone A.
- `ur5e_robotiq_description/launch/ur5e_robotiq_sim_control.launch.py` —
  intentional opt-in camera launch/world and bridge path.
- `ur5e_robotiq_description/urdf/ur5e_robotiq.urdf.xacro` — intentional frozen
  camera, optical frame, and RGB-D sensor definition.

Untracked files relevant to RGB-D:

- `ur5e_pick_place/src/object_detector.cpp` — Milestone C detector.
- `ur5e_robotiq_description/worlds/tabletop_rgbd.sdf` (inside the untracked
  `worlds/` directory) — sensor-enabled Gazebo world.
- `docs/HANDOFF_RGBD_PERCEPTION.md` — this local checkpoint.

Other pre-existing untracked local work, preserved and not altered by the RGB-D
milestones:

- `m3_grasp.csv`.
- `docs/m8*`, `docs/m9*`, `docs/m10*`, and `docs/prod_reg*` capture scripts and
  marker directories. These are classical-gripper experimental artifacts.
  Do not delete or assume they are disposable merely because they are untracked.

Tracked `git diff --stat` before adding this untracked handoff was:

```
ur5e_pick_place/CMakeLists.txt                                |  18 +++
ur5e_pick_place/package.xml                                  |   5 +
ur5e_robotiq_description/CMakeLists.txt                      |   2 +-
ur5e_robotiq_description/launch/ur5e_robotiq_sim_control...  |  57 +++++++-
ur5e_robotiq_description/urdf/ur5e_robotiq.urdf.xacro        | 145 +++++++++++++++++++++
5 files changed, 225 insertions(+), 2 deletions(-)
```

Note that `git diff --stat` excludes all untracked files, including the detector
and this handoff. Nothing in the RGB-D work is committed or pushed.

Recent history at checkpoint:

```
7b875a4 (HEAD, tag: m6-30mm-stable, upstream) docs: qualify M3-M5 figures...
f33879d fix: stabilize Robotiq grasp with direct effort control
a35d1ab test: configure 30mm object for M6 validation
20c47fb fix: derive gripper grasp width from closing axis
3ee4575 tools: add M6 pre-grasp joint trajectory recorder + capture scripts
9c26214 (origin/main, main) docs: clarify language-mix note in README
ffa27a0 evidence: commit the M3/M5 sweep CSV and M4 placement log...
71599c5 chore: remove obsolete git attribution setup script...
38a50ce chore: remove tool-specific files, correct stale topology comments
aeac80c chore: clean public release documentation and logs
```

## 6. Known warnings and backlog

### Blocking

- None for starting Milestone D from the frozen configuration.

### Non-blocking

- At the earlier 30 Hz setting, RGB/depth wall-clock gaps and bridge/native
  throughput differences were observed. These were explained primarily by
  simulation-time rate multiplied by low RTF, not DDS loss. The accepted 5 Hz
  stopped-capture path is validated; do not classify the old gaps as a new
  Milestone D failure without new evidence.
- Continuous Milestone C callbacks were normally about 28-42 ms, with occasional
  outliers up to 60.396 ms. The selected scene observations were 32.2-33.9 ms,
  and all remained well below the 200 ms sensor period.
- A one-shot `ros2 topic echo` reported one CameraInfo message loss. Detector
  synchronization continued and all matrix results were stable.
- `move_group` segfaulted during Ctrl-C teardown after the completed Milestone C
  test. Planning and M1 execution had succeeded before shutdown. Treat as a
  shutdown-only issue unless reproduced during operation.
- Gazebo exits with signal-derived status when intentionally interrupted and
  prints existing mesh-collision, deprecated-controller, and controller-period
  warnings. These are not new perception failures.
- Gazebo reported zero controller asynchronous messages lost at shutdown; one
  `publish_async_failures_` counter was printed during teardown.
- The build emits the existing MoveIt `tl_expected` deprecation warning.

### Backlog / documentation cautions

- `config/scene.yaml` still contains historical prose describing the gripper as
  position-controlled, while the validated committed controller is direct
  effort. Trust `controllers.yaml`, commit `f33879d`, and the final M10 handoff
  section; clean stale prose only in a separately scoped documentation task.
- Gazebo/DART does not natively enforce all mimic constraints. The project uses
  the validated master/fixed-tip/software-mimic arrangement. Do not “fix” this
  based solely on the recurring engine warning.
- Much of the live RGB-D evidence is in `/tmp` or captured in this handoff, not
  in committed repository artifacts. Do not reinterpret absence of raw files as
  evidence that Milestones A-C were not performed.
- No custom detection message exists. Standard ROS messages were intentionally
  used. Interface refinement, if ever needed, is backlog—not part of D's math.

## 7. Exact continuation point

STATUS UPDATE 2026-08-22, later the same day: Milestone D is now COMPLETE /
PASS. Section 8 below records it. The text immediately following is the
original pre-D continuation note, kept verbatim because it is the predeclared
scope that Milestone D was judged against.

NEXT MILESTONE (as predeclared, now satisfied):
MILESTONE D — CAMERA-FRAME 3D OBJECT POSITION

Goal:

```
detected object mask
+ validated metric depth
+ runtime CameraInfo
-> 3D object position in camera_optical_frame
```

Milestone D has NOT started.

Milestone D must use the frozen detector output and metric depth to estimate a
camera-frame position, with a controlled evaluation that does not leak Gazebo
truth into the estimator. It must NOT yet:

- transform into world frame;
- estimate yaw/orientation;
- modify manipulation;
- generate grasp poses;
- start PPO or RL.

Milestone E will later validate the `camera_optical_frame` to world-frame
transformation. Stop after Milestone D evidence; do not combine D and E.

## 8. Milestone D — camera-frame 3D object position: COMPLETE / PASS

Executed 2026-08-22 on branch `rgbd-perception`, uncommitted, against the
frozen Milestone A configuration and the frozen Milestone C detector.

### 8.1 The estimator (sensor-only) — `object_detector.cpp`

Extended in place, deliberately not as a second node: the existing detector
already owns the validated synchronized RGB + depth + CameraInfo observation
and the validated final component mask, so Milestone D consumes exactly those
rather than opening a second synchronization boundary.

Predeclared before any ground-truth query:

- **Estimated quantity: the VISIBLE TOP-SURFACE position of the object in
  `camera_optical_frame`.** It is NOT the object's geometric centre. A single
  overhead depth view observes only the top face. No half-height term is added
  anywhere in the estimator, and the truth reference is built to match (see
  8.4).
- Optical convention, confirmed at runtime and independently from the URDF:
  `+X` = image right, `+Y` = image down, `+Z` = camera forward (into scene).
- Back-projection, per valid masked pixel, using runtime `CameraInfo` only:

```
Z_i = depth_i
X_i = (u_i - cx) * Z_i / fx
Y_i = (v_i - cy) * Z_i / fy
fx = K[0]   fy = K[4]   cx = K[2]   cy = K[5]
```

- Rejection is limited to genuinely invalid depth: non-finite (NaN, ±Inf) or
  `<= 0`. There is no empirical XYZ gating and no ground-truth-derived offset;
  either would make the estimate a function of the answer.
- **Estimator = the per-axis median** over all valid points of the FINAL
  selected connected component (not the preliminary threshold mask). The
  median is the sample median: mean of the two central order statistics for an
  even count. Mean and standard deviation are computed and logged as
  diagnostics only and are never the estimate.

Runtime `CameraInfo` observed (logged once per run, verbatim):

```
frame=camera_optical_frame width=960 height=720 expected=960x720 resolution_match=yes
fx=831.574069234 fy=831.574069234 cx=480.000000000 cy=360.000000000
K=[831.574069234, 0.000000000, 480.000000000,
   0.000000000, 831.574069234, 360.000000000,
   0.000000000,   0.000000000,   1.000000000]
```

Output: `object_detector/position_camera`, `geometry_msgs/msg/PointStamped`,
`frame_id` taken from the CameraInfo (`camera_optical_frame`), stamp = the
synchronized RGB-D observation stamp used for that estimate. Publication is
conditional on a successful detection AND at least one valid back-projected
point; nothing — no placeholder, no `[0,0,0]` — is published otherwise. All
six Milestone C outputs are unchanged.

Milestone C remained frozen. Verified by brace-matched extraction and SHA-256
of the two functions that constitute detection:

```
Detection detect(...)          62 lines   680d007daab7997e -> 680d007daab7997e  IDENTICAL
double estimate_plane_depth()  23 lines   b540c0dee4eb3d77 -> b540c0dee4eb3d77  IDENTICAL
```

All eleven Milestone C parameters, the `ApproximateTime<Image, Image,
CameraInfo>` policy, queue 10, and the 0.05 s maximum interval are untouched.
The whole file diff is +205 / −1 lines, and the single removed line is the
first line of the header comment, replaced by an expanded one.

The only Milestone C-visible change is bookkeeping: the `latency_ms` field of
the `DETECTED` / `NO_OBJECT` log lines keeps its original span (callback entry
to log point) and therefore now also contains the reconstruction cost. The new
`POSITION3D` line breaks that total into `seg_ms` and `recon_ms`.

### 8.2 The ground-truth evaluation harness — SEPARATE, and outside the repo

The harness lives outside the package, was run as two distinct processes, and
is not part of the estimator:

- Phase 1 sets the scene up, reads `object_detector/position_camera`, writes
  the readings to disk, and prints `SENSOR ESTIMATE FROZEN` before exiting.
- Phase 2 is a different process that loads that frozen file and only then
  makes its first ground-truth call.

Isolation is structural, not procedural: `object_detector.cpp` has no Gazebo,
world-pose, TF, or camera-extrinsics input of any kind. It subscribes to three
sensor topics and nothing else. No transformed truth, no camera world pose,
and no evaluation output is fed back into it.

Per-scene procedure: remove any object → arm to HOME → spawn object at the
scene XY (same SDF, size, mass, friction and white material as
`scripts/08_spawn_pick_object.sh`, XY parameterised) → settle via
`scripts/lib/gz_settle.py` → arm to M1 → require all six arm joints under
1e-3 rad/s for six consecutive samples → record the sim time of stationarity →
discard every observation stamped at or before it, plus one further warm-up
frame → collect five fresh estimates → freeze → then query truth.

Truth construction, evaluation only: settled object pose is sampled with
`scripts/lib/sample_pose.py` (windowed, refuses to sample a moving pose); the
**top-surface centre** is `object centre + size_z/2` in world Z
(`0.045 / 2 = 0.022500 m`), and it is transformed into `camera_optical_frame`
two independent ways — the runtime TF, and analytically from the URDF
constants. The two agreed to **0.000000 mm** in every scene, and the analytic
optical axes came out as `+X = world −Y`, `+Y = world −X`, `+Z = world −Z`,
matching the URDF's documented mapping.

### 8.3 Predefined acceptance criteria (locked before the truth comparison)

Object-present: (1) detection succeeds; (2) estimate finite; (3) retained mask
depth validity 100%; (4) Euclidean camera-frame error <= 3.0 mm; (5) no single
coordinate error > 3.0 mm; (6) total perception computation < 200 ms.
Object-absent: (7) `NO_OBJECT`; (8) no position estimate published.
Repeatability: (9) five fresh stationary observations all finite; (10) maximum
Euclidean deviation from their mean <= 1.0 mm.

### 8.4 Four-position matrix

Detection reproduced Milestone C exactly — every bounding box and area is
identical to the values recorded in section 3, which is independent evidence
that the frozen path was not disturbed.

| Scene, world XY | bbox | area | mask px | valid | invalid | valid % |
|---|---|---:|---:|---:|---:|---:|
| A `(0.45,-0.15)` | `[559,352,23,16]` | 368 | 368 | 368 | 0 | 100.0000 |
| B `(0.80,-0.25)` | `[611,171,23,15]` | 345 | 345 | 345 | 0 | 100.0000 |
| C `(0.80, 0.25)` | `[352,171,23,15]` | 345 | 345 | 345 | 0 | 100.0000 |
| D `(0.18,-0.22)` | `[595,492,24,16]` | 384 | 384 | 384 | 0 | 100.0000 |

Sensor estimate against top-surface truth, all in `camera_optical_frame`:

| Scene | estimate XYZ [m] | truth XYZ [m] | ΔX [mm] | ΔY [mm] | ΔZ [mm] | Euclidean |
|---|---|---|---:|---:|---:|---:|
| A | `0.173706724, -0.000965037, 1.605000079` | `0.175, 0.000, 1.605` | −1.2933 | −0.9650 | +0.0001 | **1.6136 mm** |
| B | `0.274070619, -0.351273610, 1.605000138` | `0.275, −0.350, 1.605` | −0.9294 | −1.2736 | +0.0001 | **1.5767 mm** |
| C | `-0.225818749, -0.351273610, 1.605000138` | `−0.225, −0.350, 1.605` | −0.8187 | −1.2736 | +0.0001 | **1.5141 mm** |
| D | `0.244154460, 0.269245432, 1.605000138` | `0.245, 0.270, 1.605` | −0.8455 | −0.7546 | +0.0001 | **1.1333 mm** |

In metres the Euclidean errors are 0.0016136, 0.0015767, 0.0015141 and
0.0011333 m. Worst coordinate error anywhere in the matrix: 1.2736 mm.

The mean equalled the median to nine decimals in all four scenes — the masks
are filled rectangles, so the two estimators coincide here. Diagnostic
standard deviations were ~0.0128 m in X and ~0.0084-0.0089 m in Y (the spatial
extent of the mask, as expected) and 0-6e-8 m in Z, confirming a flat observed
top face with no measurable side-wall contamination.

### 8.5 Five-frame repeatability at `(0.45,-0.15)`

Five fresh stationary observations, each estimated independently — no frame
averaging before estimation:

| frame | stamp [s] | X | Y | Z |
|---:|---:|---|---|---|
| 0 | 175.600 | 0.173706724 | −0.000965037 | 1.605000079 |
| 1 | 175.800 | 0.173706724 | −0.000965037 | 1.605000079 |
| 2 | 176.000 | 0.173706724 | −0.000965037 | 1.605000079 |
| 3 | 176.200 | 0.173706724 | −0.000965037 | 1.605000079 |
| 4 | 176.400 | 0.173706724 | −0.000965037 | 1.605000079 |

mean `[0.173706724, −0.000965037, 1.605000079]`; standard deviation `[0, 0, 0]`
to double precision; maximum absolute coordinate deviation **0.000000 mm**;
maximum Euclidean deviation from the mean **0.000000 mm**. Scenes B, C and D
were equally invariant across their own five frames.

### 8.6 Latency

Steady-clock timing around the computation only, not the 200 ms camera period.
Pooled over all 495 logged detections across the four scenes:

| Stage | min | median | max |
|---|---:|---:|---:|
| A. segmentation / component selection (`seg_ms`) | 25.383 | 32.578 | 62.210 |
| B. masked 3D reconstruction (`recon_ms`) | 0.399 | 0.559 | 2.194 |
| C. total detector + reconstruction (`total_ms`) | 27.134 | 34.634 | **68.565** |

Reconstruction adds a median of 0.559 ms, about 1.6% of the total. The worst
single observation anywhere was 68.565 ms against the 200 ms criterion. The
815 `NO_OBJECT` callbacks ran 25.731 / 34.003 / 78.270 ms (min/median/max);
those do no reconstruction at all, so their spread is the pre-existing
Milestone C variance, not a Milestone D cost.

### 8.7 Object-absent behaviour

Object removed with the existing project mechanism, detector parameters
unchanged, arm held at M1: every `detected` sample was `false`, the detector
logged `NO_OBJECT` continuously, and **zero** `POSITION3D` lines were emitted.
The evaluation subscriber saw `0` messages on `object_detector/position_camera`
in total — not merely zero after the stationarity timestamp.

Latching was checked rather than assumed: the topic reports `Durability:
VOLATILE`, so a late-joining subscriber cannot be handed a stale estimate from
a previous scene, and each scene ran its subscriber in a fresh process.

### 8.8 Criterion-by-criterion result

| # | Criterion | Result |
|---:|---|---|
| 1 | Detection succeeds, every object-present scene | PASS — 4/4, bboxes identical to Milestone C |
| 2 | Position estimate finite | PASS — all finite |
| 3 | Retained mask depth validity 100% | PASS — 0 invalid in 1442 masked pixels total |
| 4 | Euclidean error <= 3.0 mm | PASS — worst 1.6136 mm |
| 5 | No coordinate error > 3.0 mm | PASS — worst 1.2736 mm |
| 6 | Total computation < 200 ms | PASS — worst 68.565 ms |
| 7 | Object-absent reports NO_OBJECT | PASS |
| 8 | No position estimate published when absent | PASS — 0 messages |
| 9 | Five fresh estimates all finite | PASS |
| 10 | Max Euclidean deviation from mean <= 1.0 mm | PASS — 0.000000 mm |

**MILESTONE D — CAMERA-FRAME 3D POSITION PASS**

### 8.9 Observations and backlog from Milestone D — not fixed here

- **The entire error budget is Milestone C's mask discretization, not the
  back-projection.** At 1.605 m one pixel is 1.9300 mm. Scene A's 1.6136 mm
  error is 0.836 px — the exact centroid error Milestone C recorded for that
  scene. Back-projection is exact for a horizontal top face under a
  straight-down camera (all top-face pixels share one depth), which is why ΔZ
  is 0.0001 mm everywhere. Any future improvement has to come from sub-pixel
  mask refinement, not from the 3D maths.
- **The XY bias is systematic and sub-pixel.** ΔX and ΔY are negative in all
  four scenes, between −0.39 px and −0.67 px. This is a mask-edge/pixel-grid
  effect, not drift. It was NOT corrected: adding an offset here would be
  exactly the ground-truth calibration Milestone D forbids. Milestone E should
  be aware of it before it is composed with a world-frame transform.
- The inferred table plane reads `1.650500 m` against a true 1.650000215 m —
  a +0.4998 mm artefact of the 1 mm histogram bin, quantified here for the
  record. It feeds only Milestone C's 10-100 mm height gate and does not enter
  the position estimate at any point.
- Gazebo continues to print `Failed to load system plugin [JointCmdProbe]` and
  an SDF `gz_frame_id` "not defined in SDF" warning. Both pre-date this
  milestone; the frame id is still applied correctly, as the observed
  `frame=camera_optical_frame` shows.
- **Milestone B's raw artefacts are confirmed gone.** `/tmp/milestone_b_capture.npz`
  and `/tmp/milestone_b_m1.csv` no longer exist on this machine, as section 3
  warned could happen. Nothing has been fabricated to replace them; Milestone
  B's tabulated numbers remain the checkpoint. Milestone D independently
  re-confirms the depth scale in passing — every scene measured the top face at
  1.605000079-1.605000138 m against an analytical 1.605 m — but that is a
  by-product, not a re-run of Milestone B.
- Milestone D's own evidence is likewise session-local and uncommitted. The
  numbers in this section are the checkpoint.

### 8.10 Continuation point after Milestone D

MILESTONE E — world-frame transformation — has NOT started. Milestone D
deliberately stopped at `camera_optical_frame`: nothing was transformed into
world or robot-base frame by the estimator, no yaw or orientation was
estimated, no grasp pose was generated, no MoveIt target was changed, and the
classical pick/place pipeline was not touched. The perceived point is not yet
used by anything.

## 9. Milestone E — camera-frame → world-frame object position: COMPLETE / PASS

Executed 2026-08-22 on branch `rgbd-perception`, uncommitted, against the
frozen Milestone A camera, the frozen Milestone C detector, and the frozen
Milestone D camera-frame estimator.

### 9.1 Production architecture

A **separate minimal node**, `ur5e_pick_place/src/object_position_world.cpp`,
subscribing to Milestone D's existing output. `object_detector.cpp` has a
**zero-line diff** for this milestone.

That was the deciding argument. Milestone D's acceptance rested on a
byte-level proof that the detection functions were unchanged; leaving the file
untouched is the cheapest way to keep that proof valid. The detector also has
no TF dependency at all, so folding a `Buffer`/`TransformListener` — and its
listener thread — into the validated process would itself have been a change
to a frozen component. The coupling between the two is a single topic
Milestone D already publishes, so the split costs nothing but one DDS hop, and
nothing is closed-loop yet.

| Item | Value |
|---|---|
| Input | `object_detector/position_camera`, `geometry_msgs/msg/PointStamped`, `camera_optical_frame` |
| Output | `object_detector/position_world`, `geometry_msgs/msg/PointStamped`, `frame_id = world` |
| Timestamp | the original camera-frame observation stamp, copied verbatim and re-asserted after the transform; never `now()` |
| TF2 API | `tf2_ros::Buffer` + `tf2_ros::TransformListener`; `buffer_->transform(*msg, out, "world")` from `tf2_geometry_msgs`, which looks up **at the message's own stamp** |
| Failure handling | `catch (tf2::TransformException)` → one concise `TF_TRANSFORM_FAILED` warning, nothing published. No retry, no sleep, no cached last point, no identity fallback. |

`tf2_ros::MessageFilter` was considered and rejected: it defers and queues
messages, which is the opposite of the simplest correct mechanism for a chain
that is static in practice.

**The camera mount constants appear nowhere in the production node.** They
reach the transform only through URDF → `robot_state_publisher` → `/tf_static`,
which is precisely what this milestone validates; hardcoding them, or deriving
world coordinates from them inside perception, would have made the check
circular. Gazebo truth is likewise absent from the node — it exists only in the
harness, in a separate process.

### 9.2 Runtime transform actually used

Looked up at runtime, `world ← camera_optical_frame`:

```
translation = [0.450000000, 0.025000000, 2.400000000]
quaternion  = [x=0.707106781, y=-0.707106781, z=0.000000000, w=-0.000000000]
optical +X -> world [ 0.000000, -1.000000, -0.000000]
optical +Y -> world [-1.000000, -0.000000, -0.000000]
optical +Z -> world [ 0.000000,  0.000000, -1.000000]
```

This matches the Milestone A geometry exactly: the frozen mount
`[0.450, 0.025, 2.400]` and the documented axis mapping `+X = world −Y`,
`+Y = world −X`, `+Z = world −Z`.

Diagnostic cross-check (NOT the production path): transforming a frozen camera
point analytically from the URDF mount constants and comparing against the
node's TF2 output gave a disagreement of **0.000000000 mm** in all four
scenes. TF2 remains the production transform regardless of the agreement.

### 9.3 Truth semantics

The Milestone D estimate is the visible TOP-SURFACE position, so the world
truth is built to match and never compared against the model centre. It is
derived from runtime state, not hardcoded: the settled object pose comes from
`scripts/lib/sample_pose.py`, the harness **asserts the runtime orientation is
identity** before applying a world-Z offset (it refuses to proceed otherwise),
and the half-height comes from `scene.yaml`'s `object.size[2]`:

```
truth_world_top = runtime_object_centre + [0, 0, size_z/2]   (size_z = 0.045, half = 0.022500)
```

### 9.4 Four-scene world-frame matrix

Detection reproduced Milestone C/D exactly again — every bounding box and area
identical to sections 3 and 8.

| Scene, spawn world XY | camera estimate XYZ [m] | world estimate XYZ [m] | world truth XYZ [m] | ΔX mm | ΔY mm | ΔZ mm | Euclidean | TF ms |
|---|---|---|---|---:|---:|---:|---:|---:|
| A `(0.45,−0.15)` | `0.173706724, −0.000965037, 1.605000079` | `0.450965037, −0.148706724, 0.794999921` | `0.450, −0.150, 0.795` | +0.9650 | +1.2933 | −0.0001 | **1.6136 mm** | ~0.027 |
| B `(0.80,−0.25)` | `0.274070619, −0.351273610, 1.605000138` | `0.801273610, −0.249070619, 0.794999862` | `0.800, −0.250, 0.795` | +1.2736 | +0.9294 | −0.0001 | **1.5767 mm** | ~0.027 |
| C `(0.80, 0.25)` | `−0.225818749, −0.351273610, 1.605000138` | `0.801273610, 0.250818749, 0.794999862` | `0.800, 0.250, 0.795` | +1.2736 | +0.8187 | −0.0001 | **1.5141 mm** | ~0.027 |
| D `(0.18,−0.22)` | `0.244154460, 0.269245432, 1.605000138` | `0.180754568, −0.219154460, 0.794999862` | `0.180, −0.220, 0.795` | +0.7546 | +0.8455 | −0.0001 | **1.1333 mm** | ~0.027 |

In metres: 0.0016136, 0.0015767, 0.0015141, 0.0011333 m. Worst single
coordinate error anywhere: **1.2736 mm**.

### 9.5 The world error is exactly Milestone D's error, re-expressed

Every world-frame Euclidean error is **bit-for-bit identical** to the
corresponding Milestone D camera-frame error, and the components are the exact
rotation predicted by the axis mapping:

```
world ΔX = −(camera ΔY)      world ΔY = −(camera ΔX)      world ΔZ = −(camera ΔZ)
```

| Scene | camera Δ (mm) | world Δ (mm) | camera Euclid | world Euclid |
|---|---|---|---:|---:|
| A | (−1.2933, −0.9650, +0.0001) | (+0.9650, +1.2933, −0.0001) | 1.6136 | 1.6136 |
| B | (−0.9294, −1.2736, +0.0001) | (+1.2736, +0.9294, −0.0001) | 1.5767 | 1.5767 |
| C | (−0.8187, −1.2736, +0.0001) | (+1.2736, +0.8187, −0.0001) | 1.5141 | 1.5141 |
| D | (−0.8455, −0.7546, +0.0001) | (+0.7546, +0.8455, −0.0001) | 1.1333 | 1.1333 |

This is the central result: **the transform contributes no error of its own.**
Milestone E adds a rigid re-expression and nothing else, so the camera
extrinsics and the TF chain are validated to the resolution the measurement
can see. The residual is entirely the inherited Milestone C/D sub-pixel mask
bias documented in 8.9, which was deliberately NOT corrected here — an offset
tuned against ground truth would have destroyed exactly the property being
measured.

### 9.6 Five-frame world repeatability at `(0.45,−0.15)`

Five fresh stationary observations, each transformed independently — no
averaging of camera frames before transformation. All five returned
`0.450965037, −0.148706724, 0.794999921` in `world`.

mean `[0.450965037, −0.148706724, 0.794999921]`; std `[0, 0, 0]`; maximum
absolute coordinate deviation **0.000000000 mm**; maximum Euclidean deviation
from the mean **0.000000000 mm**. Scenes B, C and D were equally invariant.
Every published message carried `frame_id = world`.

### 9.7 Latency

`transform_ms` is steady-clock timing around the `buffer_->transform` call
only, over 578 transforms:

| Stage | min | median | mean | p95 | max |
|---|---:|---:|---:|---:|---:|
| B. TF2 transform | 0.0194 | 0.0268 | 0.0324 | 0.0605 | **0.2216 ms** |
| A. camera-frame perception (C+D total) | 27.268 | 34.812 | — | — | 65.477 ms |
| C. worst-case end-to-end computation | — | — | — | — | **65.699 ms** |

The transform is **0.337%** of the worst-case end-to-end computation, against a
200 ms sensor period. **578 camera estimates in, 578 world estimates out — a
1:1 correspondence with zero `TF_TRANSFORM_FAILED` events across the entire
run.**

### 9.8 Object-absent behaviour

Object removed with the existing mechanism, all parameters unchanged, arm held
at M1: `detected` was `false` on every sample, and the subscriber saw **0**
messages on `object_detector/position_camera` and **0** on
`object_detector/position_world` — in total, not merely zero after the
stationarity timestamp. Both topics report `Durability: VOLATILE`, and each
scene ran its subscribers in a fresh process, so no stale estimate from a
previous scene could be mistaken for a new publication.

### 9.9 Truth-isolation procedure

Unchanged in principle from Milestone D, now covering the world estimate:

- **Structural.** `object_position_world.cpp` subscribes to one topic and
  reads TF. It has no Gazebo, ground-truth, or evaluation input, and no camera
  mount constant. `object_detector.cpp` is unchanged and equally isolated.
- **Procedural.** Two separate processes.
  `scripts/perception/milestone_e_harness.py` collects the paired camera and
  world estimates, writes `<scene>_sensor.json`, prints `SENSOR ESTIMATE
  FROZEN` and exits; `scripts/perception/milestone_e_truth.py` then loads that
  file and only afterwards makes its first ground-truth call.
- Nothing computed in the harness is fed back into either production node.

### 9.10 Acceptance criteria, locked before the truth comparison

| # | Criterion | Result |
|---:|---|---|
| 1 | Milestone C detection succeeds, all four scenes | PASS — bboxes/areas identical to sections 3 and 8 |
| 2 | Milestone D camera-frame estimate finite | PASS |
| 3 | World-frame transform succeeds | PASS — 578/578, zero failures |
| 4 | Published world point has `frame_id = world` | PASS |
| 5 | World top-surface Euclidean error <= 3.0 mm | PASS — worst 1.6136 mm |
| 6 | No world coordinate error > 3.0 mm | PASS — worst 1.2736 mm |
| 7 | Transform comfortably below the 200 ms period | PASS — max 0.2216 ms (0.11% of the period) |
| 8 | No empirical XYZ correction or calibration offset | PASS — none exists in either node |
| 9 | Five fresh stationary world estimates | PASS |
| 10 | Max Euclidean deviation from mean <= 1.0 mm | PASS — 0.000000000 mm |
| 11 | Object-absent reports NO_OBJECT | PASS |
| 12 | No new camera-frame point when absent | PASS — 0 messages |
| 13 | No new world-frame point when absent | PASS — 0 messages |

**MILESTONE E — WORLD-FRAME 3D POSITION PASS**

### 9.11 Anomalies and notes

- **One harness defect was found, diagnosed and fixed; no production defect.**
  The first scene-A run reported a missing world estimate for the fifth frame.
  The production node's own log disproved it: it had published for that exact
  stamp (`stamp=44.800000000`) at wall `1787434347.952785`, and the harness
  froze at `1787434347.953` — roughly 0.2 ms earlier. The collection loop was
  counting camera estimates and exiting before the extra DDS hop delivered the
  matching world message. `fresh_pairs()` now waits for complete camera+world
  pairs, so a genuine persistent transform failure surfaces as a loud timeout
  instead of a silently short result. Every number in 9.4-9.6 comes from runs
  after that fix. Nothing in production was changed in response.
- The inherited Milestone C/D sub-pixel bias (8.9) passes through the
  transform unchanged, rotated into world axes: world ΔX and ΔY are now both
  positive, between +0.75 mm and +1.27 mm. It is still uncorrected and still
  the entire error budget.
- Pre-existing and unrelated: Gazebo's `Failed to load system plugin
  [JointCmdProbe]`, the SDF `gz_frame_id` warning, and the MoveIt `tl_expected`
  CMake deprecation (the only build stderr).
- Milestone B's `/tmp` artefacts remain absent, as recorded in 8.9. Nothing has
  been fabricated to replace them.
- Milestone E's own evidence is session-local and uncommitted; the numbers in
  this section are the checkpoint.

### 9.12 Continuation point after Milestone E

Perception-driven manipulation has NOT started. `object_detector/position_world`
is published and validated, but **nothing consumes it**: no MoveIt goal was
changed, no grasp pose was generated, no orientation or yaw is estimated, and
the classical pick/place pipeline still uses its known object location from
`config/scene.yaml`. The next milestone is the separate exercise of replacing
that known location with the perceived one.

## 10. Milestone F1 — perception-derived pre-grasp: COMPLETE / PASS

Executed 2026-08-22/23 on branch `rgbd-perception`, uncommitted. This section
extends, and does not revise, the accepted A-E results above.

### 10.1 Scope and implementation checkpoint

The earlier Milestone F change was an **implementation checkpoint, not an
F1 PASS**. It added an opt-in consumer in `m3_grasp`: subscribe to
`object_detector/position_world`, convert the visible top-surface Z to the
object-centre semantics expected by `grasp_frame` with
`sample.z - object_height_m / 2`, and replace only the transform translation.
The configured approach orientation and `gripper_roll` remained untouched.
That checkpoint compiled but had not been exercised in live manipulation.

F1 made that interface evidence-grade and stopped after pre-grasp:

- `require_perception=false` preserves the explicit configured-position
  fallback. With `require_perception=true`, failure to receive a fresh valid
  point produces typed `PERCEPTION_TIMEOUT`; it never silently falls back.
- `position_source` is recorded immediately after `result` in the CSV and in
  the run summary: `configured`, `perceived`, `fallback_configured`, or
  `perception_timeout`. A successful classical fallback therefore cannot be
  mistaken for perception evidence.
- `pregrasp_only=true` stops after pre-grasp execution and Gazebo-ground-truth
  verification. It is distinct from `close_and_hold_only`, which stops only
  after physical contact.
- When perception is enabled the arm first moves to the frozen M1 observation
  pose, using joint names and values passed directly from `scene.yaml`.
- The node then requires six consecutive `/joint_states` samples with every
  arm-joint velocity below 0.001 rad/s. Only a `PointStamped` stamped strictly
  after that stationarity boundary is accepted. One point is frozen and its
  subscription is destroyed before planning.

### 10.2 Strict no-object defect and exact cause

The first strict no-object run correctly produced `PERCEPTION_TIMEOUT` and
`position_source=perception_timeout`, but then planned and executed the
configured pre-grasp. Log timestamps proved the violation: timeout at wall
`1787436946.819810`, execute accepted at `1787436946.849219`, execute success at
`1787436955.255497`.

The control-flow cause was exact: Stage 1's historical
`setPoseTarget`/`plan`/`execute` sequence lacked an `ok(result)` guard. Before
F1, no failure could arise inside that already-entered MoveIt scope; strict
perception introduced the first such failure. A `may_move = ok(result)` guard
now covers target setting, planning, and execution and emits `NO_MOTION` when
false.

### 10.3 Guarded-binary validation

The guarded binary was rebuilt, then validated without tuning. Strict
no-object passed:

- `result=PERCEPTION_TIMEOUT`;
- `position_source=perception_timeout`;
- no fallback and no pre-grasp target;
- `NO_MOTION` logged;
- no planning or execution request after the timeout (the earlier request in
  the log was solely the required move to M1);
- post-failure arm state matched M1 within 0.09 mrad on every arm joint and was
  stationary.

Fallback observability also passed with perception unavailable and
`require_perception=false`: `PERCEPTION_FALLBACK` was explicit,
`position_source=fallback_configured`, and configured pre-grasp verification
error was 0.0393 mm.

The frozen four-scene matrix was rerun on the guarded binary. Every run used
`position_source=perceived`, stopped before descent or gripper motion, and left
the object unmoved:

| Scene, world XY | perceived top world [m] | perception→truth pre-grasp | achieved TCP error | orientation error | object displacement |
|---|---|---:|---:|---:|---:|
| A `(0.45,-0.15)` | `[0.450965,-0.148707,0.795000]` | 1.6134 mm | 0.1344 mm | 0.053010° | 0.0000 mm |
| B `(0.80,-0.25)` | `[0.801274,-0.249071,0.795000]` | 1.5767 mm | 0.1685 mm | 0.079557° | 0.0000 mm |
| C `(0.80, 0.25)` | `[0.801274, 0.250819,0.795000]` | 1.5145 mm | 0.0978 mm | 0.076356° | 0.0000 mm |
| D `(0.18,-0.22)` | `[0.180755,-0.219154,0.795000]` | 1.1339 mm | 0.1675 mm | 0.061282° | 0.0000 mm |

Freshness rejection was observable (`rejected_stale=1` in A-C; D's first
post-boundary observation was already fresh). Truth remained evaluation-only:
the harness froze sensor-derived targets and run evidence before the separate
truth process queried Gazebo.

**MILESTONE F1 — PERCEPTION-DERIVED PRE-GRASP PASS**

F1 proves only that a fresh perceived position can generate and reach the
classical pre-grasp. It makes **no claim** about descent, contact, gripper
closure, grasp stability, lift, transport, place, or release.

## 11. Milestone F2 — perception-derived grasp: COMPLETE / PASS

**Outcome: PASS.** §11 records the F2 investigation in the order it happened.
§§11-11.2 are the two intermediate FAILURES and their root causes, retained
because those mechanisms constrain any future change to the descent
configuration. **§11.3 is the accepted result.** Do not read §§11-11.2 as the
current state.

### 11.0 First revalidation checkpoint — FAIL / STOPPED (historical)

Validated evidence only: the harness now opens and verifies the Gazebo master
joint before every independent scene. Initialization-only passed from
`0.797012` to `0.003787 rad`; sequential Scene B passed from `0.797112` to
`0.003954 rad`, eliminating its former free-air pre-close timeout.

Initialized Scene A passed the frozen scene criteria but repeated the unresolved
large seating displacement (`21.5885 mm`, previously `21.5773 mm`). Initialized
Scene B returned a clean pre-close `REACHED_GOAL`, then failed the frozen
approach-disturbance criterion: `1.6211 mm` after descent and before final
closure, maximum `1.0 mm`. C/D and strict no-object were not run. No F2 PASS is
claimed; neither grasp-only run attempted lift, transport, or place.

### 11.1 Scene-B descent collision mechanism — validated diagnostic

One unchanged initialized Scene-B repeat used opt-in contact observers on all
distal Robotiq collision meshes plus synchronized Gazebo pose and joint state.
The first robot/object contact was the
`robotiq_85_right_inner_knuckle_link_collision` against `pick_target::link::c`
at sim `117.195 s`, at the object's world −X/top edge. Object motion began in
world +X at the next pose sample, sim `117.196 s` (1 ms later). Fingertips did
not contact until sim `121.457/121.463 s`; finger and outer-knuckle streams
remained empty.

The repeated P2-P1 displacement was
`[+1.434478,-0.003538,-0.000031] mm` (`1.434482 mm`), versus the prior
`[+1.621091,+0.009743,-0.000063] mm` (`1.621121 mm`). The mechanism is thus
proven and highly reproducible: the right inner knuckle's oblique lower surface
wedges the object along the closing axis during the last part of descent.
Instrumentation was evaluation-only and opt-in; production manipulation and
all frozen perception/control/physics values were unchanged. F2 remains FAIL;
no correction was attempted.

### 11.2 One controlled Scene-B correction — validated trial

The separately authorized correction changed only the descent pre-close
configuration to command `0.130 rad`; the controller returned `REACHED_GOAL`
at `0.120134 rad`. Deterministic trial initialization passed, perception was
fresh with `position_source=perceived`, Cartesian descent completed at fraction
1.0, and the final TCP translation error was `0.00068975 mm`.

P1 after pre-grasp and P2 after descent/before final close were identical at
`[0.800000000,-0.250000000,0.772499999949] m`, so corrected approach
disturbance was `0.000000 mm`, versus the prior `1.434482/1.621121 mm` failures.
No inner-knuckle, finger, outer-knuckle, or fingertip object contact occurred
during descent. The right inner knuckle first contacted only during final
closure at sim `114.211 s`; bilateral fingertip contact followed during that
closure. `F2 STOP` was reached with no lift, transport, place, or release.

**Scene-B corrected F2 trial PASS only.** Overall F2 remains pending A-D and
strict no-object/general regression under separate authorization.

### 11.3 Final universal pre-close validation — F2 PASS

The `0.130 rad` candidate subsequently failed Scene A because the actual
right-inner follower remained more closed than Scene B's at equivalent master
angles, causing premature right-inner/top-edge contact. Synchronized follower
states plus the real collision STL led to one universal `0.070 rad` candidate.
The first controlled Scene-A trial validated it: no premature contact, zero
P1->P2 motion, and reconstructed right-inner clearance `1.198288 mm`.

The same frozen value then passed B, C, and D without tuning:

| Scene | achieved pre-close [rad] | right-inner at descent end [rad] | min right-inner clearance [mm] | Cartesian fraction | P1->P2 disturbance [mm] | final TCP error [mm] | final close | F2 stop |
|---|---:|---:|---:|---:|---:|---:|---|---|
| A | 0.060133924 | -0.519798068 | 1.198288 | 1.0 | 0.000000 | 0.000296 | TIMED_OUT_HELD | yes |
| B | 0.060137029 | -0.413621276 | 6.841447 | 1.0 | 0.000000 | 0.000339 | TIMED_OUT_HELD | yes |
| C | 0.0601319 | -0.399647024 | 7.680157 | 1.0 | 0.000000 | 0.000702 | TIMED_OUT_HELD | yes |
| D | 0.0601414 | -0.482581096 | 3.602755 | 1.0 | 0.000000 | 0.000488 | TIMED_OUT_HELD | yes |

Every scene used fresh strict perception (`position_source=perceived`), had no
monitored robot/object collision before final closure, established bilateral
physical engagement during closure, and recorded no lift, transport, place,
or release. The follower-state result is important: master position alone is
not a deterministic proxy while the coupled joints are moving; clearance was
reconstructed from directly measured link/follower poses and the real STL.

Strict no-object regression passed with `PERCEPTION_TIMEOUT`,
`position_source=perception_timeout`, no fallback, and no manipulation after
M1. A subsequent full classical regression passed with perception disabled,
`position_source=configured`, pre-close `REACHED_GOAL`, grasp
`TIMED_OUT_HELD`, and transport `SUCCESS`.

Final closure seated the object approximately 21.6--22.3 mm in all four
scenes. This remains a separate grasp-quality warning and was not hidden,
tuned, or used to redefine the frozen pre-close/descent disturbance criterion.

**MILESTONE F2 — PERCEPTION-DERIVED GRASP PASS.** The validated boundary ends
after physical grasp establishment. No perception-derived lift, transport,
place, or release is claimed.

### 11.4 Evidence-durability limitation — added 2026-08-23

**F2 raw runtime evidence is not durable; F2 PASS currently rests on the
recorded measurement tables/documentation above.** This is an evidence-quality
limitation, not grounds to revoke the validated result.

Every `/tmp` path cited in §11 — `/tmp/ur5e_f2_results/`,
`/tmp/ur5e_f2_initialized_results/`, `/tmp/f2_scene_b_sync/`,
`/tmp/f2_A_0070_results/`, `/tmp/f2_A_0070_sync/`,
`/tmp/f2_0070_generalize/`, and F1's `/tmp/ur5e_f1_guarded_results/` — no
longer exists. Lost with them: the A-D `0.070 rad` node logs, CSVs and truth
JSON; the synchronized 1 kHz joint/follower traces and all eight contact
streams the clearance figures were reconstructed from; the strict no-object
run; and the final classical regression under the frozen `0.070 rad` value.
Nothing has been fabricated or reconstructed to replace them — the same
discipline §8.9 applied to Milestone B's lost artefacts.

What does survive, on local disk:

- **Raw, local-only:** the classical regression suffixed `20260823_004208_6512`
  (`runs/prod_reg_test_c_roll90_width30_{pose,traj}_*.csv` plus the
  `docs/prod_reg_test_c_roll90_width30_*` log/CSV set and marker directory).
  It ran on the F2 code base and proves no classical regression from the F2
  additions — but at `preclose_achieved=0.2280`, i.e. the OLD `0.30` margin.
  It is not evidence for the frozen `0.070 rad` candidate. These runtime
  artifacts were removed from the tracked public-release branch during
  cleanup; the conclusion and quantitative scope remain in this document.
- **Summarized:** §§10-11 of this file and the dated F1/F2 sections of
  `HANDOFF.md`.
- **Procedure:** `scripts/perception/milestone_{d,e,f1}_{harness,truth}.py`.
  F2 reused the F1 harness with runtime flags, so the method is reproducible
  even though its output is not.

Root cause of the loss, for whoever plans the next run: the harnesses default
their output to `/tmp`, and `.gitignore` excludes `runs/`, `docs/*.log`,
`docs/*.csv` and `docs/*.txt`. Any F2 regeneration or F3 run should write
evidence under the repository from the start and confirm it is not ignored.
See `HANDOFF.md`, "Evidence durability audit", for the full audit and the
regenerate-before-F3 recommendation.

### 11.5 Durable evidence regenerated — 2026-08-23, PASS REPRODUCED

The recommendation in §11.4 was carried out. The frozen A-D matrix was rerun
without tuning, each scene on a completely fresh stack, into repository-local
non-ignored storage:

```
evidence/f2_0070_regeneration_20260823_114505/
```

| Scene | source | perception error | achieved pre-close [rad] | right-inner at descent end [rad] | fraction | P1->P2 | TCP error [mm] | premature contact |
|---|---|---:|---:|---:|---:|---:|---:|---|
| A | perceived | 1.6134 mm | 0.0601333 | -0.488294 | 1.0 | 0.0000 mm | 0.000671 | none |
| B | perceived | 1.5767 mm | 0.0601340 | -0.346623 | 1.0 | 0.0000 mm | 0.000493 | none |
| C | perceived | 1.5145 mm | 0.0601319 | -0.419677 | 1.0 | 0.0000 mm | 0.000296 | none |
| D | perceived | 1.1339 mm | 0.0601352 | -0.452537 | 1.0 | 0.0000 mm | 0.000502 | none |

Perception error reproduces within 0.00061 mm, pre-close within 6.2e-6 rad,
TCP error within 0.00041 mm, and closure seating within 0.039 mm of the
accepted figures in §11.3. Those accepted figures are NOT superseded — §11.3
remains the original F2 validation, and this section is independent
reproducibility evidence for it.

Contact evidence was re-derived from all eight raw streams rather than taken
from the node's own logging: in every scene the first `pick_target` contact of
any stream occurs after descent completion and after the stage-2 ground-truth
check, i.e. during final closure. Finger and outer-knuckle streams stayed empty
throughout, as in the accepted run, and bilateral fingertip engagement was
recorded in all four scenes.

Three limitations are stated in that directory's `README.md` and in
`HANDOFF.md`: the clearance figures come from a REBUILT tool (the original did
not survive), `milestone_f1_truth.py`'s pre-grasp comparison is not meaningful
for a `grasp_only` run and should be fixed before F3, and scene C's console
transcript was lost to an interrupted session although its artifacts are
complete. The ~21.6-22.3 mm closure seating persists in all four scenes and
remains a separate F3 warning.

Not regenerated, and therefore still documentation-only: the strict no-object
regression and the final `0.070 rad` classical full-cycle regression.

**F2 PASS REPRODUCED — durable raw evidence regenerated.** F3 had not started
when this section was written; it has since started and FAILED at Scene A
(2026-08-23). See `HANDOFF.md`.
