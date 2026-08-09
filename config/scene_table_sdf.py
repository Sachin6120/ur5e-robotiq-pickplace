# scene_table_sdf.py — single source for deriving the world's table
# collision geometry from scene.yaml's `table:` block.
#
# WHY THIS EXISTS
#   config/scene.yaml's table.size/table.pose/surface_z have described a
#   physical support surface since this file's earliest version, but
#   nothing ever read them to put a matching collision surface in the
#   world — the Gazebo world this project launches (empty.sdf) is
#   genuinely empty. M3's first live grasp test exposed the gap directly:
#   an object spawned at object.pick_pose (which assumes table.surface_z)
#   fell straight through to the floor. See docs/HANDOFF_M3.md, "Where
#   this left off — the missing table is one cause, not two anomalies."
#
#   Same treatment scene_xacro_args.py already gives robot.base_pose: one
#   function derives world geometry from scene.yaml, every consumer calls
#   it instead of hand-copying a constant or spawning a one-off temporary
#   platform (as M3's Test 2 did as a workaround).
#
# USAGE
#   Loaded via importlib.util.spec_from_file_location (not a normal
#   package import — this file lives next to scene.yaml in config/, not
#   inside any one ROS package), same pattern as scene_xacro_args.py.

def table_sdf(scene: dict, model_name: str = "table") -> str:
    """Return a single-line, static-box SDF model string for scene.yaml's
    table block. Includes an embedded <pose> for the SDF to be
    self-describing, but that embedded pose is NOT sufficient on its own —
    see table_pose_args() below. Flattened to one line already (no embedded
    newlines) — protobuf text-format request fields used by some spawn
    paths cannot cross a line boundary, the same hazard
    scripts/04_mimic_contact_probe.sh documents for its own inline object
    SDF."""
    table = scene["table"]
    sx, sy, sz = (float(v) for v in table["size"])
    pose = table["pose"]
    px, py, pz = float(pose["x"]), float(pose["y"]), float(pose["z"])
    roll, pitch, yaw = float(pose["roll"]), float(pose["pitch"]), float(pose["yaw"])
    return (
        "<?xml version='1.0'?><sdf version='1.9'>"
        f"<model name='{model_name}'>"
        "<static>true</static>"
        f"<pose>{px} {py} {pz} {roll} {pitch} {yaw}</pose>"
        "<link name='link'>"
        f"<collision name='c'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>"
        f"<visual name='v'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></visual>"
        "</link></model></sdf>"
    )


def table_pose_args(scene: dict) -> list:
    """Return ['-x', ..., '-y', ..., '-z', ..., '-R', ..., '-P', ..., '-Y', ...]
    for `ros_gz_sim create`, derived from scene.yaml's table.pose.

    REQUIRED IN ADDITION TO table_sdf()'s embedded <pose>, not redundant
    with it: verified live 2026-08-09 that `ros_gz_sim create -string`
    ignores a model-level <pose> embedded in the SDF string entirely and
    spawns at (0,0,0) unless -x/-y/-z/-R/-P/-Y are ALSO passed on the
    command line — confirmed via `gz topic -e -t /world/<world>/pose/info`
    showing the table's spawned position as empty/zero despite
    table_sdf()'s <pose>0.55 0.00 0.375 ...</pose>. This is specific to the
    `create` executable's own argument handling, not a property of SDF or
    of Gazebo generally — the `gz service .../create` EntityFactory path
    scripts/04_mimic_contact_probe.sh and friends use DOES honor an
    embedded <pose>, which is why those scripts never needed this."""
    pose = scene["table"]["pose"]
    return [
        "-x", str(float(pose["x"])),
        "-y", str(float(pose["y"])),
        "-z", str(float(pose["z"])),
        "-R", str(float(pose["roll"])),
        "-P", str(float(pose["pitch"])),
        "-Y", str(float(pose["yaw"])),
    ]
