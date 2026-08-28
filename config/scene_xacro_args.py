# scene_xacro_args.py — single source for deriving xacro arguments from
# scene.yaml that place the robot in the world.
#
# WHY THIS EXISTS
#   Before this file, three launch files each independently turned
#   robot.base_pose into "x y z" / "r p y" strings for the xacro's
#   base_xyz/base_rpy args: ur5e_robotiq_sim_control.launch.py (Gazebo
#   bringup), move_group.launch.py (MoveIt's own robot model), and
#   m2_cartesian_approach.launch.py (the M2 node's local robot model).
#   Harmless while all three agreed — but nothing enforced that they would
#   keep agreeing. Found during M2: elevating the sim's base to table height
#   changed nothing about reachability until move_group.launch.py was ALSO
#   updated, because MoveIt was silently still planning against a
#   ground-mounted robot model. That is exactly the sim/planner divergence
#   config/scene.yaml exists to prevent, and it had a gap: scene.yaml
#   governed poses, not the xacro args that place the robot itself.
#
# USAGE
#   Loaded via importlib.util.spec_from_file_location (not a normal package
#   import — this file lives next to scene.yaml in config/, not inside any
#   one ROS package, since all three consuming launch files are in
#   different packages). See any of the three launch files above for the
#   load snippet.

import math as _math
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def xacro_base_args(scene: dict) -> dict:
    """Return {'base_xyz': '...', 'base_rpy': '...'} xacro mapping strings
    derived from a loaded scene.yaml dict's robot.base_pose. Every launch
    file that constructs this project's robot_description must derive these
    two args from here, not recompute them independently."""
    base = scene["robot"]["base_pose"]
    return {
        "base_xyz": f"{base['x']} {base['y']} {base['z']}",
        "base_rpy": f"{base['roll']} {base['pitch']} {base['yaw']}",
    }


def _load_gripper_geometry_module():
    # scripts/lib/gripper_geometry.py, loaded the same way this file itself
    # is loaded by its callers (spec_from_file_location, not a package
    # import) — it lives in a different package's scripts/lib/, not next to
    # scene.yaml, and has no __init__.py chain to this file either.
    import importlib.util

    module_path = PROJECT_ROOT / "scripts/lib/gripper_geometry.py"
    spec = importlib.util.spec_from_file_location("gripper_geometry", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def xacro_gripper_args(scene: dict) -> dict:
    """Return {'fingertip_grasp_theta': '...'} — the actuated joint angle
    (radians, as gripper_geometry.theta_for_width computes it) at which the
    pads just touch the current scene object.

    WHY THIS EXISTS
        2026-08-12: both fingertip joints (left_finger_tip_joint,
        right_finger_tip_joint) changed from continuously mimic-tracking the
        knuckle to FIXED, at an angle chosen so the pad is exactly parallel
        (net tilt zero) AT THIS ANGLE — see robotiq_2f_85_macro.urdf.xacro's
        OVERRIDE comment for the full reasoning (JointCmdProbe traced the
        fingertip runaway to dartsim's contact-force resolution overriding a
        correctly-computed mimic command under load; a fixed joint has no DOF
        for that to act on). That angle is therefore OBJECT-SIZE-SPECIFIC: it
        must be re-derived whenever object.size changes, never hand-typed
        into the URDF. This function is the single source for that
        derivation, exactly as xacro_base_args() is for base_xyz/base_rpy.

        WIDTH SOURCE (2026-08-21, M6): the width comes from
        resolve_grasp_width_m(), which DERIVES which object.size[] entry the
        gripper closes across from grasp.approach_axis/gripper_roll. It used
        to index object.size with the hand-set object.grasp_width_axis, which
        named the wrong axis and silently derived this angle from the wrong
        dimension for the whole of M1-M5. See resolve_closing_axis() below.
    """
    width_m = resolve_grasp_width_m(scene)
    gripper_geometry = _load_gripper_geometry_module()
    theta_grasp = gripper_geometry.theta_for_width(width_m)
    return {"fingertip_grasp_theta": repr(theta_grasp)}


# --- closing-axis resolution -------------------------------------------------
#
# WHY THIS EXISTS (2026-08-21, M6)
#   scene.yaml carried `object.grasp_width_axis: 0` -- a HAND-SET index saying
#   "the fingers close across size[0]". Nothing validated it against the
#   direction the gripper actually closes in. It was wrong: with this
#   project's `grasp.approach_axis: [0,0,-1]`, the gripper closes along
#   object-frame +Y, so the squeezed dimension is size[1], not size[0].
#
#   That went unnoticed for the whole of M1-M5 because the object was a
#   45mm CUBE -- size[0] == size[1], so the index was unobservable. M6's
#   change to [0.030, 0.045, 0.045] was the first configuration where the
#   two differed, and it produced a 0.1069 rad grip-angle error against a
#   0.0235 rad tolerance. The aperture mathematics was never wrong; the
#   WIDTH HANDED TO IT was.
#
# THE DERIVATION, AND WHY IT MIRRORS static_scene_tf.cpp EXACTLY
#   The gripper's local closing axis is its base-frame X: the two pad faces
#   sit at local x = -/+ PAD_INSET_M (mesh-measured, both STLs, exact
#   negation -- see gripper_geometry.py). So the question "which object
#   dimension gets squeezed" is "where does gripper-base X point, in the
#   object's frame".
#
#   The chain, every link read from source rather than assumed:
#     object_frame -> grasp_frame : orientation_from_approach_axis(
#                                     approach_axis, gripper_roll)
#                                   -- static_scene_tf.cpp:54
#     grasp_frame  -> tool0       : IDENTITY rotation. m3_grasp.cpp:621
#                                   builds T_tcp_tool0 as a pure translation.
#     tool0        -> gripper base: R_z(gripper_rotation), from the vendored
#                                   ur_to_robotiq_adapter.urdf.xacro's
#                                   `<origin xyz="0 0 0" rpy="0 0 ${rotation}"/>`
#                                   then gripper_side_joint's rpy="0 0 0".
#
#   gripper_roll and gripper_rotation therefore turn about the SAME physical
#   axis (grasp_frame's local Z == tool0's local Z == the approach axis), so
#   they compose additively. Both are folded in below.
#
#   IMPORTANT: approach_axis is expressed in OBJECT_FRAME already (that is
#   what static_scene_tf publishes: object_frame -> grasp_frame). So the
#   object-frame closing axis falls out DIRECTLY, with no world round-trip.
#   The world-frame axis is computed too, but only for reporting and as a
#   round-trip self-check -- it is not on the critical path.
#
#   The basis math below is a line-by-line mirror of
#   static_scene_tf.cpp:orientation_from_approach_axis(), degenerate branch
#   included. If that function ever changes, THIS MUST CHANGE WITH IT. It is
#   duplicated rather than shared because the C++ node and these launch-time
#   Python helpers have no common runtime; the mirroring is asserted below
#   against the one case the project has independently verified live (the
#   [0,0,-1] / roll=0 configuration, whose contact positions were reproduced
#   from static geometry to ~10um on 2026-08-21).

# How close to a principal object axis the closing direction must be before a
# single `size[i]` is a meaningful grasp width. 1e-3 admits ~2.56 deg of
# obliquity, which inflates the effective width by w/cos(2.56deg) = 1.001*w --
# 0.05mm on a 45mm object, far under M3's own 0.0235 rad grip-angle tolerance.
# Beyond that the box's corner geometry starts deciding contact and no scalar
# width describes the grasp. Explicit and named so it can be argued with.
CLOSING_AXIS_ALIGNMENT_TOL = 1.0e-3

# The gripper's own closing axis in its base frame: +X. Established by the
# pad-face measurement in gripper_geometry.py (left pad at local x=-0.025259,
# right at +0.025259, exact mirror through X=0) and confirmed live by the M6
# contact reconstruction.
_GRIPPER_LOCAL_CLOSING_AXIS = (1.0, 0.0, 0.0)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0])


def _norm(v):
    return _math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])


def _unit(v):
    n = _norm(v)
    return (v[0] / n, v[1] / n, v[2] / n)


def _matvec(cols, v):
    """cols = (x_axis, y_axis, z_axis) as COLUMNS of the rotation matrix."""
    return tuple(
        sum(cols[k][i] * v[k] for k in range(3)) for i in range(3)
    )


def _basis_from_approach_axis(approach_axis, roll):
    """Mirror of static_scene_tf.cpp:orientation_from_approach_axis().

    Returns the rotation as its three COLUMNS (x_axis, y_axis, z_axis),
    already including the roll about the local Z the C++ applies as
    `q_base * q_roll`.
    """
    if _norm(approach_axis) < 1e-9:
        raise RuntimeError(
            "CONFIG_ERROR: grasp.approach_axis is the zero vector — cannot "
            "define an orientation from it"
        )
    z_axis = _unit(approach_axis)

    reference = (0.0, 0.0, 1.0)
    if abs(sum(z_axis[i] * reference[i] for i in range(3))) > 0.99:
        # approach_axis nearly parallel or antiparallel to the parent frame's
        # Z (true for this project's straight-down approach) — cross product
        # would be near-zero and numerically unstable. Same fallback, same
        # threshold, as the C++.
        reference = (1.0, 0.0, 0.0)

    x_axis = _unit(_cross(reference, z_axis))
    y_axis = _unit(_cross(z_axis, x_axis))

    # q_base * q_roll == R_base * R_z(roll): roll turns about the LOCAL z the
    # basis just established, i.e. about approach_axis itself.
    c, s = _math.cos(roll), _math.sin(roll)
    rx = tuple(x_axis[i] * c + y_axis[i] * s for i in range(3))
    ry = tuple(-x_axis[i] * s + y_axis[i] * c for i in range(3))
    return (rx, ry, z_axis)


def _rpy_columns(roll, pitch, yaw):
    """tf2::Quaternion::setRPY convention: R = Rz(yaw) * Ry(pitch) * Rx(roll).
    Returned as columns, matching _basis_from_approach_axis()."""
    cr, sr = _math.cos(roll), _math.sin(roll)
    cp, sp = _math.cos(pitch), _math.sin(pitch)
    cy, sy = _math.cos(yaw), _math.sin(yaw)
    return (
        (cy * cp, sy * cp, -sp),
        (cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, cp * sr),
        (cy * sp * cr + sy * sr, sy * sp * cr - cy * sr, cp * cr),
    )


def resolve_closing_axis(scene: dict) -> dict:
    """Derive which object.size[] entry the gripper actually closes across.

    Returns a dict with:
        closing_axis_object : unit 3-tuple, in object_frame
        closing_axis_world  : unit 3-tuple, in world (reporting only)
        axis_index          : int, the resolved index into object.size
        width_m             : float, object.size[axis_index]
        alignment           : float, |closing_axis_object[axis_index]|
        configured_axis     : int or None, scene.yaml's deprecated hand-set index

    Raises RuntimeError("CONFIG_ERROR: ...") if the closing direction is
    oblique to every principal object axis, or if the deprecated
    `object.grasp_width_axis` cross-check field disagrees with the derivation.
    """
    obj = scene["object"]
    grasp = scene["grasp"]
    size = [float(v) for v in obj["size"]]

    approach_axis = tuple(float(v) for v in grasp["approach_axis"])
    gripper_roll = float(grasp.get("gripper_roll", 0.0) or 0.0)

    # Mount rotation about the same axis (the xacro's `gripper_rotation` arg,
    # default 0.0). KNOWN GAP, stated rather than papered over: nothing yet
    # forces the xacro arg to equal this key, because scene.yaml does not
    # currently carry it and the xacro is not invoked from here. If a
    # non-zero mount rotation is ever introduced, it must be added to
    # scene.yaml AND threaded into the xacro from here, or this derivation
    # silently goes stale.
    mount_rotation = float((scene.get("gripper") or {}).get("mount_rotation", 0.0) or 0.0)

    cols = _basis_from_approach_axis(approach_axis, gripper_roll + mount_rotation)
    closing_object = _unit(_matvec(cols, _GRIPPER_LOCAL_CLOSING_AXIS))

    pick = obj["pick_pose"]
    obj_cols = _rpy_columns(
        float(pick["roll"]), float(pick["pitch"]), float(pick["yaw"])
    )
    closing_world = _unit(_matvec(obj_cols, closing_object))

    alignment = max(abs(c) for c in closing_object)
    axis_index = max(range(3), key=lambda i: abs(closing_object[i]))

    if alignment < 1.0 - CLOSING_AXIS_ALIGNMENT_TOL:
        raise RuntimeError(
            "CONFIG_ERROR: the gripper's closing direction is oblique to every "
            f"principal object axis. closing_axis (object frame) = "
            f"({closing_object[0]:+.6f}, {closing_object[1]:+.6f}, "
            f"{closing_object[2]:+.6f}), best alignment {alignment:.6f} < "
            f"{1.0 - CLOSING_AXIS_ALIGNMENT_TOL:.6f}. No single object.size[i] "
            "is the grasp width in this configuration — the box's corner "
            "geometry decides contact, not one face. Re-check grasp.approach_axis, "
            "grasp.gripper_roll and object.pick_pose's rpy."
        )

    configured = obj.get("grasp_width_axis")
    if configured is not None and int(configured) != axis_index:
        raise RuntimeError(
            f"CONFIG_ERROR: configured object.grasp_width_axis={int(configured)} "
            f"disagrees with the derived closing axis, which selects "
            f"object.size[{axis_index}]. The gripper closes along object-frame "
            f"axis {axis_index} (closing_axis = ({closing_object[0]:+.6f}, "
            f"{closing_object[1]:+.6f}, {closing_object[2]:+.6f}), alignment "
            f"{alignment:.6f}). Configured width would be "
            f"{size[int(configured)]:.4f} m; derived width is "
            f"{size[axis_index]:.4f} m. grasp_width_axis is DEPRECATED and kept "
            "only as this cross-check — fix it in scene.yaml to match the "
            "derivation, or fix grasp.approach_axis/grasp.gripper_roll if the "
            "derivation is what is wrong."
        )

    return {
        "closing_axis_object": closing_object,
        "closing_axis_world": closing_world,
        "axis_index": axis_index,
        "width_m": size[axis_index],
        "alignment": alignment,
        "configured_axis": None if configured is None else int(configured),
    }


def resolve_grasp_width_m(scene: dict) -> float:
    """The object dimension the gripper actually closes across, in metres.

    THE single source of grasp width. Every consumer -- xacro_gripper_args()'s
    fingertip_grasp_theta derivation, m3_grasp.launch.py's object_width_m
    parameter and its release-aperture solve -- must come through here rather
    than indexing object.size directly.
    """
    return resolve_closing_axis(scene)["width_m"]


# --- self-tests --------------------------------------------------------------
# Run at import, matching gripper_geometry.py's own _assert_parallel_jaw() /
# _assert_fixed_tip_matches_at_grasp() idiom. Pure arithmetic on synthetic
# scene dicts -- no file I/O, no gripper_geometry load -- so this costs
# nothing at launch time and cannot fail for environmental reasons. These
# exist because the bug they guard against was SILENT for five milestones.

def _synthetic_scene(size, axis, roll=0.0, approach=(0.0, 0.0, -1.0), rpy=(0.0, 0.0, 0.0)):
    return {
        "object": {
            "size": list(size),
            "grasp_width_axis": axis,
            "pick_pose": {"roll": rpy[0], "pitch": rpy[1], "yaw": rpy[2]},
        },
        "grasp": {"approach_axis": list(approach), "gripper_roll": roll},
    }


def _assert_closing_axis_resolution():
    # A -- the M6 configuration that exposed the bug. approach_axis [0,0,-1]
    # with zero roll puts the closing axis on object +Y, so the 45mm
    # dimension is the one squeezed, NOT the 30mm one the old hand-set
    # index named. `axis=1` here because that is now the corrected value in
    # scene.yaml; the mismatch case is exercised separately below.
    a = resolve_closing_axis(_synthetic_scene([0.030, 0.045, 0.045], 1))
    assert a["axis_index"] == 1, a
    assert abs(a["width_m"] - 0.045) < 1e-12, a
    assert max(abs(a["closing_axis_object"][i] - (0.0, 1.0, 0.0)[i])
               for i in range(3)) < 1e-12, a

    # B -- the historical cube. The derivation must land on the same 45mm
    # width it always effectively used, so every M1-M5 result stays valid.
    # Note the derived index is still 1: with a cube that is a distinction
    # without a difference, which is exactly why the bug hid here.
    b = resolve_closing_axis(_synthetic_scene([0.045, 0.045, 0.045], 1))
    assert b["axis_index"] == 1, b
    assert abs(b["width_m"] - 0.045) < 1e-12, b

    # C -- roll the gripper a quarter turn about the approach axis and the
    # closing direction moves to object +X, selecting the 30mm dimension.
    # This is the configuration that would actually grasp M6's narrow face.
    c = resolve_closing_axis(
        _synthetic_scene([0.030, 0.045, 0.045], 0, roll=_math.pi / 2.0))
    assert c["axis_index"] == 0, c
    assert abs(c["width_m"] - 0.030) < 1e-12, c
    assert abs(abs(c["closing_axis_object"][0]) - 1.0) < 1e-12, c

    # D -- oblique. Half a quarter-turn leaves the closing axis 45 deg from
    # both X and Y; no single size[i] is the width. Must raise, NOT silently
    # take the larger component.
    try:
        resolve_closing_axis(
            _synthetic_scene([0.030, 0.045, 0.045], 0, roll=_math.pi / 4.0))
    except RuntimeError as exc:
        assert "CONFIG_ERROR" in str(exc) and "oblique" in str(exc), exc
    else:
        raise AssertionError("oblique closing axis did not raise CONFIG_ERROR")

    # The deprecated cross-check must fire on the exact M6 misconfiguration
    # (derived 1, configured 0) rather than quietly preferring either one.
    try:
        resolve_closing_axis(_synthetic_scene([0.030, 0.045, 0.045], 0))
    except RuntimeError as exc:
        assert "CONFIG_ERROR" in str(exc) and "grasp_width_axis=0" in str(exc), exc
    else:
        raise AssertionError("axis mismatch did not raise CONFIG_ERROR")

    # A missing/None grasp_width_axis is allowed -- the field is deprecated,
    # so the derivation must stand on its own once it is eventually removed.
    s = _synthetic_scene([0.030, 0.045, 0.045], 1)
    del s["object"]["grasp_width_axis"]
    assert resolve_closing_axis(s)["axis_index"] == 1

    # Object yaw must move the WORLD axis while leaving the OBJECT axis
    # alone -- approach_axis is object-frame, so the selected size[] index
    # cannot depend on how the object is turned in the world. Guards against
    # anyone "fixing" this by transforming in the wrong direction.
    y = resolve_closing_axis(
        _synthetic_scene([0.030, 0.045, 0.045], 1, rpy=(0.0, 0.0, _math.pi / 2.0)))
    assert y["axis_index"] == 1, y
    assert max(abs(y["closing_axis_object"][i] - (0.0, 1.0, 0.0)[i])
               for i in range(3)) < 1e-12, y
    assert max(abs(y["closing_axis_world"][i] - (-1.0, 0.0, 0.0)[i])
               for i in range(3)) < 1e-12, y

    # The zero-vector guard must behave exactly like the C++ it mirrors.
    try:
        resolve_closing_axis(
            _synthetic_scene([0.030, 0.045, 0.045], 1, approach=(0.0, 0.0, 0.0)))
    except RuntimeError as exc:
        assert "CONFIG_ERROR" in str(exc) and "zero vector" in str(exc), exc
    else:
        raise AssertionError("zero approach_axis did not raise CONFIG_ERROR")


_assert_closing_axis_resolution()
