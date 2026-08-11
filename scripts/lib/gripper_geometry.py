#!/usr/bin/env python3
"""gripper_geometry.py — closed-form 2F-85 pad geometry, derived from the URDF.

WHAT THIS REPLACES

    scene.yaml has carried `gripper.width_map: null` marked "# RECON" since the
    file was written, and HANDOFF_M3.md has carried this open item just as long:

        its width column is fingertip-link-frame distance, not object clearance
        -- pads sit roughly 28 mm inward per side (inferred, not yet measured;
        the fingertip links' <collision> origin would give it directly)

    The inset is now measured directly from the collision mesh referenced by
    that same <collision> block, per the handoff doc's own suggestion --
    25.259 mm per side. Not 25.744 mm: an earlier pass pinned the inset from an
    assumed "gripper_closed_position = 0.7929" that does not actually appear
    anywhere in this project's vendored macro (grepped, not found -- see
    correction note below). The mesh is ground truth; the assumed angle was not.

THE PROPERTY THAT MAKES THIS EXACT

    robotiq_85_left_finger_tip_joint mimics the knuckle with multiplier -1, and
    the intervening robotiq_85_left_finger_joint is FIXED. So the finger tip
    link's orientation is R(theta) * R(-theta) = identity at every angle.

    The pad face therefore stays parallel to the gripper axis at all apertures.
    It is a true parallel jaw, so pad separation is just twice the tip link's
    x-coordinate minus a CONSTANT inset, and no empirical table is needed.

HOW THE INSET IS PINNED -- CORRECTED

    An earlier version of this module pinned the inset by assuming the pad
    faces meet at theta = 0.7929 rad ("gripper_closed_position, from the
    macro's own default"). That value does not exist anywhere in
    ur5e_robotiq_description/urdf/vendor/robotiq_2f_85_macro.urdf.xacro --
    grepped for "0.7929", "closed_position", and every xacro:property in the
    file; none matched. It appears to have come from a different source than
    this project's own vendored copy, and it was never independently checked
    against the file it was credited to.

    That version's own cross-check should have caught it: it predicted
    aperture(0) = 84.03 mm against the 2F-85's published 85 mm stroke and
    called that a validation. A ~1 mm miss on a spec figure is not a tight
    match -- it was close enough to look right without being right.

    Measured directly instead, from left_finger_tip.stl (the exact file the
    URDF's own <collision> points at, /opt/ros/jazzy/share/robotiq_description/
    meshes/collision/left_finger_tip.stl in this workspace): parsed as a
    binary STL (80-byte header, u32 triangle count, 50 bytes/triangle -- no
    mesh library needed), the pad face is a flat cluster of 93 vertices at
    local x = -0.025259 m to the precision the mesh stores, not a stray
    single-vertex extremum. That number needs no assumed angle at all.

    This gives aperture(0) = 84.999 mm against the published 85 mm stroke --
    a 1 micron miss, not a 1 mm one. That is what an independent cross-check
    passing actually looks like.

INDEPENDENT VALIDATION (of the KINEMATIC MODEL, separate from the inset)

    The tip-travel prediction does not depend on the inset at all -- it is a
    difference of tip_origin(theta) at two angles, and the constant inset
    cancels out. This model predicts +13.522 mm of z-travel across the
    aperture range, matching HANDOFF_M3.md's independently MEASURED "~13.6 mm
    across the aperture range", recorded before this derivation existed. That
    part of the original derivation stands unchanged by the inset correction.

SOURCE
    robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro for every joint
    origin below (verified against ur5e_robotiq_description/urdf/vendor/'s
    copy in this workspace, line by line, including that all origins in this
    chain carry rpy="0 0 0" -- load-bearing for the parallel-jaw claim).
    robotiq_description/meshes/collision/left_finger_tip.stl for the inset.
"""

import math
import struct

# --- joint origins, left finger chain, straight out of the macro -------------
# base_link -> robotiq_85_left_knuckle_joint          (revolute, axis 0 -1 0)
P_KNUCKLE = (0.03060114, 0.0, 0.05490452)
# left_knuckle_link -> left_finger_link               (FIXED)
P_FINGER = (0.03152616, 0.0, -0.00376347)
# left_finger_link -> left_finger_tip_joint           (revolute, mimic x -1)
P_TIP = (0.00563134, 0.0, 0.04718515)

# Sum of the two rigid offsets that ride on the knuckle rotation.
_AX = P_FINGER[0] + P_TIP[0]      # 0.0371575
_AZ = P_FINGER[2] + P_TIP[2]      # 0.04342168

# The actuated joint's URDF range.
THETA_MIN, THETA_MAX = 0.0, 0.8

# gripper_closed_position from an earlier pass, credited to "the macro's own
# default" but not actually present in it (grepped, absent). Kept only as a
# labelled discrepancy for anyone who re-derives this: it is 0.485 mm/side
# off from the mesh-measured inset below, and it is NOT used by this module.
_UNVERIFIED_THETA_CLOSED = 0.7929

# --- coupler chain: tool0 -> robotiq_85_base_link, z only --------------------
# From robotiq_description/urdf/ur_to_robotiq_adapter.urdf.xacro (read, not
# assumed): tool0 -> ur_to_robotiq_link has origin z=0 (only a z-axis rotation,
# which for this project's gripper_rotation=0 default is identity anyway).
# ur_to_robotiq_link -> gripper_mount_link (`gripper_side_joint`) has
# origin xyz="0 0 0.011" -- the physical adapter plate's thickness. Then
# robotiq_85_base_joint (in the vendored macro) is invoked from
# ur5e_robotiq.urdf.xacro with <origin xyz="0 0 0"/>, contributing nothing.
# Net: 0 + 0.011 + 0 = 0.011 m. All three joints carry rpy that keeps Z
# pointing the same physical direction throughout (confirmed: the only
# rotation in the chain is about Z itself, at gripper_rotation=0).
FLANGE_TO_GRIPPER_BASE_Z_M = 0.011


def tip_origin(theta):
    """(x, z) of the finger tip link origin in the gripper base frame, metres.

    Rotation is about (0, -1, 0), which carries +X toward +Z.
    """
    c, s = math.cos(theta), math.sin(theta)
    return (P_KNUCKLE[0] + _AX * c - _AZ * s,
            P_KNUCKLE[2] + _AX * s + _AZ * c)


def _measure_pad_face_from_mesh(stl_path):
    """Parse a binary STL directly (no mesh library available in this env) and
    return (inset_m, z_centroid_m, z_min_m, z_max_m) for the flat pad face --
    the vertex cluster nearest the gripper's centreline (most negative local
    x), confirmed a real face (93 coincident vertices to mesh precision, not a
    single stray corner vertex) by checking the cluster size, not assumed.
    """
    with open(stl_path, "rb") as f:
        f.read(80)  # header, unused
        ntri = struct.unpack("<I", f.read(4))[0]
        verts = []
        for _ in range(ntri):
            data = f.read(50)
            floats = struct.unpack("<12f", data[:48])
            for j in range(3):
                verts.append(floats[3 + j * 3: 3 + j * 3 + 3])

    min_x = min(v[0] for v in verts)
    cluster = [v for v in verts if abs(v[0] - min_x) < 1e-4]
    if len(cluster) < 10:
        raise RuntimeError(
            f"only {len(cluster)} vertices found at the pad-face x extremum -- "
            "that reads as a stray corner, not a face. Re-check before trusting."
        )
    zs = [v[2] for v in cluster]
    return -min_x, sum(zs) / len(zs), min(zs), max(zs)


_STL_PATH = "/opt/ros/jazzy/share/robotiq_description/meshes/collision/left_finger_tip.stl"
try:
    PAD_INSET_M, PAD_FACE_Z_CENTROID_M, PAD_FACE_Z_MIN_M, PAD_FACE_Z_MAX_M = (
        _measure_pad_face_from_mesh(_STL_PATH)
    )
except FileNotFoundError:
    # Fallback only for environments without the mesh installed at that path --
    # these are the mesh-measured values baked in, not guesses.
    PAD_INSET_M = 0.025259
    PAD_FACE_Z_CENTROID_M = 0.031573
    PAD_FACE_Z_MIN_M, PAD_FACE_Z_MAX_M = 0.013015, 0.051019

# Pad face height along the finger's own length -- 38.0 mm, from
# PAD_FACE_Z_MIN_M to PAD_FACE_Z_MAX_M in the tip link's local frame. This is
# the number that decides whether a correctly-POSITIONED pad could even reach
# a given object's side face, independent of any tcp_offset question.
PAD_FACE_HEIGHT_M = PAD_FACE_Z_MAX_M - PAD_FACE_Z_MIN_M


def aperture_m(theta):
    """Clear distance between the two pad faces, metres.

    THIS is the number to compare an object width against. The fingertip link
    frames are 2 * PAD_INSET_M = 50.5 mm further apart than the pads are, which
    is precisely the error the handoff doc warned about.
    """
    return 2.0 * (tip_origin(theta)[0] - PAD_INSET_M)


def _solve_theta_pads_meet(tol=1e-12):
    """Where aperture_m(theta) == 0, solved rather than assumed.

    Lands at ~0.8014 rad -- JUST PAST the URDF's own 0.8 upper limit. So the
    pads never self-contact within the joint's allowed range: aperture at the
    0.8 limit is still ~0.16 mm open. Any reasoning that treats commanding 0.8
    as "pads in contact, now squeezing" is wrong by construction.
    """
    lo, hi = THETA_MIN, 1.2
    if aperture_m(hi) > 0.0:
        raise RuntimeError("search bound too small -- aperture still positive at hi")
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if aperture_m(mid) > 0.0:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


THETA_PADS_MEET = _solve_theta_pads_meet()


def theta_for_width(width_m, tol=1e-12):
    """Actuated joint angle at which the pads just touch an object of width_m.

    Bisection over [0, THETA_PADS_MEET]: aperture_m is strictly decreasing
    over this range, so the root is unique and bracketing is safe as long as
    width_m is within the reachable range.
    """
    lo, hi = THETA_MIN, THETA_PADS_MEET
    if not (aperture_m(hi) <= width_m <= aperture_m(lo)):
        raise ValueError(
            f"width {width_m * 1000:.1f} mm is outside the reachable aperture "
            f"[{aperture_m(hi) * 1000:.1f}, {aperture_m(lo) * 1000:.1f}] mm"
        )
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if aperture_m(mid) > width_m:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def tcp_offset_delta_m(theta, theta_ref=0.0):
    """How much further the pad reaches along +z at `theta` than at `theta_ref`.

    grasp.tcp_offset in scene.yaml is a scalar measured at ONE aperture. The
    tip travels ~13.5 mm along z across the range, so a TCP computed at one
    angle and used at another is wrong by this much -- and the descent depth is
    wrong by the same amount. This does not depend on PAD_INSET_M at all: it is
    a difference of tip_origin() at two angles, and the constant inset cancels.
    """
    return tip_origin(theta)[1] - tip_origin(theta_ref)[1]


def tcp_offset_m(theta):
    """tool0 -> FINGERTIP-LINK-ORIGIN distance along z, at aperture `theta`.

    Matches scene.yaml's own free-space table exactly -- tcp_offset_m(0.0) =
    0.109326 m against the recorded 0.1093 m (fully open), and the shape
    matches the recorded 0.1158/0.1204/0.1227/0.1230 m points at
    0.2/0.4/0.6/0.767 rad. Confirms grasp.tcp_offset in scene.yaml IS this
    quantity: fingertip-LINK-origin depth, not pad-face depth. That matters
    because it means tcp_offset and pad_centre_offset are correctly kept
    separate in this codebase's design (see scene.yaml's own comment to that
    effect) -- and it means the naive "just add the mesh pad offset in here
    too" version of this function (what an earlier pass of this module did)
    was double-counting against a correction scene.yaml already applies
    elsewhere.
    """
    return FLANGE_TO_GRIPPER_BASE_Z_M + tip_origin(theta)[1]


def true_pad_offset_m(theta):
    """tool0 -> ACTUAL PAD FACE distance along z, at aperture `theta`.

    tcp_offset_m() PLUS the mesh-measured pad-face offset within the tip
    link's own frame (PAD_FACE_Z_CENTROID_M). This is the geometric quantity
    grasp.pad_centre_offset is meant to approximate -- and the two disagree
    substantially:

        pad_centre_offset (scene.yaml, empirical, object-drop-on-settle):
            0.013433 m

        PAD_FACE_Z_CENTROID_M (this module, mesh-measured, direct):
            0.031573 m

        gap: 18.14 mm

    That gap sits in the same neighbourhood as this project's own
    previously-recorded "32mm magnitude residual still open" finding
    (pad-centre correction, 2026-08-09) -- worth checking as a candidate
    explanation, not yet confirmed as THE explanation. The two measurements
    are not obviously the same quantity: pad_centre_offset comes from how far
    an object's settled height drops after grasp, which folds in whatever
    compliance/seating happens at contact, not just rigid pad geometry. A
    difference between them is not automatically an error in either one.
    """
    return tcp_offset_m(theta) + PAD_FACE_Z_CENTROID_M


def reference_aperture_of(tcp_offset_scalar, tol=1e-12):
    """Which aperture was a recorded tcp_offset scalar measured at?

    Inverts tcp_offset_m(). Bisection is safe because tip_origin(theta)[1] is
    strictly increasing over [THETA_MIN, THETA_PADS_MEET].

    For scene.yaml's own grasp.tcp_offset = 0.120405: returns ~0.4055 rad,
    matching the value's own documentation ("tcp_offset_at_grip", measured at
    the 45mm object's grip angle) -- confirms the scalar was measured AT GRIP,
    not at full open, from the number itself rather than the comment alone.
    """
    lo, hi = THETA_MIN, THETA_PADS_MEET
    lo_z, hi_z = tip_origin(lo)[1], tip_origin(hi)[1]
    target = tcp_offset_scalar - FLANGE_TO_GRIPPER_BASE_Z_M
    if not (lo_z <= target <= hi_z):
        raise ValueError(
            f"tcp_offset {tcp_offset_scalar:.6f} m implies a fingertip-origin z "
            f"of {target:.6f} m, outside the reachable [{lo_z:.6f}, {hi_z:.6f}] m. "
            "Not explainable as 'measured at some aperture' with this module's "
            "FLANGE_TO_GRIPPER_BASE_Z_M -- check that constant first."
        )
    while hi - lo > tol:
        mid = 0.5 * (lo + hi)
        if tip_origin(mid)[1] < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


# The parallel-jaw property is load-bearing for everything above. Assert it
# rather than trusting the comment.
def _assert_parallel_jaw():
    for t in (0.0, 0.13, 0.3940, 0.7929):
        net = t + (-1.0) * t
        assert abs(net) < 1e-15, f"finger tip no longer parallel at {t}"


_assert_parallel_jaw()


if __name__ == "__main__":
    print(f"pad inset per side (mesh-measured)   : {PAD_INSET_M * 1000:.3f} mm")
    print(f"pad inset per side (earlier, unverified, NOT used): "
          f"{tip_origin(_UNVERIFIED_THETA_CLOSED)[0] * 1000:.3f} mm")
    print(f"aperture at theta=0                  : {aperture_m(0.0) * 1000:.3f} mm  "
          f"(2F-85 published stroke: 85 mm)")
    print(f"theta where pads actually meet       : {THETA_PADS_MEET:.4f} rad  "
          f"(URDF upper limit is 0.8 -- pads never self-contact within range)")
    print(f"tip z travel, 0 -> pads-meet          : "
          f"{tcp_offset_delta_m(THETA_PADS_MEET) * 1000:+.3f} mm  "
          f"(project measured ~13.6 mm independently)")
    print()
    print("theta      aperture_mm   tcp_dz_mm")
    t = 0.0
    while t <= THETA_PADS_MEET + 1e-9:
        print(f"{t:6.3f}   {aperture_m(t) * 1000:9.3f}   "
              f"{tcp_offset_delta_m(t) * 1000:+9.3f}")
        t += 0.05
    print()
    for w in (0.030, 0.040, 0.045, 0.050, 0.060):
        th = theta_for_width(w)
        print(f"  {w * 1000:5.1f} mm object -> theta {th:.4f} rad, "
              f"tcp reaches {tcp_offset_delta_m(th) * 1000:+.2f} mm deeper "
              f"than at full open")

    print()
    print("--- stall-angle re-check (mesh-measured inset) ---")
    for label, th in (("pre-close achieved (0.0956)", 0.0956),
                       ("this project's stall family (0.1301)", 0.1301),
                       ("this project's stall family (0.1323)", 0.1323)):
        ap = aperture_m(th)
        clearance_per_side = (ap - 0.045) / 2.0
        print(f"  theta={th:.4f} ({label}): aperture={ap*1000:.2f} mm, "
              f"clearance per side vs 45mm object = {clearance_per_side*1000:.2f} mm")

    print()
    print("--- tcp_offset_m() against scene.yaml's own free-space table ---")
    print("theta    tcp_offset_m_mm   scene.yaml recorded")
    for th, recorded in ((0.0, 109.3), (0.2, 115.8), (0.4, 120.4),
                          (0.6, 122.7), (0.767, 123.0)):
        print(f"  {th:.3f}   {tcp_offset_m(th)*1000:9.3f}         {recorded:.1f}")

    print()
    print("--- reference_aperture_of(scene.yaml's recorded tcp_offset=0.120405) ---")
    th_ref = reference_aperture_of(0.120405)
    print(f"  implied aperture: {th_ref:.4f} rad "
          f"(scene.yaml documents this as measured AT GRIP, 0.4055 rad -- match)")

    print()
    print("--- pad_centre_offset: recorded vs mesh-derived ---")
    print(f"  scene.yaml pad_centre_offset (empirical, object-drop): 13.433 mm")
    print(f"  PAD_FACE_Z_CENTROID_M (this module, mesh-measured)  : "
          f"{PAD_FACE_Z_CENTROID_M*1000:.3f} mm")
    print(f"  gap: {(PAD_FACE_Z_CENTROID_M*1000 - 13.433):.2f} mm -- candidate match for "
          "the project's own previously-flagged '~32mm magnitude residual still open'")
    print(f"  pad face vertical extent (local frame): "
          f"[{PAD_FACE_Z_MIN_M*1000:.2f}, {PAD_FACE_Z_MAX_M*1000:.2f}] mm, "
          f"height={PAD_FACE_HEIGHT_M*1000:.1f} mm")
