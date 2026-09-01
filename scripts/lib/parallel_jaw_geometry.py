#!/usr/bin/env python3
"""parallel_jaw_geometry.py -- closed-form geometry for gripper_model:=parallel_jaw.

Companion to gripper_geometry.py (the vendor Robotiq module), NOT a
replacement or edit of it. gripper_geometry.py's theta-based functions
describe a DIFFERENT joint (revolute, radians, one master + mimic followers)
and are untouched. This module describes gripper_jaw_joint (prismatic,
metres, one DOF, no mimic) as defined in
ur5e_robotiq_description/urdf/parallel_jaw_gripper.urdf.xacro and derived in
docs/GRIPPER_REDESIGN_DESIGN.md Section 3.5.

Every formula here was cross-checked against a live simulation (V0: FK on
the xacro-generated URDF; V1: 6330-sample recording of a real free-space
open->close->open run under gz_ros2_control/dartsim) before this module was
written -- see the pre-object-contact calibration report, 2026-08-25.
"""

import math

# --- core geometry, V0-proven exact (aperture(0)=85.000mm to 1 micron, see
# scripts/lib/gripper_geometry.py's own note on why 85mm is the right
# provenance to reuse) ---------------------------------------------------
APERTURE_FULL_OPEN_M = 0.085   # A0, matches parallel_jaw_gripper.urdf.xacro's
                                # aperture_m default and the 2F-85's published
                                # stroke (same number, new joint).
Q_MIN_M = 0.0
Q_MAX_M = 0.085

# tool0 -> gripper_base_link, Z ONLY, metres. NOT the vendor's tcp_offset
# (which is aperture-dependent, tool0 -> FINGERTIP-LINK-origin, ~0.109-
# 0.123 m across the stroke -- an order of magnitude larger, because the
# vendor's knuckle+finger+fingertip chain extends the pad far forward of
# the mounting plate). This gripper's pads carry NO Z offset of their own
# relative to gripper_base_link (parallel_jaw_gripper.urdf.xacro places
# pad_fixed_link/pad_moving_link with z=0 in every joint origin in their
# chain -- purely an X-axis design, by construction), so the pad
# contact-height coincides EXACTLY with gripper_base_link's own Z. That
# collapses the vendor's two-term correction (tcp_offset + pad_centre_offset)
# into ONE constant with no residual pad_centre_offset needed at all.
#
# FLANGE_TO_GRIPPER_BASE_Z_M: tool0 -> gripper_base_link, Z ONLY, metres (0.011 m).
# Coupler chain: tool0 -> ur_to_robotiq_link (0) -> gripper_mount_link (0.011 m)
# -> gripper_base_joint (0) -> gripper_base_link. Net: 0.011 m.
FLANGE_TO_GRIPPER_BASE_Z_M = 0.011

# FINGER_STANDOFF_Z_M: gripper_base_link -> pad contact centre along Z, metres (0.038 m).
# Derived offline 2026-08-25: Lz_geometric_min (25.4 mm) + margin_z (12.5 mm)
# = 37.9 mm, rounded to 38.0 mm (0.038 m). Provides 12.6 mm clearance between
# ur_to_robotiq_link adapter plate and 45 mm object top surface at grasp height.
FINGER_STANDOFF_Z_M = 0.038

# TCP_OFFSET_Z_M: tool0 -> pad contact centre along Z, metres (0.049 m).
# Derived: FLANGE_TO_GRIPPER_BASE_Z_M (0.011 m) + FINGER_STANDOFF_Z_M (0.038 m) = 0.049 m.
TCP_OFFSET_Z_M = FLANGE_TO_GRIPPER_BASE_Z_M + FINGER_STANDOFF_Z_M


def aperture_m(q):
    """Clear distance between the two pad faces, metres, at jaw joint value q.

    aperture(q) = A0 - q. Exact by construction (prismatic joint, no
    mimic, no theta) -- proven via FK on the generated URDF in V0, and
    confirmed live via gripper_jaw_joint's actual /joint_states values in
    V1 (e.g. q=0.055 -> 30.478mm achieved against 30.000mm commanded,
    within the 0.5mm goal_tolerance).
    """
    return APERTURE_FULL_OPEN_M - q


def q_for_aperture(aperture_m_value):
    """Inverse of aperture_m(): the jaw command (metres) that produces a
    given clear aperture. This IS the width -> q conversion -- there is no
    bisection, no theta, no mesh lookup: aperture(q) is linear and exactly
    invertible by subtraction.
    """
    return APERTURE_FULL_OPEN_M - aperture_m_value


# width->q is the same function under the name callers will actually reach
# for; kept as a separate name so intent is unambiguous at call sites.
q_for_width = q_for_aperture


def grasp_centre_offset_m(q):
    """Where the MIDPOINT between the two pad inner faces sits, in the
    gripper_base_link frame, along the closing axis (local X), at jaw
    value q. Positive return value = magnitude of the shift TOWARD THE
    FIXED JAW (which sits at local -X in parallel_jaw_gripper.urdf.xacro's
    convention).

    offset(q) = q/2. Proven exact in V0 (grasp_centre_offset_m FK check,
    err 8.7e-19 m at every sampled q) and IS ZERO at q=0 (full open) --
    the fixed and moving jaws are placed symmetrically about the base
    frame's own x=0 at rest, by construction.

    THIS DID NOT EXIST FOR THE VENDOR GRIPPER. A symmetric two-sided jaw
    has offset(q) == 0 for all q (both pads move equally), so the vendor
    pipeline never needed a term like this and none of its TCP/grasp-pose
    math (tcp_offset, pad_centre_offset) computes it. Reusing the vendor's
    TCP placement UNCHANGED for parallel_jaw is equivalent to silently
    assuming offset(q) == 0 for all q here too, which is FALSE for q > 0.
    That silent-zero assumption is the exact mechanism this function
    exists to remove.
    """
    return q / 2.0


def predicted_nominal_sweep_m(declared_clearance_m):
    """Predicted one-sided object displacement during closing, assuming the
    PRE-CLOSE arm pose was chosen using grasp_centre_offset_m(q_preclose)
    (this module's recommended strategy -- see
    preclose_pose_offset_m()) and PERFECT (zero-error) positioning.

    Derivation (algebraic, general, independent of object width w):
    let clearance = aperture_preclose - w. Centring the object in the
    pre-close aperture gives it clearance/2 of margin from EACH pad. The
    fixed pad never moves; only the moving pad's face advances as q rises
    from q_preclose to q_final. Solving for the q at which the moving
    face first reaches the object (q_contact) and comparing to q_final
    gives, after simplification, q_final - q_contact = clearance/2
    EXACTLY -- the object needs to travel only half the declared
    clearance to reach the (stationary) fixed pad, not the full amount.
    """
    return declared_clearance_m / 2.0


def predicted_worst_case_sweep_m(declared_clearance_m):
    """Upper bound if perception/placement error consumes the ENTIRE
    pre-close margin on one side (i.e. the object starts already touching
    the moving pad's initial position). Matches
    docs/GRIPPER_REDESIGN_DESIGN.md Section 3.3's own bound,
    shove <= aperture_preclose - width, exactly.
    """
    return declared_clearance_m


# --- pre-close clearance, derived (see the calibration report for the full
# derivation) --------------------------------------------------------------
# F2's own validated worst-case perception error (Euclidean, camera-frame ->
# world), from docs/HANDOFF_RGBD_PERCEPTION.md / HANDOFF.md's F2 table:
#     worst Euclidean error = 1.6136 mm, repeatability = 0.000000000 mm.
# This dominates: F2's achieved TCP execution error after Cartesian descent
# was sub-micrometre (0.000296-0.000702 mm across scenes A-D), three orders
# of magnitude smaller, so it is not a separate term here.
# Pre-close clearances (metres) for Scene-A fixed-pad-referenced precision grasp:
# The pre-close aperture split is unchanged.  The grasp TCP's fixed-side X
# clearance is deliberately maintained separately below: it must absorb the
# captured closing-axis perception bias during the vertical descent.
FIXED_SIDE_CLEARANCE_M = 0.0005
MOVING_SIDE_CLEARANCE_M = 0.0035

# Total declared clearance (both sides): preclose_aperture = width + this (4.0 mm).
DECLARED_CLEARANCE_M = FIXED_SIDE_CLEARANCE_M + MOVING_SIDE_CLEARANCE_M  # 0.0040 m = 4.0 mm

# Fixed-side clearance used solely to position the grasp TCP. Raised from
# 1.5 mm to 2.0 mm (2026-08-31) after Stage-2D case D3 (spawn offset
# +30/-30 mm from configured, spawn yaw +45 deg) showed a closing-axis
# perception-error projection of 1.576 mm -- 0.076 mm past the 1.5 mm
# budget -- producing a deterministic fixed-pad/object-top contact during
# Cartesian descent (evidence/stage2d_pose/D3_retry1_diagnostics). 2.0 mm
# restores positive margin (+0.424 mm at that same case); D1/D2 (smaller
# closing-axis projections) gain additional margin with no observed
# regression (evidence/stage2d_pose/D{1,2,3}_clearance2mm_diag). It
# changes preclose_pose_offset_m(0.030) from 0.0260 m to 0.0255 m without
# changing the pre-close aperture or final close target.
GRASP_TCP_FIXED_SIDE_CLEARANCE_M = 0.0020


def preclose_aperture_m(width_m, clearance_m=DECLARED_CLEARANCE_M):
    """preclose_aperture = object_width + declared_clearance, per the
    parallel-jaw redesign brief. NOT theta, NOT preclose_margin_rad --
    this is the entire formula; there is no residual/margin correction
    term the way the vendor pipeline needed one (see this module's
    docstring on grasp_centre_offset_m for why: this joint has no mimic
    dynamics to compensate for).
    """
    return width_m + clearance_m


def preclose_pose_offset_m(
    width_m,
    clearance_m=DECLARED_CLEARANCE_M,
    c_fixed_m=GRASP_TCP_FIXED_SIDE_CLEARANCE_M,
):
    """Arm/TCP positioning offset magnitude (metres, positive toward fixed jaw) for pre-close pose.

    Derivation:
    Fixed pad inner face is at local x = -0.0425 m relative to tool0.
    Positioning the fixed pad inner face at distance c_fixed_m from the object's
    fixed-side face (object_x - width_m / 2.0) requires:
        x_fixed_inner = tool0_x - 0.0425 = object_x - width_m / 2.0 - c_fixed_m
    ==> tool0_x = object_x + (0.0425 - width_m / 2.0 - c_fixed_m).
    Since m3_grasp computes tool0_x = object_x - (-offset_x), the positive offset
    magnitude toward the fixed jaw is:
        offset_x = 0.0425 - (width_m / 2.0) - c_fixed_m.
    For width_m = 0.030 m and c_fixed_m = 0.0020 m (2.0 mm):
        offset_x = 0.0425 - 0.0150 - 0.0020 = +0.0255 m (+25.5 mm).
    """
    return 0.0425 - (width_m / 2.0) - c_fixed_m


def final_grasp_pose_offset_m(width_m):
    """The THEORETICAL fully-centred-at-final-grasp offset:
    grasp_centre_offset_m evaluated at q_final = q_for_width(width_m).
    Reported for comparison/verification only -- preclose_pose_offset_m()
    is the one actually used for arm positioning (see its docstring).
    """
    q_final = q_for_width(width_m)
    return grasp_centre_offset_m(q_final)
