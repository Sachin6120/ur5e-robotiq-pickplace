#!/usr/bin/env python3
"""slip.py — M3's slip measurement. Ground truth only, no TF, no tcp_offset.

WHAT SLIP IS HERE

    How far the object moved RELATIVE TO THE GRIPPER between two moments.
    Not how far it moved in the world -- the whole point of a transport leg is
    that the object moves a long way in the world while not moving at all
    relative to the fingers holding it.

    M3's criterion: 20 cycles, at least 18 with slip under 5 mm, zero ejections
    or penetrations. That threshold was fixed in advance and does not move.

TWO DECISIONS THAT KEEP THE NUMBER HONEST

    1. Poses come from Gazebo's own link/model pose topics. Never TF. TF
       reports what the system BELIEVES, and belief is the thing under test --
       a grasp that has silently failed still has a perfectly consistent TF
       tree, because MoveIt's attachObject is collision bookkeeping and nothing
       more. This has been the project's rule since M0 and it applies hardest
       here, where the measurement decides a pass/fail.

    2. Slip is measured relative to the FLANGE LINK, not to grasp_tcp.

       The flange -> TCP transform is constant during a cycle, so it cancels
       exactly in a before/after difference:

           slip = | R_f(t1)^T (p_o(t1) - p_f(t1))
                   - R_f(t0)^T (p_o(t0) - p_f(t0)) |

       Any fixed offset applied to both samples subtracts out. So this number
       does not depend on tcp_offset being right -- which matters, because
       tcp_offset WAS wrong by 18 mm until 2026-08-11 and every trial before
       that fix was silently mispositioned. A slip measure that inherited that
       error would have reported the geometry bug as friction.

       Rotation of the object in the fingers is NOT captured by this scalar.
       That is deliberate: the criterion is written in millimetres. Log the
       quaternion alongside if rotation ever needs to enter the criterion, but
       do not quietly fold it into a distance.

WHEN TO SAMPLE

    Not immediately after the stall. The gripper is still settling and the
    object is still seating; sampling into that produced a 6 mm error once
    already (05_measure_gripper_geometry.sh, the 0.2 rad anomaly) and 6 mm is
    larger than the criterion it would be judged against.

    Take the baseline through a WINDOWED quiescence check -- gz_settle_pose_
    windowed in scripts/lib/gz_settle.sh -- not a fixed sleep and not a single
    below-threshold sample. gz_settle_pose's non-windowed form produced false
    quiescence before and was replaced for exactly this reason.

SELF-TEST
    python3 scripts/lib/slip.py --selftest

    Runs without a simulator. The load-bearing case is the third one: move the
    gripper AND the object together as one rigid body, through a large
    translation and rotation, and require slip == 0. A measure that reports
    slip when nothing slipped would fail M3 cycles for the wrong reason, and
    that is a harder failure to notice than the reverse.
"""

import argparse
import math
import sys


def quat_to_matrix(x, y, z, w):
    """Rotation matrix from a quaternion, as row-major nested tuples.

    Normalises first: Gazebo's quaternions are close to unit but not exactly,
    and an unnormalised quaternion silently scales the result.
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        raise ValueError("degenerate quaternion (norm ~ 0)")
    x, y, z, w = x / n, y / n, z / n, w / n
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def transpose_apply(R, v):
    """R^T v — express a world vector in the frame whose rotation is R."""
    return tuple(sum(R[k][i] * v[k] for k in range(3)) for i in range(3))


def object_in_flange(flange_pose, object_pose):
    """Object position expressed in the flange frame.

    Each pose is (x, y, z, qx, qy, qz, qw) in the SAME world frame — both
    straight from Gazebo, never mixed with a TF lookup.
    """
    fx, fy, fz, fqx, fqy, fqz, fqw = flange_pose
    ox, oy, oz = object_pose[0], object_pose[1], object_pose[2]
    R = quat_to_matrix(fqx, fqy, fqz, fqw)
    return transpose_apply(R, (ox - fx, oy - fy, oz - fz))


def slip_m(flange_t0, object_t0, flange_t1, object_t1):
    """Straight-line distance the object moved relative to the gripper, metres."""
    a = object_in_flange(flange_t0, object_t0)
    b = object_in_flange(flange_t1, object_t1)
    return math.sqrt(sum((b[i] - a[i]) ** 2 for i in range(3)))


# ---------------------------------------------------------------------------


def _selftest():
    fails = 0

    def check(name, got, want, tol):
        nonlocal fails
        ok = abs(got - want) <= tol
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}: got {got:.9f}, want {want:.9f}")
        if not ok:
            fails += 1

    ident = (0.0, 0.0, 0.0, 1.0)

    # 1. Nothing moves -> no slip.
    f0 = (0.4, 0.0, 0.9) + ident
    o0 = (0.4, 0.0, 0.85) + ident
    check("static scene", slip_m(f0, o0, f0, o0), 0.0, 1e-12)

    # 2. Object slides 3 mm in the gripper while the gripper holds still.
    o1 = (0.403, 0.0, 0.85) + ident
    check("3 mm slide, gripper static", slip_m(f0, o0, f0, o1), 0.003, 1e-12)

    # 3. THE ONE THAT MATTERS. Rigid-body transport: rotate the gripper 37
    #    degrees about z and translate it a long way, carrying the object with
    #    it exactly. Nothing slipped, so the measure must return zero. If a
    #    world-frame delta were being reported instead of a relative one, this
    #    would come back around 0.5 m and every transport leg would fail.
    ang = math.radians(37.0)
    c, s = math.cos(ang / 2), math.sin(ang / 2)
    q = (0.0, 0.0, s, c)                      # rotation about world z
    R = quat_to_matrix(*q)
    t = (0.35, -0.42, 0.11)                   # arbitrary translation

    def rigid(p):
        v = tuple(sum(R[i][k] * p[k] for k in range(3)) for i in range(3))
        return tuple(v[i] + t[i] for i in range(3))

    f2 = rigid(f0[:3]) + q
    o2 = rigid(o0[:3]) + q
    check("rigid transport, no slip", slip_m(f0, o0, f2, o2), 0.0, 1e-12)

    # 4. Rigid transport PLUS a real 4 mm slide along the flange's own x.
    #    The transport must not contaminate the 4 mm.
    o0_in_f = object_in_flange(f0, o0)
    slid = (o0_in_f[0] + 0.004, o0_in_f[1], o0_in_f[2])
    Rf = quat_to_matrix(*f2[3:])
    world = tuple(
        f2[i] + sum(Rf[i][k] * slid[k] for k in range(3)) for i in range(3)
    )
    check("transport + 4 mm slide", slip_m(f0, o0, f2, world + q), 0.004, 1e-12)

    # 5. Unnormalised quaternion must not change the answer.
    f0_scaled = f0[:3] + tuple(2.5 * v for v in f0[3:])
    check("unnormalised quat", slip_m(f0_scaled, o0, f0_scaled, o1), 0.003, 1e-12)

    print(f"\n{'ALL PASS' if fails == 0 else f'{fails} FAILURE(S)'}")
    return 1 if fails else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--flange-t0", nargs=7, type=float, metavar="V")
    ap.add_argument("--object-t0", nargs=7, type=float, metavar="V")
    ap.add_argument("--flange-t1", nargs=7, type=float, metavar="V")
    ap.add_argument("--object-t1", nargs=7, type=float, metavar="V")
    ap.add_argument("--threshold-m", type=float, default=0.005,
                    help="M3 criterion. Fixed in advance; do not tune to fit.")
    args = ap.parse_args()

    if args.selftest:
        return _selftest()

    need = (args.flange_t0, args.object_t0, args.flange_t1, args.object_t1)
    if any(v is None for v in need):
        print(
            "CONFIG_ERROR: all four poses required, each as "
            "x y z qx qy qz qw from Gazebo's own pose topics. "
            "Never from TF.",
            file=sys.stderr,
        )
        return 2

    d = slip_m(*need)
    verdict = "PASS" if d <= args.threshold_m else "FAIL"
    print(f"slip_m={d:.6f} threshold_m={args.threshold_m:.6f} verdict={verdict}")
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
