#!/usr/bin/env python3
"""m0c_eval.py — evaluate the M0-C contact-load result.

Called by scripts/m0_verify.sh. Separate file rather than an inline heredoc so
it can be unit-tested against synthetic samples without a running simulator.

argv:
  1 joints_closed.txt   "<name> <position> <velocity>" per line, Gazebo ground truth
  2 mimic_spec.txt      "<name> <multiplier> <offset>" per line, from robot_description
  3 actuated            master joint name
  4 commanded           commanded master angle, rad
  5 box_before          "x y z"
  6 box_after           "x y z"
  7 min_shortfall       rad
  8 max_box_disp        m
  9 max_follower_err    rad
 10 floor_z             m
 11 stalled             "true" | "false" | ""
 12 reached_goal        "true" | "false" | ""

exit 0 = M0-C contact criterion met, 1 = not met
"""
import sys


def main() -> int:
    (js_p, spec_p, actuated, commanded, box_b, box_a,
     min_short, max_disp, max_ferr, floor_z, stalled, reached) = sys.argv[1:13]

    commanded = float(commanded)
    min_short, max_disp = float(min_short), float(max_disp)
    max_ferr, floor_z = float(max_ferr), float(floor_z)

    joints = {}
    for line in open(js_p):
        f = line.split()
        if len(f) >= 2:
            joints[f[0]] = (float(f[1]),
                            None if len(f) < 3 or f[2] == 'NA' else float(f[2]))

    spec = {}
    for line in open(spec_p):
        f = line.split()
        if len(f) == 3:
            spec[f[0]] = (float(f[1]), float(f[2]))

    fail = 0

    def ok(m):
        print(f"  [PASS] {m}")

    def bad(m):
        nonlocal fail
        print(f"  [FAIL] {m}")
        fail = 1

    # --- 1. shortfall: did the linkage stop short of command? ---------------
    if actuated not in joints:
        bad(f"master joint '{actuated}' absent from Gazebo ground truth")
        print("       joints Gazebo reports:")
        for n in sorted(joints):
            print(f"         {n}")
        return 1

    achieved = joints[actuated][0]
    shortfall = commanded - achieved
    print(f"  commanded  : {commanded:.4f} rad")
    print(f"  achieved   : {achieved:.4f} rad")
    print(f"  shortfall  : {shortfall:+.4f} rad   (need > {min_short})")

    if shortfall > min_short:
        ok(f"linkage stalled {shortfall:.4f} rad short — contact opposes it")
    else:
        bad(f"shortfall {shortfall:.4f} rad is below {min_short} — the linkage "
            "drove to command THROUGH the object")
        print("       Contact is not opposing the mimic override.")
        print("       Do NOT proceed to M3 friction tuning; see report §3.5")
        print("       Consequence 1 and work the physics-engine decision.")

    # --- 2. controller's own stall report -----------------------------------
    print(f"  controller : stalled={stalled or '<unparsed>'} "
          f"reached_goal={reached or '<unparsed>'}")
    if stalled == 'true':
        ok("controller independently reported stalled=true")
    elif stalled == 'false':
        bad("controller reported stalled=false — it believes it reached the goal")
    else:
        print("  [WARN] could not parse stalled from the action result; "
              "shortfall above is the deciding signal")

    if reached == 'true' and shortfall > min_short:
        bad("contradiction: reached_goal=true but Gazebo shows a large shortfall. "
            "Reconcile before trusting either source.")

    # --- 3. object retained? -------------------------------------------------
    try:
        bx, by, bz = (float(v) for v in box_b.split())
        ax, ay, az = (float(v) for v in box_a.split())
    except (ValueError, AttributeError):
        bad("could not read box pose before/after — object retention unverified")
        return 1

    disp = ((ax - bx) ** 2 + (ay - by) ** 2 + (az - bz) ** 2) ** 0.5
    print(f"  box before : ({bx:.3f}, {by:.3f}, {bz:.3f})")
    print(f"  box after  : ({ax:.3f}, {ay:.3f}, {az:.3f})")
    print(f"  displacement: {disp:.4f} m   (need < {max_disp})")

    if disp < max_disp:
        ok(f"object retained, moved {disp*1000:.1f} mm")
    else:
        bad(f"object moved {disp*1000:.1f} mm — ejected or driven through")

    if az < floor_z:
        bad(f"object at z={az:.3f} is at/below floor ({floor_z}) — it was dropped")
    else:
        ok(f"object clear of the floor (z={az:.3f})")

    # M3's slip criterion is 5 mm. Settling during closure is not transport
    # slip, but it is worth surfacing every run rather than only at M3.
    drop = bz - az
    if drop > 0.005:
        print(f"  [WARN] object settled {drop*1000:.1f} mm downward during closure. "
              f"M3's slip criterion is 5 mm.")
        print("         Settling as the pads seat is not the same as slip under")
        print("         transport load — but if the object is sliding BETWEEN the")
        print("         pads, M3 has a problem that predates friction tuning.")
        print("         Worth one GUI look before M3. See report §3.5.")

    # --- 4. followers still consistent under load ---------------------------
    print(f"\n  followers vs multiplier x master (tolerance {max_ferr} rad):")
    # `seen` is tracked separately from `worst`. Using `worst_n is None` as the
    # "found any?" test is wrong: with err > worst and worst starting at 0.0, a
    # set of perfectly-tracking followers (err == 0.0 for all) never updates
    # worst_n, and perfect tracking gets reported as "no follower joints found".
    # Caught by the drove-through test case, where every error was exactly zero.
    seen = 0
    worst, worst_n = -1.0, None
    for name, (mult, off) in sorted(spec.items()):
        if name not in joints:
            bad(f"{name} absent from Gazebo ground truth")
            continue
        seen += 1
        got = joints[name][0]
        exp = mult * achieved + off
        err = abs(got - exp)
        if err > worst:
            worst, worst_n = err, name
        flag = "  <-- exceeds tolerance" if err > max_ferr else ""
        print(f"    {name:<42} got {got:+.5f}  exp {exp:+.5f}  err {err:.5f}{flag}")

    if seen == 0:
        bad("no follower joints found in Gazebo ground truth")
    elif worst > max_ferr:
        bad(f"worst follower error {worst:.5f} rad ({worst_n}) exceeds {max_ferr}")
    else:
        ok(f"all followers within {max_ferr} rad (worst {worst:.5f}, {worst_n})")
        if worst > 0.01:
            print(f"  [note] {worst_n} diverges {worst:.5f} rad under load. Report "
                  "§3.5 records this as an open, low-priority watch item — it is "
                  "the only follower declared revolute rather than continuous, and "
                  "it tracked exactly in free space.")

    return fail


if __name__ == '__main__':
    sys.exit(main())
