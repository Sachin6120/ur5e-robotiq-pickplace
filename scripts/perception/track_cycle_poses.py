#!/usr/bin/env python3
import time, sys, pathlib, yaml
project = pathlib.Path.cwd()
sys.path.insert(0, str(project / "scripts/perception"))
import milestone_f1_harness as harness

stages = [
    "stage.liftdone_ready",
    "stage.transportdone_ready",
    "stage.run_summary_ready"
]

# Clear existing marker files
for s in stages:
    p = project / "evidence/branch_b_full_cycle_run" / s
    if p.exists():
        p.unlink()

print("Tracking started. Initial pose:", harness.instantaneous_object_pose())

last_pose = harness.instantaneous_object_pose()
while True:
    pose = harness.instantaneous_object_pose()
    if pose:
        # Check if rotated or significantly displaced
        if abs(pose[4]) > 0.1 or abs(pose[2] - 0.7725) > 0.01:
            print(f"POSETILT! t={time.time()} pose={pose}")
            break
    time.sleep(0.05)
