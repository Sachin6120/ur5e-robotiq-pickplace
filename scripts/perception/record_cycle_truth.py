#!/usr/bin/env python3
import time, sys, pathlib, csv
project = pathlib.Path.cwd()
sys.path.insert(0, str(project / "scripts/perception"))
import milestone_f1_harness as harness

stages = [
    "stage.liftdone_ready",
    "stage.transportdone_ready",
    "stage.run_summary_ready"
]

for s in stages:
    p = project / "evidence/branch_b_full_cycle_run" / s
    if p.exists():
        p.unlink()

out_csv = project / "evidence/branch_b_full_cycle_run/object_trajectory.csv"
records = []

start_time = time.time()
print("Recording object trajectory...")
while True:
    pose = harness.instantaneous_object_pose()
    now = time.time() - start_time
    if pose:
        records.append([now] + pose)

    summary_marker = project / "evidence/branch_b_full_cycle_run/stage.run_summary_ready"
    if summary_marker.exists() and now > 5.0:
        # Wait an extra second to capture settled pose
        time.sleep(1.0)
        pose = harness.instantaneous_object_pose()
        records.append([time.time() - start_time] + pose)
        break

    if now > 60.0:
        print("Timeout waiting for run summary marker.")
        break
    time.sleep(0.05)

with open(out_csv, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time", "x", "y", "z", "qx", "qy", "qz", "qw"])
    writer.writerows(records)

print(f"Recorded {len(records)} samples to {out_csv}")
