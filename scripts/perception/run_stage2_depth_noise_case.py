#!/usr/bin/env python3
"""One guarded, perception-only Stage-2 depth-noise diagnostic case."""
import argparse,json,math,os,subprocess,sys,time
from pathlib import Path
REPO=Path(__file__).resolve().parents[2];sys.path.insert(0,str(REPO/"scripts/perception"))
import milestone_f1_harness as harness
import run_stage2a_yaw_case as yaw
CONDITIONS={"N0":(0,0,0,0),"N1":(1,0,0,0),"N2":(2,0,0,0),"N3":(5,0,0,0),"N4":(1,.02,0,0),"N5":(1,0,.01,50),"N6":(10,0,0,0)};YAW={"P0":0.,"P1":15.,"P3":30.};SEED=20260829
PRE_PATTERN="m3_grasp|static_scene_tf|move_group|object_detector|object_position_world|[g]z sim|robot_state_publisher|ros2_control_node|gz_pose_observer|depth_noise_shim|depth_stats_probe"
RUNTIME_PATTERN="m3_grasp|static_scene_tf|move_group"
def atomic_json(path,v):
    path=Path(path);tmp=path.with_name(path.name+".tmp")
    with open(tmp,"w") as f:json.dump(v,f,indent=2,sort_keys=True);f.write("\n")
    os.replace(tmp,path)
def head():return subprocess.run(["git","-C",str(REPO),"rev-parse","HEAD"],capture_output=True,text=True,check=True).stdout.strip()
def pids(pattern):
    r=subprocess.run(["pgrep","-f",pattern],capture_output=True,text=True);return r.stdout.strip().splitlines() if r.returncode==0 else []
def clean_guard(pattern=PRE_PATTERN):
    hit=pids(pattern)
    if hit:raise RuntimeError("CONTAMINATED_ENVIRONMENT: "+",".join(hit))
def active(text,name):return any(line.split() and line.split()[0]==name and line.split()[-1]=="active" for line in text.splitlines())
def present(text,name):return any(line.split() and line.split()[0]==name for line in text.splitlines())
def require_n0(case):
    marker=REPO/f"evidence/stage2_depth_noise/{case}_N0/n0_pass.json"
    try: ok=json.loads(marker.read_text()).get("status")=="PASS"
    except (OSError,json.JSONDecodeError):ok=False
    if not ok:raise RuntimeError(f"N0_PASS_REQUIRED: {marker}")
def run(case,condition):
    out=REPO/f"evidence/stage2_depth_noise/{case}_{condition}"
    if out.exists():raise RuntimeError(f"evidence directory already exists: {out}")
    if condition!="N0":require_n0(case)
    clean_guard();out.mkdir(parents=True,exist_ok=False);sigma,drop,ofrac,omag=CONDITIONS[condition]
    atomic_json(out/"case_metadata.json",{"case":case,"condition":condition,"configured_yaw_deg":YAW[case],"configured_yaw_rad":math.radians(YAW[case]),"seed":SEED,"sigma_mm":sigma,"dropout_fraction":drop,"outlier_fraction":ofrac,"outlier_magnitude_mm":omag,"target_frames":100,"object_size_m":harness.OBJ_SIZE,"git_head":head(),"sim_launch":"ur5e_robotiq_sim_control.launch.py gripper_model:=parallel_jaw enable_camera:=true gazebo_gui:=false","no_arm_or_gripper_action_commands":True})
    procs=[];files=[];failure=None
    def start(cmd,log):
        f=open(out/log,"w");files.append(f);p=yaw.start_process(cmd,stdout=f,stderr=subprocess.STDOUT);procs.append(p);return p
    try:
        start(f"python3 {REPO}/scripts/perception/gz_pose_observer.py --out {out/'gt_pose_stream.csv'}","gz_pose_observer.log")
        start(f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && ros2 launch ur5e_robotiq_description ur5e_robotiq_sim_control.launch.py gripper_model:=parallel_jaw enable_camera:=true gazebo_gui:=false","sim.log")
        for _ in range(60):
            text,_=yaw.run_cmd(f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && ros2 control list_controllers",10)
            if active(text,"arm_controller") and active(text,"parallel_jaw_gripper_controller") and not present(text,"gripper_controller"):break
            time.sleep(1)
        else:raise RuntimeError("controller assertion failed: require active arm_controller + parallel_jaw_gripper_controller; linkage gripper_controller absent")
        ready,detail=yaw.wait_for_camera_topics()
        if not ready:raise RuntimeError("camera unavailable: "+detail)
        clean_guard(RUNTIME_PATTERN);harness.remove_object();time.sleep(1);spawn=yaw.spawn_object_yaw(.45,-.15,math.radians(YAW[case]))
        if "data: true" not in spawn.lower() and "true" not in spawn.lower():raise RuntimeError("object spawn rejected: "+spawn)
        ok,msg=harness.settle_object()
        if not ok:raise RuntimeError("object failed settle: "+msg)
        initial=harness.instantaneous_object_pose()
        if not initial:raise RuntimeError("initial GT unavailable")
        atomic_json(out/"gt_settled_pose.json",initial)
        start(f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && python3 {REPO}/scripts/perception/depth_noise_shim.py --sigma-mm {sigma} --dropout-fraction {drop} --outlier-fraction {ofrac} --outlier-magnitude-mm {omag} --seed {SEED} --audit-out {out/'shim_audit.json'}","depth_noise_shim.log")
        start(f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && ros2 run ur5e_pick_place object_detector --ros-args -p use_sim_time:=true -p depth_topic:=/overhead_camera/depth_image_noisy","object_detector.log")
        start(f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && ros2 run ur5e_pick_place object_position_world --ros-args -p use_sim_time:=true","object_position_world.log")
        n0arg=" --require-n0-identity" if condition=="N0" else ""
        probe=start(f"source /opt/ros/jazzy/setup.bash && source {REPO}/install/setup.bash && python3 {REPO}/scripts/perception/depth_stats_probe.py --out {out/'depth_stats.csv'} --raw-out {out/'masked_depth_evidence.npz'} --calibration-out {out/'camera_world_calibration.json'} --audit-out {out/'stream_audit.json'} --frames 100{n0arg}","depth_stats_probe.log")
        try:rc=probe.wait(timeout=90)
        except subprocess.TimeoutExpired:
            yaw.stop_process(probe);raise RuntimeError("probe timeout after 90 s; partial evidence retained; no retry")
        if rc!=0:raise RuntimeError("depth probe failed; partial evidence retained")
        clean_guard(RUNTIME_PATTERN);samples=[]
        for _ in range(5):
            sample,detail=yaw.wait_for_perception_point(15)
            if sample is None:raise RuntimeError("detector point evidence failure: "+detail)
            samples.append(sample)
        atomic_json(out/"detector_world_samples.json",samples);final=harness.instantaneous_object_pose()
        if not final:raise RuntimeError("final GT unavailable")
        atomic_json(out/"gt_final_pose.json",final);drift=math.dist(initial[:3],final[:3])*1000
        if drift>.1:raise RuntimeError(f"GT drift {drift:.6f} mm exceeds 0.1 mm")
        audit=json.loads((out/"stream_audit.json").read_text())
        if not audit.get("stamp_contract_pass") or audit.get("failure"):raise RuntimeError("RGB/depth stamp contract failure")
        if condition=="N0":
            if audit.get("n0_identity_failures") or audit.get("frames_compared",0)<100 or audit.get("source_sha256")!=audit.get("noisy_sha256"):raise RuntimeError("N0 independent stream identity failure")
            baseline=json.loads((REPO/f"evidence/stage2_perception_yaw/{case}/perceived_samples.json").read_text());expected=[sum(s["xyz"][i] for s in baseline)/len(baseline) for i in range(3)];observed=[sum(s["xyz"][i] for s in samples)/len(samples) for i in range(3)];errs=[abs(x-y)*1000 for x,y in zip(observed,expected)]
            atomic_json(out/"n0_checks.json",{"baseline_top_world_m":expected,"observed_top_world_m":observed,"axis_abs_error_mm":errs,"tolerance_mm":.01,"stream_audit":audit})
            if max(errs)>.01:raise RuntimeError("N0 reproduction outside 0.01 mm")
        subprocess.run([sys.executable,str(REPO/"scripts/perception/stage2_depth_noise_analyzer.py"),"--case-dir",str(out)],check=True)
        if condition=="N0":atomic_json(out/"n0_pass.json",{"status":"PASS","case":case,"git_head":head(),"stream_identity_pass":True,"reproduction_tolerance_mm":.01})
    except Exception as e:
        failure=str(e);atomic_json(out/"run_failure.json",{"failure":failure,"no_retry":True});raise
    finally:
        for p in reversed(procs):yaw.stop_process(p)
        for f in files:f.close()
        time.sleep(.5);left=pids(PRE_PATTERN);atomic_json(out/"cleanup.json",{"clean_slate":not bool(left),"remaining_pids":left,"prior_failure":failure})
        if left and failure is None:raise RuntimeError("post-cleanup contamination: "+",".join(left))
def main():
    p=argparse.ArgumentParser(description="Run one no-command perception-only depth-noise case.");p.add_argument("--case",required=True,choices=YAW);p.add_argument("--condition",required=True,choices=CONDITIONS);a=p.parse_args()
    try:run(a.case,a.condition)
    except Exception as e:print("HARD_ABORT:",e,file=sys.stderr);raise SystemExit(2)
if __name__=="__main__":main()
