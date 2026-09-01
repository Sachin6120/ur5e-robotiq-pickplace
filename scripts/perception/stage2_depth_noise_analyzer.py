#!/usr/bin/env python3
"""Offline diagnostic-only comparison of C1/C2/D10/D20 depth estimators."""
import argparse,csv,json,math,os
from pathlib import Path
import numpy as np
GT_DRIFT_LIMIT_MM=.1
DECISION_RULES={"minimum_usable_frames":100,"rank_order":["mean_euclidean_error_mm","std_euclidean_error_mm","abs_mean_harmful_projection_mm"],"hard_stops":["stamp_contract_failure","gt_drift_exceeds_0_1mm","fewer_than_100_usable_frames"]}
def atomic_json(path,v):
    path=Path(path);tmp=path.with_name(path.name+".tmp")
    with open(tmp,"w") as f:json.dump(v,f,indent=2,sort_keys=True);f.write("\n")
    os.replace(tmp,path)
def summary(a):
    a=np.asarray(a,float)
    if not a.size:return {k:math.nan for k in ("mean","std","min","max","peak_to_peak")}
    return {"mean":float(a.mean()),"std":float(a.std()),"min":float(a.min()),"max":float(a.max()),"peak_to_peak":float(a.max()-a.min())}
def trim_mean(a,f):
    a=np.sort(np.asarray(a,float));k=int(math.floor(a.size*f));return float(a[k:a.size-k].mean()) if a.size>2*k else math.nan
def camera_to_world(camera_xyz,world_from_camera):
    v=np.asarray(camera_xyz,float);M=np.asarray(world_from_camera,float)
    if M.shape!=(4,4):raise ValueError("world_from_camera must be 4x4")
    return (M@np.r_[v,1.])[:3]
def depth_values(z):
    v=np.asarray(z,float);v=v[np.isfinite(v)&(v>0)]
    if not v.size:return None
    return {"C1":float(v.mean()),"C2":float(np.median(v)),"D10":trim_mean(v,.1),"D20":trim_mean(v,.2)}
def reconstruct(raw,calibration,gt_top,yaw_rad):
    M=calibration["world_from_camera"];K=calibration["intrinsics"];fx,fy,cx,cy=(float(K[x]) for x in ("fx","fy","cx","cy"));out={n:[] for n in ("C1","C2","D10","D20")}
    offs=raw["offsets"];u=raw["u"];v=raw["v"];z=raw["depth_m"]
    for i in range(len(offs)-1):
        lo,hi=int(offs[i]),int(offs[i+1]);vals=depth_values(z[lo:hi])
        if vals is None or hi==lo:continue
        uu=float(np.asarray(u[lo:hi]).mean());vv=float(np.asarray(v[lo:hi]).mean())
        for n,d in vals.items():
            if math.isfinite(d):out[n].append(camera_to_world([(uu-cx)*d/fx,(vv-cy)*d/fy,d],M))
    axis=np.array([math.cos(yaw_rad),math.sin(yaw_rad)]);result={}
    for n,points in out.items():
        p=np.asarray(points,float);err=(p-np.asarray(gt_top))*1000 if p.size else np.empty((0,3));eu=np.linalg.norm(err,axis=1) if err.size else np.empty(0);harm=err[:,:2]@axis if err.size else np.empty(0)
        result[n]={"frames":int(len(points)),"world_xyz_m": {"x":summary(p[:,0] if p.size else []),"y":summary(p[:,1] if p.size else []),"z":summary(p[:,2] if p.size else [])},"ex_mm":summary(err[:,0] if err.size else []),"ey_mm":summary(err[:,1] if err.size else []),"ez_mm":summary(err[:,2] if err.size else []),"z_bias_mm":summary(err[:,2] if err.size else []),"euclidean_error_mm":summary(eu),"harmful_closing_axis_projection_mm":summary(harm)}
    return result
def decide(metrics,hard):
    if any(hard.values()):return {"status":"INCONCLUSIVE","recommendation":"No estimator recommendation: hard-stop evidence failure.","rules":DECISION_RULES}
    ranked=[]
    for name,m in metrics.items():
        if m["frames"]<100:continue
        e=m["euclidean_error_mm"];h=m["harmful_closing_axis_projection_mm"]
        ranked.append((e["mean"],e["std"],abs(h["mean"]),name))
    if not ranked:return {"status":"INCONCLUSIVE","recommendation":"No estimator recommendation: no candidate has 100 valid frames.","rules":DECISION_RULES}
    ranked.sort();return {"status":"DIAGNOSTIC_RECOMMENDATION_ONLY","recommended_candidate":ranked[0][3],"ranking": [x[3] for x in ranked],"recommendation":"Lowest mean Euclidean error, then error standard deviation, then absolute harmful projection; not adopted into production.","rules":DECISION_RULES}
def analyze(case_dir):
    d=Path(case_dir);meta=json.loads((d/"case_metadata.json").read_text());initial=json.loads((d/"gt_settled_pose.json").read_text());final=json.loads((d/"gt_final_pose.json").read_text());cal=json.loads((d/"camera_world_calibration.json").read_text());audit=json.loads((d/"stream_audit.json").read_text())
    with np.load(d/"masked_depth_evidence.npz") as raw: raw={k:raw[k] for k in raw.files}
    rows=list(csv.DictReader(open(d/"depth_stats.csv",newline="")));gt_top=np.asarray(initial[:3],float);gt_top[2]+=float(meta["object_size_m"][2])/2;drift=math.dist(initial[:3],final[:3])*1000
    valid=np.asarray([int(r.get("valid_count",0)) for r in rows]);area=np.asarray([int(r.get("mask_area",0)) for r in rows]);table=np.asarray([float(r["table_depth_m"]) for r in rows if r.get("table_depth_m","") not in ("","nan","NaN")]);usable=int((valid>0).sum());empty=int((area==0).sum());zero_valid_nonempty=int(((area>0)&(valid==0)).sum())
    hard={"stamp_contract_failure":not bool(audit.get("stamp_contract_pass")) or bool(audit.get("failure")),"gt_drift_exceeds_0_1mm":drift>GT_DRIFT_LIMIT_MM,"fewer_than_100_usable_frames":usable<100}
    nonempty=area[area>0];candidates=reconstruct(raw,cal,gt_top,float(meta["configured_yaw_rad"]));result={"case":meta["case"],"condition":meta["condition"],"gt_top_world_m":gt_top.tolist(),"gt_drift_mm":drift,"frames_recorded":len(rows),"usable_frames":usable,"mask_health":{"mask_area_px":summary(area),"valid_depth_count":summary(valid),"empty_mask_frames":empty,"nonempty_mask_zero_valid_depth_frames":zero_valid_nonempty,"successful_mask_area_below_frozen_160_px":int((nonempty<160).sum()),"successful_mask_area_above_frozen_5000_px":int((nonempty>5000).sum()),"component_filter_rejection_causes":"not observable from published mask/log; not inferred"},"table_depth_stability_m":summary(table),"xy_centroid_drift_px":{"u":summary([float(r["centroid_u_px"]) for r in rows if r.get("centroid_u_px","") not in ("","nan")]),"v":summary([float(r["centroid_v_px"]) for r in rows if r.get("centroid_v_px","") not in ("","nan")])},"candidates":candidates,"hard_stops":hard,"decision":decide(candidates,hard),"production_change":"NONE"}
    atomic_json(d/"analysis.json",result);return result
def main():
    p=argparse.ArgumentParser(description="Analyze Stage-2 depth-noise evidence; diagnostic recommendation only.");p.add_argument("--case-dir",required=True);a=p.parse_args();print(json.dumps(analyze(a.case_dir),indent=2))
if __name__=="__main__":main()
