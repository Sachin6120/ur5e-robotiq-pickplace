#!/usr/bin/env python3
"""Deterministic TYPE_32FC1 depth perturbation shim; N0 is byte-identical."""
import argparse, hashlib, json, os
from pathlib import Path
import numpy as np

def atomic_json(path, value):
    path=Path(path); tmp=path.with_name(path.name+".tmp")
    with open(tmp,"w") as f: json.dump(value,f,indent=2,sort_keys=True); f.write("\n")
    os.replace(tmp,path)

def perturb_depth(src,sigma_mm=0.,dropout_fraction=0.,outlier_fraction=0.,outlier_magnitude_mm=0.,rng=None):
    """Copy, then Gaussian -> dropout -> signed outlier over finite pixels."""
    src=np.asarray(src)
    if src.dtype!=np.float32: raise ValueError("TYPE_32FC1 requires float32")
    if not 0<=dropout_fraction<=1 or not 0<=outlier_fraction<=1: raise ValueError("fractions must be in [0,1]")
    if sigma_mm<0 or outlier_magnitude_mm<0: raise ValueError("magnitudes must be non-negative")
    out=src.copy(); finite=np.isfinite(src); idx=np.flatnonzero(finite.ravel())
    s={"pixels":int(src.size),"finite_input":int(idx.size),"nonfinite_input":int(src.size-idx.size),"gaussian_count":0,"dropout_count":0,"outlier_count":0,"max_abs_input_output_diff_m":0.}
    # N0: no arithmetic and no RNG access, preserving all IEEE-754 bits.
    if sigma_mm==dropout_fraction==outlier_fraction==0.: return out,s
    rng=np.random.default_rng(0) if rng is None else rng
    if sigma_mm:
        out.ravel()[idx]+=rng.normal(0.,sigma_mm/1000.,idx.size).astype(np.float32); s["gaussian_count"]=int(idx.size)
    if dropout_fraction and idx.size:
        chosen=idx[rng.random(idx.size)<dropout_fraction];out.ravel()[chosen]=np.nan;s["dropout_count"]=int(chosen.size)
    survivors=idx[np.isfinite(out.ravel()[idx])]
    if outlier_fraction and survivors.size:
        chosen=survivors[rng.random(survivors.size)<outlier_fraction];out.ravel()[chosen]+=rng.choice(np.array([-1.,1.],np.float32),chosen.size)*np.float32(outlier_magnitude_mm/1000.);s["outlier_count"]=int(chosen.size)
    c=finite&np.isfinite(out)
    if c.any():s["max_abs_input_output_diff_m"]=float(np.abs(out[c]-src[c]).max())
    return out,s

def bits_equal(a,b):
    a,b=np.asarray(a,np.float32),np.asarray(b,np.float32)
    return a.shape==b.shape and np.array_equal(a.view(np.uint32),b.view(np.uint32))

def main():
    p=argparse.ArgumentParser(description="Publish deterministic noisy depth from /overhead_camera/depth_image.")
    p.add_argument("--sigma-mm",type=float,default=0.);p.add_argument("--dropout-fraction",type=float,default=0.);p.add_argument("--outlier-fraction",type=float,default=0.);p.add_argument("--outlier-magnitude-mm",type=float,default=50.);p.add_argument("--seed",type=int,required=True);p.add_argument("--audit-out",required=True);a=p.parse_args()
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    class Shim(Node):
        def __init__(self):
            super().__init__("depth_noise_shim");self.rng=np.random.default_rng(a.seed);self.frames=0;self.bad=0;self.totals={k:0 for k in ("gaussian_count","dropout_count","outlier_count")};self.ih=hashlib.sha256();self.oh=hashlib.sha256();self.pub=self.create_publisher(Image,"/overhead_camera/depth_image_noisy",10);self.create_subscription(Image,"/overhead_camera/depth_image",self.cb,10)
        def audit(self,last): atomic_json(a.audit_out,{"seed":a.seed,"frames":self.frames,"identity_internal_failures":self.bad,"cumulative":self.totals,"source_sha256":self.ih.hexdigest(),"published_sha256":self.oh.hexdigest(),"last_frame":last,"n0_internal_pass":self.bad==0})
        def cb(self,m):
            if m.encoding!="32FC1" or m.is_bigendian or m.step!=m.width*4 or len(m.data)!=m.height*m.step:
                self.bad+=1;self.get_logger().error(f"DEPTH_INPUT_REJECTED encoding={m.encoding}");self.audit({"rejected":True});return
            src=np.frombuffer(m.data,dtype=np.float32).reshape(m.height,m.width);out,s=perturb_depth(src,a.sigma_mm,a.dropout_fraction,a.outlier_fraction,a.outlier_magnitude_mm,self.rng);raw=out.tobytes();self.frames+=1;self.ih.update(m.data);self.oh.update(raw)
            for k in self.totals:self.totals[k]+=s[k]
            if a.sigma_mm==a.dropout_fraction==a.outlier_fraction==0. and (not bits_equal(src,out) or s["max_abs_input_output_diff_m"]!=0.):self.bad+=1
            r=Image();r.header=m.header;r.height=m.height;r.width=m.width;r.encoding="32FC1";r.is_bigendian=False;r.step=m.step;r.data=raw;self.pub.publish(r);self.audit(s)
            if self.frames==1 or self.frames%25==0:self.get_logger().info(f"frame={self.frames} seed={a.seed} cumulative={self.totals}")
    rclpy.init();n=Shim()
    try:rclpy.spin(n)
    finally:n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
