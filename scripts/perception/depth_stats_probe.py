#!/usr/bin/env python3
"""Exact-stamp mask/noisy-depth recorder with RGB-depth and N0 stream guards."""
import argparse,csv,hashlib,json,math,os,signal,time
from collections import OrderedDict
from pathlib import Path
import numpy as np
MAX_BUFFER=64
def atomic_json(path,v):
    path=Path(path);tmp=path.with_name(path.name+".tmp")
    with open(tmp,"w") as f:json.dump(v,f,indent=2,sort_keys=True);f.write("\n")
    os.replace(tmp,path)
def stamp(h):return(int(h.stamp.sec),int(h.stamp.nanosec))
def contract_keys(rgb,source):return set(rgb)&set(source)
def bits_equal(a,b):
    a,b=np.asarray(a,np.float32),np.asarray(b,np.float32);return a.shape==b.shape and np.array_equal(a.view(np.uint32),b.view(np.uint32))
def insert(buf,k,v):
    if k in buf:raise ValueError("duplicate pending timestamp")
    if len(buf)>=MAX_BUFFER:raise ValueError("timestamp buffer overflow")
    buf[k]=v
class SyncState:
    """Exact timestamp state machine: warm-up is discard-only; armed is strict."""
    def __init__(self):
        self.buffers={n:OrderedDict() for n in ("rgb","src","noisy","mask")};self.contracted=set();self.done=set();self.armed=False;self.first_armed_timestamp=None;self.warmup_discards={n:0 for n in self.buffers};self.stamp_contract_failures=[];self.exact_pair_count=0
    def add(self,name,k,value,now):
        if k in self.done:raise ValueError("duplicate completed timestamp")
        insert(self.buffers[name],k,(now,value));self.contract()
    def contract(self):
        rgb,src=self.buffers["rgb"],self.buffers["src"]
        for k in list(contract_keys(rgb,src)):rgb.pop(k);self.contracted.add(k)
    def ready_keys(self):return sorted(set(self.buffers["mask"])&set(self.buffers["noisy"])&set(self.buffers["src"])&self.contracted)
    def pop_ready(self):
        pairs=[]
        for k in self.ready_keys():
            ma=self.buffers["mask"].pop(k)[1];no=self.buffers["noisy"].pop(k)[1];so=self.buffers["src"].pop(k)[1];self.contracted.remove(k);self.done.add(k);self.exact_pair_count+=1
            if not self.armed:
                self.armed=True;self.first_armed_timestamp=k;self.clear_older(k)
            pairs.append((k,ma,no,so))
        return pairs
    def clear_older(self,k):
        for name,b in self.buffers.items():
            for old in [x for x in b if x<k]:b.pop(old);self.warmup_discards[name]+=1
        self.contracted={x for x in self.contracted if x>=k}
    def expire(self,now,timeout_s=2.):
        failures=[]
        for name,b in self.buffers.items():
            for k in [x for x,v in b.items() if v[0]<now-timeout_s]:
                was_contracted=k in self.contracted;b.pop(k);self.contracted.discard(k)
                if not self.armed:self.warmup_discards[name]+=1;continue
                if name=="rgb" or (name=="src" and not was_contracted):
                    msg=f"RGB/source stamp-contract failure at {k} ({name} unmatched)";self.stamp_contract_failures.append(msg)
                else:msg=f"stale unmatched {name} timestamp {k}"
                failures.append(msg)
        return failures
def stats(mask,depth,k):
    if mask.shape!=depth.shape:raise ValueError("mask/depth shape mismatch")
    selected=mask!=0; z=np.asarray(depth,np.float32)[selected];valid=z[np.isfinite(z)&(z>0)];n=valid.size;ys,xs=np.nonzero(selected)
    r={"stamp_sec":k[0],"stamp_nanosec":k[1],"mask_area":int(z.size),"valid_count":int(n),"invalid_count":int(z.size-n),"centroid_u_px":float(xs.mean()) if xs.size else math.nan,"centroid_v_px":float(ys.mean()) if ys.size else math.nan}
    for name in ("mean_m","median_m","std_m","trimmed_mean_10_m","trimmed_mean_20_m","p01_m","p05_m","p10_m","p25_m","p75_m","p90_m","p95_m","p99_m","mad_m","table_depth_m"):r[name]=math.nan
    r.update(outlier_count_mad=0,outlier_fraction_mad=math.nan)
    whole=np.asarray(depth,np.float32);table=whole[np.isfinite(whole)&(whole>=1.2)&(whole<2.)]
    if table.size:h,e=np.histogram(table,bins=800,range=(1.2,2.));i=int(h.argmax());r["table_depth_m"]=float((e[i]+e[i+1])/2)
    if not n:return r,xs.astype(np.int32),ys.astype(np.int32),z
    o=np.sort(valid.astype(float));med=float(np.median(o));mad=float(np.median(np.abs(o-med)));out=np.abs(o-med)>3*1.4826*mad if mad>0 else np.zeros(n,bool)
    def trim(f):q=int(math.floor(n*f));return float(o[q:n-q].mean()) if n>2*q else math.nan
    r.update(mean_m=float(o.mean()),median_m=med,std_m=float(o.std()),trimmed_mean_10_m=trim(.1),trimmed_mean_20_m=trim(.2),mad_m=mad,outlier_count_mad=int(out.sum()),outlier_fraction_mad=float(out.mean()));r.update(**{f"p{x:02d}_m":float(np.percentile(o,x)) for x in (1,5,10,25,75,90,95,99)})
    return r,xs.astype(np.int32),ys.astype(np.int32),z
def matrix_from_transform(t):
    q=t.transform.rotation;x,y,z,w=q.x,q.y,q.z,q.w;n=math.sqrt(x*x+y*y+z*z+w*w)
    if not n:raise ValueError("zero transform quaternion")
    x,y,z,w=x/n,y/n,z/n,w/n;M=np.eye(4);M[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]];M[:3,3]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z];return M
def main():
    p=argparse.ArgumentParser(description="Record exact detector-mask/noisy-depth pairs; no segmentation.");p.add_argument("--out",required=True);p.add_argument("--raw-out",required=True);p.add_argument("--calibration-out",required=True);p.add_argument("--audit-out",required=True);p.add_argument("--frames",type=int,default=100);p.add_argument("--require-n0-identity",action="store_true");a=p.parse_args()
    if a.frames<1:p.error("--frames must be positive")
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from sensor_msgs.msg import Image,CameraInfo
    from tf2_ros import Buffer,TransformListener,TransformException
    class P(Node):
        def __init__(self):
            super().__init__("depth_stats_probe");self.sync=SyncState();self.rows=[];self.raw=[];self.failure=None;self.info=None;self.M=None;self.started=time.monotonic();self.n0bad=0;self.n0frames=0;self.sh=hashlib.sha256();self.nh=hashlib.sha256();self.tf=Buffer(node=self);self.listener=TransformListener(self.tf,self);self.create_subscription(Image,"/overhead_camera/image",self.rgb_cb,20);self.create_subscription(Image,"/overhead_camera/depth_image",self.src_cb,20);self.create_subscription(Image,"/overhead_camera/depth_image_noisy",self.noisy_cb,20);self.create_subscription(Image,"object_detector/mask",self.mask_cb,20);self.create_subscription(CameraInfo,"/overhead_camera/camera_info",self.info_cb,5);self.timer=self.create_timer(.1,self.tick)
        def fail(self,s):
            if not self.failure:self.failure=s;self.get_logger().error(f"HARD_ABORT: {s}")
        def decode(self,m,d,e):
            if m.encoding!=e:raise ValueError(f"encoding {m.encoding}, expected {e}")
            if m.is_bigendian or m.step!=m.width*np.dtype(d).itemsize or len(m.data)!=m.height*m.step:raise ValueError("invalid image layout")
            return np.frombuffer(m.data,dtype=d).reshape(m.height,m.width)
        def add(self,name,k,v):
            try:self.sync.add(name,k,v,time.monotonic())
            except ValueError as e:self.fail(str(e))
        def rgb_cb(self,m):self.add("rgb",stamp(m.header),None);self.pair()
        def src_cb(self,m):
            try:self.add("src",stamp(m.header),self.decode(m,np.float32,"32FC1"));self.pair()
            except ValueError as e:self.fail(str(e))
        def noisy_cb(self,m):
            try:self.add("noisy",stamp(m.header),self.decode(m,np.float32,"32FC1"));self.pair()
            except ValueError as e:self.fail(str(e))
        def mask_cb(self,m):
            try:self.add("mask",stamp(m.header),self.decode(m,np.uint8,"mono8"));self.pair()
            except ValueError as e:self.fail(str(e))
        def info_cb(self,m):
            self.info=m
            if self.M is None:
                try:
                    t=self.tf.lookup_transform("world",m.header.frame_id,Time());self.M=matrix_from_transform(t);atomic_json(a.calibration_out,{"target_frame":"world","source_frame":m.header.frame_id,"intrinsics":{"fx":m.k[0],"fy":m.k[4],"cx":m.k[2],"cy":m.k[5],"width":m.width,"height":m.height},"world_from_camera":self.M.tolist()})
                except TransformException:pass
        def pair(self):
            if self.M is None:return
            for k,ma,no,so in self.sync.pop_ready():
                self.n0frames+=1;self.sh.update(so.tobytes());self.nh.update(no.tobytes())
                if a.require_n0_identity and not bits_equal(so,no):self.n0bad+=1;self.fail("N0 source/noisy bit mismatch")
                r,x,y,z=stats(ma,no,k);r.update(fx=self.info.k[0],fy=self.info.k[4],cx=self.info.k[2],cy=self.info.k[5]);self.rows.append(r);self.raw.append((k,x,y,z))
        def tick(self):
            now=time.monotonic()
            for failure in self.sync.expire(now):self.fail(failure)
            if (self.info is None or self.M is None) and now-self.started>10:self.fail("camera intrinsics/world_from_camera unavailable")
        def finalize(self):
            Path(a.out).parent.mkdir(parents=True,exist_ok=True);fields=list(self.rows[0]) if self.rows else ["stamp_sec","stamp_nanosec","mask_area","valid_count","invalid_count"];tmp=Path(a.out).with_suffix(".tmp")
            with open(tmp,"w",newline="") as f:w=csv.DictWriter(f,fieldnames=fields,extrasaction="ignore");w.writeheader();w.writerows(self.rows)
            os.replace(tmp,a.out);offs=[0];ss=[];xs=[];ys=[];zs=[]
            for k,x,y,z in self.raw:ss.append(k);xs.append(x);ys.append(y);zs.append(z);offs.append(offs[-1]+len(z))
            rawtmp=str(a.raw_out)+".tmp"
            with open(rawtmp,"wb") as f:np.savez_compressed(f,stamp=np.asarray(ss,np.int64),offsets=np.asarray(offs,np.int64),u=np.concatenate(xs) if xs else np.empty(0,np.int32),v=np.concatenate(ys) if ys else np.empty(0,np.int32),depth_m=np.concatenate(zs) if zs else np.empty(0,np.float32))
            os.replace(rawtmp,a.raw_out);atomic_json(a.audit_out,{"armed":self.sync.armed,"first_armed_timestamp":self.sync.first_armed_timestamp,"warmup_discard_counts":self.sync.warmup_discards,"stamp_contract_failures":self.sync.stamp_contract_failures,"stamp_contract_pass":not self.sync.stamp_contract_failures,"exact_pair_count":self.sync.exact_pair_count,"frames_compared":self.n0frames,"n0_identity_failures":self.n0bad,"source_sha256":self.sh.hexdigest(),"noisy_sha256":self.nh.hexdigest(),"failure":self.failure,"complete_rows":len(self.rows)})
    rclpy.init();n=P();signal.signal(signal.SIGTERM,lambda *_:setattr(n,"failure",n.failure or "terminated"))
    try:
        while rclpy.ok() and not n.failure and len(n.rows)<a.frames:rclpy.spin_once(n,timeout_sec=.2)
    finally:n.finalize();n.destroy_node();rclpy.try_shutdown()
    if n.failure or len(n.rows)<a.frames:raise SystemExit(n.failure or "fewer than requested paired frames")
if __name__=="__main__":main()
