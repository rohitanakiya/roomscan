import numpy as np, sys, json, time
from roomscan.io.stray import load_stray
from roomscan.geometry.fusion import select_keyframes, fuse
from roomscan.geometry.drift import correct_drift, lidar_cloud_fn
from roomscan.geometry.frame import PlanFrame
from roomscan.geometry.cells import extract_segments
name=sys.argv[1]
cap=load_stray(f'data/raw/{name}'); kf=select_keyframes(cap.poses,0.10,8.0,30)
def sharp(poses):
    pc=fuse(cap,kf,poses=poses,stride=2,max_depth=5.0,voxel=0.02)
    F=PlanFrame.fit(pc.xyz,pc.normals); q=F.to_plan(pc.xyz); nq=F.normals_to_plan(pc.normals)
    segs=[s for s in extract_segments(q,nq) if s.t1-s.t0>1.0]
    fr=[]
    for s in segs:
        ax_n,ax_t=(1,0) if s.orient=='H' else (0,1)
        m=(np.abs(nq[:,ax_n])>0.9)&(q[:,ax_t]>s.t0)&(q[:,ax_t]<s.t1)&(np.abs(q[:,ax_n]-s.coord)<0.15)&(q[:,2]>0.1)
        d=np.abs(q[m,ax_n]-s.coord)
        if len(d)>200: fr.append(((d<0.02).mean(), len(d)))
    fr=np.array(fr)
    return float(np.average(fr[:,0],weights=fr[:,1])), len(fr)
res={}
pass
for tag,kw in [('interp_default',{}),('interp_frag8_d4',dict(frag_len=8,max_pair_dist=4.0))]:
    t=time.time(); P,info=correct_drift(cap.poses,kf,lidar_cloud_fn(cap),log=lambda *a:None,**kw)
    res[tag]=sharp(P); print(tag,res[tag],info.get('loop_edges'),info.get('loop_edges_kept'),info.get('mean_loop_residual_before_m'),info.get('mean_loop_residual_after_m'),round(time.time()-t),flush=True)
