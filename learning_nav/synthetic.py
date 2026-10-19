"""Generate explicitly synthetic plumbing data, never real robotics evidence."""
import argparse
import json
from pathlib import Path
import numpy as np
from .data import save_dataset,SCHEMA_VERSION,ACTION_SOURCES,DEFAULT_LIMITS_S


def generate(directory,sessions=6,rows=36,seed=42):
    out=Path(directory); out.mkdir(parents=True,exist_ok=True); rng=np.random.default_rng(seed)
    meta={"schema_version":SCHEMA_VERSION,"odometry_source":"synthetic","environment":"synthetic","action_source":ACTION_SOURCES["synthetic"],"recording_limits_s":dict(DEFAULT_LIMITS_S),"map_id":"SYNTHETIC-NOT-A-REAL-MAP","sectors":24,"lidar_max_m":5.,"synthetic":True,"candidates":[{"name":"STOP","pose":None},{"name":"desk","pose":[1.,0.,0.]},{"name":"home","pose":[0.,1.,0.]}]}
    (out/"metadata.json").write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding="utf-8")
    commands=["стоп","поезжай к столу","езжай домой"]
    for session in range(sessions):
        samples=[]
        for i in range(rows):
            label=i%3; dt=.2; v=float(rng.uniform(0,.15)); w=float(rng.uniform(-.3,.3)); odom=np.array([rng.uniform(-1,1),rng.uniform(-1,1),rng.uniform(-np.pi,np.pi)],np.float32)
            dx=v*dt*np.cos(odom[2]); dy=v*dt*np.sin(odom[2]); nxt=odom+np.array([dx,dy,w*dt],np.float32)
            scan=rng.uniform(.2,.9,meta["sectors"]).astype(np.float32); mask=rng.random(meta["sectors"])>.1
            nxtscan=np.clip(scan-v*dt/5*np.cos(np.linspace(-np.pi,np.pi,meta["sectors"])),0,1).astype(np.float32)
            rgb=np.zeros((128,128,3),np.uint8); rgb[...,label]=150; rgb+=rng.integers(0,30,rgb.shape,dtype=np.uint8)
            ns=int(1e9*(100+session*100+i*.25))
            samples.append(dict(rgb=rgb,lidar=scan,lidar_mask=mask,odom=odom,map_pose=odom.copy(),map_pose_valid=np.array(True),next_lidar=nxtscan,next_lidar_mask=mask.copy(),next_odom=nxt,action=np.array([v,w],np.float32),dt=np.array(dt,np.float32),action_valid=np.array(True),instruction=np.array(commands[label]),label=np.array(label,np.int64),session=np.array(f"SYNTHETIC_{session:03d}"),start_ns=np.array(ns,np.int64),end_ns=np.array(ns+200000000,np.int64),ack_ns=np.array(ns-10000000,np.int64),rgb_ns=np.array(ns-20000000,np.int64),next_rgb_ns=np.array(ns+180000000,np.int64),scan_ns=np.array(ns,np.int64),next_scan_ns=np.array(ns+200000000,np.int64),odom_ns=np.array(ns-10000000,np.int64),next_odom_ns=np.array(ns+190000000,np.int64),odom_healthy=np.array(True),next_odom_healthy=np.array(True),snapshot_received_ns=np.array(ns+30000000,np.int64),next_snapshot_received_ns=np.array(ns+230000000,np.int64),health_received_ns=np.array(ns+20000000,np.int64),next_health_received_ns=np.array(ns+220000000,np.int64)))
        save_dataset(out/f"session_{session:03d}.npz",samples,meta)
    return meta


def main():
    p=argparse.ArgumentParser(__doc__); p.add_argument("--out",required=True); p.add_argument("--sessions",type=int,default=6); p.add_argument("--rows",type=int,default=36); args=p.parse_args(); generate(args.out,args.sessions,args.rows)


if __name__=="__main__": main()
