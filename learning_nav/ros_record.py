"""Conservative ROS 2 transition recorder. Ctrl-C writes one session NPZ.

    python -m learning_nav.ros_record --meta metadata.json --out session01.npz

ACK is a device-acknowledged nominal setpoint, not measured wheel speed. RF2O
odometry is estimated from the SAME lidar observations, not ground truth.
This collector
keeps only constant ACK intervals with fresh source-stamped sensors. It does not
guess missing actions and does not reconstruct motor dynamics from messages.
"""
from __future__ import annotations
import argparse
from collections import deque
import math
import json
from pathlib import Path
import uuid
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from sensor_msgs.msg import Image,LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String,Bool
from tf2_ros import Buffer,TransformListener,TransformException
from .data import load_metadata,sector_scan,save_dataset,ODOMETRY_SOURCE,DEFAULT_LIMITS_S
from .recording import scan_ready,synchronized_pair


def stamp_ns(stamp):
    return int(stamp.sec)*1000000000+int(stamp.nanosec)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def decode_image(msg):
    channels={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4,"mono8":1}.get(msg.encoding)
    if channels is None or msg.width<1 or msg.height<1 or msg.step<msg.width*channels:
        raise ValueError("Only raw rgb8/bgr8/rgba8/bgra8/mono8 images are supported")
    raw=np.frombuffer(bytes(msg.data),np.uint8)
    if len(raw)!=msg.height*msg.step: raise ValueError("Image data/step mismatch")
    image=raw.reshape(msg.height,msg.step)[:,:msg.width*channels].reshape(msg.height,msg.width,channels)
    if channels==1: image=np.repeat(image,3,-1)
    else:
        image=image[...,:3]
        if msg.encoding.startswith("bgr"): image=image[...,::-1]
    ys=np.linspace(0,msg.height-1,128).astype(int); xs=np.linspace(0,msg.width-1,128).astype(int)
    return image[ys[:,None],xs[None,:]].copy()


class Recorder(Node):
    def __init__(self,args,name="learning_recorder"):
        super().__init__(name)
        self.args=args; self.meta=load_metadata(args.meta); self.rows=[]; self.previous=None
        if self.meta["odometry_source"] != ODOMETRY_SOURCE:
            raise ValueError("Live recorder/shadow requires guarded RF2O metadata; synthetic metadata is offline only")
        self.limits=dict(self.meta["recording_limits_s"])
        for key in DEFAULT_LIMITS_S:
            value=getattr(args,"max_"+key,None)
            if value is not None:
                if not 0 < value <= self.limits[key]:
                    raise ValueError(f"--max-{key.replace('_','-')} must be positive and <= metadata recording limit")
                self.limits[key]=value
        if not 0 < args.odom_wait <= self.limits["scan_age"]:
            raise ValueError("odom_wait must be positive and <= maximum scan age")
        self.rgb=None; self.odom=None; self.rgb_history=deque(maxlen=60); self.odom_history=deque(maxlen=400); self.instruction=""; self.label=-1; self.context_generation=-1; self.acks=deque(maxlen=2000)
        self.session=args.session or str(uuid.uuid4()); self.last_scan_ns=0; self.dropped=0; self.armed=False; self.armed_received_ns=0
        self.action_source=None; self.action_source_received_ns=0
        self.odom_healthy=False; self.health_received_ns=0; self.health_generation=0; self.pending_scans=deque(maxlen=10)
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.create_subscription(Image,args.image_topic,self.image_cb,qos_profile_sensor_data)
        self.create_subscription(Odometry,args.odom_topic,self.odom_cb,qos_profile_sensor_data)
        self.create_subscription(LaserScan,args.scan_topic,self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(TwistStamped,args.ack_topic,self.ack_cb,10)
        self.create_subscription(Bool,"/room_base/armed",self.armed_cb,10)
        self.create_subscription(Bool,args.health_topic,self.health_cb,10)
        self.create_subscription(String,"/room_base/action_source",self.action_source_cb,1)
        context_qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String,"/room_nav/demo_context",self.context_cb,context_qos)
        self.create_timer(10.,self.report)
        self.create_timer(.05,self.drain_pending)

    def report(self): self.get_logger().info(f"session={self.session} transitions={len(self.rows)} dropped={self.dropped}")

    def image_cb(self,msg):
        ns=stamp_ns(msg.header.stamp)
        if ns<=0 or (self.rgb is not None and ns<=self.rgb[0]): return
        try: rgb=decode_image(msg)
        except ValueError as e: self.get_logger().warning(str(e)); return
        self.rgb=(ns,rgb)
        self.rgb_history.append(self.rgb)

    def odom_cb(self,msg):
        ns=stamp_ns(msg.header.stamp)
        if ns<=0 or msg.header.frame_id!=self.args.odom_frame or msg.child_frame_id!=self.args.base_frame:
            self.previous=None; return
        if self.odom is not None and ns<=self.odom[0]:
            if ns<self.odom[0]: self.previous=None; self.odom_history.clear()
            else: return
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        pose=np.array([p.x,p.y,yaw(q)],np.float32)
        if np.isfinite(pose).all(): self.odom=(ns,pose); self.odom_history.append(self.odom)

    def action_source_cb(self,msg):
        self.action_source=str(msg.data); self.action_source_received_ns=self.get_clock().now().nanoseconds
        if self.action_source != self.meta["action_source"]:
            self.previous=None; self.pending_scans.clear(); self.acks.clear()

    def ack_cb(self,msg):
        ns=stamp_ns(msg.header.stamp); cmd=np.array([msg.twist.linear.x,msg.twist.angular.z],np.float32)
        if ns<=0 or not np.isfinite(cmd).all(): return
        if self.acks and ns<=self.acks[-1][0]:
            # Reset temporal continuity on a time jump/out-of-order ACK.
            self.previous=None; self.acks.clear()
        self.acks.append((ns,cmd))

    def armed_cb(self,msg):
        was_armed=self.armed
        self.armed=bool(msg.data); self.armed_received_ns=self.get_clock().now().nanoseconds
        if not self.armed:
            if was_armed: self.previous=None
            self.acks.clear()

    def health_cb(self,msg):
        healthy=bool(msg.data)
        if healthy!=self.odom_healthy: self.health_generation+=1; self.previous=None
        self.odom_healthy=healthy; self.health_received_ns=self.get_clock().now().nanoseconds
        if not healthy:
            self.previous=None; self.pending_scans.clear(); self.odom_history.clear(); self.odom=None

    def context_cb(self,msg):
        try:
            context=json.loads(msg.data)
            instruction=context["instruction"]; target=context["target"]; generation=int(context["generation"])
            if not isinstance(instruction,str) or not isinstance(target,str): return
        except (ValueError,KeyError,TypeError): return
        names=[c["name"] for c in self.meta["candidates"]]
        # Atomic context changes invalidate the interval straddling the label boundary.
        self.previous=None; self.pending_scans.clear(); self.context_generation=generation
        self.instruction=instruction[:512]; self.label=names.index(target) if target in names else -1

    def on_snapshot(self,snapshot):
        """Overridden by read-only shadow evaluator."""

    def scan_cb(self,msg):
        ns=stamp_ns(msg.header.stamp); now=self.get_clock().now().nanoseconds
        if ns<=self.last_scan_ns or ns<=0:
            self.previous=None; self.pending_scans.clear(); self.dropped+=1
            if 0<ns<self.last_scan_ns: self.last_scan_ns=ns
            return
        self.last_scan_ns=ns
        if not 0<=now-ns<=int(self.limits["scan_age"]*1e9):
            self.previous=None; self.dropped+=1; return
        if len(self.pending_scans)==self.pending_scans.maxlen:
            self.previous=None; self.dropped+=1
        self.pending_scans.append((msg,now))
        self.drain_pending()

    def drain_pending(self):
        now=self.get_clock().now().nanoseconds
        while self.pending_scans:
            msg,received_ns=self.pending_scans[0]; ns=stamp_ns(msg.header.stamp)
            if not scan_ready(ns,received_ns,now,self.odom_history,self.args.odom_wait): return
            self.pending_scans.popleft(); self.record_scan(msg,now)

    def record_scan(self,msg,now):
        ns=stamp_ns(msg.header.stamp)
        if self.action_source != self.meta["action_source"] or not 0 <= now-self.action_source_received_ns <= 300000000:
            self.previous=None; self.acks.clear(); self.dropped+=1; return
        # RF2O output arrives after its scan. A short queue allows same-stamp
        # matching; bounded histories never pair a future odometry/image stamp.
        pair=synchronized_pair(ns,now,self.rgb_history,self.odom_history,self.odom_healthy,self.health_received_ns,self.limits)
        if pair is None:
            self.previous=None; self.dropped+=1; return
        rgb,odom=pair
        lidar,mask=sector_scan(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,self.meta["sectors"],self.meta["lidar_max_m"])
        map_pose=np.zeros(3,np.float32); map_valid=False
        try:
            transform=self.tf.lookup_transform(self.args.map_frame,self.args.base_frame,Time.from_msg(msg.header.stamp)).transform
            map_pose=np.array([transform.translation.x,transform.translation.y,yaw(transform.rotation)],np.float32)
            map_valid=bool(np.isfinite(map_pose).all())
        except TransformException: pass
        # Timestamp identifies the latest ACK preceding this sensor frame.
        before=[a for a in self.acks if a[0]<=min(ns,odom[0])]
        armed_fresh=self.armed and 0<=now-self.armed_received_ns<=int(self.limits["armed_age"]*1e9)
        ack=before[-1] if armed_fresh and before and ns-before[-1][0]<=int(self.limits["ack_gap"]*1e9) else None
        snapshot=dict(ns=ns,rgb_ns=rgb[0],rgb=rgb[1],odom_ns=odom[0],odom=odom[1],lidar=lidar,lidar_mask=mask,map_pose=map_pose,map_pose_valid=map_valid,instruction=self.instruction,label=self.label,ack=ack,received_ns=now,health_received_ns=self.health_received_ns,health_generation=self.health_generation)
        self.on_snapshot(snapshot)
        old=self.previous
        if old is None: self.previous=snapshot; return
        dt=(ns-old["ns"])*1e-9
        if dt<self.args.min_dt: return
        self.previous=snapshot
        if dt>self.args.max_dt:
            self.dropped+=1; return
        if snapshot["odom_ns"]<=old["odom_ns"] or snapshot["health_generation"]!=old["health_generation"]:
            self.dropped+=1; return
        action_valid=old["ack"] is not None and ack is not None
        if action_valid:
            history=[a for a in self.acks if old["ack"][0]<=a[0]<=ns]
            action_valid=bool(history and history[0][0]==old["ack"][0] and all(np.allclose(a[1],old["ack"][1],rtol=0,atol=1e-5) for a in history))
            times=[a[0] for a in history]+[ns]
            action_valid=action_valid and np.diff(times).max(initial=0)<=self.limits["ack_gap"]*1e9
        if old["instruction"]!=snapshot["instruction"] or old["label"]!=snapshot["label"]:
            self.dropped+=1; return
        if not self.args.out: return
        self.rows.append(dict(rgb=old["rgb"],lidar=old["lidar"],lidar_mask=old["lidar_mask"],odom=old["odom"],map_pose=old["map_pose"],map_pose_valid=np.array(old["map_pose_valid"]),next_lidar=lidar,next_lidar_mask=mask,next_odom=snapshot["odom"],action=old["ack"][1] if action_valid else np.full(2,np.nan,np.float32),dt=np.array(dt,np.float32),action_valid=np.array(action_valid),instruction=np.array(old["instruction"]),label=np.array(old["label"],np.int64),session=np.array(self.session),start_ns=np.array(old["ns"],np.int64),end_ns=np.array(ns,np.int64),ack_ns=np.array(old["ack"][0] if action_valid else 0,np.int64),rgb_ns=np.array(old["rgb_ns"],np.int64),next_rgb_ns=np.array(snapshot["rgb_ns"],np.int64),scan_ns=np.array(old["ns"],np.int64),next_scan_ns=np.array(ns,np.int64),odom_ns=np.array(old["odom_ns"],np.int64),next_odom_ns=np.array(snapshot["odom_ns"],np.int64),odom_healthy=np.array(True),next_odom_healthy=np.array(True),snapshot_received_ns=np.array(old["received_ns"],np.int64),next_snapshot_received_ns=np.array(now,np.int64),health_received_ns=np.array(old["health_received_ns"],np.int64),next_health_received_ns=np.array(snapshot["health_received_ns"],np.int64)))
        if len(self.rows)>=self.args.max_rows: raise KeyboardInterrupt

    def save(self):
        if not self.args.out or not self.rows: return
        path=Path(self.args.out)
        if path.exists(): raise FileExistsError("Refusing to overwrite a recording")
        path.parent.mkdir(parents=True,exist_ok=True); save_dataset(path,self.rows,self.meta)
        self.get_logger().info(f"Saved {len(self.rows)} transitions to {path}")


def parser():
    p=argparse.ArgumentParser(__doc__)
    p.add_argument("--meta",required=True); p.add_argument("--out",required=True); p.add_argument("--session")
    p.add_argument("--image-topic",default="/camera/image_raw"); p.add_argument("--scan-topic",default="/scan")
    p.add_argument("--odom-topic",default="/odom"); p.add_argument("--ack-topic",default="/room_base/applied_cmd_vel")
    p.add_argument("--health-topic",default="/room_base/odom_healthy")
    p.add_argument("--map-frame",default="map"); p.add_argument("--odom-frame",default="odom"); p.add_argument("--base-frame",default="base_link")
    for name in DEFAULT_LIMITS_S:
        p.add_argument("--max-"+name.replace("_","-"),type=float,default=None,help="Optional stricter limit than metadata recording_limits_s")
    p.add_argument("--odom-wait",type=float,default=.12,help="Wait for RF2O output after a scan, bounded by max scan age")
    p.add_argument("--min-dt",type=float,default=.15); p.add_argument("--max-dt",type=float,default=.35); p.add_argument("--max-rows",type=int,default=2000)
    return p


def main():
    args,ros_args=parser().parse_known_args()
    if Path(args.out).exists(): raise FileExistsError("Use a new session output path")
    rclpy.init(args=ros_args); node=Recorder(args)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.save(); node.destroy_node(); rclpy.shutdown()


if __name__=="__main__": main()
