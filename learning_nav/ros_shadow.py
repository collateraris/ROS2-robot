"""ROS read-only ML observer; publishes JSON diagnostics, never goals or Twist."""
import argparse
import json
import numpy as np
import rclpy
from std_msgs.msg import String
from .ros_record import Recorder,parser
from .infer import load_checkpoint,predict
from .train_utils import tensors


class Shadow(Recorder):
    def __init__(self,args):
        super().__init__(args,"learning_shadow")
        self.models,self.kind=load_checkpoint(args.checkpoint,self.meta,args.device)
        self.publisher=self.create_publisher(String,"/room_learning/shadow_prediction",10)
        self.last_prediction=0

    def on_snapshot(self,s):
        if s["ns"]-self.last_prediction<500000000: return
        if self.kind=="world" and s["ack"] is None: return
        if self.kind=="vla" and (not s["map_pose_valid"] or not s["instruction"]): return
        # World predictions describe holding the last ACKed setpoint for horizon;
        # no future applied action is claimed or sent to the robot.
        data={"rgb":s["rgb"][None],"lidar":s["lidar"][None],"lidar_mask":s["lidar_mask"][None],"odom":s["odom"][None],"map_pose":s["map_pose"][None],"next_lidar":s["lidar"][None],"next_lidar_mask":s["lidar_mask"][None],"next_odom":s["odom"][None],"action":np.array([s["ack"][1] if s["ack"] else [0.,0.]],np.float32),"dt":np.array([self.args.horizon],np.float32),"instruction":np.array([s["instruction"]]),"label":np.array([-1])}
        b=tensors(data,np.array([0]),self.args.device,self.meta)
        result=predict(self.models,self.kind,b,self.meta,self.args.confidence)
        result["source_scan_stamp_ns"]=s["ns"]; result["hypothetical_hold_seconds"]=self.args.horizon if self.kind=="world" else None
        result["odometry_source"]=self.meta["odometry_source"]; result["action_source"]=self.meta["action_source"]
        result["odom_healthy"]=True; result["motion_target_is_ground_truth"]=False
        msg=String(); msg.data=json.dumps(result,ensure_ascii=False); self.publisher.publish(msg); self.last_prediction=s["ns"]


def main():
    p=parser(); p.description=__doc__
    # Observer stores no dataset and cannot publish navigation goals.
    for action in p._actions:
        if action.dest=="out": action.required=False; action.default=None
    p.add_argument("--checkpoint",required=True); p.add_argument("--device",default="cpu"); p.add_argument("--horizon",type=float,default=.2); p.add_argument("--confidence",type=float,default=.8)
    args,ros_args=p.parse_known_args()
    if not 0<args.horizon<=1: p.error("horizon must be in (0,1]")
    args.out=None; rclpy.init(args=ros_args); node=Shadow(args)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()


if __name__=="__main__": main()
