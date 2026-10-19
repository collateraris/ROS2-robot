"""Week 3: model -> symbolic plan -> grounded metric goal -> Nav2 action.

This node never arms a motor and never publishes Twist. Run instead of the old
room_mission. Default shadow_only means neither path nor navigation requests.
"""
from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path
import io
import json
import math
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid as GridMessage
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from room_planning.backends import make_plan
from room_planning.core import (GenerationGate, MetricGoal, OccupancyGrid, Plan,
    PlanRejected, Scene, is_stop, parse_command, parse_scene, standoff_goal,
    validate_grounding, finite, planar_quaternion, validate_path, validate_grid_path)


def planar_yaw(q):
    return planar_quaternion(q.x,q.y,q.z,q.w)


@dataclass
class InferenceJob:
    future: object
    token: int
    request_id: str
    instruction: str
    scene: Scene
    started: float
    image_stamp_ns: int


class TaskCoordinator(Node):
    def __init__(self):
        super().__init__("room_tasks")
        defaults=dict(backend="rules",model="qwen2.5vl:7b",ollama_endpoint="http://127.0.0.1:11434",
            shadow_only=True,enable_execution=False,waypoints_file="",waypoints_confirmed=False,
            use_image=False,image_topic="/camera/image_raw",scene_topic="/room_semantics/objects",
            max_scene_age_s=1.0,max_image_age_s=.5,planning_timeout_s=30.,http_timeout_s=25.,
            path_timeout_s=5.,goal_timeout_s=60.,max_target_movement_m=.3,max_uncertainty_m=.3,
            minimum_score=.5,robot_radius_m=.18,clearance_margin_m=.05,standoff_m=.8,
            tf_max_age_s=.5,odom_health_timeout_s=.3,
            log_file="logs/week03_tasks.jsonl")
        self.p={name:self.declare_parameter(name,value).value for name,value in defaults.items()}
        for name in ("max_scene_age_s","max_image_age_s","planning_timeout_s","http_timeout_s","path_timeout_s",
                     "goal_timeout_s","max_target_movement_m","max_uncertainty_m","robot_radius_m",
                     "clearance_margin_m","standoff_m","tf_max_age_s","odom_health_timeout_s"):
            if not finite(self.p[name]) or self.p[name]<=0: raise ValueError(f"{name} must be positive")
        if self.p["planning_timeout_s"]>30 or self.p["http_timeout_s"]>30 or not .3<=self.p["standoff_m"]<=3:
            raise ValueError("invalid model deadline/standoff")
        if not 0<=self.p["minimum_score"]<=1 or self.p["backend"] not in ("rules","ollama"):
            raise ValueError("invalid score/backend")
        self.waypoints=self.load_waypoints()
        self.scene=None;self.grid=None;self.image=None
        self.healthy=False;self.health_at=-math.inf
        self.armed=False;self.armed_seen=False
        self.gate=GenerationGate();self.request_id="";self.instruction=""
        self.plan=None;self.reference=None;self.step_index=0;self.phase="idle";self.deadline=math.inf
        self.action_handle=None;self.path_handle=None;self.goal=None;self.anchor=None
        self.pool=ThreadPoolExecutor(max_workers=2,thread_name_prefix="room_planner")
        self.jobs=[]
        self.log_path=Path(self.p["log_file"]).expanduser();self.log_path.parent.mkdir(parents=True,exist_ok=True)
        # Fail at startup if evidence cannot be written. No images, tokens, HTTP bodies or credentials are logged.
        with self.log_path.open("a",encoding="utf-8"): pass
        self.events=self.create_publisher(String,"/room_tasks/events",20)
        self.plans=self.create_publisher(String,"/room_tasks/plan",10)
        latch=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.estop=self.create_publisher(Bool,"/room_base/estop",latch)
        self.demo_context=self.create_publisher(String,"/room_nav/demo_context",latch)
        self.context("","")
        self.path_client=ActionClient(self,ComputePathToPose,"/compute_path_to_pose")
        self.nav_client=ActionClient(self,NavigateToPose,"/navigate_to_pose")
        self.tf=Buffer();self.tf_listener=TransformListener(self.tf,self)
        self.create_subscription(String,"/room_tasks/command",self.command,10)
        self.create_subscription(String,self.p["scene_topic"],self.scene_cb,10)
        self.create_subscription(GridMessage,"/map",self.map_cb,latch)
        self.create_subscription(Bool,"/room_base/odom_healthy",self.health_cb,1)
        self.create_subscription(Bool,"/room_base/armed",self.armed_cb,latch)
        if self.p["use_image"]:
            self.create_subscription(Image,self.p["image_topic"],self.image_cb,qos_profile_sensor_data)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.event("ready","shadow" if self.shadow else "execution_enabled",{"backend":self.p["backend"],"waypoint_names":sorted(self.waypoints)})

    @property
    def shadow(self): return self.p["shadow_only"] or not self.p["enable_execution"]

    def now_ns(self): return self.get_clock().now().nanoseconds

    def context(self,instruction,target):
        self.demo_context.publish(String(data=json.dumps({"instruction":instruction,"target":target,
            "generation":self.gate.generation},ensure_ascii=False)))

    def load_waypoints(self):
        if not self.p["waypoints_confirmed"]: return {}
        if not self.p["waypoints_file"]: raise ValueError("confirmed waypoints need a file")
        import yaml
        document=yaml.safe_load(Path(self.p["waypoints_file"]).read_text(encoding="utf-8"))
        if not isinstance(document,dict) or document.get("frame_id")!="map" or not isinstance(document.get("waypoints"),dict):
            raise ValueError("waypoint file requires frame_id map and waypoints mapping")
        result={}
        for name,row in document["waypoints"].items():
            if not isinstance(name,str) or not name or len(name)>80 or not isinstance(row,dict): raise ValueError("invalid waypoint")
            pose=row.get("pose")
            if not isinstance(pose,list) or len(pose)!=3 or not all(finite(v) for v in pose): raise ValueError("invalid waypoint pose")
            result[name]=MetricGoal(*map(float,pose))
        if not result: raise ValueError("empty confirmed waypoint file")
        return result

    def event(self,event,status,details=None,request_id=None,generation=None):
        value={"schema_version":1,"stamp_ns":self.now_ns(),"request_id":self.request_id if request_id is None else request_id,
               "generation":self.gate.generation if generation is None else generation,
               "event":event,"status":status,"details":details or {}}
        text=json.dumps(value,ensure_ascii=False,allow_nan=False)
        with self.log_path.open("a",encoding="utf-8") as stream: stream.write(text+"\n")
        self.events.publish(String(data=text))
        self.get_logger().info(f"{event}: {status}: {value['request_id']}")

    def scene_cb(self,msg):
        try:
            value=parse_scene(msg.data)
            if self.scene is not None and value.stamp_ns<self.scene.stamp_ns: return
            self.scene=value
        except PlanRejected as exc:
            self.scene=None
            self.event("scene","rejected",{"reason":str(exc)})

    def map_cb(self,msg):
        try:
            if msg.header.frame_id!="map" or abs(msg.info.origin.position.z)>.02: raise PlanRejected("map must be planar map-frame")
            self.grid=OccupancyGrid(msg.info.width,msg.info.height,float(msg.info.resolution),
                msg.info.origin.position.x,msg.info.origin.position.y,planar_yaw(msg.info.origin.orientation),tuple(int(v) for v in msg.data))
        except (ValueError,PlanRejected) as exc:
            self.grid=None;self.event("map","rejected",{"reason":str(exc)})

    def health_cb(self,msg):
        self.healthy=bool(msg.data);self.health_at=time.monotonic()
        if not self.healthy and self.gate.active and not self.shadow and self.phase in ("path_pending","path_running","nav_pending","navigating"):
            self.finish("failed","laser odometry health lost",halt=True)

    def armed_cb(self,msg):
        previous=self.armed;self.armed=bool(msg.data);self.armed_seen=True
        if previous and not self.armed and self.gate.active and not self.shadow and self.phase in ("nav_pending","navigating"):
            self.finish("failed","motor ARM lost; explicit new task and re-arm required",halt=True)

    def image_cb(self,msg):
        # Optional VLM context. Root camera/perception supplies proper acquisition stamps.
        try:
            stamp=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            if stamp<=0 or self.image is not None and stamp<=self.image[0]: return
            channels={"rgb8":3,"bgr8":3,"rgba8":4,"bgra8":4,"mono8":1}.get(msg.encoding)
            if channels is None or not 0<msg.width<=4096 or not 0<msg.height<=4096 or msg.step<msg.width*channels or len(msg.data)!=msg.step*msg.height:
                raise ValueError("unsupported raw camera frame")
            from PIL import Image as PILImage
            rows=b"".join(bytes(msg.data[y*msg.step:y*msg.step+msg.width*channels]) for y in range(msg.height))
            mode={1:"L",3:"RGB",4:"RGBA"}[channels]
            im=PILImage.frombytes(mode,(msg.width,msg.height),rows)
            if msg.encoding.startswith("bgr"):
                red,green,blue,*extra=im.split();im=PILImage.merge("RGB",(blue,green,red))
            im=im.convert("RGB");im.thumbnail((640,480));buffer=io.BytesIO();im.save(buffer,format="JPEG",quality=80)
            self.image=(stamp,buffer.getvalue())
        except (ValueError,ImportError): self.image=None

    def command(self,msg):
        try: request_id,instruction=parse_command(msg.data)
        except PlanRejected as exc:
            self.event("task_result","rejected",{"reason":str(exc)},request_id="");return
        request_id=request_id or str(uuid.uuid4())
        if is_stop(instruction):
            if self.gate.active: self.finish("canceled","operator STOP",halt=True)
            else: self.halt()
            self.context("стоп","STOP")
            self.event("task_result","canceled",{"reason":"operator STOP"},request_id=request_id);return
        if self.gate.active: self.finish("canceled","superseded by a new task; explicit re-arm required",halt=not self.shadow)
        self.request_id=request_id;self.instruction=instruction
        self.step_index=0
        token=self.gate.begin();self.phase="planning"
        if len(self.jobs)>=2:
            self.finish("rejected","planner workers busy; wait for previous bounded requests");return
        # A waypoint-only task may work with an empty scene; objects can never be invented.
        scene=self.scene or Scene(self.now_ns(),self.now_ns(),self.now_ns(),())
        image=None;image_stamp=0
        if self.p["use_image"]:
            if self.image is None or not 0<=(self.now_ns()-self.image[0])*1e-9<=self.p["max_image_age_s"]:
                self.finish("rejected","optional VLM image is missing/stale");return
            image_stamp,image=self.image
        self.event("request","planning",{"instruction":instruction,"backend":self.p["backend"],
                   "scene_stamp_ns":scene.stamp_ns,"image_input_stamp_ns":image_stamp})
        future=self.pool.submit(make_plan,self.p["backend"],instruction,scene,self.waypoints,
            endpoint=self.p["ollama_endpoint"],model=self.p["model"],image_jpeg=image,timeout_s=self.p["http_timeout_s"])
        self.jobs.append(InferenceJob(future,token,request_id,instruction,scene,time.monotonic(),image_stamp))
        self.deadline=time.monotonic()+self.p["planning_timeout_s"]

    def halt(self):
        self.estop.publish(Bool(data=True))
        for handle in (self.path_handle,self.action_handle):
            if handle is not None: handle.cancel_goal_async()
        self.path_handle=None;self.action_handle=None

    def finish(self,status,reason,halt=False):
        # Invalidate BEFORE cancel; future action accepts/results cannot dispatch more work.
        generation=self.gate.generation
        self.gate.cancel()
        if halt: self.halt()
        self.event("task_result",status,{"reason":reason,"completed_steps":self.step_index},generation=generation)
        self.context("","")
        self.plan=None;self.reference=None;self.goal=None;self.anchor=None;self.phase="idle";self.deadline=math.inf

    def robot_pose(self):
        transform=self.tf.lookup_transform("map","base_link",rclpy.time.Time())
        stamp=transform.header.stamp.sec*1_000_000_000+transform.header.stamp.nanosec
        if not 0<=(self.now_ns()-stamp)*1e-9<=self.p["tf_max_age_s"]: raise PlanRejected("robot map localization is stale")
        p=transform.transform.translation
        if not all(finite(v) for v in (p.x,p.y,p.z)) or abs(p.z)>.02: raise PlanRejected("invalid robot map pose")
        return MetricGoal(p.x,p.y,planar_yaw(transform.transform.rotation))

    def check_object(self,step):
        if self.scene is None: raise PlanRejected("trusted scene is unavailable")
        validate_grounding(Plan((step,)),self.instruction,self.scene,self.now_ns(),
            self.p["max_scene_age_s"],self.p["max_uncertainty_m"],self.p["minimum_score"],
            self.reference,self.p["max_target_movement_m"])
        obj=self.scene.lookup(step.object_id)
        if self.anchor is not None and math.hypot(obj.x-self.anchor.x,obj.y-self.anchor.y)>self.p["max_target_movement_m"]:
            raise PlanRejected("active target moved; no automatic person chasing")
        return obj

    def movement_ready(self):
        if self.grid is None: raise PlanRejected("map is absent/invalid")
        if not self.healthy or time.monotonic()-self.health_at>self.p["odom_health_timeout_s"]:
            raise PlanRejected("laser odometry health is false/stale")
        return self.robot_pose()

    def pose_message(self,goal):
        msg=PoseStamped();msg.header.frame_id="map";msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x=goal.x;msg.pose.position.y=goal.y
        msg.pose.orientation.z=math.sin(goal.yaw/2);msg.pose.orientation.w=math.cos(goal.yaw/2)
        return msg

    def dispatch(self):
        if not self.gate.active or self.plan is None: return
        if self.step_index>=len(self.plan.steps): self.finish("succeeded","all steps completed");return
        step=self.plan.steps[self.step_index]
        self.anchor=None
        if step.action=="stop":
            self.finish("succeeded","terminal stop step",halt=True)
            self.context("стоп","STOP");return
        if step.action=="wait":
            self.phase="waiting";self.deadline=time.monotonic()+step.seconds
            self.event("step","waiting",{"step":self.step_index,"seconds":step.seconds});return
        try:
            robot=self.movement_ready()
            if step.action=="goto_object":
                self.anchor=self.check_object(step)
                goal=standoff_goal(robot.x,robot.y,self.anchor,self.p["standoff_m"])
                if goal is None:
                    self.event("step","already_at_standoff",{"step":self.step_index});self.step_index+=1;self.dispatch();return
            else: goal=self.waypoints[step.name]
            self.grid.check(goal,self.p["robot_radius_m"]+self.p["clearance_margin_m"])
            if not self.path_client.server_is_ready() or not self.nav_client.server_is_ready(): raise PlanRejected("Nav2 path/navigation server unavailable")
            self.goal=goal;token=self.gate.generation;index=self.step_index
            request=ComputePathToPose.Goal();request.goal=self.pose_message(goal)
            request.start=self.pose_message(robot);request.use_start=True;request.planner_id="GridBased"
            self.phase="path_pending";self.deadline=time.monotonic()+self.p["path_timeout_s"]
            self.event("goal","path_requested",{"step":index,"action":step.document(),"goal":{"x":goal.x,"y":goal.y,"yaw":goal.yaw}})
            future=self.path_client.send_goal_async(request)
            future.add_done_callback(lambda f:self.path_accepted(f,token,index))
        except Exception as exc:
            self.finish("rejected",f"movement precondition: {exc}",halt=True)

    def path_accepted(self,future,token,index):
        try: handle=future.result()
        except Exception as exc:
            if self.gate.current(token): self.finish("failed",f"path request: {type(exc).__name__}",halt=True)
            return
        if not self.gate.accept_handle(token,handle):
            if self.gate.current(token): self.finish("rejected","Nav2 rejected path request",halt=True)
            return
        if index!=self.step_index: handle.cancel_goal_async();return
        self.path_handle=handle;self.phase="path_running"
        handle.get_result_async().add_done_callback(lambda f:self.path_result(f,token,index,handle))

    def path_result(self,future,token,index,handle):
        if not self.gate.current(token) or index!=self.step_index or self.path_handle is not handle: return
        self.path_handle=None
        try:
            wrapped=future.result();result=wrapped.result
            if wrapped.status!=4 or getattr(result,"error_code",0)!=0: raise PlanRejected("no successful Nav2 path")
            poses=result.path.poses
            robot=self.movement_ready()
            step=self.plan.steps[self.step_index]
            if step.action=="goto_object": self.check_object(step)
            self.grid.check(self.goal,self.p["robot_radius_m"]+self.p["clearance_margin_m"])
            path_rows=[(pose.header.frame_id,pose.pose.position.x,pose.pose.position.y,pose.pose.position.z,
                        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w)
                       for pose in poses]
            validate_path(path_rows,result.path.header.frame_id,robot,self.goal)
            validate_grid_path(path_rows,self.grid,self.p["robot_radius_m"]+self.p["clearance_margin_m"])
            request=NavigateToPose.Goal();request.pose=self.pose_message(self.goal)
            self.phase="nav_pending";self.deadline=time.monotonic()+self.p["goal_timeout_s"]
            self.event("goal","navigation_requested",{"step":index,"path_poses":len(poses)})
            sent=self.nav_client.send_goal_async(request)
            sent.add_done_callback(lambda f:self.nav_accepted(f,token,index))
        except Exception as exc: self.finish("rejected",f"path/dispatch validation: {exc}",halt=True)

    def nav_accepted(self,future,token,index):
        try: handle=future.result()
        except Exception as exc:
            if self.gate.current(token): self.finish("failed",f"navigation request: {type(exc).__name__}",halt=True)
            return
        if not self.gate.accept_handle(token,handle):
            if self.gate.current(token): self.finish("rejected","Nav2 rejected navigation goal",halt=True)
            return
        if index!=self.step_index: handle.cancel_goal_async();return
        self.action_handle=handle;self.phase="navigating"
        step=self.plan.steps[self.step_index]
        self.context(self.instruction,step.name if step.action=="goto_waypoint" else step.object_id)
        self.event("step","navigating",{"step":index})
        handle.get_result_async().add_done_callback(lambda f:self.nav_result(f,token,index,handle))

    def nav_result(self,future,token,index,handle):
        if not self.gate.current(token) or index!=self.step_index or self.action_handle is not handle: return
        self.action_handle=None
        try:
            wrapped=future.result()
            if wrapped.status!=4 or getattr(wrapped.result,"error_code",0)!=0:
                self.finish("failed",f"Nav2 result status={wrapped.status}",halt=True);return
            self.movement_ready()
            step=self.plan.steps[self.step_index]
            if step.action=="goto_object": self.check_object(step)
            self.context("","")
            self.event("step","succeeded",{"step":index});self.step_index+=1;self.dispatch()
        except Exception as exc: self.finish("failed",f"Nav2 result: {type(exc).__name__}",halt=True)

    def tick(self):
        now=time.monotonic()
        for job in list(self.jobs):
            if not job.future.done(): continue
            self.jobs.remove(job)
            if not self.gate.current(job.token):
                self.event("plan","discarded",{"reason":"obsolete generation"},request_id=job.request_id,generation=job.token);continue
            try:
                if now-job.started>self.p["planning_timeout_s"]: raise PlanRejected("planning deadline exceeded")
                plan=job.future.result()
                current=self.scene or Scene(self.now_ns(),self.now_ns(),self.now_ns(),())
                validate_grounding(plan,job.instruction,current,self.now_ns(),self.p["max_scene_age_s"],
                    self.p["max_uncertainty_m"],self.p["minimum_score"],job.scene,self.p["max_target_movement_m"])
                self.plan=plan;self.reference=job.scene;self.step_index=0
                details={"plan":plan.document(),"backend":self.p["backend"],"inference_seconds":now-job.started,
                         "scene_input_stamp_ns":job.scene.stamp_ns,"image_input_stamp_ns":job.image_stamp_ns}
                self.plans.publish(String(data=json.dumps(details,ensure_ascii=False)))
                self.event("plan","validated",details)
                if self.shadow: self.finish("shadow","validated plan; execution disabled")
                else: self.dispatch()
            except Exception as exc: self.finish("rejected",str(exc))
        if not self.gate.active: return
        if self.phase in ("path_pending","path_running","nav_pending","navigating"):
            try:
                self.movement_ready()
                step=self.plan.steps[self.step_index]
                if step.action=="goto_object": self.check_object(step)
                if self.goal is not None:
                    self.grid.check(self.goal,self.p["robot_radius_m"]+self.p["clearance_margin_m"])
            except Exception as exc:
                self.finish("failed",f"active goal invalidated: {exc}",halt=True);return
        if now>=self.deadline:
            if self.phase=="waiting": self.step_index+=1;self.dispatch()
            else: self.finish("failed",f"{self.phase} timeout",halt=self.phase!="planning")

    def close(self):
        if self.gate.active: self.finish("canceled","coordinator shutdown",halt=not self.shadow)
        self.pool.shutdown(wait=False,cancel_futures=True)


def main(args=None):
    rclpy.init(args=args);node=TaskCoordinator()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__=="__main__": main()
