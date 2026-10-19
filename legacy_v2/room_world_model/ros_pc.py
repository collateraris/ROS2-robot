"""ROS2 PC worker. One latest frame, one inference at a time, no GPU work on Pi."""
from __future__ import annotations
import argparse
import json
import threading
import time
import traceback
from .configuration import load_config,resolve_path,calibration_fingerprint
from .ros_interfaces import (message_types, frame_from_ros, proposal_to_ros,
    task_to_ros, task_from_ros, stop_to_ros, planner_status_to_ros)
import math
import uuid
from .perception import CameraCalibration,CameraPerception,PerceptionConfig
from .engine import ControlEngine


def main(argv=None):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    from rclpy.action import ActionServer, GoalResponse, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.task import Future
    from rclpy.duration import Duration
    from rclpy.clock import Clock, ClockType
    from roomwm_msgs.action import ApproachPerson
    interfaces = message_types()
    parser=argparse.ArgumentParser(description="GPU perception/world-model inference, shadow by default")
    parser.add_argument("--config",required=True)
    parser.add_argument("--mode",choices=["shadow","teleop","baseline","wm"])
    parser.add_argument("--shadow-controller",choices=["baseline","wm","teleop"],default="baseline")
    parser.add_argument("--checkpoint")
    parser.add_argument("--device")
    parser.add_argument("--wm-trial",action="store_true",
                        help="Reviewed supervised WM comparison trial; Pi must allow_wm_trials")
    args,ros_args=parser.parse_known_args(argv)
    c=load_config(args.config)
    mode=args.mode or c["runtime"]["mode"]
    if args.wm_trial and mode!="wm":
        raise SystemExit("--wm-trial is valid only with --mode wm")
    device=args.device or c["runtime"]["device"]
    calibration=CameraCalibration.from_json(resolve_path(c,c["camera"]["calibration_file"]))
    rectifier=None
    if c["camera"].get("intrinsics_file"):
        from .calibrate import ImageRectifier
        rectifier=ImageRectifier(resolve_path(c,c["camera"]["intrinsics_file"]))
        rectifier.validate_calibration(calibration)
    elif calibration.accepted and not c["camera"].get("already_rectified",False):
        raise SystemExit("Accepted camera needs intrinsics_file, or explicitly verified already_rectified=true")
    perception=CameraPerception.from_pretrained(calibration,
        detector_weights=str(resolve_path(c,c["perception"]["detector_weights"])),
        depth_model_id=c["perception"]["depth_model_id"],device=device,
        depth_model_revision=c["perception"].get("depth_model_revision"),
        local_files_only=c["perception"]["local_files_only"],
        config=PerceptionConfig(depth_scale_m=c["perception"]["depth_scale_m"]))
    model=planner=None
    model_hash=""
    if mode=="wm" or (mode=="shadow" and args.shadow_controller=="wm"):
        from .model import load_checkpoint
        from .acceptance import load_acceptance,load_trial_eligibility
        from .planner import MPCPlanner,PlannerConfig
        model,metadata=load_checkpoint(resolve_path(c,args.checkpoint or c["runtime"]["checkpoint"]),device,
             for_live=mode=="wm",calibration_fingerprint=calibration_fingerprint(c),
             depth_scale_m=c["perception"]["depth_scale_m"])
        checkpoint_path=resolve_path(c,args.checkpoint or c["runtime"]["checkpoint"])
        import hashlib
        model_hash=hashlib.sha256(checkpoint_path.read_bytes()).hexdigest()
        if mode=="wm":
            enabled="allow_wm_trials" if args.wm_trial else "world_model_validated"
            evidence="world_model_trial_file" if args.wm_trial else "world_model_acceptance_file"
            if not c["runtime"].get(enabled):
                raise SystemExit(f"runtime.{enabled}=false; use --mode shadow --shadow-controller wm")
            check=load_trial_eligibility if args.wm_trial else load_acceptance
            check(resolve_path(c,c["runtime"].get(evidence, "")),
                calibration_fingerprint=calibration_fingerprint(c), checkpoint_path=checkpoint_path)
            if model_hash!=c["runtime"].get("world_model_sha256"):
                raise SystemExit("Configured WM hash differs from checkpoint")
        planner=MPCPlanner(model,PlannerConfig(horizon=c["planner"]["horizon"],pulse_s=c["safety"]["pulse_s"],
             observe_wait_s=c["planner"]["wait_s"],depth_scale_m=c["perception"]["depth_scale_m"],
             stop_distance_m=c["perception"]["person_standoff_m"],max_forward=1.,max_yaw=.30,
             max_forward_change=1.,max_yaw_change=.30,max_aux_disagreement=c["planner"]["uncertainty_limit"]))
    engine=ControlEngine(c,perception,model=model,planner=planner,rectifier=rectifier,
                         mode=mode,shadow_controller=args.shadow_controller)
    rclpy.init(args=ros_args)
    sensor_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
    control_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
    proposal_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE,
        lifespan=Duration(seconds=c["safety"]["max_response_delay_s"]))

    class PCNode(Node):
        def __init__(self):
            super().__init__("roomwm_pc")
            if self.get_parameter("use_sim_time").value:
                raise ValueError("Real Pi/PC controller forbids use_sim_time")
            self._condition=threading.Condition()
            self._latest=None
            self._pending_task=None
            self._goal_id=""
            self._lease=""
            self._armed=False
            self._status_received=0.
            self._pi_status=None
            self._last_observation=None
            self._closed=False
            self._active_action=None
            self._action_records={}
            self._action_reserved=False
            self.pub=self.create_publisher(interfaces.ActuationProposal,"/roomwm/actuation_proposal",proposal_qos)
            self.perception_pub=self.create_publisher(interfaces.PerceptionState,"/roomwm/perception",sensor_qos)
            self.stop_pub=self.create_publisher(interfaces.Stop,"/roomwm/emergency_stop",control_qos)
            self.status_pub=self.create_publisher(interfaces.PlannerStatus,"/roomwm/pc_status",control_qos)
            self.task_pub=self.create_publisher(interfaces.TaskRequest,"/roomwm/task",control_qos)
            self.create_subscription(interfaces.FramePacket,"/roomwm/frame",self.frame,sensor_qos)
            self.create_subscription(interfaces.SafetyStatus,"/roomwm/status",self.status,control_qos)
            self.create_subscription(interfaces.TaskRequest,"/roomwm/task",self.task,control_qos)
            self.create_subscription(interfaces.Stop,"/roomwm/emergency_stop",self.emergency_stop,control_qos)
            self.create_subscription(interfaces.ActuationAck,"/roomwm/actuation_ack",self.ack,
                QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE))
            self.actions=ActionServer(self,ApproachPerson,"/roomwm/approach_person",
                execute_callback=self.execute_action,goal_callback=self.goal_request,
                cancel_callback=self.cancel_request,handle_accepted_callback=self.goal_accepted,
                callback_group=ReentrantCallbackGroup())
            self.create_timer(.05,self.action_tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
            self.worker=threading.Thread(target=self.run_worker,daemon=True,name="latest-only-world-model")
            self.worker.start()
            self.get_logger().info(f"PC controller={mode}, device={device}; awaiting typed Pi frames")

        def frame(self,msg):
            with self._condition:
                self._latest=(msg,time.monotonic())
                self._condition.notify()

        def status(self,msg):
            if msg.schema_version!=2:
                self._lease=""; self._armed=False
                return
            matches_goal = msg.goal_id == self._goal_id
            self._lease=msg.lease_id if matches_goal else ""
            self._armed=msg.armed if matches_goal else False
            self._pi_status=msg
            self._status_received=time.monotonic()

        def install_task(self,value):
            with self._condition:
                if value["goal_id"]==self._goal_id:
                    return
                self._pending_task=value
                self._goal_id=value["goal_id"]
                self._lease=""
                self._armed=False
                self._last_observation=None
                self._condition.notify()

        def task(self,msg):
            try:
                value=task_from_ros(msg)
                if self._active_action and value["goal_id"]!=self._active_action["goal_id"]:
                    self.finish_action("superseded_by_new_task",publish_stop=False)
                self.install_task(value)
            except (ValueError,TypeError,AttributeError):
                self.stop_pub.publish(stop_to_ros("malformed_text_goal"))

        def emergency_stop(self,msg):
            if msg.goal_id and msg.goal_id!=self._goal_id:
                return
            if msg.reason=="action_cancel_requested" and self._active_action:
                self.install_task({"goal_id":uuid.uuid4().hex,"kind":"text","text":"стоп"})
                return  # action_tick reports CANCELED once rclpy enters canceling
            if self._active_action:
                self.finish_action(msg.reason or "operator_stop",publish_stop=False)
            # Invalidate an in-flight GPU result immediately, including a text/manual task.
            self.install_task({"goal_id":uuid.uuid4().hex,"kind":"text","text":"стоп"})

        def goal_request(self,request):
            valid=(mode!="teleop" and request.avoid_obstacles
                and math.isfinite(request.standoff_m)
                and abs(request.standoff_m-c["perception"]["person_standoff_m"])<1e-6
                and math.isfinite(request.timeout_s) and 0.<request.timeout_s<=c["safety"]["task_timeout_s"])
            with self._condition:
                if not valid or self._action_reserved or self._active_action is not None:
                    return GoalResponse.REJECT
                self._action_reserved=True
            return GoalResponse.ACCEPT

        def goal_accepted(self,handle):
            goal_id=bytes(handle.goal_id.uuid).hex()
            self._active_action={"handle":handle,"goal_id":goal_id,"future":Future(),
                "started":time.monotonic(),"timeout":handle.request.timeout_s,
                "pulses":0,"seen_acks":set(),"was_armed":False}
            self._action_records[id(handle)]=self._active_action
            handle.execute()

        async def execute_action(self,handle):
            action=self._action_records.get(id(handle))
            if action is None:
                if handle.is_active:
                    handle.abort()
                result=ApproachPerson.Result(); result.reason="goal_lifecycle_error"
                return result
            try:
                if not action["future"].done():
                    task={"goal_id":action["goal_id"],"kind":"text","text":"подъедь к человеку и обходи препятствия"}
                    self.install_task(task)
                    self.task_pub.publish(task_to_ros(task))
                # This await yields the executor; cancellation/status never waits for CUDA.
                # An already finalized pre-execute goal returns its cached result.
                return await action["future"]
            finally:
                self._action_records.pop(id(handle),None)

        def cancel_request(self,handle):
            if self._active_action and self._active_action["handle"] is handle:
                # Physical stop is sent before waiting for a cancelled action result.
                self.stop_pub.publish(stop_to_ros("action_cancel_requested",self._active_action["goal_id"]))
                return CancelResponse.ACCEPT
            return CancelResponse.REJECT

        def ack(self,msg):
            action=self._active_action
            if (action and msg.schema_version==2 and msg.goal_id==action["goal_id"]
                    and msg.has_interval and msg.interval.complete and msg.interval.actuation_confirmed):
                key=(msg.boot_id,msg.nonce,msg.seq)
                if key not in action["seen_acks"]:
                    action["seen_acks"].add(key); action["pulses"]+=1

        def finish_action(self,reason,success=False,cancelled=False,publish_stop=True):
            action=self._active_action
            if action is None:
                return
            self._active_action=None
            self._action_reserved=False
            if publish_stop:
                self.stop_pub.publish(stop_to_ros(reason,action["goal_id"]))
            self.install_task({"goal_id":uuid.uuid4().hex,"kind":"text","text":"стоп"})
            handle=action["handle"]
            if cancelled:
                handle.canceled()
            elif success:
                handle.succeed()
            else:
                handle.abort()
            result=ApproachPerson.Result()
            result.success=bool(success); result.reason=str(reason)
            result.completed_pulses=action["pulses"]
            if not action["future"].done():
                action["future"].set_result(result)

        def action_tick(self):
            action=self._active_action
            if action is None:
                return
            handle=action["handle"]
            if handle.is_cancel_requested:
                self.finish_action("action_cancelled",cancelled=True); return
            now=time.monotonic()
            if now-action["started"]>action["timeout"]:
                self.finish_action("action_timeout"); return
            if now-action["started"]>1. and now-self._status_received>1.:
                self.finish_action("pi_status_lost"); return
            with self._condition:
                observed=self._last_observation
            action_armed=(self._armed and self._goal_id==action["goal_id"]
                and self._pi_status is not None and self._pi_status.goal_id==action["goal_id"])
            feedback=ApproachPerson.Feedback()
            feedback.phase="waiting_for_arm" if not action_armed else "waiting_for_fresh_observation"
            feedback.completed_pulses=action["pulses"]
            if observed and observed["goal_id"]==action["goal_id"]:
                p=observed["diagnostics"]["perception"]
                feedback.person_visible=p["person_visible"]
                feedback.person_distance_m=float(p["person_distance_m"])
                feedback.person_bearing_rad=float(p["person_bearing_rad"])
                feedback.phase=observed["diagnostics"]["reason"]
                if observed["abort"]:
                    success=feedback.phase=="goal_reached_stop"
                    self.finish_action(feedback.phase,success=success); return
            if action_armed:
                action["was_armed"]=True
            elif action["was_armed"] and self._pi_status is not None:
                self.finish_action("pi_disarmed:"+self._pi_status.reason); return
            handle.publish_feedback(feedback)

        def run_worker(self):
            while not self._closed:
                with self._condition:
                    self._condition.wait_for(lambda:self._closed or self._latest is not None,timeout=.5)
                    if self._closed:
                        return
                    if self._latest is None:
                        continue
                    item,self._latest=self._latest,None
                    pending,self._pending_task=self._pending_task,None
                raw,received=item
                try:
                    if pending is not None:
                        engine.set_task(pending)
                    packet,rgb=frame_from_ros(raw)
                    if not packet["settled"] and mode!="shadow":
                        continue
                    lease=self._lease if self._armed and time.monotonic()-self._status_received<1. else ""
                    proposal=engine.process(packet,rgb,lease)
                    with self._condition:
                        if self._closed or proposal["goal_id"]!=self._goal_id:
                            continue
                    total=time.monotonic()-received
                    proposal["command"]["inference_ms"]=total*1000
                    if total>c["safety"]["max_inference_s"]:
                        proposal["command"]["linear_mps"]=0.
                        proposal["command"]["angular_rps"]=0.
                        proposal["abort"]=bool(self._armed)
                        proposal["diagnostics"]["reason"]="pc_decode_perception_planning_deadline_exceeded"
                    if not lease:
                        proposal["command"]["linear_mps"]=0.
                        proposal["command"]["angular_rps"]=0.
                    if args.wm_trial:
                        proposal["controller"]="wm_trial"
                    typed=proposal_to_ros(proposal,engine.fingerprint,model_hash)
                    with self._condition:
                        if self._closed or proposal["goal_id"]!=self._goal_id:
                            continue
                        # Record the cause before the Pi can report its resulting STOP.
                        self._last_observation=proposal
                    self.pub.publish(typed)
                    self.perception_pub.publish(typed.current_perception)
                    self.status_pub.publish(planner_status_to_ros(typed,filter_steps=proposal["diagnostics"]["filter_steps"]))
                except Exception as exc:
                    if not self._closed:
                        self.stop_pub.publish(stop_to_ros("pc_inference_exception:"+str(exc)[:120]))
                        self.get_logger().error(traceback.format_exc())

        def close(self):
            self._closed=True
            self.finish_action("pc_shutdown")
            self.stop_pub.publish(stop_to_ros("pc_shutdown"))
            with self._condition:
                self._condition.notify_all()
            self.worker.join(timeout=1.)
            self.actions.destroy()

    node=None
    try:
        node=PCNode()
        # Async actions yield within the executor; CUDA remains in a separate worker.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close(); node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
