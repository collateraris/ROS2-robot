"""ROS2 Jazzy Pi node: camera, recording, safety governor and exclusive MCU owner."""
from __future__ import annotations
import argparse
from collections import deque
from dataclasses import asdict
import json
import time
import uuid

from .actuator import PulseSerial, mix_pwm
from .camera import LatestCamera
from .configuration import load_config, resolve_path, calibration_fingerprint, recording_provenance
from .frames import make_frame
from .protocol import FrameRef, MotionCommand, MetricGeometry
from .ros_interfaces import (message_types, frame_to_ros, proposal_from_ros, task_from_ros,
                             ack_to_ros, status_to_ros)
from .record import EpisodeRecorder
from .safety import SafetyConfig, SafetyGovernor


def build_safety_config(c: dict) -> SafetyConfig:
    r,s,rt=c["robot"],c["safety"],c["runtime"]
    return SafetyConfig(
        calibration_confirmed=rt["calibration_confirmed"],
        allow_camera_only_research=rt["allow_camera_only_research"],
        pulse_mode=True,max_motion_pulse_s=s["pulse_s"],
        settle_extra_s=s["settle_extra_s"],
        max_frame_age_s=s["max_frame_age_s"],max_geometry_age_s=s["max_frame_age_s"],
        max_capture_latency_bound_s=c["camera"].get("max_capture_age_at_dequeue_s", 1.0),
        max_response_delay_s=s["max_response_delay_s"],max_inference_s=s["max_inference_s"],
        deadman_timeout_s=s["deadman_timeout_s"],
        driver_latency_s=r["actuator_latency_bound_s"],
        max_linear_mps=r["linear_command_scale_mps"],max_angular_rps=r["angular_command_scale_rps"],
        speed_bound_mps=r["physical_speed_bound_mps"],
        angular_speed_bound_rps=r.get("physical_angular_speed_bound_rps",0.40),
        braking_decel_min_mps2=r["min_brake_decel_mps2"],
        angular_braking_decel_min_rps2=r.get("min_angular_brake_decel_rps2",0.80),
        motor_watchdog_s=r.get("motor_cutoff_bound_s",0.11),
        static_margin_m=s["clearance_margin_m"],
        obstacle_speed_bound_mps=s.get("obstacle_speed_bound_mps",0.0),
    )


def dispatch_motor(governor, motor, decision, previous_decision, robot, last_pulse_id, now_s):
    """Apply a decision immediately, also usable in tests without importing ROS.

    Normal pulse completion relies on the MCU's independent expiry. An explicit
    zero/fault while a pulse is pending sends a latched stop even if a callback
    has already overwritten the previous armed state. A pulse is sent once.
    """
    if motor is None:
        return decision,last_pulse_id
    explicit_zero=(not decision.moving and motor.pending is not None
                   and decision.reason!="pulse_complete")
    disarmed_stop=(not decision.armed and (motor.pending is not None or previous_decision.armed))
    if explicit_zero or disarmed_stop:
        if decision.armed:
            decision=governor.stop(decision.reason,now_s)
        if not motor._faulted:
            motor.emergency_stop(decision.reason,now_s)
        return decision,last_pulse_id
    if decision.moving and decision.pulse_id!=last_pulse_id:
        try:
            left,right=mix_pwm(decision.linear_mps,decision.angular_rps,robot["track_width_m"],
                               robot["wheel_command_scale_mps"],robot["pwm_limit"])
            if motor.send_pulse(decision.pulse_id,left,right,decision.pulse_remaining_s,now_s):
                last_pulse_id=decision.pulse_id
        except (ValueError,OSError) as exc:
            reason="motor_submit_failed:"+str(exc)
            decision=governor.stop(reason,now_s)
            if not motor._faulted:
                motor.emergency_stop(reason,now_s)
    return decision,last_pulse_id


def advance_pending_arm(governor, motor, deadline_s, now_s):
    """Retry only within the original explicit operator grant; never renew it."""
    if not deadline_s:
        return 0.0
    if now_s>=deadline_s:
        governor.stop("explicit_arm_handshake_timeout",now_s)
        return 0.0
    if motor is not None and motor.ready and motor.pending is None:
        if governor.arm(now_s,stationary_confirmed=True) is not None:
            return 0.0
    # Fresh geometry or physical settling may follow the MCU handshake. The
    # governor still checks deadman/ready/camera/geometry at every arm attempt.
    return deadline_s


def main(argv=None):
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.clock import Clock, ClockType
    from rclpy.qos import QoSProfile,ReliabilityPolicy,HistoryPolicy,DurabilityPolicy
    from std_msgs.msg import Bool
    from sensor_msgs.msg import CompressedImage
    from geometry_msgs.msg import TwistStamped
    from std_srvs.srv import SetBool, Trigger
    from rclpy.duration import Duration
    interfaces = message_types()

    parser=argparse.ArgumentParser(description="Pi capture / pulse safety node (default no actuation)")
    parser.add_argument("--config",required=True)
    args,ros_args=parser.parse_known_args(argv)
    config=load_config(args.config)
    rclpy.init(args=ros_args)
    sensor_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
    control_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                          history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE)
    proposal_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,durability=DurabilityPolicy.VOLATILE,
        lifespan=Duration(seconds=config["safety"]["max_response_delay_s"]))
    initializing={}

    class PiNode(Node):
        def __init__(self):
            super().__init__("roomwm_pi")
            # Keep a cleanup handle even if later camera/recorder setup raises.
            initializing["node"]=self
            self.motor=self.camera=self.recorder=None
            if self.get_parameter("use_sim_time").value:
                raise ValueError("Real motor node forbids use_sim_time")
            self.config=config
            self.expected_model_hash = config["runtime"].get("world_model_sha256", "")
            if config["runtime"].get("world_model_validated"):
                from .acceptance import load_acceptance
                load_acceptance(resolve_path(config, config["runtime"].get("world_model_acceptance_file", "")),
                    calibration_fingerprint=calibration_fingerprint(config),
                    checkpoint_sha256=self.expected_model_hash)
            if config["runtime"].get("allow_wm_trials"):
                from .acceptance import load_trial_eligibility
                load_trial_eligibility(resolve_path(config,config["runtime"]["world_model_trial_file"]),
                    calibration_fingerprint=calibration_fingerprint(config),
                    checkpoint_sha256=self.expected_model_hash)
            self.session=uuid.uuid4().hex
            self.fingerprint=calibration_fingerprint(config)
            self.gov=SafetyGovernor(build_safety_config(config))
            self.steady=Clock(clock_type=ClockType.STEADY_TIME)
            self.history=deque()
            self.pulse_goals={}
            self.history_start=time.monotonic()
            self.goal_id=""
            self.last_frame=None
            self.last_publish=0.
            self.last_status=0.
            self.last_log=0.
            self.last_decision=self.gov.tick()
            self.last_motor_pulse_id=-1
            self.last_pi_fault=""
            self.history_confirmed = not config["runtime"]["allow_actuation"]
            self.arm_pending_until=0.
            self.record_metadata = {
                "robot": config["robot"], "camera": config["camera"],
                "calibration_fingerprint": self.fingerprint,
                "depth_scale_m": config["perception"]["depth_scale_m"],
                "actuation_allowed": config["runtime"]["allow_actuation"],
                "synthetic": False, "firmware": "roomwm-pulse-v3",
                "calibration_confirmed": config["runtime"]["calibration_confirmed"],
                **recording_provenance(config),
                **config.get("session", {})}
            self.recorder=EpisodeRecorder(resolve_path(config,config["runtime"]["record_session"]),
                                          self.record_metadata)
            self.motor=None
            if type(config["runtime"]["allow_actuation"]) is not bool:
                raise ValueError("allow_actuation must be boolean")
            if config["runtime"]["allow_actuation"]:
                import serial
                port=serial.Serial(config["robot"]["serial_port"],config["robot"]["baud"],
                                    timeout=0,write_timeout=0,exclusive=True)
                self.motor=PulseSerial(port,on_event=self.motor_event,
                    clock_rate_bound_ppm=config["robot"].get("mcu_clock_rate_bound_ppm",20000.0))
            self.camera=LatestCamera(config["camera"])
            self.recorder.event({"type":"camera_settings","time_s":time.monotonic(),
                                 "settings":self.camera.metadata})
            self.frame_pub=self.create_publisher(interfaces.FramePacket,"/roomwm/frame",sensor_qos)
            self.info_pub=self.create_publisher(interfaces.FrameInfo,"/roomwm/frame_info",sensor_qos)
            self.status_pub=self.create_publisher(interfaces.SafetyStatus,"/roomwm/status",control_qos)
            self.ack_pub=self.create_publisher(interfaces.ActuationAck,"/roomwm/actuation_ack",
                QoSProfile(depth=64,reliability=ReliabilityPolicy.RELIABLE))
            self.camera_pub=self.create_publisher(CompressedImage,"/roomwm/camera/image",sensor_qos)
            # Telemetry never goes to the old motor bridge's /robot/cmd_vel.
            self.executed_pub=self.create_publisher(TwistStamped,"/roomwm/executed",control_qos)
            self.create_subscription(interfaces.ActuationProposal,"/roomwm/actuation_proposal",self.proposal,proposal_qos)
            self.create_subscription(interfaces.TaskRequest,"/roomwm/task",self.task,control_qos)
            self.create_subscription(Bool,"/roomwm/deadman",self.deadman,control_qos)
            self.create_subscription(interfaces.Stop,"/roomwm/emergency_stop",self.stop_message,control_qos)
            self.create_service(SetBool,"/roomwm/arm",self.arm)
            self.create_service(Trigger,"/roomwm/new_episode",self.new_episode)
            self.create_timer(.02,self.tick,clock=self.steady)
            self.get_logger().info("Recording: "+str(self.recorder.directory))
            self.get_logger().info("Pulse controller starts DISARMED. allow_actuation="+str(config["runtime"]["allow_actuation"]))

        def motor_event(self,event):
            event=dict(event)
            key=(event.get("boot_id"),event.get("nonce"),event.get("seq"))
            if event["type"]=="motor_sent":
                self.pulse_goals[key]=self.goal_id
            event["goal_id"]=self.pulse_goals.get(key, "")
            self.recorder.event(event)
            if hasattr(self, "ack_pub"):
                self.ack_pub.publish(ack_to_ros(event))
            if event["type"]=="motor_interval":
                self.history.append(event)
                self.gov.confirm_motor_stop(event["end_upper_s"],event.get("time_s",time.monotonic()))
                self.pulse_goals.pop(key,None)
            if event["type"]=="motor_ready" and event.get("zero_confirmed"):
                self.history_start = event["history_start_s"]
                self.history_confirmed = True
                self.history.clear()
                self.recorder.boundary("mcu_zero_handshake", event["time_s"])
            if event["type"] in {"motor_fault", "motor_unknown", "motor_reboot"}:
                # No gap is silently relabeled as zero after unknown execution.
                self.history_confirmed = False
                self.history_start = event["time_s"]
                self.history.clear()
                self.recorder.boundary(event["type"],event["time_s"])

        def new_episode(self,req,response):
            if self.gov.armed or (self.motor is not None and self.motor.pending is not None):
                response.success=False
                response.message="Disarm and wait for confirmed PWM-off before changing episodes."
                return response
            self.stop_all("manual_episode_boundary")
            self.recorder.close()
            self.recorder=EpisodeRecorder(resolve_path(config,config["runtime"]["record_session"]),
                                          self.record_metadata)
            self.history.clear()
            self.history_start=time.monotonic()
            self.goal_id=""
            response.success=True
            response.message="New episode opened; set a new goal after any manual repositioning."
            return response

        def stop_all(self,reason):
            self.arm_pending_until=0.
            self.last_decision=self.gov.stop(reason)
            if self.motor is not None and (self.motor.ready or self.motor.pending is not None):
                self.motor.emergency_stop(reason)
            self.recorder.event({"type":"operator_stop","time_s":time.monotonic(),"reason":reason})

        def task(self,msg):
            try:
                value=task_from_ros(msg)
                if not isinstance(value,dict) or not isinstance(value.get("goal_id"),str) or not value["goal_id"] or len(value["goal_id"])>128:
                    raise ValueError("Invalid task goal_id")
                if value["goal_id"]==self.goal_id:
                    return
                self.stop_all("new_task_requires_explicit_arm")
                self.goal_id=value["goal_id"]
                self.recorder.event({"type":"task","time_s":time.monotonic(),"task":value})
            except (ValueError,TypeError):
                self.stop_all("malformed_task")

        def deadman(self,msg):
            now=time.monotonic()
            self.gov.set_deadman(bool(msg.data),now)
            if not msg.data:
                self.arm_pending_until=0.
                self.last_decision,self.last_motor_pulse_id=dispatch_motor(
                    self.gov,self.motor,self.gov.tick(now),self.last_decision,config["robot"],
                    self.last_motor_pulse_id,now)

        def stop_message(self,msg):
            if msg.goal_id and msg.goal_id != self.goal_id:
                return
            self.stop_all(msg.reason[:160] or "operator_stop")

        def arm(self,req,response):
            if not req.data:
                self.stop_all("operator_disarm")
                response.success=True
                response.message="Stopped and disarmed"
                return response
            if not config["runtime"]["allow_actuation"]:
                response.success=False
                response.message="Shadow configuration: allow_actuation=false"
                return response
            if not self.goal_id:
                response.success=False
                response.message="Set a reviewed task/direction first"
                return response
            if self.motor is None:
                response.success=False; response.message="No pulse firmware connection"; return response
            if self.motor.pending is not None:
                response.success=False; response.message="Wait for the MCU STOP acknowledgement"; return response
            if not self.motor.ready:
                try:
                    self.gov.stop("explicit_rearm_waiting_for_drive")
                    self.motor.reset_handshake()
                    self.arm_pending_until=time.monotonic()+3.
                    response.success=True
                    response.message="Explicit arm requested: awaiting zero handshake and fresh geometry for at most 3 s while hold-to-run remains pressed"
                except ValueError as exc:
                    response.success=False; response.message=str(exc)
                return response
            lease=self.gov.arm(stationary_confirmed=True)
            response.success=lease is not None
            response.message="Armed; waiting for a newly exposed settled frame" if lease else self.gov.reason
            return response

        def proposal(self,msg):
            try:
                value=proposal_from_ros(msg)
                command=MotionCommand.from_dict(value["command"])
                geometry=MetricGeometry.from_dict(value["geometry"])
                if command.frame!=geometry.frame:
                    raise ValueError("Command/geometry frame mismatch")
                if value["goal_id"]!=self.goal_id:
                    return  # in-flight result for an already cancelled task
                if value["controller"] not in {"shadow","teleop","baseline","wm","wm_trial"}:
                    raise ValueError("Unknown controller")
                if value["diagnostics"]["calibration_fingerprint"] != self.fingerprint:
                    raise ValueError("Current geometry calibration mismatch")
                moving=command.linear_mps!=0 or command.angular_rps!=0
                if moving and not self.history_confirmed:
                    raise ValueError("MCU history not confirmed")
                if moving and value["controller"] in {"wm","wm_trial"}:
                    if value["diagnostics"]["model_hash"] != self.expected_model_hash:
                        raise ValueError("WM checkpoint differs from reviewed evidence")
                    if (not value["diagnostics"]["model_uncertainty_valid"]
                        or value["diagnostics"]["model_uncertainty"] > config["planner"]["uncertainty_limit"]):
                        raise ValueError("WM uncertainty invalid or above limit")
                if moving and (value["controller"]=="shadow" or not config["runtime"]["allow_actuation"]):
                    raise ValueError("Shadow result attempted actuation")
                if moving and value["controller"]=="wm" and not config["runtime"]["world_model_validated"]:
                    raise ValueError("World-model supervised acceptance is not recorded")
                if moving and value["controller"]=="wm_trial" and not config["runtime"].get("allow_wm_trials"):
                    raise ValueError("Supervised WM trial is not enabled")
                self.recorder.event({"type":"proposal","time_s":time.monotonic(),
                    "frame":command.frame.to_dict(),"goal_id":value["goal_id"],
                    "command":value["command"],"diagnostics":value["diagnostics"]})
                if value["abort"]:
                    self.stop_all("task_abort:"+str(value["diagnostics"].get("reason","stop"))[:120])
                    return
                now=time.monotonic()
                self.gov.update_geometry(geometry,now_s=now)
                decision=self.gov.submit(command,now)
                # Start the one MCU pulse in this callback. Waiting for the
                # next 20 ms timer would shorten an 80 ms pulse to about 60 ms.
                self.last_decision,self.last_motor_pulse_id=dispatch_motor(
                    self.gov,self.motor,decision,self.last_decision,config["robot"],
                    self.last_motor_pulse_id,now)
            except (ValueError,TypeError,AttributeError,KeyError) as exc:
                self.stop_all("bad_proposal:"+str(exc)[:120])

        def tick(self):
            now=time.monotonic()
            if self.motor is not None:
                self.motor.poll(now)
                # This is a fresh MCU reply, not a cached ROS ready=True heartbeat.
                healthy=bool(self.motor.ready and now-self.motor._last_rx<.80)
                self.gov.set_drive_ready(healthy,now)
            else:
                self.gov.set_drive_ready(False,now)
            if self.camera.error:
                self.gov.stop(self.camera.error,now)
            if self.recorder.failed:
                self.gov.stop("recorder_overflow_or_disk_error",now)
            if self.arm_pending_until:
                self.arm_pending_until=advance_pending_arm(
                    self.gov,self.motor,self.arm_pending_until,now)
            decision=self.gov.tick(now)
            decision,self.last_motor_pulse_id=dispatch_motor(
                self.gov,self.motor,decision,self.last_decision,config["robot"],
                self.last_motor_pulse_id,now)
            self.last_decision=decision
            if now-self.last_publish>=1/config["camera"]["publish_max_fps"]:
                sample=self.camera.take_latest()
                if sample is not None:
                    self.last_publish=now
                    ref=FrameRef(self.session,sample.epoch,sample.sequence)
                    low=sample.exposure_earliest_ns
                    high=sample.exposure_latest_ns
                    registered=self.gov.register_exposure_interval(ref,sample.read_mono_s,
                        None if low is None else low/1e9, None if high is None else high/1e9,
                        now_s=time.monotonic())
                    if not registered or (decision.armed and not self.gov.armed):
                        decision=self.gov.tick(time.monotonic())
                        decision,self.last_motor_pulse_id=dispatch_motor(self.gov,self.motor,decision,
                            self.last_decision,config["robot"],self.last_motor_pulse_id,time.monotonic())
                        self.last_decision=decision
                    self.last_frame=ref
                    self.history_start=max(self.history_start,now-30.)
                    while self.history and self.history[0]["end_s"]<self.history_start:
                        self.history.popleft()
                    settled=(sample.timing.timing_valid and not decision.moving
                        and now>=self.gov._settle_until and low/1e9>=self.gov._settle_until
                        and (self.motor is None or (self.motor.pending is None and self.motor.ready)))
                    ok,jpeg=cv2.imencode(".jpg",sample.bgr,[cv2.IMWRITE_JPEG_QUALITY,config["camera"]["jpeg_quality"]])
                    if not ok:
                        self.gov.stop("jpeg_encoding_failed")
                    else:
                        body=make_frame(ref, fingerprint=self.fingerprint, history=list(self.history),
                            history_start_s=self.history_start, settled=settled,
                            mode=config["runtime"]["mode"], timing=sample.timing)
                        msg=frame_to_ros(body,jpeg.tobytes(),ros_now_ns=self.get_clock().now().nanoseconds,
                                         pi_now_ns=time.monotonic_ns())
                        self.frame_pub.publish(msg)
                        self.info_pub.publish(msg.info)
                        self.camera_pub.publish(msg.image)
                        self.recorder.frame(jpeg.tobytes(),body)
            if now-self.last_status>=.10:
                self.last_status=now
                status={**asdict(decision),"goal_id":self.goal_id,"session_id":self.session,
                    "state": ("DISARMED" if not decision.armed else
                        "PULSE_ACTIVE" if (decision.moving or self.motor is not None and self.motor.pending is not None) else
                        "WAIT_SETTLE" if now < self.gov._settle_until else "ARMED_STOPPED"),
                    "arm_pending":bool(self.arm_pending_until>now),
                    "arm_pending_seconds":max(0.,self.arm_pending_until-now),
                    "frame":None if self.last_frame is None else self.last_frame.to_dict(),
                    "motor_ready":bool(self.motor and self.motor.ready),
                    "actuation_enabled":config["runtime"]["allow_actuation"],
                    "calibration_fingerprint":self.fingerprint}
                self.status_pub.publish(status_to_ros(status))
                cmd=TwistStamped(); cmd.header.stamp=self.get_clock().now().to_msg(); cmd.header.frame_id="nominal_setpoint_not_odometry"
                cmd.twist.linear.x=decision.linear_mps; cmd.twist.angular.z=decision.angular_rps
                self.executed_pub.publish(cmd)
            if now-self.last_log>3.:
                self.last_log=now
                self.get_logger().info(f"{decision.reason}; armed={decision.armed}; frame_age_upper={decision.frame_age_s}")

        def close(self):
            if hasattr(self,"gov"):
                self.gov.stop("shutdown")
            if self.motor:
                self.motor.close()
            if self.camera:
                self.camera.close()
            if self.recorder:
                self.recorder.close()

    node=None
    try:
        node=PiNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is None:
            node=initializing.get("node")
        if node is not None:
            node.close(); node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
