"""Testable PC perception/filter/planner, independent of ROS transport."""
from __future__ import annotations
from dataclasses import dataclass
import math
import time
import numpy as np
from .actuator import mix_pwm, unmix_pwm
from .baseline_controller import reactive_action
from .commands import parse_command
from .configuration import calibration_fingerprint
from .protocol import FrameRef, MotionCommand, MetricGeometry
from .record import intervals_to_segments
from .safety import stopping_distance


MANUAL_ACTIONS = {"forward": (1.0,0.0), "left": (1.0,0.10),
                  "right": (1.0,-0.10), "stop": (0.0,0.0)}


@dataclass
class Task:
    goal_id: str = ""
    kind: str = "stop"
    text: str = ""
    direction: str = "stop"
    start_s: float = 0.0


class ControlEngine:
    def __init__(self, config: dict, perception, *, model=None, planner=None,
                 rectifier=None, mode="shadow", shadow_controller="baseline"):
        if mode not in {"shadow", "teleop", "baseline", "wm"}:
            raise ValueError("Unknown controller mode")
        self.config, self.perception = config, perception
        self.model, self.planner, self.rectifier = model, planner, rectifier
        self.mode = mode
        self.controller = shadow_controller if mode == "shadow" else mode
        if self.controller == "wm" and (model is None or planner is None):
            raise ValueError("World-model controller requires a checkpoint and MPC planner")
        self.fingerprint = calibration_fingerprint(config)
        if self.controller == "wm" and mode != "shadow":
            from .model import check_live_metadata
            check_live_metadata(model.checkpoint_metadata,
                                calibration_fingerprint=self.fingerprint,
                                depth_scale_m=config["perception"]["depth_scale_m"])
        self.task = Task()
        self.state = None
        self.last_exposure = None
        self.last_exposure_interval = None
        self.last_epoch = None
        self.filter_reset_reason = "no_history"
        self.filter_steps = 0
        burn_in = model.checkpoint_metadata.get("burn_in_frames", 2) if model is not None else 2
        if type(burn_in) is not int or burn_in < 0:
            raise ValueError("Checkpoint burn_in_frames must be a nonnegative integer")
        self.required_filter_steps = burn_in + 1
        self.command_seq = 0
        self.last_person = None
        self.best_distance = math.inf
        self.progress_s = 0.0

    def set_task(self, value: dict, now_s=None):
        now = time.monotonic() if now_s is None else now_s
        if not isinstance(value, dict) or not isinstance(value.get("goal_id"),str) or not value["goal_id"] or len(value["goal_id"])>128:
            raise ValueError("Task requires a nonempty goal_id")
        if value.get("kind")=="manual" and set(value)=={"goal_id","kind","direction"}:
            if value["direction"] not in MANUAL_ACTIONS:
                raise ValueError("Unknown manual action")
            kind = "manual" if self.controller=="teleop" else "stop"
            task = Task(value["goal_id"],kind,direction=value["direction"],start_s=now)
        elif value.get("kind")=="text" and set(value)=={"goal_id","kind","text"}:
            parsed = parse_command(value["text"], self.config["perception"]["person_standoff_m"])
            task = Task(value["goal_id"],parsed.kind if parsed.motion_requested else "stop",
                        text=str(value["text"])[:500],start_s=now)
        else:
            raise ValueError("Unknown task message")
        self.task=task
        self.last_person=None
        self.best_distance=math.inf
        self.progress_s=now

    def _quantize(self, action):
        r=self.config["robot"]
        v=float(action[0])*r["linear_command_scale_mps"]
        w=float(action[1])*r["angular_command_scale_rps"]
        if v<=0:
            return 0.0,0.0
        left,right=mix_pwm(v,w,r["track_width_m"],r["wheel_command_scale_mps"],r["pwm_limit"])
        # At the speed cap, rounding .06 m/s upward can produce PWM34 and
        # .060714 m/s in the example calibration. Choose a nearby bounded PWM
        # pair instead of turning every straight proposal into an invalid zero.
        choices=[]
        for l in {left,max(0,left-1)}:
            for rr in {right,max(0,right-1)}:
                qv,qw=unmix_pwm(l,rr,r["track_width_m"],r["wheel_command_scale_mps"],r["pwm_limit"])
                if 0<qv<=r["linear_command_scale_mps"]+1e-9 and abs(qw)<=r["angular_command_scale_rps"]+1e-9:
                    error=((qv-v)/r["linear_command_scale_mps"])**2+((qw-w)/r["angular_command_scale_rps"])**2
                    choices.append((error,qv,qw))
        if not choices:
            return 0.0,0.0
        _,v,w=min(choices)
        return v,w

    def _filter(self, packet, rgb):
        if self.model is None:
            return
        import cv2
        import torch
        image=cv2.resize(cv2.cvtColor(rgb,cv2.COLOR_RGB2GRAY),(128,128),interpolation=cv2.INTER_AREA)
        tensor=torch.from_numpy(image.copy()).to(self.model.device,dtype=torch.float32)[None,None]/255.0
        low, high = packet.get("exposure_earliest_ns"), packet.get("exposure_latest_ns")
        precise=(type(low) is int and type(high) is int and 0<=low<=high and
                 (high-low)/2e9<=self.config["camera"].get("max_training_exposure_uncertainty_s",.10))
        if packet.get("timing_valid") is not True or not precise or packet.get("settled") is not True:
            # A dequeue-time placeholder is not an exposure. Do not warm up
            # a model state on an observation with unknown temporal ordering.
            self.state = None
            self.filter_steps = 0
            self.last_exposure = self.last_exposure_interval = self.last_epoch = None
            self.filter_reset_reason = "unsettled_or_uncertain_camera_exposure"
            return
        now=packet["exposure_estimate_s"]
        current_interval = (low / 1e9, high / 1e9)
        if not current_interval[0] <= now <= current_interval[1]:
            raise ValueError("Exposure estimate lies outside its Pi timestamp bounds")
        epoch=(packet["frame"]["session_id"],packet["frame"]["camera_epoch"])
        with torch.inference_mode():
            valid=(self.state is not None and self.last_epoch==epoch and self.last_exposure is not None
                   and 0<now-self.last_exposure<=self.model.config.max_dt_s
                   and packet["history_start_s"]<=self.last_exposure_interval[0])
            if valid:
                segments,ok=intervals_to_segments(self.last_exposure,now,packet["history"],self.config["robot"],
                    max_timing_uncertainty_s=self.config["robot"]["motor_time_uncertainty_max_s"],
                    history_start_s=packet["history_start_s"],start_interval=self.last_exposure_interval,
                    end_interval=current_interval,require_mcu_confirmation=True)
                if ok and len(segments)<=24:
                    seg=torch.tensor(segments,device=self.model.device)[None]
                    mask=torch.ones(seg.shape[:2],device=self.model.device,dtype=torch.bool)
                    dt=now-self.last_exposure
                    action=(seg[...,:2]*seg[...,2:3]).sum(1)/dt
                    self.state=self.model.advance_interval(self.state,action,dt,seg,mask)
                    self.state=self.model.assimilate(self.state,tensor).detach()
                    self.filter_steps+=1
                    self.filter_reset_reason = "confirmed_history_assimilated"
                else:
                    valid=False
            if not valid:
                self.state=self.model.initial_state(tensor).detach()
                self.filter_steps=1
                self.filter_reset_reason="initial_or_unconfirmed_history_reset"
        self.last_exposure,self.last_epoch,self.last_exposure_interval=now,epoch,current_interval

    def process(self, packet: dict, raw_rgb: np.ndarray, lease_id: str, now_s=None) -> dict:
        started=time.monotonic() if now_s is None else now_s
        task=self.task  # immutable-for-this-inference snapshot; ROS caller checks ID again
        frame=FrameRef.from_dict(packet["frame"])
        if packet["calibration_fingerprint"]!=self.fingerprint:
            raise ValueError("Pi/PC camera and motor calibration fingerprints differ")
        rgb=self.rectifier(raw_rgb) if self.rectifier is not None else raw_rgb
        observed=self.perception.observe(rgb,int(packet["exposure_estimate_s"]*1e9),
              image_valid_mask=self.rectifier.valid_mask if self.rectifier is not None else None)
        self._filter(packet,raw_rgb)
        reason="waiting_for_task"
        abort=False
        action=np.zeros(2,np.float32)
        desired_action=action.copy()
        r,s,p=self.config["robot"],self.config["safety"],self.config["perception"]
        # Use the same conservative physical bound as the Pi. PC age is only an
        # upper prediction budget; the authoritative final check is on Pi time.
        required=stopping_distance(r["physical_speed_bound_mps"],
              s["max_frame_age_s"]+r["actuator_latency_bound_s"]+.19,
              r["min_brake_decel_mps2"],s["clearance_margin_m"],s.get("obstacle_speed_bound_mps",0.0))
        cache={}
        def geometry_for(a):
            try:
                v,w=self._quantize(a)
            except (ValueError,TypeError):
                return (0.,0.,None)
            if v<=0 or abs(w/v)>3.0:
                return (v,w,None)
            key=(round(v,6),round(w,6))
            if key not in cache:
                # Clearance has to cover the entire stopping distance, not only
                # the few millimetres in a commanded pulse.
                horizon=max(p["movement_horizon_s"],(required+.05)/v)
                cache[key]=observed.path_clearance(v,w,r["footprint_radius_m"],horizon,.05)
            return v,w,cache[key]
        def allowed(a):
            if float(a[0])==0 and float(a[1])==0:
                return True
            v,w,g=geometry_for(a)
            return bool(v>0 and g is not None and g.metric_valid and g.covers_swept_footprint and g.clearance_m>=required)
        # WM inputs use the SAME rounded PWM setpoints that the Pi executes.
        # Float requested controls would otherwise create an avoidable train/live
        # action mismatch, particularly at the low-speed PWM quantization limit.
        primitive_options=[np.zeros(2,np.float32)]
        for yaw in (0.,.25,-.25,.10,-.10):
            qv,qw=self._quantize(np.array([1.,yaw],np.float32))
            option=np.array([qv/r["linear_command_scale_mps"],qw/r["angular_command_scale_rps"]],np.float32)
            if qv>0 and not any(np.array_equal(option,old) for old in primitive_options):
                primitive_options.append(option)
        primitive_options=np.asarray(primitive_options,np.float32)
        diagnostics={"perception":observed.to_dict(),"filter_steps":self.filter_steps,
                     "required_filter_steps":self.required_filter_steps,"required_clearance_pc_m":required,
                     "filter_history_status":self.filter_reset_reason}
        if started-task.start_s>s["task_timeout_s"] and task.kind!="stop":
            reason,abort="task_timeout",True
        elif task.kind=="manual":
            desired_action=np.asarray(MANUAL_ACTIONS[task.direction],np.float32)
            action=desired_action if allowed(desired_action) else np.zeros(2,np.float32)
            reason="teleop_proposal" if action[0]>0 else "manual_path_not_observed_or_clear"
        elif task.kind=="approach_person":
            if not observed.person_visible or observed.person_count!=1 or not observed.person_distance_valid:
                reason,abort="unique_visible_person_with_metric_estimate_required",True
            elif observed.person_distance_m<=p["person_standoff_m"]:
                reason,abort="goal_reached_stop",True
            elif self.last_person is not None and (
                    abs(observed.person_bearing_rad-self.last_person[0])>p["target_jump_bearing_rad"] or
                    abs(observed.person_distance_m-self.last_person[1])>p["target_jump_distance_m"]):
                reason,abort="target_jump_stop_reselect",True
            elif self.controller=="wm":
                if self.filter_steps<self.required_filter_steps:
                    reason="world_model_history_warmup"
                else:
                    result=self.planner.plan(self.state,goal="approach_person",for_live=self.mode!="shadow",
                         action_allowed=allowed,first_action_options=primitive_options)
                    desired_action=np.asarray(result.action,np.float32)
                    action=desired_action
                    reason=result.reason
                    diagnostics["planner"]=result.diagnostics
            else:
                # Transparent, nonlearned comparison policy. It only proposes
                # gentle visible arcs and is used before a robot WM is trained.
                action=reactive_action(observed.person_bearing_rad,primitive_options,
                    angular_scale_rps=r["angular_command_scale_rps"],action_allowed=allowed)
                if action[0]>0:
                    desired_action=action
                    reason="baseline_visual_approach"
                else:
                    reason="no_currently_observed_safe_arc"
            if observed.person_visible and observed.person_count==1:
                self.last_person=(observed.person_bearing_rad,observed.person_distance_m)
                if observed.person_distance_m<self.best_distance-.015:
                    self.best_distance=observed.person_distance_m
                    self.progress_s=started
                elif started-self.progress_s>s["progress_timeout_s"]:
                    action=np.zeros(2,np.float32)
                    reason,abort="no_progress_stop",True
        # Prepare straight-ahead geometry even while stopped for explicit arming.
        geometry_action=action if action[0]>0 else np.array([1.,0.],np.float32)
        v,w,g=geometry_for(geometry_action)
        geometry=MetricGeometry(frame,0. if g is None else float(g.clearance_m),w,
            False if g is None else bool(g.metric_valid),observed.horizontal_fov_rad/2,
            False if g is None else bool(g.covers_swept_footprint),v)
        diagnostics.update(reason=reason,desired_action=desired_action.tolist(),
                           path_reason="invalid_motion" if g is None else g.reason,
                           proposed_velocity=[v,w],mode=self.mode)
        if (action[0]<=0 or self.mode=="shadow" or not packet["settled"] or abort
                or packet.get("timing_valid") is not True):
            v=w=0.
        elapsed=max(0.,time.monotonic()-started)
        if elapsed>s["max_inference_s"]:
            v=w=0.
            diagnostics["reason"]="inference_budget_exceeded"
        self.command_seq+=1
        command=MotionCommand(frame,lease_id,self.command_seq,v,w,
               int(s["pulse_s"]*1000),elapsed*1000)
        return {"version":1,"type":"proposal","goal_id":task.goal_id,"controller":self.mode,
                "abort":abort,"command":command.to_dict(),"geometry":geometry.to_dict(),
                "diagnostics":diagnostics}
