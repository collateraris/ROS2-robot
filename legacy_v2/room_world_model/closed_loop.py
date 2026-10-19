"""Paired CLOSED-LOOP toy comparison, with zero access to real motor transports.

The same ray-cast observation, initial layout, virtual latency, primitive set
and conservative toy guard are used for reactive and learned MPC controllers.
Only the controller changes. This is a software regression experiment, not a
physics simulator, realistic monocular perception, or permission to drive.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import time

import numpy as np
import torch

from .baseline_controller import reactive_action
from .model import load_checkpoint
from .planner import MPCPlanner, PlannerConfig
from .simulate import render_scene, wrap_angle


@dataclass
class ToyRoom:
    seed: int
    changed_layout: bool = False
    radius_m: float = .16
    virtual_forward_mps: float = .65
    virtual_yaw_rps: float = 1.1

    def __post_init__(self):
        rng = np.random.default_rng(self.seed)
        self.position = np.array([-1.25, float(rng.uniform(-.08, .08))])
        self.yaw = float(rng.uniform(-.04, .04))
        self.person = np.array([1.65, float(rng.uniform(-.12, .12))])
        self.obstacles = np.array([[.0, .85, .24], [.8, -.8, .24], [-1.65, 1.0, .2]])
        self.obstacles[:, :2] += rng.uniform(-.12, .12, size=(3, 2))
        self.time_s, self.path_length_m, self.collisions = 0., 0., 0
        self.changed = False
        self.minimum_clearance_m = self.clearance(self.position)

    def clearance(self, position):
        values = [float(np.min(2.5 - np.abs(position))) - self.radius_m,
                  float(np.linalg.norm(position-self.person)) - .20 - self.radius_m]
        values.extend(float(np.linalg.norm(position-x[:2])) - x[2] - self.radius_m for x in self.obstacles)
        return min(values)

    def observe(self):
        return render_scene(self.position, self.yaw, self.obstacles, self.person)

    def advance(self, action, seconds):
        # Integrate motor pulse substeps instead of endpoint teleportation. The
        # collision metric is computed from toy ground truth, never model output.
        count = max(1, int(math.ceil(seconds/.01)))
        dt = seconds/count
        for _ in range(count):
            self.time_s += dt
            if self.changed_layout and not self.changed and self.time_s >= 12.:
                self.obstacles[0, :2] = np.array([.10, .30])
                self.changed = True
            yaw_next = wrap_angle(self.yaw + float(action[1])*self.virtual_yaw_rps*dt)
            midpoint = self.yaw + .5*float(action[1])*self.virtual_yaw_rps*dt
            delta = float(action[0])*self.virtual_forward_mps*dt*np.array([math.cos(midpoint),math.sin(midpoint)])
            self.position += delta
            self.path_length_m += float(np.linalg.norm(delta))
            self.yaw = yaw_next
            clearance = self.clearance(self.position)
            self.minimum_clearance_m = min(self.minimum_clearance_m,clearance)
            if clearance <= 0:
                self.collisions += 1
                return


@torch.no_grad()
def run_trial(model, *, seed: int, controller: str, steps: int, pulse_s: float,
              wait_s: float, camera_latency_s: float, changed_layout: bool,
              candidates: int = 12) -> dict:
    room = ToyRoom(seed,changed_layout)
    if controller not in {"reactive","wm_mpc"}:
        raise ValueError("Controller must be reactive or wm_mpc")
    scale = float(model.checkpoint_metadata.get("depth_scale_m",3.0))
    source_scale = model.checkpoint_metadata.get("action_scale",{})
    if source_scale != {"virtual_forward_m_s":room.virtual_forward_mps,"virtual_yaw_rad_s":room.virtual_yaw_rps}:
        raise ValueError("Toy benchmark requires matching synthetic control scales; it does not test real checkpoints")
    config=PlannerConfig(horizon=2,pulse_s=pulse_s,observe_wait_s=wait_s+camera_latency_s,
                         candidates=candidates,elite=max(2,candidates//4),iterations=2,
                         max_forward=.8,max_forward_change=.8,max_yaw=.3,max_yaw_change=.3,
                         clearance_m=.4,stop_distance_m=1.,clearance_error_margin_m=.05,
                         depth_scale_m=scale,seed=seed)
    planner=MPCPlanner(model,config)
    options=np.array([[0,0],[.8,0],[.8,.10],[.8,-.10],[.8,.25],[.8,-.25]],np.float32)

    def tensor(image):
        return torch.from_numpy(image.copy()).to(model.device,dtype=torch.float32)[None,None]/255.

    image,aux,mask=room.observe()
    state=model.initial_state(tensor(image))
    # Stationary burn-in is a recorded virtual history. No target motion or
    # imagined action is fed back as if it had actually happened.
    for _ in range(int(model.checkpoint_metadata.get("burn_in_frames",2))):
        room.advance([0,0],wait_s+camera_latency_s)
        state=model.advance(state,torch.zeros(1,2,device=model.device),wait_s+camera_latency_s)
        image,aux,mask=room.observe()
        state=model.assimilate(state,tensor(image))
    started_virtual=room.time_s
    initial_distance=float(np.linalg.norm(room.person-room.position))
    trajectory=[]
    stops=guard_rejections=0
    success=False
    wall_started=time.monotonic()
    for step in range(steps):
        distance=float(np.linalg.norm(room.person-room.position))
        if bool(aux[0]>.5) and distance <= 1.:
            success=True
            break
        current_min=float(np.min(aux[3:])*scale)

        def allowed(action):
            # Toy raycast has no learned depth errors. Explicitly shared guard,
            # front-only arcs and an allowance for one pulse + robot radius.
            if np.all(np.abs(action)<1e-8):
                return True
            required=.4+room.radius_m+room.virtual_forward_mps*float(action[0])*pulse_s
            return bool(mask[3:].all() and current_min>required and action[0]>0
                        and abs(float(action[1])*room.virtual_yaw_rps*pulse_s)<math.radians(15))

        # While waiting on a delayed frame the virtual robot remains stationary.
        # An environmental change during that wait is real simulator state, so
        # the following action really does use an older observation.
        room.advance([0,0],camera_latency_s)
        if aux[0]<.5 or not mask[1] or not mask[2]:
            action=np.zeros(2,np.float32);reason="target_unknown_stop"
        elif controller=="reactive":
            action=reactive_action(float(aux[1])*math.radians(68)/2,options,
                                   angular_scale_rps=room.virtual_yaw_rps,action_allowed=allowed)
            reason="reactive_arc" if action[0]>0 else "reactive_guard_stop"
        else:
            planning_state=model.advance(state,torch.zeros(1,2,device=model.device),camera_latency_s)
            result=planner.plan(planning_state,goal="approach_person",for_live=False,
                                first_action_options=options,action_allowed=allowed)
            action=result.action.copy();reason=result.reason
        if not allowed(action):
            action[:]=0;guard_rejections+=1;reason="shared_guard_rejected"
        if not np.any(action):
            stops+=1
        room.advance(action,pulse_s)
        collision=room.collisions>0
        if not collision:
            room.advance([0,0],wait_s)
        next_image,next_aux,next_mask=room.observe()
        trajectory.append({"step":step,"observation_age_at_action_s":camera_latency_s,
                           "time_s":room.time_s-started_virtual,"action":action.tolist(),"reason":reason,
                           "position_m":room.position.tolist(),"person_distance_m":float(np.linalg.norm(room.person-room.position)),
                           "current_min_raycast_range_m":current_min,"minimum_body_clearance_m":room.minimum_clearance_m})
        if collision:
            break
        history=torch.tensor([[[0.,0.,camera_latency_s],[float(action[0]),float(action[1]),pulse_s],
                               [0.,0.,wait_s]]],device=model.device)
        elapsed=camera_latency_s+pulse_s+wait_s
        summary=(history[...,:2]*history[...,2:3]).sum(1)/elapsed
        state=model.advance_interval(state,summary,elapsed,history,torch.ones((1,3),dtype=torch.bool,device=model.device))
        state=model.assimilate(state,tensor(next_image))
        image,aux,mask=next_image,next_aux,next_mask
    final_distance=float(np.linalg.norm(room.person-room.position))
    success=success or (not room.collisions and aux[0]>.5 and final_distance<=1.)
    return {"seed":seed,"controller":controller,"changed_layout":changed_layout,"success":bool(success),
            "collisions":room.collisions,"virtual_interventions":int(room.collisions>0),
            "task_time_s":room.time_s-started_virtual,"controller_wall_time_s":time.monotonic()-wall_started,
            "number_of_zero_action_steps":stops,"shared_guard_rejections":guard_rejections,
            "steps":len(trajectory),"initial_person_distance_m":initial_distance,"final_person_distance_m":final_distance,
            "minimum_clearance_m":room.minimum_clearance_m,"path_length_m":room.path_length_m,
            "path_efficiency":min(1.,max(0.,initial_distance-1.)/room.path_length_m) if success and room.path_length_m>0 else None,
            "trajectory":trajectory}


def run_benchmark(args) -> dict:
    torch.set_num_threads(args.cpu_threads)
    if (args.trials<1 or args.steps<1 or args.cpu_threads<1
            or not all(math.isfinite(x) and x>0 for x in (args.camera_latency,args.pulse,args.wait))):
        raise ValueError("Positive trials, steps, threads and finite timing intervals required")
    model,metadata=load_checkpoint(args.checkpoint,device=args.device)
    if metadata.get("synthetic") is not True:
        raise ValueError("This ray-cast smoke benchmark only accepts synthetic checkpoints")
    trials=[]
    for i in range(args.trials):
        for controller in ("reactive","wm_mpc"):
            trial=run_trial(model,seed=args.seed+i,controller=controller,steps=args.steps,
                            pulse_s=args.pulse,wait_s=args.wait,camera_latency_s=args.camera_latency,
                            changed_layout=bool(i%2),candidates=args.candidates)
            trials.append(trial)
            print(json.dumps({key:trial[key] for key in ("seed","controller","success","steps","collisions")}),flush=True)
    summary={}
    for controller in ("reactive","wm_mpc"):
        group=[trial for trial in trials if trial["controller"]==controller]
        succeeded=[trial for trial in group if trial["success"]]
        summary[controller]={"success_rate":len(succeeded)/len(group),"trials":len(group),
                             "minimum_clearance_m":min(trial["minimum_clearance_m"] for trial in group),
                             "collisions":sum(trial["collisions"] for trial in group),
                             "interventions":sum(trial["virtual_interventions"] for trial in group),
                             "median_success_task_time_s":float(np.median([trial["task_time_s"] for trial in succeeded])) if succeeded else None,
                             "mean_zero_action_steps":float(np.mean([trial["number_of_zero_action_steps"] for trial in group]))}
    report={"schema_version":1,"domain":"synthetic_toy","synthetic":True,"real_robot_acceptance":False,
            "checkpoint":str(args.checkpoint),"summary":summary,"paired_trials":trials,
            "configuration":{key:str(value) if isinstance(value,Path) else value for key,value in vars(args).items()},
            "limitations":["2D circles, ideal ray-cast labels, invented dynamics/speeds and fixed virtual latency; no real camera/MCU.",
                           "Both controllers receive the same toy observations, guard, action primitives and initial conditions.",
                           "Alternating trials move one obstacle at virtual time 12 s; latest observations can contradict history.",
                           "A virtual intervention means the collision monitor ended a toy trial; it is not a real emergency-stop statistic.",
                           "Zero collisions while stopping or failing every task demonstrates no useful navigation.",
                           "Any synthetic success, action sensitivity or speed result is insufficient to authorize real motors."]}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding="utf-8")
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint",required=True,type=Path)
    parser.add_argument("--output",type=Path,default=Path("runs/closed_loop_toy.json"))
    parser.add_argument("--trials",type=int,default=4)
    parser.add_argument("--steps",type=int,default=64)
    parser.add_argument("--seed",type=int,default=900)
    parser.add_argument("--pulse",type=float,default=.08)
    parser.add_argument("--wait",type=float,default=.65)
    parser.add_argument("--camera-latency",type=float,default=.35)
    parser.add_argument("--candidates",type=int,default=12)
    parser.add_argument("--device",default="cpu")
    parser.add_argument("--cpu-threads",type=int,default=2)
    run_benchmark(parser.parse_args())


if __name__=="__main__":
    main()
