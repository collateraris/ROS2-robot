"""Reproducible CPU-only synthetic data -> train -> ablate -> closed-loop smoke.

No camera, serial device, ROS graph or real motor permissions are accessed.
Generated weights are synthetic and are rejected by all live model gates.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=Path("runs/smoke_v2"))
    parser.add_argument("--trials",type=int,default=4)
    parser.add_argument("--steps",type=int,default=64)
    args=parser.parse_args()
    if args.trials<1 or args.steps<1:
        parser.error("Positive --trials and --steps required")
    destination=args.output.resolve()
    if destination.exists() and any(destination.iterdir()):
        parser.error("Use a new/empty output folder to avoid mixing recordings or previous results")
    destination.mkdir(parents=True,exist_ok=True)
    data,run=destination/"data",destination/"model"
    commands=[
        ["simulate","--output",str(data),"--episodes","6","--frames","24","--seed","100"],
        ["train","--data",str(data),"--output",str(run),"--device","cpu","--epochs","2",
         "--latent-dim","16","--hidden-dim","24","--ensemble-size","2","--batch-size","2",
         "--sequence-length","4","--burn-in","1","--max-train-batches","2","--max-val-batches","1",
         "--split-by","layout","--cpu-threads","2"],
        ["evaluate","--checkpoint",str(run/"best.pt"),"--data",str(data),"--output",str(destination/"offline_evaluation.json"),
         "--device","cpu","--batch-size","2","--max-batches","2","--cpu-threads","2"],
        ["closed_loop","--checkpoint",str(run/"best.pt"),"--output",str(destination/"closed_loop.json"),
         "--trials",str(args.trials),"--steps",str(args.steps),"--device","cpu","--cpu-threads","2"],
    ]
    logs=[]
    for command in commands:
        argv=[sys.executable,"-m",f"room_world_model.{command[0]}",*command[1:]]
        print(json.dumps({"running":command[0],"synthetic_only":True}),flush=True)
        completed=subprocess.run(argv,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=600)
        logs.append({"argv":argv,"returncode":completed.returncode,"output":completed.stdout})
        (destination/"execution_log.json").write_text(json.dumps(logs,indent=2,ensure_ascii=False),encoding="utf-8")
        if completed.returncode:
            print(completed.stdout)
            raise SystemExit(completed.returncode)
        print(completed.stdout,end="",flush=True)
    from .model import load_checkpoint
    try:
        load_checkpoint(run/"best.pt",for_live=True)
    except ValueError as exc:
        if "Synthetic" not in str(exc):
            raise
    else:
        raise AssertionError("Synthetic checkpoint incorrectly passed the live model gate")
    offline=json.loads((destination/"offline_evaluation.json").read_text())
    closed_loop=json.loads((destination/"closed_loop.json").read_text())
    summary={"pipeline_executed":True,"synthetic_live_gate_rejected":True,"real_robot_acceptance":False,
             "offline_diagnostics":offline["diagnostics"],"toy_controller_comparison":closed_loop["summary"],
             "interpretation":"A completed smoke run verifies executable code paths only. Negative gains "
                              "and failed tasks are recorded as failures of model utility, not hidden. "
                              "This tiny model is not an autonomy policy for a physical robot."}
    (destination/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False),encoding="utf-8")
    print(json.dumps(summary,indent=2,ensure_ascii=False))


if __name__=="__main__":
    main()
