"""Offline inference. No publishers and no motor control."""
import argparse
import json
import numpy as np
import torch
from .data import load_metadata,metadata_fingerprint,load_dataset,SCHEMA_VERSION
from .models import WorldModel,TinyVLA
from .train_utils import tensors,world_forward,vla_forward


def load_checkpoint(path,meta,device="cpu"):
    # weights_only rejects arbitrary pickle execution. Only use your own checkpoints.
    ckpt=torch.load(path,map_location=device,weights_only=True)
    if ckpt.get("schema_version")!=SCHEMA_VERSION or ckpt.get("fingerprint")!=metadata_fingerprint(meta):
        raise ValueError("Checkpoint goals, sensor/action source or recording metadata differ")
    if ckpt["kind"]=="tiny_vla":
        model=TinyVLA(meta["sectors"],len(meta["candidates"])).to(device); model.load_state_dict(ckpt["state"]); model.eval(); return [model],"vla"
    if ckpt["kind"]=="world_ensemble":
        models=[]
        for state in ckpt["states"]:
            m=WorldModel(meta["sectors"]).to(device); m.load_state_dict(state); m.eval(); models.append(m)
        return models,"world"
    raise ValueError("Unknown checkpoint kind")


@torch.no_grad()
def predict(models,kind,b,meta,confidence_threshold=.8,uncertainty_m=.25):
    if kind=="vla":
        probabilities=torch.softmax(vla_forward(models[0],b),dim=-1)[0]
        confidence,index=probabilities.max(-1); index=int(index); accepted=float(confidence)>=confidence_threshold
        waypoint=meta["candidates"][index]["name"]
        return {"kind":"finite_vla_shadow","predicted_waypoint":waypoint,"confidence":float(confidence),"proposal":waypoint if accepted else "ABSTAIN","stop_proposal":bool(accepted and index==0),"actuation":False,"calibrated":False}
    outputs=[world_forward(m,b) for m in models]
    scans=torch.stack([o[0] for o in outputs]); motions=torch.stack([o[1] for o in outputs]); mean=scans.mean(0)[0]*meta["lidar_max_m"]; std=scans.std(0,unbiased=False)[0]*meta["lidar_max_m"]
    observed=b["mask"][0].bool()
    max_std=float(std[observed].max()) if observed.any() else None
    return {"kind":"world_shadow","predicted_lidar_m":mean.cpu().tolist(),"current_observed_mask":observed.cpu().tolist(),"ensemble_lidar_std_m":std.cpu().tolist(),"predicted_body_dx_dy_dyaw":motions.mean(0)[0].cpu().tolist(),"odometry_source":meta["odometry_source"],"action_source":meta["action_source"],"motion_target_is_ground_truth":False,"max_observed_std_m":max_std,"uncertain":max_std is None or max_std>uncertainty_m,"actuation":False,"calibrated":False,"note":"Predictions for currently unobserved sectors are unverified; RF2O motion targets share lidar observations and are not independent ground truth."}


def main():
    p=argparse.ArgumentParser(__doc__); p.add_argument("--checkpoint",required=True); p.add_argument("--meta",required=True); p.add_argument("--data",required=True); p.add_argument("--row",type=int,default=0); p.add_argument("--instruction"); p.add_argument("--action",nargs=2,type=float); p.add_argument("--dt",type=float); p.add_argument("--device",default="cpu"); p.add_argument("--confidence",type=float,default=.8); args=p.parse_args()
    meta=load_metadata(args.meta); data=load_dataset([args.data],meta)
    if not 0<=args.row<len(data["rgb"]): p.error("row outside dataset")
    # Override inputs explicitly for counterfactual research; never rewrite target labels.
    if args.instruction is not None: data["instruction"]=data["instruction"].astype("U512"); data["instruction"][args.row]=args.instruction
    if args.action is not None: data["action"][args.row]=args.action
    if args.dt is not None:
        if args.dt<=0: p.error("dt must be positive")
        data["dt"][args.row]=args.dt
    models,kind=load_checkpoint(args.checkpoint,meta,args.device)
    if kind=="vla" and not data["map_pose_valid"][args.row]: p.error("VLA requires a valid synchronized map->base pose")
    if kind=="world" and not data["action_valid"][args.row] and args.action is None: p.error("Row has no acknowledged action; provide explicit counterfactual --action")
    b=tensors(data,np.array([args.row]),args.device,meta)
    print(json.dumps(predict(models,kind,b,meta,args.confidence),ensure_ascii=False,indent=2))


if __name__=="__main__": main()
