"""python -m learning_nav.train_world --data ... --meta ... --out ..."""
import argparse
from pathlib import Path
import copy
import numpy as np
import torch
from .data import load_metadata,load_dataset,metadata_fingerprint,SCHEMA_VERSION
from .models import WorldModel,masked_scan_mse
from .train_utils import add_common,seed_all,tensors,split_usable,batches,world_forward,write_metrics


@torch.no_grad()
def evaluate(models,data,idx,meta,device,batch_size):
    sums={"scan_se":0.,"persistence_scan_se":0.,"valid_sectors":0,"motion_se":0.,"zero_motion_se":0.,"kinematic_motion_se":0.,"rows":0,"ensemble_scan_std":0.}
    for m in models: m.eval()
    for rows in batches(idx,batch_size):
        b=tensors(data,rows,device,meta)
        predictions=[world_forward(m,b) for m in models]
        scans=torch.stack([p[0] for p in predictions]); motion=torch.stack([p[1] for p in predictions]).mean(0)
        scan=scans.mean(0); valid=b["next_mask"]
        # Compare on jointly observed sectors, so persistence has the same target support.
        common=valid*b["mask"]
        sums["scan_se"]+=float((((scan-b["next_lidar"])**2)*common).sum())
        sums["persistence_scan_se"]+=float((((b["lidar"]-b["next_lidar"])**2)*common).sum())
        sums["valid_sectors"]+=int(common.sum())
        sums["motion_se"]+=float(((motion-b["motion"])**2).sum())
        sums["zero_motion_se"]+=float((b["motion"]**2).sum())
        dt=b["dt"]; v,w=b["action"].unbind(-1); theta=w*dt
        safe_w=torch.where(w.abs()>1e-6,w,torch.ones_like(w))
        dx=torch.where(w.abs()>1e-6,v/safe_w*torch.sin(theta),v*dt)
        dy=torch.where(w.abs()>1e-6,v/safe_w*(1-torch.cos(theta)),torch.zeros_like(v))
        kinematic=torch.stack((dx,dy,theta),-1)
        sums["kinematic_motion_se"]+=float(((kinematic-b["motion"])**2).sum())
        sums["ensemble_scan_std"]+=float((scans.std(0,unbiased=False)*valid).sum())
        sums["rows"]+=len(rows)
    # Motion error mixes m and rad; retain component metrics in real experiments.
    c=max(1,sums["valid_sectors"]); n=max(1,sums["rows"]*3)
    return {"rows":sums["rows"],"valid_comparison_sectors":sums["valid_sectors"],"scan_rmse_m":(sums["scan_se"]/c)**.5*meta["lidar_max_m"],"persistence_scan_rmse_m":(sums["persistence_scan_se"]/c)**.5*meta["lidar_max_m"],"motion_m_rad_rmse":(sums["motion_se"]/n)**.5,"zero_motion_m_rad_rmse":(sums["zero_motion_se"]/n)**.5,"kinematic_motion_m_rad_rmse":(sums["kinematic_motion_se"]/n)**.5,"ensemble_scan_std_sum_m":sums["ensemble_scan_std"]*meta["lidar_max_m"]}


def main():
    p=argparse.ArgumentParser(__doc__); add_common(p)
    p.add_argument("--members",type=int,default=3)
    args=p.parse_args()
    if args.members<2 or args.epochs<1: p.error("Use >=2 ensemble members and >=1 epoch")
    seed_all(args.seed); meta=load_metadata(args.meta); data=load_dataset(args.data,meta)
    eligible=data["action_valid"].astype(bool)&(data["next_lidar_mask"]&data["lidar_mask"]).any(1)
    splits=split_usable(data,eligible,args.seed)
    models=[]; train_ids=splits["train"]
    for member in range(args.members):
        seed_all(args.seed+member)
        model=WorldModel(meta["sectors"]).to(args.device)
        optimizer=torch.optim.AdamW(model.parameters(),lr=args.lr)
        bootstrap=np.random.default_rng(args.seed+member).choice(train_ids,len(train_ids),replace=True)
        best=None; best_score=float("inf")
        for epoch in range(args.epochs):
            model.train()
            for rows in batches(bootstrap,args.batch_size,True):
                b=tensors(data,rows,args.device,meta); scan,motion=world_forward(model,b)
                loss=masked_scan_mse(scan,b["next_lidar"],b["next_mask"])+torch.mean((motion-b["motion"])**2)
                optimizer.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(),1.); optimizer.step()
            val=evaluate([model],data,splits["val"],meta,args.device,args.batch_size)
            score=val["scan_rmse_m"]/meta["lidar_max_m"]+val["motion_m_rad_rmse"]
            if score<best_score: best_score=score; best={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
            print(f"member={member} epoch={epoch+1} validation_score={score:.6f}",flush=True)
        model.load_state_dict(best); models.append(model)
    out=Path(args.out); out.mkdir(parents=True,exist_ok=True)
    torch.save({"kind":"world_ensemble","schema_version":SCHEMA_VERSION,"metadata":meta,"fingerprint":metadata_fingerprint(meta),"states":[m.cpu().state_dict() for m in models]},out/"world.pt")
    for m in models: m.to(args.device)
    metrics={"synthetic":bool(meta.get("synthetic",False)),"odometry_source":meta["odometry_source"],"action_source":meta["action_source"],"motion_target_is_ground_truth":False,"split_sessions":{k:sorted(set(data["session"][v].tolist())) for k,v in splits.items()},"validation":evaluate(models,data,splits["val"],meta,args.device,args.batch_size),"test":evaluate(models,data,splits["test"],meta,args.device,args.batch_size),"note":"Ensemble disagreement is uncalibrated. RF2O targets share input lidar and are not independent motion truth."}
    write_metrics(out,metrics)


if __name__=="__main__": main()
