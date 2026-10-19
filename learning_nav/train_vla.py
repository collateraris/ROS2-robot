"""Train finite-command VLA from human-labeled map waypoint demonstrations."""
import argparse
from pathlib import Path
import numpy as np
import torch
from .data import load_metadata,load_dataset,metadata_fingerprint,SCHEMA_VERSION
from .models import TinyVLA
from .train_utils import add_common,seed_all,tensors,split_usable,batches,vla_forward,write_metrics


@torch.no_grad()
def evaluate(model,data,ids,meta,device,batch_size,majority):
    model.eval(); k=len(meta["candidates"]); confusion=np.zeros((k,k),int); baseline=0
    for rows in batches(ids,batch_size):
        b=tensors(data,rows,device,meta); pred=vla_forward(model,b).argmax(-1).cpu().numpy(); target=data["label"][rows]
        np.add.at(confusion,(target,pred),1); baseline+=int(np.sum(target==majority))
    recalls=[float(confusion[i,i]/confusion[i].sum()) for i in range(k) if confusion[i].sum()]
    return {"rows":len(ids),"accuracy":float(np.trace(confusion)/len(ids)),"balanced_accuracy_observed_classes":float(np.mean(recalls)),"majority_class_baseline_accuracy":baseline/len(ids),"confusion_true_rows_pred_columns":confusion.tolist(),"class_names":[c["name"] for c in meta["candidates"]]}


def main():
    p=argparse.ArgumentParser(__doc__); add_common(p); args=p.parse_args()
    if args.epochs<1: p.error("epochs must be positive")
    seed_all(args.seed); meta=load_metadata(args.meta); data=load_dataset(args.data,meta)
    splits=split_usable(data,(data["label"]>=0)&data["map_pose_valid"].astype(bool)&(np.char.str_len(data["instruction"].astype(str))>0),args.seed)
    counts=np.bincount(data["label"][splits["train"]],minlength=len(meta["candidates"]))
    if np.any(counts==0): raise ValueError("Every candidate including STOP needs examples in the training sessions")
    model=TinyVLA(meta["sectors"],len(meta["candidates"])).to(args.device)
    optimizer=torch.optim.AdamW(model.parameters(),lr=args.lr)
    # Inverse frequency class weighting; validation/test keep their natural distribution.
    weights=torch.as_tensor(counts.sum()/(len(counts)*counts),dtype=torch.float32,device=args.device)
    criterion=torch.nn.CrossEntropyLoss(weight=weights); best=None; best_score=-1
    majority=int(counts.argmax())
    for epoch in range(args.epochs):
        model.train()
        for rows in batches(splits["train"],args.batch_size,True):
            b=tensors(data,rows,args.device,meta); loss=criterion(vla_forward(model,b),b["label"])
            optimizer.zero_grad(); loss.backward(); torch.nn.utils.clip_grad_norm_(model.parameters(),1.); optimizer.step()
        score=evaluate(model,data,splits["val"],meta,args.device,args.batch_size,majority)["balanced_accuracy_observed_classes"]
        if score>best_score: best_score=score; best={k:v.detach().cpu().clone() for k,v in model.state_dict().items()}
        print(f"epoch={epoch+1} validation_balanced_accuracy={score:.4f}",flush=True)
    model.load_state_dict(best); out=Path(args.out); out.mkdir(parents=True,exist_ok=True)
    torch.save({"kind":"tiny_vla","schema_version":SCHEMA_VERSION,"metadata":meta,"fingerprint":metadata_fingerprint(meta),"state":{k:v.cpu() for k,v in model.state_dict().items()}},out/"vla.pt")
    metrics={"synthetic":bool(meta.get("synthetic",False)),"odometry_source":meta["odometry_source"],"action_source":meta["action_source"],"split_sessions":{k:sorted(set(data["session"][v].tolist())) for k,v in splits.items()},"training_class_counts":counts.tolist(),"validation":evaluate(model,data,splits["val"],meta,args.device,args.batch_size,majority),"test":evaluate(model,data,splits["test"],meta,args.device,args.batch_size,majority),"note":"Finite known waypoint vocabulary; softmax confidence is not calibrated."}
    write_metrics(out,metrics)


if __name__=="__main__": main()
