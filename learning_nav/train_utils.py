from __future__ import annotations
import json
from pathlib import Path
import numpy as np
import torch
from .data import candidate_geometry, relative_motion, text_bytes, grouped_split


def seed_all(seed):
    np.random.seed(seed)
    torch.manual_seed(seed)


def tensors(data,idx,device,meta):
    def t(key,dtype=torch.float32):
        # Sensor-only VLA rows carry NaN action with action_valid=false. Their
        # unused action tensor is explicit padding; world training excludes them.
        if key=="action": return torch.as_tensor(np.nan_to_num(data[key][idx],nan=0.),dtype=dtype,device=device)
        return torch.as_tensor(data[key][idx],dtype=dtype,device=device)
    return {"rgb":t("rgb").permute(0,3,1,2)/255.,"lidar":t("lidar"),"mask":t("lidar_mask"),"action":t("action"),"dt":t("dt"),"next_lidar":t("next_lidar"),"next_mask":t("next_lidar_mask"),"motion":torch.as_tensor(relative_motion(data["odom"][idx],data["next_odom"][idx]),device=device),"tokens":torch.as_tensor(text_bytes(data["instruction"][idx]),device=device),"goals":torch.as_tensor(candidate_geometry(data["map_pose"][idx],meta),device=device),"label":t("label",torch.long)}


def split_usable(data,predicate,seed):
    splits=grouped_split(data["session"],seed)
    result={key:indices[predicate[indices]] for key,indices in splits.items()}
    if any(not len(i) for i in result.values()):
        raise ValueError("Each train/val/test group must have usable samples; record more sessions")
    return result


def write_metrics(directory,metrics):
    Path(directory).mkdir(parents=True,exist_ok=True)
    (Path(directory)/"metrics.json").write_text(json.dumps(metrics,indent=2,ensure_ascii=False),encoding="utf-8")


def batches(indices,batch_size,shuffle=False):
    indices=np.array(indices,copy=True)
    if shuffle: np.random.shuffle(indices)
    for start in range(0,len(indices),batch_size):
        yield indices[start:start+batch_size]


def add_common(parser):
    parser.add_argument("--data",nargs="+",required=True,help="One or more session NPZ files")
    parser.add_argument("--meta",required=True)
    parser.add_argument("--out",required=True)
    parser.add_argument("--epochs",type=int,default=20)
    parser.add_argument("--batch-size",type=int,default=32)
    parser.add_argument("--lr",type=float,default=1e-3)
    parser.add_argument("--seed",type=int,default=17)
    parser.add_argument("--device",default="cpu")


def world_forward(model,b):
    return model(b["rgb"],b["lidar"],b["mask"],b["action"],b["dt"])


def vla_forward(model,b):
    return model(b["rgb"],b["lidar"],b["mask"],b["tokens"],b["goals"])
