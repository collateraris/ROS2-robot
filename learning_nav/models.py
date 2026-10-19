"""Small train-from-scratch models. These are research baselines, not Nav2."""
import torch
from torch import nn


class VisionEncoder(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(nn.Conv2d(3,16,5,2,2),nn.ReLU(),nn.Conv2d(16,32,3,2,1),nn.ReLU(),nn.Conv2d(32,48,3,2,1),nn.ReLU(),nn.AdaptiveAvgPool2d(1),nn.Flatten(),nn.Linear(48,64),nn.ReLU())

    def forward(self, rgb):
        return self.net(rgb)


class WorldModel(nn.Module):
    """p(next clearance, body displacement | RGB, scan, mask, action, dt).

    Action enters the prediction head directly; it is ACKed setpoint, while
    odometry supplies measured motion targets. Predictions never command a base.
    """
    def __init__(self, sectors):
        super().__init__()
        self.vision = VisionEncoder()
        self.trunk = nn.Sequential(nn.Linear(64+2*sectors+3,128),nn.SiLU(),nn.Linear(128,128),nn.SiLU())
        self.scan_head = nn.Linear(128,sectors)
        self.motion_head = nn.Linear(128,3)

    def forward(self,rgb,lidar,mask,action,dt):
        h=self.trunk(torch.cat((self.vision(rgb),lidar*mask,mask,action,dt.reshape(-1,1)),dim=-1))
        return torch.sigmoid(self.scan_head(h)),self.motion_head(h)


class TinyVLA(nn.Module):
    """Finite waypoint selection with images + lidar + text + known map goals.

    This is a task-specific supervised classifier, not a universal pretrained VLA.
    Output class 0 is STOP; remaining classes are fixed checkpoint metadata.
    """
    def __init__(self,sectors,candidates):
        super().__init__()
        self.vision=VisionEncoder()
        self.byte_embed=nn.Embedding(257,24,padding_idx=0)
        self.text=nn.GRU(24,48,batch_first=True)
        self.head=nn.Sequential(nn.Linear(64+2*sectors+48+2*candidates,128),nn.ReLU(),nn.Linear(128,candidates))

    def forward(self,rgb,lidar,mask,tokens,goals):
        encoded,_=self.text(self.byte_embed(tokens))
        last=(tokens!=0).sum(-1).clamp(min=1)-1
        language=encoded[torch.arange(len(tokens),device=tokens.device),last]
        return self.head(torch.cat((self.vision(rgb),lidar*mask,mask,language,goals.flatten(1)),dim=-1))


def masked_scan_mse(pred,target,mask):
    """Rows with no observations contribute zero, never divide by zero."""
    return (((pred-target)**2)*mask).sum()/mask.sum().clamp_min(1)
