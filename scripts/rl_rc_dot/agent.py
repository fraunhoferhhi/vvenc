"""PPO (3-layer CNN, Gaussian actor, value head, block-reward head).

The 5x7 coarse action is the public reproduction's configuration. This is a
research implementation and not the original authors' unreleased code.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import torch
from torch import nn, Tensor
from torch.distributions import Normal
from torch.nn import functional as F
from spec import *

class RLRCDoT(nn.Module):
    def __init__(self):
        super().__init__()
        self.cnn=nn.Sequential(
          nn.Conv2d(4,16,3,padding=1),nn.ReLU(),
          nn.Conv2d(16,32,3,padding=1),nn.ReLU(),
          nn.Conv2d(32,32,3,padding=1),nn.ReLU(),
          nn.AdaptiveAvgPool2d((4,6)),nn.Flatten())
        self.fuse=nn.Sequential(nn.Linear(32*4*6+16,64),nn.ReLU())
        self.actor=nn.Linear(64,35)
        self.log_std=nn.Parameter(torch.full((35,),-.5))
        self.critic=nn.Linear(64,1)
        self.reward_head=nn.Linear(64,600)

    def forward(self,local:Tensor,global_:Tensor):
        if local.ndim!=4 or tuple(local.shape[1:])!=(4,20,30):
            raise ValueError("local tensor must be [N,4,20,30]")
        if global_.ndim!=2 or tuple(global_.shape)!=(local.shape[0],16):
            raise ValueError("global tensor must be [N,16]")
        hidden=self.fuse(torch.cat([self.cnn(local),global_],dim=1))
        mean=self.actor(hidden)
        distribution=Normal(mean,self.log_std.clamp(-5,2).exp().expand_as(mean))
        return distribution,self.critic(hidden).flatten(),self.reward_head(hidden).reshape(-1,20,30)

    @staticmethod
    def logprob(dist:Normal,raw:Tensor)->Tensor:
        jac=2*(math.log(2.)-raw-F.softplus(-2*raw))
        return (dist.log_prob(raw)-jac).sum(dim=-1)

    def sample(self,local:Tensor,global_:Tensor,deterministic=False):
        dist,value,aux=self(local,global_)
        raw=dist.mean if deterministic else dist.rsample()
        return raw,self.logprob(dist,raw),value,aux

    def evaluate(self,local:Tensor,global_:Tensor,raw:Tensor):
        dist,value,aux=self(local,global_)
        return self.logprob(dist,raw),value,aux,dist.entropy().sum(dim=-1)

def interpolate_qp(raw:Tensor,max_delta:int=12)->list[int]:
    if raw.numel()!=35 or not 1<=max_delta<=12:
        raise ValueError("Expected 35 action components; max_delta 1..12")
    grid=raw.detach().float().reshape(1,1,5,7).tanh()
    grid=F.interpolate(grid,size=(20,30),mode="bilinear",align_corners=False)
    return grid.mul(max_delta).round().clamp(-12,12).int().flatten().tolist()

@dataclass
class Transition:
    local:Tensor
    global_:Tensor
    raw:Tensor
    old_logp:float
    old_value:float
    reward:float
    done:bool
    aux_target:Tensor|None=None
    aux_mask:Tensor|None=None

def gae(steps:list[Transition],gamma=.99,lam=.95,bootstrap=0.):
    if not steps:raise ValueError("Empty rollout")
    advantages=torch.zeros(len(steps))
    returns=torch.zeros(len(steps))
    next_value=bootstrap
    running=0.
    for i in range(len(steps)-1,-1,-1):
        t=steps[i]
        not_done=not t.done
        error=t.reward+gamma*next_value*not_done-t.old_value
        running=error+gamma*lam*not_done*running
        advantages[i]=running
        returns[i]=running+t.old_value
        next_value=t.old_value
    return advantages,returns

class PPOTrainer:
    def __init__(self,model:RLRCDoT,lr=3e-4,clip=.2,epochs=5,minibatch=64,aux_weight=.1):
        self.model=model
        self.opt=torch.optim.Adam(model.parameters(),lr=lr)
        self.clip=clip
        self.epochs=epochs
        self.minibatch=minibatch
        self.aux_weight=aux_weight

    def update(self,steps:list[Transition]):
        adv,returns=gae(steps)
        adv=(adv-adv.mean())/(adv.std(unbiased=False)+1e-8)
        local=torch.stack([s.local for s in steps]).float()
        glob=torch.stack([s.global_ for s in steps]).float()
        raw=torch.stack([s.raw for s in steps]).float()
        oldp=torch.tensor([s.old_logp for s in steps])
        losses=[]
        for _ in range(self.epochs):
            for idx in torch.randperm(len(steps)).split(self.minibatch):
                logp,value,aux,entropy=self.model.evaluate(local[idx],glob[idx],raw[idx])
                ratio=(logp-oldp[idx]).clamp(-20,20).exp()
                pg=-torch.minimum(ratio*adv[idx],
                    ratio.clamp(1-self.clip,1+self.clip)*adv[idx]).mean()
                loss=pg+.5*F.mse_loss(value,returns[idx])-.001*entropy.mean()
                chosen=[j for j,i in enumerate(idx.tolist())
                        if steps[i].aux_target is not None and steps[i].aux_mask is not None]
                if chosen:
                    targets=torch.stack([steps[idx[j].item()].aux_target for j in chosen]).float()
                    mask=torch.stack([steps[idx[j].item()].aux_mask for j in chosen]).float()
                    if mask.sum().item()>0:
                        loss=loss+self.aux_weight*(((aux[chosen]-targets).square()*mask).sum()/mask.sum())
                self.opt.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(self.model.parameters(),.5)
                self.opt.step()
                losses.append(loss.detach().item())
        return {"loss":sum(losses)/len(losses),"optimizer_steps":len(losses),"transitions":len(steps)}
