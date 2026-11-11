"""Stable-Baselines3 2.7.0 PPO + the paper's 0.1 block-reward MSE.

The PPO optimization follows SB3's MIT-licensed PPO update. Environment,
buffer and finite-value checks bind rewards to the action consumed by VVenC.
"""
from typing import NamedTuple
import numpy as np
import torch
from torch import nn
from torch.nn import functional as F
from gymnasium import spaces
from stable_baselines3 import PPO
from stable_baselines3.common.buffers import DictRolloutBuffer
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from stable_baselines3.common.utils import explained_variance
from stable_baselines3.common.vec_env import VecEnvWrapper
from .method import action_map,wire_map

OBSERVATIONS=spaces.Dict({"global":spaces.Box(-np.inf,np.inf,(16,),dtype=np.float32),
                         "mb":spaces.Box(-np.inf,np.inf,(4,20,30),dtype=np.float32)})
ACTIONS=spaces.Box(-10,10,(35,),dtype=np.float32)

def finite(name,*values):
    for value in values:
        valid=torch.isfinite(value).all().item() if isinstance(value,torch.Tensor) else np.isfinite(value).all()
        if not valid:raise ValueError(f"Nonfinite {name}; update/checkpoint refused")

class StatsCNNExtractor(BaseFeaturesExtractor):
    def __init__(self,observation_space):
        super().__init__(observation_space,features_dim=64)
        self.cnn=nn.Sequential(nn.Conv2d(4,16,3,padding=1),nn.ReLU(),
            nn.Conv2d(16,32,3,padding=1),nn.ReLU(),nn.Conv2d(32,32,4,padding=1),nn.ReLU(),nn.Flatten())
        self.fc=nn.Sequential(nn.Linear(32*19*29+16,128),nn.ReLU(),nn.Linear(128,64),nn.ReLU())
    def forward(self,observations):
        return self.fc(torch.cat((self.cnn(observations["mb"]),observations["global"]),dim=1))

class AuxiliaryLabels(VecEnvWrapper):
    def __init__(self,env):
        super().__init__(env)
        self.last_block_reward=np.zeros((self.num_envs,20,30),dtype=np.float32)
        self.last_consumed_action=np.zeros((self.num_envs,20,30),dtype=np.float64)
    def reset(self):
        self.last_block_reward.fill(0)
        self.last_consumed_action.fill(0)
        return self.venv.reset()
    def step_wait(self):
        obs,rewards,done,infos=self.venv.step_wait()
        labels=np.asarray([info["block_reward"] for info in infos],dtype=np.float32)
        consumed=np.asarray([info["consumed_action"] for info in infos],dtype=np.float64).reshape(self.num_envs,20,30)
        finite("auxiliary targets/actions",labels,consumed,rewards,*obs.values())
        if labels.shape!=self.last_block_reward.shape or (labels<0).any() or (labels>1).any():
            raise ValueError("Invalid block targets")
        np.copyto(self.last_block_reward,labels)
        np.copyto(self.last_consumed_action,consumed)
        return obs,rewards,done,infos

class AuxSamples(NamedTuple):
    observations:dict
    actions:torch.Tensor
    old_values:torch.Tensor
    old_log_prob:torch.Tensor
    advantages:torch.Tensor
    returns:torch.Tensor
    auxiliary:torch.Tensor

class AuxBuffer(DictRolloutBuffer):
    def reset(self):
        super().reset()
        self.auxiliary=np.zeros((self.buffer_size,self.n_envs,20,30),dtype=np.float32)
        self.label_source=None
    def add(self,obs,action,reward,episode_start,value,log_prob):
        finite("rollout",action,reward,value,log_prob,*obs.values())
        if self.label_source is None:raise ValueError("Auxiliary label source is missing")
        labels=self.label_source.last_block_reward.copy()
        consumed=self.label_source.last_consumed_action
        expected=np.stack([wire_map(action_map(a)) for a in np.asarray(action).reshape(self.n_envs,35)])
        if not np.array_equal(expected,consumed):
            raise ValueError("Raw Gaussian action does not match the encoder-consumed map")
        self.auxiliary[self.pos]=labels
        super().add(obs,action,reward,episode_start,value,log_prob)
    def get(self,batch_size=None):
        if not self.generator_ready:
            self.auxiliary=self.swap_and_flatten(self.auxiliary)
        return super().get(batch_size)
    def _get_samples(self,batch_inds,env=None):
        data=super()._get_samples(batch_inds,env)
        return AuxSamples(*data,self.to_torch(self.auxiliary[batch_inds].reshape(len(batch_inds),600)))

class AuxPPO(PPO):
    def __init__(self,*args,aux_loss_coef=.1,**kwargs):
        self.aux_loss_coef=aux_loss_coef
        self.completed_rollouts=0
        self.trained_frames=0
        self.last_update_metrics={}
        kwargs.setdefault("rollout_buffer_class",AuxBuffer)
        super().__init__(*args,**kwargs)
    def _setup_model(self):
        super()._setup_model()
        self.policy.reward_head=nn.Sequential(nn.Linear(64,128),nn.ReLU(),nn.Linear(128,600)).to(self.device)
        self.policy.optimizer.add_param_group({"params":self.policy.reward_head.parameters()})
    def collect_rollouts(self,env,callback,rollout_buffer,n_rollout_steps):
        # PPO resets the buffer inside collect_rollouts; retain its wrapper
        # reference across that reset without retaining the VecEnv in saves.
        original=rollout_buffer.reset
        def reset():
            original()
            rollout_buffer.label_source=env
        rollout_buffer.reset=reset
        try:return super().collect_rollouts(env,callback,rollout_buffer,n_rollout_steps)
        finally:
            rollout_buffer.reset=original
            rollout_buffer.label_source=None
    def assert_checkpoint_finite(self):
        finite("policy parameters",*self.policy.state_dict().values())
        for state in self.policy.optimizer.state.values():
            finite("optimizer state",*[v for v in state.values() if isinstance(v,torch.Tensor)])
    def save(self,*args,**kwargs):
        self.assert_checkpoint_finite()
        return super().save(*args,**kwargs)
    def train(self):
        self.assert_checkpoint_finite()
        self.policy.set_training_mode(True)
        self._update_learning_rate(self.policy.optimizer)
        clip=self.clip_range(self._current_progress_remaining)
        clip_vf=self.clip_range_vf(self._current_progress_remaining) if self.clip_range_vf else None
        # Validate the whole fresh rollout before any optimizer step.
        with torch.no_grad():
            for data in self.rollout_buffer.get(self.batch_size):
                finite("stored rollout",data.actions,data.old_log_prob,data.old_values,
                       data.advantages,data.returns,data.auxiliary,*data.observations.values())
                values,logp,_=self.policy.evaluate_actions(data.observations,data.actions)
                finite("collecting-policy verification",values,logp)
                if (logp-data.old_log_prob).abs().max().item()>1e-3 or (values.flatten()-data.old_values).abs().max().item()>1e-3:
                    raise ValueError("Off-policy rollout; no update performed")
        records=[]
        stop=False
        for epoch in range(self.n_epochs):
            for data in self.rollout_buffer.get(self.batch_size):
                values,logp,entropy=self.policy.evaluate_actions(data.observations,data.actions)
                values=values.flatten()
                advantage=data.advantages
                if self.normalize_advantage and len(advantage)>1:
                    advantage=(advantage-advantage.mean())/(advantage.std()+1e-8)
                ratio=torch.exp(logp-data.old_log_prob)
                policy_loss=-torch.minimum(advantage*ratio,advantage*ratio.clamp(1-clip,1+clip)).mean()
                predicted=values if clip_vf is None else data.old_values+(values-data.old_values).clamp(-clip_vf,clip_vf)
                value_loss=F.mse_loss(predicted,data.returns)
                entropy_loss=-entropy.mean() if entropy is not None else logp.mean()
                auxiliary=F.mse_loss(self.policy.reward_head(self.policy.extract_features(data.observations)),data.auxiliary)
                loss=policy_loss+self.vf_coef*value_loss+self.ent_coef*entropy_loss+self.aux_loss_coef*auxiliary
                log_ratio=logp-data.old_log_prob
                kl=(torch.exp(log_ratio)-1-log_ratio).mean()
                finite("PPO objective",ratio,loss,kl)
                if self.target_kl is not None and kl.item()>1.5*self.target_kl:
                    stop=True
                    break
                self.policy.optimizer.zero_grad()
                loss.backward()
                norm=nn.utils.clip_grad_norm_(self.policy.parameters(),self.max_grad_norm,error_if_nonfinite=True)
                self.policy.optimizer.step()
                self.assert_checkpoint_finite()
                records.append({"loss":loss.item(),"policy_loss":policy_loss.item(),"value_loss":value_loss.item(),
                    "auxiliary_loss":auxiliary.item(),"entropy_loss":entropy_loss.item(),"approx_kl":kl.item(),
                    "clip_fraction":((ratio-1).abs()>clip).float().mean().item(),"gradient_norm":norm.item()})
            self._n_updates+=1
            if stop:break
        if not records:raise RuntimeError("PPO completed no optimizer steps")
        self.completed_rollouts+=1
        self.trained_frames+=self.n_steps*self.n_envs
        self.last_update_metrics={key:float(np.mean([r[key] for r in records])) for key in records[0]}
        self.last_update_metrics.update(optimizer_steps=len(records),completed_rollouts=self.completed_rollouts,
                                        trained_frames=self.trained_frames,num_timesteps=self.num_timesteps)
        coarse=12*np.tanh(np.clip(self.rollout_buffer.actions,-10,10))
        self.last_update_metrics.update(policy_std=self.policy.log_std.exp().mean().item(),
            coarse_delta_mean=float(coarse.mean()),coarse_delta_std=float(coarse.std()),
            coarse_delta_saturated_fraction=float((np.abs(coarse)>=11.5).mean()))
        for key,value in self.last_update_metrics.items():self.logger.record("train/"+key,value)
        self.logger.record("train/std",self.policy.log_std.exp().mean().item())
        ev=explained_variance(self.rollout_buffer.values.flatten(),self.rollout_buffer.returns.flatten())
        if np.isfinite(ev):self.logger.record("train/explained_variance",float(ev))
        self.logger.record("train/explained_variance_defined",int(np.isfinite(ev)))

def build(env,seed,device="cpu",**options):
    if env.observation_space!=OBSERVATIONS or env.action_space!=ACTIONS:raise ValueError("Method space differs")
    return AuxPPO("MultiInputPolicy",AuxiliaryLabels(env),seed=seed,device=device,
                  policy_kwargs={"features_extractor_class":StatsCNNExtractor,"net_arch":[],
                                 "share_features_extractor":True,"normalize_images":False},**options)
