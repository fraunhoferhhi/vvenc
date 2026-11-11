"""Real SB3 forward/backward and replay tests using explicit synthetic tasks."""
import os
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import tempfile
import unittest
import numpy as np
import torch
import gymnasium as gym
from stable_baselines3.common.vec_env import DummyVecEnv,SubprocVecEnv
from scripts.rl_rc_dot.model import OBSERVATIONS,ACTIONS,AuxPPO,AuxiliaryLabels,build
from scripts.rl_rc_dot.method import action_map,wire_map

torch.set_num_threads(2)

class Fixture(gym.Env):
    observation_space=OBSERVATIONS
    action_space=ACTIONS
    def __init__(self,bad_action=False):self.bad_action=bad_action
    def reset(self,*,seed=None,options=None):
        super().reset(seed=seed);self.t=0
        return self.obs(),{}
    def obs(self):
        return {"global":np.full(16,self.t/10,np.float32),"mb":np.full((4,20,30),self.t/10,np.float32)}
    def step(self,action):
        self.t+=1
        consumed=wire_map(action_map(action))
        if self.bad_action:consumed[:]=10
        return self.obs(),float(self.t)/10,self.t==4,False,{"block_reward":np.zeros((20,30),np.float32),
                  "consumed_action":consumed.ravel().tolist()}
    def rng_state(self):return {"generator":self.np_random.bit_generator.state,"episodes_started":0}
    def restore_rng_state(self,state):self.np_random.bit_generator.state=state["generator"]
    def suspend_episode(self):pass

class IntegratedModel(unittest.TestCase):
    def test_two_spawned_collectors_share_one_on_policy_update(self):
        model=build(SubprocVecEnv([Fixture,Fixture],start_method="spawn"),7,n_steps=4,batch_size=4,n_epochs=1,device="cpu")
        try:
            model.learn(8)
            self.assertEqual(model.completed_rollouts,1)
            self.assertEqual(model.trained_frames,8)
            model.assert_checkpoint_finite()
        finally:model.get_env().close()
    def test_sb3_update_checkpoint_reload_and_fresh_resume(self):
        env=DummyVecEnv([Fixture])
        model=build(env,7,n_steps=4,batch_size=4,n_epochs=2,device="cpu")
        before={k:v.clone() for k,v in model.policy.state_dict().items()}
        try:
            model.learn(4)
            for key in ("action_net.weight","value_net.weight","reward_head.2.weight"):
                self.assertFalse(torch.equal(before[key],model.policy.state_dict()[key]),key)
            obs=Fixture().reset()[0]
            expected=model.predict(obs,deterministic=True)[0]
            with tempfile.TemporaryDirectory() as tmp:
                path=Path(tmp)/"checkpoint.zip";model.save(path)
                resumed=AuxPPO.load(path,env=AuxiliaryLabels(DummyVecEnv([Fixture])),device="cpu")
                np.testing.assert_array_equal(expected,resumed.predict(obs,deterministic=True)[0])
                self.assertEqual(model.completed_rollouts,resumed.completed_rollouts)
                resumed.learn(4,reset_num_timesteps=False)
                self.assertEqual(resumed.completed_rollouts,2)
                resumed.get_env().close()
            # Corrupted stored logprob must fail before any optimizer step.
            model.rollout_buffer.log_probs[0]=np.nan
            saved={k:v.clone() for k,v in model.policy.state_dict().items()}
            with self.assertRaisesRegex(ValueError,"Nonfinite"):model.train()
            for k,v in saved.items():self.assertTrue(torch.equal(v,model.policy.state_dict()[k]))
        finally:model.get_env().close()
    def test_target_kl_guard_reports_rejected_minibatch(self):
        model=build(DummyVecEnv([Fixture]),7,n_steps=4,batch_size=4,n_epochs=10,
                    learning_rate=1e-3,target_kl=1e-10,device="cpu")
        try:
            model.learn(4)
            metrics=model.last_update_metrics
            self.assertEqual(metrics["completed_rollouts"],1)
            self.assertEqual(metrics["stopped_by_target_kl"],1)
            self.assertGreater(metrics["max_approx_kl_observed"],1.5e-10)
            self.assertGreaterEqual(metrics["max_approx_kl_observed"],metrics["approx_kl"])
            self.assertLess(metrics["optimizer_steps"],10)
            model.assert_checkpoint_finite()
        finally:model.get_env().close()

    def test_wrong_consumed_map_is_rejected_before_training(self):
        model=build(DummyVecEnv([lambda:Fixture(True)]),7,n_steps=4,batch_size=4,n_epochs=1,device="cpu")
        try:
            with self.assertRaisesRegex(ValueError,"encoder-consumed"):model.learn(4)
            self.assertEqual(model.completed_rollouts,0)
        finally:model.get_env().close()

if __name__=="__main__":unittest.main()
