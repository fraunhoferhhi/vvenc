"""Validation selection and final checkpoint regression without Test data."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import json
import tempfile
import unittest
from stable_baselines3.common.vec_env import DummyVecEnv
from scripts.rl_rc_dot.model import build
from scripts.rl_rc_dot.test_integrated_model import Fixture
from scripts.rl_rc_dot.session import SaveAndValidate
from scripts.rl_rc_dot.artifacts import read_checkpoint
from scripts.rl_rc_dot.data import atomic_json

class Session(unittest.TestCase):
    def test_ppo_parameters_are_validated(self):
        from scripts.rl_rc_dot.session import parser,ppo_options
        args=parser().parse_args(["--input-root","data","--encoder","enc","--decoder","dec",
                                  "--yolo-repo","yolo","--weights","weights","--output","run"])
        self.assertEqual(ppo_options(args),{"learning_rate":3e-5,"n_epochs":3,"target_kl":0.05})
        for field,bad in (("ppo_learning_rate",0),("ppo_learning_rate",float("nan")),
                          ("ppo_epochs",0),("ppo_target_kl",-0.1),("ppo_target_kl",float("inf"))):
            original=getattr(args,field);setattr(args,field,bad)
            with self.assertRaisesRegex(ValueError,"PPO requires"):ppo_options(args)
            setattr(args,field,original)

    def test_resume_contract_binds_ppo_hyperparameters(self):
        from scripts.rl_rc_dot.session import contract_for
        from unittest.mock import patch
        spec={"encoder":"enc","weights":"weights","frames":0,"seed":7,"workers":2,
              "rollout_steps":64,"encoder_threads":2,"validation_videos":2,
              "validation_rates":[50,120],"validation_frames":64,
              "ppo":{"learning_rate":3e-5,"n_epochs":3,"target_kl":0.05}}
        with patch("scripts.rl_rc_dot.session.digest",return_value="test-hash"), \
             patch("scripts.rl_rc_dot.session.source_fingerprint",return_value="source"):
            first=contract_for(spec)
            spec["ppo"]={**spec["ppo"],"learning_rate":1e-5}
            second=contract_for(spec)
        self.assertNotEqual(first,second)
        self.assertEqual(first["ppo"]["target_kl"],0.05)


    def test_final_update_is_saved_and_validated_even_without_next_rollout(self):
        with tempfile.TemporaryDirectory() as tmp:
            output=Path(tmp)
            spec={"output":str(output),"val_every_updates":25}
            method={"schema":"fixture"}
            atomic_json(output/"method.json",method)
            seen=[]
            def validator(model,spec,deadline):
                seen.append(model.completed_rollouts)
                return {"score":.2,"completed_rollouts":model.completed_rollouts,"points":[]}
            model=build(DummyVecEnv([Fixture]),7,n_steps=4,batch_size=4,n_epochs=1,device="cpu")
            try:
                callback=SaveAndValidate(spec,method,validator=validator)
                model.learn(4,callback=callback)
                self.assertEqual(seen,[1])
                progress=json.loads((output/"progress.json").read_text())
                self.assertEqual(progress["best"]["score"],.2)
                _,m=read_checkpoint(output,progress["latest"])
                self.assertEqual(m["completed_rollouts"],1)
                self.assertEqual(m["num_timesteps"],4)
            finally:model.get_env().close()

    def test_detector_gpu_routing_does_not_remask_visible_devices(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        import os
        import torch
        from scripts.rl_rc_dot.data import detector_device
        helper=SimpleNamespace(__file__=str(Path("fixture_yolo/utils/torch_utils.py").resolve()),
                               select_device=lambda *args: "original")
        original=helper.select_device
        before=os.environ.get("CUDA_VISIBLE_DEVICES")
        with patch("torch.cuda.is_available",return_value=True), \
             patch("torch.cuda.device_count",return_value=2), \
             patch("torch.cuda.set_device") as select, \
             patch("scripts.rl_rc_dot.data.importlib.import_module",return_value=helper):
            with detector_device("fixture_yolo","cuda:1") as device:
                self.assertEqual(str(device),"cuda:1")
                self.assertEqual(helper.select_device("cuda:0"),torch.device("cuda:1"))
            select.assert_called_once_with(torch.device("cuda:1"))
        self.assertIs(helper.select_device,original)
        self.assertEqual(os.environ.get("CUDA_VISIBLE_DEVICES"),before)

    def test_detector_loader_restores_helper_after_failure(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from scripts.rl_rc_dot.data import detector_device
        helper=SimpleNamespace(__file__=str(Path("fixture_yolo/utils/torch_utils.py").resolve()),
                               select_device=lambda *args: "original")
        original=helper.select_device
        with patch("scripts.rl_rc_dot.data.importlib.import_module",return_value=helper):
            with self.assertRaisesRegex(RuntimeError,"load failed"):
                with detector_device("fixture_yolo","cpu"):
                    raise RuntimeError("load failed")
        self.assertIs(helper.select_device,original)

if __name__=="__main__":unittest.main()
