"""Native + SB3 + checkpoint integration. Detector is an explicit fixture."""
import os
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import shutil
import tempfile
import unittest
import numpy as np
import torch
import json
from functools import partial
from stable_baselines3.common.vec_env import DummyVecEnv
from scripts.rl_rc_dot.model import build,AuxPPO,AuxiliaryLabels
from scripts.rl_rc_dot.environment import VVenCEnv
from scripts.rl_rc_dot.preflight import make_fixture
from scripts.rl_rc_dot.session import validate,SaveAndValidate
from scripts.rl_rc_dot.data import atomic_json

class FixtureCatalog:
    lock={"splits":{"train":["fixture"],"val":["val_fixture"],"test":["test_fixture"]}}
    def prepare(self,video,path,limit=0):
        frames=limit or 8
        make_fixture(path,frames)
        return frames
class FixtureDetector:
    def __call__(self,frame):return np.array([[20.,30.,60.,70.,.9]])
    def close(self):pass

@unittest.skipUnless(os.environ.get("RCDOT_TEST_ENCODER"),"Set RCDOT_TEST_ENCODER for real codec integration")
class NativeIntegration(unittest.TestCase):
    def test_final_update_runs_paired_native_validation_and_selects_checkpoint(self):
        torch.set_num_threads(2)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            spec={"output":str(root),"encoder":os.environ["RCDOT_TEST_ENCODER"],"encoder_threads":2,
                  "input_root":str(root),"ffmpeg":"unused","ffprobe":"unused","yolo_repo":"fixture",
                  "weights":"fixture","gpu_ids":[],"validation_videos":1,"validation_rates":[120],
                  "validation_frames":8,"seed":7,"val_every_updates":25}
            contract={"synthetic_task_fixture":True};atomic_json(root/"method.json",contract)
            factory=lambda:VVenCEnv(FixtureCatalog(),FixtureDetector(),spec["encoder"],root/"episodes",frame_limit=8,threads=2)
            model=build(DummyVecEnv([factory]),7,n_steps=8,batch_size=4,n_epochs=1,device="cpu")
            validator=partial(validate,detector_factory=lambda *args:FixtureDetector(),catalog_factory=lambda *args:FixtureCatalog())
            try:model.learn(8,callback=SaveAndValidate(spec,contract,validator=validator))
            finally:model.get_env().close()
            progress=json.loads((root/"progress.json").read_text())
            self.assertIsNotNone(progress["best"])
            result=json.loads((root/progress["best"]["validation"]).read_text())
            self.assertEqual(len(result["points"]),1)
            self.assertIn("anchor",result["points"][0]);self.assertIn("agent",result["points"][0])

    def test_real_rollout_update_and_fresh_checkpoint_rollout(self):
        torch.set_num_threads(2)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            factory=lambda:VVenCEnv(FixtureCatalog(),FixtureDetector(),os.environ["RCDOT_TEST_ENCODER"],
                                    root/"episodes",frame_limit=8,threads=2)
            model=build(DummyVecEnv([factory]),7,n_steps=8,batch_size=4,n_epochs=2,device="cpu")
            try:
                model.learn(8)
                self.assertEqual(model.completed_rollouts,1)
                checkpoint=root/"checkpoint.zip";model.save(checkpoint)
            finally:model.get_env().close()
            resumed=AuxPPO.load(checkpoint,env=AuxiliaryLabels(DummyVecEnv([factory])),device="cpu")
            try:
                resumed.learn(8,reset_num_timesteps=False)
                self.assertEqual(resumed.completed_rollouts,2)
                resumed.assert_checkpoint_finite()
            finally:resumed.get_env().close()
            logs=list((root/"episodes").rglob("events.jsonl.gz"))
            self.assertGreaterEqual(len(logs),2)

if __name__=="__main__":unittest.main()
