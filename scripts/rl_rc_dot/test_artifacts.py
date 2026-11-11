"""Checkpoint transaction, RNG and Kaggle-safe resume regressions."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import json
import random
import tempfile
import unittest
import zipfile
import numpy as np
import torch
from stable_baselines3.common.vec_env import DummyVecEnv
from scripts.rl_rc_dot.test_integrated_model import Fixture
from scripts.rl_rc_dot.model import build
from scripts.rl_rc_dot.artifacts import (capture_rng,restore_rng,save_checkpoint,read_checkpoint,
                                        export_run,import_run,load_resume)
from scripts.rl_rc_dot.data import atomic_json

class Artifacts(unittest.TestCase):
    def test_rng_restores_parent_and_env_without_pending_reseed(self):
        env=DummyVecEnv([Fixture])
        env.reset()
        try:
            state=capture_rng(env)
            expected=(random.random(),np.random.rand(),torch.rand(3),env.env_method("rng_state"))
            env.seed(999) # pending VecEnv seeds must not defeat restoration
            restore_rng(state,env)
            self.assertEqual(random.random(),expected[0]);self.assertEqual(np.random.rand(),expected[1])
            self.assertTrue(torch.equal(torch.rand(3),expected[2]))
            self.assertEqual(env.env_method("rng_state"),expected[3])
            self.assertEqual(env._seeds,[None])
        finally:env.close()
    def test_generation_resume_safe_export_and_corruption_rejection(self):
        model=build(DummyVecEnv([Fixture]),7,n_steps=4,batch_size=4,n_epochs=1,device="cpu")
        try:
            model.learn(4)
            with tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp)/"run";root.mkdir()
                contract={"schema":"test-contract"};atomic_json(root/"method.json",contract)
                ref=save_checkpoint(model,model.get_env(),root,contract)
                # An interrupted, uncommitted generation is never chosen.
                orphan=root/"checkpoints/step999999999999_update99999999_deadbeef";orphan.mkdir()
                (orphan/"model.zip").write_bytes(b"incomplete")
                resumed_env=DummyVecEnv([Fixture])
                loaded,progress=load_resume(root,contract,resumed_env)
                self.assertEqual(loaded.completed_rollouts,1)
                loaded.get_env().close()
                evidence=root/"episodes/worker0/proof.json";evidence.parent.mkdir(parents=True)
                evidence.write_text('{"fixture":true}')
                exported=export_run(root,Path(tmp)/"session.zip")
                target=Path(tmp)/"restored"
                history=import_run(exported["safe_input"],target)
                self.assertIn("episodes/worker0/proof.json",history["parent_files"])
                self.assertFalse((target/"episodes").exists())
                directory,_=read_checkpoint(target,progress["latest"])
                self.assertTrue((directory/"model.zip").is_file())
                # Flip checkpoint bytes while retaining the export manifest.
                bad=Path(tmp)/"corrupt.zip"
                with zipfile.ZipFile(exported["export"]) as src,zipfile.ZipFile(bad,"w") as dst:
                    for name in src.namelist():
                        data=src.read(name)
                        if name.endswith("/model.zip"):data=data+b"corrupt"
                        dst.writestr(name,data)
                with self.assertRaisesRegex(ValueError,"Corrupt export"):import_run(bad,Path(tmp)/"bad")
                self.assertFalse(list((Path(tmp)/"bad").iterdir()))
        finally:model.get_env().close()
    def test_archive_traversal_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            archive=Path(tmp)/"bad.zip"
            with zipfile.ZipFile(archive,"w") as z:z.writestr("../escape","bad")
            with self.assertRaisesRegex(ValueError,"Unsafe"):import_run(archive,Path(tmp)/"out")
            self.assertFalse((Path(tmp)/"escape").exists())

if __name__=="__main__":unittest.main()
