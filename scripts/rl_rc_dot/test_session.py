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

if __name__=="__main__":unittest.main()
