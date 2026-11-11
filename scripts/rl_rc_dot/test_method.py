import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import unittest
import numpy as np
import torch
from scripts.rl_rc_dot.method import action_map,detection_targets,score,observation,SCHEMA

class Method(unittest.TestCase):
    def test_action_is_finite_bounded_and_rejects_nan(self):
        self.assertTrue(np.all(action_map(np.zeros(35))==0))
        result=action_map(np.arange(35)-17)
        self.assertEqual(result.shape,(20,30));self.assertLessEqual(np.abs(result).max(),12)
        with self.assertRaises(ValueError):action_map(np.full(35,np.nan))
    def test_half_pixel_interpolation_matches_torch(self):
        raw=np.random.default_rng(5).normal(size=35)
        coarse=torch.from_numpy(12*np.tanh(raw)).reshape(1,1,5,7)
        expected=torch.nn.functional.interpolate(coarse,size=(20,30),mode="bilinear",align_corners=False).numpy()[0,0]
        np.testing.assert_allclose(action_map(raw),expected,atol=1e-12,rtol=1e-12)
    def test_unique_match_and_false_positive_auxiliary_zero(self):
        ref=np.array([[10,10,30,30,.9]])
        pred=np.array([[10,10,30,30,.9],[10,10,30,30,.8],[80,10,100,30,.7]])
        labels,m=detection_targets(ref,pred)
        self.assertEqual((m["tp"],m["fp"],m["fn"]),(1,2,0))
        self.assertEqual(labels[1,1],.5)
        self.assertEqual(labels[1,5],0)
    def test_empty_frame_policy_and_rate_reward(self):
        reward,labels,m=score([],[],120000,30,120000)
        self.assertEqual(reward,0);self.assertEqual(m["precision"],0);self.assertEqual(labels.sum(),0)
        with self.assertRaises(ValueError):score([],[],0,1,120000)
    def test_cumulative_bits_use_bounded_scale_not_unbounded_linear_value(self):
        g=[0.]*16
        g[2]=3_500_000
        event={"schema":SCHEMA,"global":g,"maps":np.zeros((4,600)).tolist(),
               "history_poc":-1,"poc":0,"lookahead_last_poc":0}
        self.assertAlmostEqual(observation(event,1200)["global"][2],np.log1p(35),places=5)

    def test_completed_history_cannot_silently_have_zero_modes(self):
        event={"schema":SCHEMA,"global":[0.]*16,"maps":np.zeros((4,600)).tolist(),
               "history_poc":0,"poc":1,"lookahead_last_poc":3}
        with self.assertRaisesRegex(ValueError,"mode fractions"):observation(event,4)
        event["global"][5]=1
        self.assertEqual(observation(event,4)["mb"].shape,(4,20,30))

if __name__=="__main__":unittest.main()
