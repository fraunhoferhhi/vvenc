import unittest
try:import torch
except ImportError:torch=None
@unittest.skipIf(torch is None,"torch not installed")
class ModelTest(unittest.TestCase):
    def test_ppo(self):
        from agent import RLRCDoT,PPOTrainer,Transition,interpolate_qp
        model=RLRCDoT()
        l=torch.rand(2,4,20,30);g=torch.rand(2,16)
        raw,lp,v,aux=model.sample(l,g)
        self.assertEqual(tuple(raw.shape),(2,35))
        self.assertEqual(len(interpolate_qp(raw[0])),600)
        self.assertTrue(torch.isfinite(lp).all())
        steps=[Transition(l[i],g[i],raw[i].detach(),float(lp[i]),float(v[i]),1.,i==1) for i in range(2)]
        self.assertEqual(PPOTrainer(model,epochs=1,minibatch=2).update(steps)["optimizer_steps"],1)
if __name__=="__main__":unittest.main()
