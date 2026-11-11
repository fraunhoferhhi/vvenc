import unittest
try:
    import torch
except ImportError:
    torch=None

@unittest.skipIf(torch is None,"torch not installed")
class ModelTest(unittest.TestCase):
    def test_ppo_and_auxiliary_update(self):
        from agent import RLRCDoT,PPOTrainer,Transition,interpolate_qp
        torch.manual_seed(3)
        model=RLRCDoT()
        l=torch.rand(2,4,20,30)
        g=torch.rand(2,16)
        with torch.no_grad():
            raw,lp,v,aux=model.sample(l,g)
        self.assertEqual(tuple(raw.shape),(2,35))
        self.assertEqual(tuple(aux.shape),(2,20,30))
        self.assertEqual(model.fuse[0].in_features,32*19*29+16)
        self.assertEqual(model.fuse[0].out_features,128)
        self.assertEqual(model.reward_head[-1].out_features,600)
        self.assertEqual(len(interpolate_qp(raw[0])),600)
        self.assertTrue(torch.isfinite(lp).all())
        labels=torch.zeros(20,30)
        labels[1,1]=1.
        masks=torch.zeros(20,30)
        masks[1,1]=1.
        masks[2,2]=1. # False positive with zero reward is supervised
        before=model.reward_head[-1].weight.detach().clone()
        steps=[Transition(l[i],g[i],raw[i].detach(),lp[i].item(),v[i].item(),
                          reward=1.,done=i==1,aux_target=labels,aux_mask=masks)
               for i in range(2)]
        result=PPOTrainer(model,epochs=1,minibatch=2).update(steps)
        self.assertEqual(result["optimizer_steps"],1)
        self.assertFalse(torch.equal(before,model.reward_head[-1].weight.detach()))

    def test_gaussian_logprob_stability(self):
        from agent import RLRCDoT
        model=RLRCDoT()
        dist,value,aux=model(torch.zeros(1,4,20,30),torch.zeros(1,16))
        raw=torch.full((1,35),15.)
        self.assertTrue(torch.isfinite(model.logprob(dist,raw)).all())

if __name__=="__main__":
    unittest.main()
