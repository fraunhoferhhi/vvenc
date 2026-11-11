import unittest
from types import SimpleNamespace
from auxiliary import block_reward_targets

def box(x,y,w=10,h=10,confidence=.9):
    return SimpleNamespace(x1=x,y1=y,x2=x+w,y2=y+h,confidence=confidence)

class AuxiliaryLabels(unittest.TestCase):
    def test_correct_detection_and_false_positive(self):
        ref=[box(16,16)]
        pred=[box(16,16),box(80,16,confidence=.6)]
        target,mask=block_reward_targets(ref,pred)
        matched=1*30+1
        false_positive=1*30+5
        self.assertEqual((target[matched],mask[matched]),(1.,1.))
        self.assertEqual((target[false_positive],mask[false_positive]),(0.,1.))
        self.assertEqual(sum(mask),2.)

    def test_zero_reward_is_supervised(self):
        target,mask=block_reward_targets([], [box(10,10)])
        self.assertEqual(sum(target),0.)
        self.assertEqual(sum(mask),1.)

    def test_empty_frame_is_unlabelled(self):
        target,mask=block_reward_targets([], [])
        self.assertEqual(sum(target),0.)
        self.assertEqual(sum(mask),0.)

    def test_duplicate_detection_is_not_double_true_positive(self):
        ref=[box(16,16)]
        pred=[box(16,16,confidence=.9),box(16,16,confidence=.7)]
        target,mask=block_reward_targets(ref,pred)
        self.assertEqual(sum(mask),1.)
        self.assertEqual(sum(target),.5)

    def test_invalid_threshold(self):
        with self.assertRaises(ValueError):
            block_reward_targets([],[],threshold=0.)

if __name__=="__main__":
    unittest.main()
