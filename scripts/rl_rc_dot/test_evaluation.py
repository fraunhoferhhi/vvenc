import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import unittest
from scripts.rl_rc_dot.evaluate import bd_metrics

def points(scale=1):
    return [{"actual_kbps":r*scale,"mean_precision":q} for r,q in [(50,.2),(80,.3),(120,.4),(160,.5),(200,.6)]]

class Evaluation(unittest.TestCase):
    def test_exact_rate_scaling(self):
        self.assertAlmostEqual(bd_metrics(points(),points(.8))["bd_rate_percent"],-20,places=8)
    def test_duplicate_rate_is_not_a_pchip_crash(self):
        duplicate=points()+[{"actual_kbps":80,"mean_precision":.32}]
        self.assertIsNotNone(bd_metrics(duplicate,points(.8))["bd_rate_percent"])
    def test_disjoint_quality_has_no_extrapolated_bd_rate(self):
        other=[{**p,"mean_precision":p["mean_precision"]+.7} for p in points()]
        # Keep valid precision while ensuring no quality overlap.
        other=[{**p,"mean_precision":.8+i*.03} for i,p in enumerate(other)]
        self.assertIsNone(bd_metrics(points(),other)["bd_rate_percent"])
    def test_nonfinite_rd_point_rejected(self):
        bad=points();bad[0]["actual_kbps"]=float("nan")
        with self.assertRaises(ValueError):bd_metrics(bad,points())

if __name__=="__main__":unittest.main()
