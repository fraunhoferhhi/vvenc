import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import unittest
import tempfile
import json
from scripts.rl_rc_dot.evaluate import bd_metrics,report,evaluation_protocol,evaluation_deadline

def points(scale=1):
    return [{"actual_kbps":r*scale,"mean_precision":q} for r,q in [(50,.2),(80,.3),(120,.4),(160,.5),(200,.6)]]

class Evaluation(unittest.TestCase):
    def test_mini_five_rate_and_quick_protocols(self):
        for rates in ([50,120,200],[50,80,120,160,200]):
            evaluation_protocol("mini",256,rates)
        for frames,rates in ((64,[50,120,200]),(256,[50,120]),(256,[200,120,50])):
            with self.assertRaises(ValueError):evaluation_protocol("mini",frames,rates)

    def test_notebook_setup_consumes_evaluation_budget(self):
        self.assertEqual(evaluation_deadline(1,1000,now=50,wall=1600),2690)
        self.assertLess(evaluation_deadline(1,1000,now=50,wall=5000),50)
        self.assertEqual(evaluation_deadline(0,1000,now=50,wall=5000),float("inf"))

    def test_five_rate_mini_report_computes_bd_rate(self):
        pairs=[]
        for a,b in zip(points(),points(.8)):
            rate=a['actual_kbps']
            pairs.append(tuple(dict(p,video_id='mini',target_kbps=rate,mean_recall=.5,f1=.45) for p in (a,b)))
        with tempfile.TemporaryDirectory() as tmp:
            report(Path(tmp),pairs)
            summary=json.loads((Path(tmp)/'summary.json').read_text())
            self.assertEqual(summary['bd_rate_valid_videos'],1)
            self.assertAlmostEqual(summary['mean_bd_rate_percent'],-20,places=8)

    def test_three_rate_mini_summary_reports_pairs_without_bd_rate(self):
        rates=(50,120,200)
        pairs=[]
        for vid in ("testA","testB"):
            for rate in rates:
                anchor={"video_id":vid,"target_kbps":rate,"actual_kbps":rate,
                        "mean_precision":.4,"mean_recall":.5,"f1":.44}
                agent={"video_id":vid,"target_kbps":rate,"actual_kbps":rate*1.05,
                       "mean_precision":.45,"mean_recall":.52,"f1":.48}
                pairs.append((anchor,agent))
        with tempfile.TemporaryDirectory() as tmp:
            from pathlib import Path
            report(Path(tmp),pairs,rates)
            summary=json.loads((Path(tmp)/"summary.json").read_text())
            self.assertEqual(summary["complete_videos"],2)
            self.assertEqual(summary["completed_comparable_pairs"],6)
            self.assertAlmostEqual(summary["mean_delta_precision"],.05)
            self.assertAlmostEqual(summary["mean_rate_ratio_vs_anchor"],.05)
            self.assertIsNone(summary["mean_bd_rate_percent"])
            self.assertEqual(summary["precision_win_fraction"],1.0)

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
