import csv
import tempfile
import unittest
from pathlib import Path
from spec import STATE_FIELDS
from smoke_codec import (expected_ctu_delta,verify_final_qp_trace,
                         verify_poc_response,verify_spatial_response,verify_state_pocs)


def state(poc):
    return f"DOT_PROXY {poc} "+" ".join(["0.1"]*(STATE_FIELDS-2))


class CodecEvidenceContracts(unittest.TestCase):
    def test_state_pocs_allow_reordering_but_reject_duplicates_and_gaps(self):
        self.assertEqual(verify_state_pocs([state(1),state(0)],2),[1,0])
        for pocs in ([0,0],[0,2],[0]):
            with self.assertRaises(AssertionError):
                verify_state_pocs([state(poc) for poc in pocs],2)

    def test_spatial_pooling_and_cpp_rounding(self):
        half=[6 if col<15 else -6 for row in range(20) for col in range(30)]
        self.assertEqual([expected_ctu_delta(half,0,x,0) for x in (0,128,256,384)],
                         [6,5,-6,-6])
        tie=[1 if col%2 else 0 for row in range(20) for col in range(30)]
        self.assertEqual(expected_ctu_delta(tie,0,0,0),1)
        self.assertEqual(expected_ctu_delta([-v for v in tie],0,0,0),-1)

    def test_trace_rejects_wrong_requested_poc_and_nonfinite_mean(self):
        fields=["poc","ctu","x","y","requested_delta","num_luma_cus", "min_qp","max_qp","area_mean_qp"]
        rows=[dict(zip(fields,[poc,ctu,(ctu%4)*128,(ctu//4)*128,
                              6 if poc else -6,1,30,30,30.]))
              for poc in range(2) for ctu in range(12)]
        action=lambda poc:[6 if poc else -6]*600
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"trace.csv"
            def write():
                with path.open("w",newline="") as f:
                    writer=csv.DictWriter(f,fieldnames=fields)
                    writer.writeheader()
                    writer.writerows(rows)
            write()
            self.assertEqual(len(verify_final_qp_trace(path,2,action=action)),24)
            rows[0]["requested_delta"]=6
            write()
            with self.assertRaisesRegex(AssertionError,"Requested delta"):
                verify_final_qp_trace(path,2,action=action)
            rows[0]["requested_delta"]=-6
            rows[0]["area_mean_qp"]=float("nan")
            write()
            with self.assertRaisesRegex(AssertionError,"Invalid winning"):
                verify_final_qp_trace(path,2,action=action)

    def test_spatial_response_rejects_reversed_regions(self):
        zero={(0,ctu):30. for ctu in range(12)}
        good={k:v+(6 if k[1]%4==0 else -6 if k[1]%4==3 else 0) for k,v in zero.items()}
        verify_spatial_response(zero,good,0)
        with self.assertRaises(AssertionError):
            verify_spatial_response(zero,{k:60-v for k,v in good.items()},0)

    def test_isolated_poc_response_rejects_changed_prefix_and_ignored_action(self):
        minus={(poc,ctu):30. for poc in range(2) for ctu in range(12)}
        plus={k:v+(6 if k[0]==1 else 0) for k,v in minus.items()}
        verify_poc_response(plus,minus,1)
        with self.assertRaises(AssertionError):
            verify_poc_response(minus,minus,1)
        plus[(0,0)]+=1
        with self.assertRaisesRegex(AssertionError,"before"):
            verify_poc_response(plus,minus,1)


if __name__=="__main__":
    unittest.main()
