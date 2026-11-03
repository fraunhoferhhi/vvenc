"""Tests that reward follows VIDEO+POC, not arbitrary CSV/JSONL row order."""
import json
import tempfile
import unittest
from pathlib import Path

from reward_join import join_episode


class TestRewardJoin(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.video_id = "train_case"
        self.trajectory = self.root / "trajectory.jsonl"
        self.au = self.root / "au.csv"
        self.bitstream = self.root / "video.266"
        self.source = self.root / "source.jsonl"
        self.reconstructed = self.root / "recon.jsonl"
        self.bitstream.write_bytes(b"Z" * 30)

        # Decision order is POC 2,0,1; output AU order is POC 0,2,1.
        trajectory = []
        for poc in (2, 0, 1):
            trajectory.append({
                "poc": poc, "global": [0.0]*8,
                "spatial": [[[0.0]*4 for _ in range(3)] for _ in range(3)],
                "raw_action": [0.25]*12, "discrete_action": [1]*12,
                "value": float(poc + 1), "log_prob": -5.0,
                "reward": None, "done": None,
            })
        self._write_jsonl(self.trajectory, trajectory)
        self.au.write_text(
            "output_step,poc,au_bytes,au_bits\n"
            "1,0,10,80\n"
            "2,2,10,80\n"
            "3,1,10,80\n"
        )
        car = {"x1":10,"y1":10,"x2":40,"y2":40,"confidence":0.9,"class_id":2}
        self._write_jsonl(self.source, [
            {"video_id":self.video_id,"poc":poc,"detections":[car]}
            for poc in (1, 2, 0)
        ])
        # Exactly one missed detection at POC 2.
        self._write_jsonl(self.reconstructed, [
            {"video_id":self.video_id,"poc":poc,"detections":([] if poc==2 else [car])}
            for poc in (2, 0, 1)
        ])

    @staticmethod
    def _write_jsonl(path, rows):
        path.write_text("".join(json.dumps(r) + "\n" for r in rows))

    def tearDown(self):
        self.tmp.cleanup()

    def _join(self):
        return join_episode(
            video_id=self.video_id,
            trajectory_file=self.trajectory,
            au_csv=self.au,
            bitstream=self.bitstream,
            source_detections=self.source,
            decoded_detections=self.reconstructed,
            frame_rate=30.0,
            target_bps=2400.0,
            beta=0.1,
        )

    def test_join_does_not_confuse_poc_and_au_order(self):
        rows = self._join()
        self.assertEqual([r["poc"] for r in rows], [2,0,1])
        self.assertEqual([r["reward_diagnostics"]["encoded_step"] for r in rows], [1,2,3])
        self.assertEqual([r["au_output_step"] for r in rows], [2,1,3])
        self.assertEqual([r["reward_diagnostics"]["counts"]["fn"] for r in rows], [1,0,0])
        self.assertEqual([r["done"] for r in rows], [False, False, True])
        self.assertEqual([r["next_value"] for r in rows], [1.0, 2.0, 0.0])
        self.assertEqual(rows[0]["reward_diagnostics"]["cumulative_bits"], 80)
        self.assertEqual(rows[1]["reward_diagnostics"]["cumulative_bits"], 160)
        self.assertEqual(rows[2]["reward_diagnostics"]["cumulative_bits"], 240)

    def test_reject_duplicate_source_poc(self):
        rows = [json.loads(x) for x in self.source.read_text().splitlines()]
        rows[0]["poc"] = rows[1]["poc"]
        self._write_jsonl(self.source, rows)
        with self.assertRaises(ValueError):
            self._join()

    def test_reject_missing_decoded_frame(self):
        rows = [json.loads(x) for x in self.reconstructed.read_text().splitlines()]
        self._write_jsonl(self.reconstructed, rows[:-1])
        with self.assertRaises(ValueError):
            self._join()

    def test_reject_wrong_video(self):
        rows = [json.loads(x) for x in self.source.read_text().splitlines()]
        rows[0]["video_id"] = "test_dataset_video"
        self._write_jsonl(self.source, rows)
        with self.assertRaises(ValueError):
            self._join()

    def test_reject_wrong_total_au_bits(self):
        self.bitstream.write_bytes(b"bad")
        with self.assertRaises(ValueError):
            self._join()


if __name__ == "__main__":
    unittest.main()
