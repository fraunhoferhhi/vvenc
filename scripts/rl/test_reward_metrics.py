"""Tests for QP-only frozen-YOLO pseudo-GT F1/rate reward invariants."""
import math
import unittest

from reward_metrics import (
    Detection, bbox_iou, match_one_to_one, calculate_frame_reward,
)


class RewardTests(unittest.TestCase):
    def setUp(self):
        self.ref = [Detection(10, 10, 40, 40)]
        self.correct = [Detection(10, 10, 40, 40, 0.8)]

    def test_bbox_iou_and_dedup(self):
        self.assertAlmostEqual(bbox_iou(self.ref[0], self.correct[0]), 1.0)
        self.assertEqual(bbox_iou(self.ref[0], Detection(50, 50, 60, 60)), 0.0)
        # Two predictions may NOT claim the same reference GT.
        c = match_one_to_one(self.ref, self.correct + self.correct)
        self.assertEqual((c.tp, c.fp, c.fn), (1, 1, 0))
        self.assertAlmostEqual(c.f1, 2.0 / 3.0)

    def test_greedy_iou_tie_uses_baseline_lowest_reference_index(self):
        # Equal-IoU first prediction must match GT index 0, leaving index 1
        # available for the second prediction at IoU threshold 0.7.
        reference = [Detection(0, 0, 10, 10), Detection(2, 0, 12, 10)]
        candidates = [
            Detection(1, 0, 11, 10, 0.99),
            Detection(2, 0, 12, 10, 0.90),
        ]
        counts = match_one_to_one(reference, candidates, iou_threshold=0.7)
        self.assertEqual((counts.tp, counts.fp, counts.fn), (2, 0, 0))

    def test_missed_boxes_reduce_f1(self):
        c = match_one_to_one(self.ref, [])
        self.assertEqual((c.tp, c.fp, c.fn), (0, 0, 1))
        self.assertEqual(c.f1, 0.0)
        self.assertEqual(c.recall, 0.0)

    def test_exact_f1_rate_at_target(self):
        r = calculate_frame_reward(
            video_id="train_video_1", poc=7, encoded_step=3,
            pseudo_gt=self.ref, reconstructed=self.correct,
            actual_frame_bits=4000, cumulative_bits=12000,
            frame_rate=30.0, target_bps=120000.0, beta=0.3,
        )
        self.assertEqual(r.counts.tp, 1)
        self.assertAlmostEqual(r.detection_term, 1.0)
        self.assertAlmostEqual(r.bitrate_penalty, 0.0)
        self.assertAlmostEqual(r.reward, 1.0)
        self.assertEqual(r.poc, 7)
        self.assertEqual(r.encoded_step, 3) # POC is not the encoding-order index

    def test_bitrate_penalizes_both_over_and_under_target(self):
        common = dict(
            video_id="train_video_1", poc=0, encoded_step=1,
            pseudo_gt=self.ref, reconstructed=self.correct,
            frame_rate=30.0, target_bps=120000.0, beta=0.5,
        )
        a = calculate_frame_reward(actual_frame_bits=2000, cumulative_bits=2000, **common)
        b = calculate_frame_reward(actual_frame_bits=8000, cumulative_bits=8000, **common)
        self.assertAlmostEqual(a.bitrate_penalty, b.bitrate_penalty)
        self.assertLess(a.reward, 1.0)
        self.assertLess(b.reward, 1.0)

    def test_empty_reference_no_free_reward_and_fp_penalty(self):
        common = dict(
            video_id="train_video_1", poc=1, encoded_step=2,
            actual_frame_bits=4000, cumulative_bits=8000,
            frame_rate=30, target_bps=120000, beta=0.3,
            pseudo_gt=[],
        )
        a = calculate_frame_reward(reconstructed=[], **common)
        b = calculate_frame_reward(reconstructed=self.correct, **common)
        self.assertEqual((a.counts.tp, a.counts.fp, a.counts.fn), (0, 0, 0))
        self.assertEqual(a.detection_term, 0.0)
        self.assertEqual(b.counts.fp, 1)
        self.assertAlmostEqual(b.detection_term, -0.25)
        self.assertGreater(a.reward, b.reward)

    def test_invalid_inputs_are_rejected(self):
        with self.assertRaises(ValueError):
            Detection(10, 0, 5, 10)
        with self.assertRaises(ValueError):
            match_one_to_one([], [], iou_threshold=0)
        with self.assertRaises(ValueError):
            calculate_frame_reward(
                video_id="x", poc=0, encoded_step=0,
                pseudo_gt=[], reconstructed=[],
                actual_frame_bits=0, cumulative_bits=0,
                frame_rate=30, target_bps=120000, beta=0.3,
            )


if __name__ == "__main__":
    unittest.main()
