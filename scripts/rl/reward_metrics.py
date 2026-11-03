"""Reward computation primitives for the QP-only VVenC PPO pilot.

This module does not run YOLO and does not guess bitstream frame boundaries.
The caller MUST supply frozen-YOLOv5n detections on the original and
reconstructed frames after matching them by (video_id, POC), and must supply
actual per-access-unit coded bits (not YUV bytes or target bits).

The score threshold, NMS, car-class filter, resizing and box-matching rules
MUST be locked to the original Vanilla Baseline before scientific evaluation.
This module supplies an explicit confidence-ordered one-to-one IoU matcher;
if the Baseline uses a different matcher, adapt this module and re-test it.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


@dataclass(frozen=True)
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float = 1.0

    def __post_init__(self):
        nums = (self.x1, self.y1, self.x2, self.y2, self.confidence)
        if not all(math.isfinite(x) for x in nums):
            raise ValueError("Non-finite bounding box or score")
        if self.x2 <= self.x1 or self.y2 <= self.y1:
            raise ValueError("Bounding boxes must have positive width and height")
        if not (0.0 <= self.confidence <= 1.0):
            raise ValueError("Detection confidence outside [0,1]")


@dataclass(frozen=True)
class DetectionCounts:
    tp: int
    fp: int
    fn: int

    @property
    def precision(self) -> float:
        return self.tp / (self.tp + self.fp) if self.tp + self.fp else 0.0

    @property
    def recall(self) -> float:
        return self.tp / (self.tp + self.fn) if self.tp + self.fn else 0.0

    @property
    def f1(self) -> float:
        denom = 2 * self.tp + self.fp + self.fn
        return 2 * self.tp / denom if denom else 0.0


@dataclass(frozen=True)
class FrameReward:
    video_id: str
    poc: int
    encoded_step: int
    actual_frame_bits: int
    cumulative_bits: int
    actual_cumulative_bps: float
    counts: DetectionCounts
    detection_term: float
    bitrate_penalty: float
    reward: float


def bbox_iou(a: Detection, b: Detection) -> float:
    left = max(a.x1, b.x1)
    top = max(a.y1, b.y1)
    right = min(a.x2, b.x2)
    bottom = min(a.y2, b.y2)
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    area_a = (a.x2 - a.x1) * (a.y2 - a.y1)
    area_b = (b.x2 - b.x1) * (b.y2 - b.y1)
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def match_one_to_one(
    pseudo_gt: Sequence[Detection],
    reconstructed: Sequence[Detection],
    iou_threshold: float = 0.5,
) -> DetectionCounts:
    """Greedy one-to-one matching ordered by prediction confidence.

    Both inputs must already contain only class=car predictions after the
    Baseline-matched confidence/NMS filters. Original-frame detections serve
    as pseudo-labels, NOT human-annotated ground truth.
    """
    if not (0 < iou_threshold <= 1):
        raise ValueError("IoU threshold must be in (0,1]")
    unmatched = set(range(len(pseudo_gt)))
    tp = 0
    for p in sorted(reconstructed, key=lambda x: x.confidence, reverse=True):
        viable = [(bbox_iou(p, pseudo_gt[i]), i) for i in unmatched]
        if not viable:
            continue
        # Baseline tie-break: equal IoU prefers the lower reference index.
        best_iou, best_idx = max(viable, key=lambda pair: (pair[0], -pair[1]))
        if best_iou >= iou_threshold:
            unmatched.remove(best_idx)
            tp += 1
    return DetectionCounts(tp, len(reconstructed) - tp, len(unmatched))


def calculate_frame_reward(
    *,
    video_id: str,
    poc: int,
    encoded_step: int,
    pseudo_gt: Sequence[Detection],
    reconstructed: Sequence[Detection],
    actual_frame_bits: int,
    cumulative_bits: int,
    frame_rate: float,
    target_bps: float,
    beta: float,
    iou_threshold: float = 0.5,
    empty_reference_fp_penalty: float = 0.25,
) -> FrameReward:
    """Per-step F1/rate reward in the encoder's *policy decision* order.

    encoded_step starts at 1. Do NOT replace it with POC+1: VVC can reorder.
    Use the sum of actual bits from all access units encoded through this
    policy step, and the corresponding encoded_step/frame_rate budget clock.
    This is an initial rate penalty; GOP startup behavior needs validation.
    """
    if not video_id or poc < 0 or encoded_step < 1:
        raise ValueError("Missing video ID, invalid POC or encoded_step")
    if actual_frame_bits < 0 or cumulative_bits < actual_frame_bits:
        raise ValueError("Actual frame bits and cumulative bits are inconsistent")
    if frame_rate <= 0 or target_bps <= 0 or beta < 0 or empty_reference_fp_penalty < 0:
        raise ValueError("Invalid frame rate, target bitrate or reward weight")
    if not all(math.isfinite(x) for x in (frame_rate, target_bps, beta, empty_reference_fp_penalty)):
        raise ValueError("Non-finite reward parameters")

    counts = match_one_to_one(pseudo_gt, reconstructed, iou_threshold)
    if not pseudo_gt:
        # No free positive reward for an empty-empty pair; penalize FPs.
        detection_term = -empty_reference_fp_penalty * counts.fp
    else:
        detection_term = counts.f1

    elapsed = encoded_step / frame_rate
    measured_bps = max(1.0, cumulative_bits / elapsed)
    bitrate_penalty = beta * abs(math.log(measured_bps / target_bps))
    return FrameReward(
        video_id=video_id,
        poc=poc,
        encoded_step=encoded_step,
        actual_frame_bits=actual_frame_bits,
        cumulative_bits=cumulative_bits,
        actual_cumulative_bps=measured_bps,
        counts=counts,
        detection_term=detection_term,
        bitrate_penalty=bitrate_penalty,
        reward=detection_term - bitrate_penalty,
    )
