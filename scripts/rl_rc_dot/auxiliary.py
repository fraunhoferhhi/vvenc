"""Auxiliary block-target construction for the 16+4 PPO reward head.

Engineering interpretation: assign each reconstructed car detection to the
16x16 cell containing its box centre. Correct one-to-one IoU matches earn 1,
unmatched detections earn 0. Every occupied cell is supervised, INCLUDING
zero-reward false-positive cells. This is an explicit approximation to the
paper's auxiliary block rewards, not a claim of exact author implementation.
"""
from __future__ import annotations
from typing import Sequence

WIDTH=480
HEIGHT=320
CELL=16
GRID_WIDTH=30
GRID_HEIGHT=20
N=600

def _iou(a,b)->float:
    ix=max(0.,min(a.x2,b.x2)-max(a.x1,b.x1))
    iy=max(0.,min(a.y2,b.y2)-max(a.y1,b.y1))
    intersection=ix*iy
    area_a=(a.x2-a.x1)*(a.y2-a.y1)
    area_b=(b.x2-b.x1)*(b.y2-b.y1)
    union=area_a+area_b-intersection
    return intersection/union if union>0 else 0.

def block_reward_targets(pseudo_gt:Sequence,reconstructed:Sequence,threshold:float=.5):
    if not 0<threshold<=1:
        raise ValueError("IoU threshold must be in (0,1]")
    totals=[0.]*N
    counts=[0]*N
    available=set(range(len(pseudo_gt)))
    for pred in sorted(reconstructed,key=lambda b:b.confidence,reverse=True):
        # Same confidence ordered, unique best-IoU matcher as reward_metrics.
        index=None
        score=-1.
        for candidate in available:
            sim=_iou(pred,pseudo_gt[candidate])
            if sim>score or (sim==score and (index is None or candidate<index)):
                score,index=sim,candidate
        matched=index is not None and score>=threshold
        if matched:
            available.remove(index)
        cx=max(0.,min(float(WIDTH)-1e-6,(pred.x1+pred.x2)*.5))
        cy=max(0.,min(float(HEIGHT)-1e-6,(pred.y1+pred.y2)*.5))
        block=int(cy//CELL)*GRID_WIDTH+int(cx//CELL)
        counts[block]+=1
        totals[block]+=float(matched)
    values=[totals[i]/counts[i] if counts[i] else 0. for i in range(N)]
    mask=[1. if counts[i] else 0. for i in range(N)]
    return values,mask
