#!/usr/bin/env python3
"""One verifiable on-policy PPO update from frozen-YOLO and actual AU bits.

Batches across full BDD100K are not automated here. The default strict mode
rejects proxy states; allowing them is engineering ONLY, not paper reproduction.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys

def main():
    p=argparse.ArgumentParser()
    for field in ("trajectory","au_csv","bitstream","source_detections","decoded_detections","output"):
        p.add_argument("--"+field.replace("_","-"),type=Path,required=True)
    p.add_argument("--video-id",required=True)
    p.add_argument("--fps",type=float,default=30.)
    p.add_argument("--target-bps",type=float,required=True)
    p.add_argument("--checkpoint-in",type=Path)
    p.add_argument("--task-weight",type=float,default=20.)
    p.add_argument("--allow-proxy-state",action="store_true")
    p.add_argument("--seed",type=int,default=7)
    args=p.parse_args()
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent/"rl"))
    from au_rate_audit import read_au_bits
    from reward_join import read_detections
    from reward_metrics import match_one_to_one
    from reward import paper_reward
    from agent import RLRCDoT,PPOTrainer,Transition
    import torch
    if args.output.exists():p.error("Checkpoint output already exists")
    trajectory=[json.loads(line) for line in args.trajectory.read_text().splitlines() if line.strip()]
    n=len(trajectory)
    if n<2:raise ValueError("Need at least two on-policy transitions")
    pocs=[r["poc"] for r in trajectory]
    if set(pocs)!=set(range(n)) or len(pocs)!=len(set(pocs)):
        raise ValueError("Missing, duplicate or unexpected POC")
    for r in trajectory:
        if r.get("proxy") and not args.allow_proxy_state:
            raise ValueError("DOT_PROXY cannot be used for scientific training")
        if len(r.get("global",[]))!=16 or len(r.get("local",[]))!=4 or any(len(c)!=600 for c in r["local"]):
            raise ValueError("Not a 16+4 state")
        if len(r.get("raw_action") or [])!=35 or len(r.get("discrete_action") or [])!=600:
            raise ValueError("Missing genuine 35-D PPO actions (zero-policy not trainable)")
    source=read_detections(args.source_detections,args.video_id,set(range(n)))
    decoded=read_detections(args.decoded_detections,args.video_id,set(range(n)))
    au=read_au_bits(args.au_csv,args.bitstream,n)
    au_by_poc={a.poc:a for a in au}
    torch.manual_seed(args.seed)
    model=RLRCDoT()
    trainer=PPOTrainer(model)
    if args.checkpoint_in:
        data=torch.load(args.checkpoint_in,map_location="cpu",weights_only=True)
        model.load_state_dict(data["model"])
        trainer.opt.load_state_dict(data["optimizer"])
    steps=[]
    accbits=0
    rewards=[]
    for index,row in enumerate(trajectory):
        poc=row["poc"]
        accbits+=au_by_poc[poc].au_bits
        counts=match_one_to_one(source[poc],decoded[poc],iou_threshold=.5)
        value=paper_reward(counts.precision,accbits,index+1,args.fps,
                           args.target_bps,args.task_weight,bool(source[poc]))
        steps.append(Transition(
            torch.tensor(row["local"],dtype=torch.float32).view(4,20,30),
            torch.tensor(row["global"],dtype=torch.float32),
            torch.tensor(row["raw_action"],dtype=torch.float32),
            float(row["old_logp"]),float(row["old_value"]),value,index==n-1))
        rewards.append(value)
    # Critical on-policy gate: the checkpoint model must be exactly the
    # collecting policy; all raw-action logprobs and values must match.
    with torch.no_grad():
        local=torch.stack([s.local for s in steps])
        global_=torch.stack([s.global_ for s in steps])
        raw=torch.stack([s.raw for s in steps])
        lp,v,_,_=model.evaluate(local,global_,raw)
        max_logp=max(abs(float(lp[i])-s.old_logp) for i,s in enumerate(steps))
        max_value=max(abs(float(v[i])-s.old_value) for i,s in enumerate(steps))
    if max_logp>1e-3 or max_value>1e-3:
        raise RuntimeError(f"Off-policy rollout (max logprob err={max_logp}, value err={max_value})")
    metrics=trainer.update(steps)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    torch.save({"model":model.state_dict(),"optimizer":trainer.opt.state_dict(),
                "source_trajectory_sha256":hashlib.sha256(args.trajectory.read_bytes()).hexdigest(),
                "metrics":metrics,"video_id":args.video_id,
                "is_proxy":any(r.get("proxy") for r in trajectory)},args.output)
    print(json.dumps({"result":"DOT_PPO_UPDATE_PASS","steps":n,"mean_reward":sum(rewards)/n,
                      "on_policy_logp_error":max_logp,"on_policy_value_error":max_value,
                      "checkpoint":str(args.output),"metrics":metrics},indent=2))
if __name__=="__main__":
    main()
