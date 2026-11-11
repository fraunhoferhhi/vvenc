#!/usr/bin/env python3
"""Causal per-POC Python policy bridge for DOT_STATE / DOT_PROXY.
Strict by default: proxy states are for transport smoke tests only.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import time
from spec import parse_state,write_action

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--states",type=Path,required=True)
    p.add_argument("--actions",type=Path,required=True)
    p.add_argument("--trajectory",type=Path,required=True)
    p.add_argument("--frames",type=int,required=True)
    p.add_argument("--checkpoint",type=Path)
    p.add_argument("--mode",choices=("zero","ppo"),default="ppo")
    p.add_argument("--allow-proxy-state",action="store_true")
    p.add_argument("--max-delta",type=int,default=12)
    p.add_argument("--idle-timeout",type=float,default=600.)
    p.add_argument("--seed",type=int,default=7)
    args=p.parse_args()
    if args.frames<=0 or args.idle_timeout<=0:p.error("frames and idle-timeout must be positive")
    if args.states.exists():p.error("State file must not exist before a new encoder process")
    if args.trajectory.exists():p.error("Trajectory already exists")
    if args.actions.exists() and any(args.actions.iterdir()):p.error("Action directory is nonempty")
    if args.mode=="ppo":
        import torch
        from agent import RLRCDoT,interpolate_qp
        torch.manual_seed(args.seed)
        model=RLRCDoT().eval()
        if args.checkpoint:
            checkpoint=torch.load(args.checkpoint,map_location="cpu",weights_only=True)
            model.load_state_dict(checkpoint.get("model",checkpoint))
    args.actions.mkdir(parents=True,exist_ok=True)
    args.states.parent.mkdir(parents=True,exist_ok=True)
    args.trajectory.parent.mkdir(parents=True,exist_ok=True)
    touched=time.monotonic()
    while not args.states.is_file():
        if time.monotonic()-touched>args.idle_timeout:raise TimeoutError("VVenC state stream did not appear")
        time.sleep(.02)
    seen=set()
    with args.states.open(encoding="utf-8") as inp,args.trajectory.open("x",encoding="utf-8") as out:
        while len(seen)<args.frames:
            line=inp.readline()
            if not line:
                if time.monotonic()-touched>args.idle_timeout:
                    raise TimeoutError(f"Waiting for observation: got {len(seen)} / {args.frames}")
                time.sleep(.02)
                continue
            touched=time.monotonic()
            state=parse_state(line,allow_proxy=args.allow_proxy_state)
            if state.poc<0 or state.poc>=args.frames or state.poc in seen:
                raise RuntimeError(f"Duplicate or unexpected POC={state.poc}")
            if args.mode=="zero":
                delta=[0]*600
                rec={"poc":state.poc,"proxy":state.proxy,"global":list(state.globals),
                     "local":[list(m) for m in state.maps],"discrete_action":delta,
                     "raw_action":None,"old_logp":None,"old_value":None}
            else:
                import torch
                spatial=torch.tensor(state.maps,dtype=torch.float32).view(1,4,20,30)
                glob=torch.tensor(state.globals,dtype=torch.float32).view(1,16)
                with torch.no_grad():
                    raw,lp,value,_=model.sample(spatial,glob)
                delta=interpolate_qp(raw[0],max_delta=args.max_delta)
                rec={"poc":state.poc,"proxy":state.proxy,"global":list(state.globals),
                     "local":[list(m) for m in state.maps],"discrete_action":delta,
                     "raw_action":raw[0].tolist(),"old_logp":float(lp.item()),
                     "old_value":float(value.item())}
            write_action(args.actions,state.poc,delta)
            out.write(json.dumps(rec,separators=(",",":"))+"\n")
            out.flush()
            seen.add(state.poc)
    print(f"DOT_BRIDGE_PASS frames={len(seen)} proxy={args.allow_proxy_state}",flush=True)

if __name__=="__main__":
    main()
