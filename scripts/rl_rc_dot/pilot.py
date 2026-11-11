#!/usr/bin/env python3
"""Locked BDD100K Train-only VVC + RL-RC-DoT one-video pilot.

Requires Kaggle input manifests, installed VVdeC, GPU and locked YOLOv5n.
This is the 16+4 branch, not the old 8+3 prototype. The present encoder
only emits DOT_PROXY; explicit --allow-proxy-state enables smoke use ONLY.
"""
from __future__ import annotations
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import math

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent/"rl"))
from real_reward_pilot import (
    read_lock_manifest,read_baseline_config,dataset_source,
    preprocess_mov,convert_10bit_to_8bit,temporal_alignment_check,
    load_frozen_yolov5,detect_all,run_cmd,sha256_file,
    W,H,FPS,BITRATE_KBPS
)
from au_rate_audit import read_au_bits
from reward_join import read_detections
from reward_metrics import match_one_to_one
from reward import paper_reward

def encode_dot(args,ref:Path,out:Path):
    out.mkdir(parents=True,exist_ok=False)
    state_file=out/"dot_states.txt"
    action_dir=out/"actions"
    trajectory=out/"trajectory.jsonl"
    bridge_log=out/"bridge.log"
    command=[sys.executable,"-u",str(HERE/"bridge.py"),
             "--states",str(state_file),"--actions",str(action_dir),
             "--trajectory",str(trajectory),"--frames",str(args.frames),
             "--mode","ppo","--seed",str(args.seed),"--idle-timeout","900"]
    if args.allow_proxy_state:command.append("--allow-proxy-state")
    if args.checkpoint_in:command.extend(["--checkpoint",str(args.checkpoint_in)])
    with bridge_log.open("w") as log:
        bridge=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        try:
            deadline=time.monotonic()+60
            while not action_dir.exists() and bridge.poll() is None:
                if time.monotonic()>deadline:raise TimeoutError("DOT policy bridge startup")
                time.sleep(.05)
            if bridge.poll() is not None:
                raise RuntimeError("DOT bridge exited early; read bridge.log")
            env=os.environ.copy()
            for name in ("VVENC_RL_QP_MAP","VVENC_RL_ACTION_DIR","VVENC_RL_STATE_FILE",
                         "VVENC_DOT_STATE_FILE","VVENC_DOT_ACTION_DIR","VVENC_RL_AU_BITS_FILE"):
                env.pop(name,None)
            env["VVENC_DOT_STATE_FILE"]=str(state_file)
            env["VVENC_DOT_ACTION_DIR"]=str(action_dir)
            env["VVENC_RL_AU_BITS_FILE"]=str(out/"au_bits.csv")
            bitstream=out/"dot.266"
            cmd=[str(args.encoder),"-i",str(ref),"-s",f"{W}x{H}",
                 "-r",str(FPS),"-f",str(args.frames),"-c","yuv420",
                 "--preset","medium","--internal-bitdepth","10",
                 "--bitrate",f"{args.bitrate_kbps}k","--passes","1","--qpa","1",
                 "--intraperiod","32","--refreshtype","cra",
                 "--sdr","sdr_709","--threads","4","--mtprofile","-1",
                 "--additional","GOPSize=32:LookAhead=1","-o",str(bitstream)]
            run_cmd(cmd,out/"encoder.log",env=env,timeout=2400)
            if bridge.wait(timeout=90):
                raise RuntimeError("DOT policy bridge failed: "+bridge_log.read_text()[-2000:])
            with trajectory.open() as stream:
                records=[json.loads(line) for line in stream if line.strip()]
            if len(records)!=args.frames or set(r["poc"] for r in records)!=set(range(args.frames)):
                raise ValueError("DOT trajectory missing or duplicate POCs")
            aus=read_au_bits(out/"au_bits.csv",bitstream,args.frames)
            return bitstream,trajectory,aus
        finally:
            if bridge.poll() is None:
                bridge.kill()
                bridge.wait()

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--repo",type=Path,required=True)
    p.add_argument("--encoder",type=Path)
    p.add_argument("--decoder",type=Path,required=True)
    p.add_argument("--out",type=Path,required=True)
    p.add_argument("--train-index",type=int,default=0)
    p.add_argument("--frames",type=int,default=64)
    p.add_argument("--bitrate-kbps",type=int,default=120,choices=[50,80,120,160,200])
    p.add_argument("--seed",type=int,default=7)
    p.add_argument("--checkpoint-in",type=Path)
    p.add_argument("--update",action="store_true",help="Perform ONE true on-policy PPO update")
    p.add_argument("--allow-proxy-state",action="store_true",
                   help="Engineering only; never use for scientific paper claims")
    args=p.parse_args()
    if not 0<=args.train_index<65 or not 32<=args.frames<=512 or args.seed<0:
        p.error("Train index 0..64; frames 32..512; seed >=0")
    if args.out.exists():p.error("Choose an unused output directory")
    args.encoder=args.encoder or args.repo/"bin/release-static/vvencapp"
    for path in (args.encoder,args.decoder):
        if not path.is_file():p.error("Missing encoder/decoder executable: "+str(path))
    if args.checkpoint_in and not args.checkpoint_in.is_file():
        p.error("Collecting policy checkpoint not found")
    # No implicit proxy opt-in: bridge will refuse DOT_PROXY until --allow-proxy-state.
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError("Kaggle CUDA GPU required to reproduce locked YOLOv5n evaluation")
    root=Path("/kaggle/input")
    splits,manifest=read_lock_manifest(root)
    lock=read_baseline_config(root)
    video_id=splits["train"][args.train_index]
    if video_id in splits["val"] or video_id in splits["test"]:
        raise RuntimeError("Split leakage")
    source_mov=dataset_source(manifest,video_id)
    args.out.mkdir(parents=True)
    source_yuv=args.out/"source_8bit.yuv"
    preprocess=preprocess_mov(source_mov,source_yuv,args.frames,args.out/"ffmpeg.log")
    bitstream,traj,aus=encode_dot(args,source_yuv,args.out/"online")
    raw10=args.out/"decoded_10bit.yuv"
    run_cmd([str(args.decoder),"-b",str(bitstream),"-o",str(raw10),
             "--threads","4"],args.out/"vvdec.log",timeout=900)
    recon8=args.out/"decoded_8bit.yuv"
    convert_10bit_to_8bit(raw10,recon8,args.frames)
    alignment=temporal_alignment_check(source_yuv,raw10,args.frames)
    detector,model_info=load_frozen_yolov5(args.repo,lock)
    source_boxes=args.out/"source_detections.jsonl"
    decoded_boxes=args.out/"decoded_detections.jsonl"
    n_source=detect_all(detector,source_yuv,args.frames,video_id,source_boxes)
    n_decoded=detect_all(detector,recon8,args.frames,video_id,decoded_boxes)
    expected=set(range(args.frames))
    ref=read_detections(source_boxes,video_id,expected)
    rec=read_detections(decoded_boxes,video_id,expected)
    records=[json.loads(line) for line in traj.read_text().splitlines() if line.strip()]
    by_poc={r.poc:r for r in aus}
    cumulative=0
    tp=fp=fn=0
    all_rewards=[]
    # Rewards follow POLICY DECISION ORDER and actual AU bits are joined by POC.
    for step,row in enumerate(records,start=1):
        poc=row["poc"]
        cumulative+=by_poc[poc].au_bits
        counts=match_one_to_one(ref[poc],rec[poc],iou_threshold=.5)
        tp+=counts.tp;fp+=counts.fp;fn+=counts.fn
        all_rewards.append(paper_reward(
            counts.precision,cumulative,step,FPS,args.bitrate_kbps*1000.0,
            task_weight=20.,has_reference=bool(ref[poc])))
    checkpoint=None
    if args.update:
        checkpoint=args.out/"dot_checkpoint.pt"
        traincmd=[sys.executable,str(HERE/"train.py"),
                  "--trajectory",str(traj),"--au-csv",str(args.out/"online"/"au_bits.csv"),
                  "--bitstream",str(bitstream),"--source-detections",str(source_boxes),
                  "--decoded-detections",str(decoded_boxes),"--video-id",video_id,
                  "--target-bps",str(args.bitrate_kbps*1000),
                  "--seed",str(args.seed),"--output",str(checkpoint)]
        if args.allow_proxy_state:traincmd.append("--allow-proxy-state")
        if args.checkpoint_in:traincmd.extend(["--checkpoint-in",str(args.checkpoint_in)])
        run_cmd(traincmd,args.out/"train.log",timeout=900)
    summary={"purpose":"TRAIN_ONLY_DOT_PROXY_PILOT_NOT_PAPER_REPRODUCTION",
        "train_video":video_id,"split":"train","frames":args.frames,
        "target_kbps":args.bitrate_kbps,
        "actual_kbps":bitstream.stat().st_size*8*FPS/args.frames/1000.,
        "au_bits_verified":sum(a.au_bits for a in aus)==bitstream.stat().st_size*8,
        "source_boxes":n_source,"decoded_boxes":n_decoded,
        "tp":tp,"fp":fp,"fn":fn,"micro_precision":tp/(tp+fp) if tp+fp else 0.,
        "micro_recall":tp/(tp+fn) if tp+fn else 0.,
        "mean_precision_reward":sum(all_rewards)/len(all_rewards),
        "checkpoint":str(checkpoint) if checkpoint else None,
        "is_proxy":any(row.get("proxy") for row in records),
        "detector":model_info,"preprocess":preprocess,
        "alignment":alignment,"lock_hash":lock["lock_sha256"],
        "collecting_checkpoint_sha256":sha256_file(args.checkpoint_in) if args.checkpoint_in else None}
    (args.out/"summary.json").write_text(json.dumps(summary,indent=2,ensure_ascii=False))
    print(json.dumps({"result":"DOT_TRAIN_PILOT_PASS","split":"train","video_id":video_id,
                      "is_proxy":summary["is_proxy"],"checkpoint":summary["checkpoint"],
                      "actual_kbps":summary["actual_kbps"],"micro_precision":summary["micro_precision"]},indent=2))

if __name__=="__main__":
    main()
