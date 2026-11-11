#!/usr/bin/env python3
"""Real-encoder smoke gate: hook-off vs zero action vs nonzero action.

Produces its own deterministic YUV420P 8-bit 480x320 sequence. This is not a
BDD100K or detector benchmark; decoder tests remain a separate requirement.
"""
import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import tempfile
from spec import MAP_ELEMENTS,parse_state,write_action

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def run(encoder,source,folder,name,action=None,frames=6):
    bitstream=folder/(name+".266")
    state=folder/(name+".states")
    env=os.environ.copy()
    for key in ("VVENC_RL_QP_MAP","VVENC_RL_ACTION_DIR","VVENC_RL_STATE_FILE",
                "VVENC_DOT_ACTION_DIR","VVENC_DOT_STATE_FILE"):
        env.pop(key,None)
    if action is not None:
        action_dir=folder/(name+"_maps")
        for poc in range(frames):
            write_action(action_dir,poc,[action]*MAP_ELEMENTS)
        env["VVENC_DOT_ACTION_DIR"]=str(action_dir)
        env["VVENC_DOT_STATE_FILE"]=str(state)
    cmd=[str(encoder),"-i",str(source),"-s","480x320","-r","30","-f",str(frames),
         "-c","yuv420","--preset","medium","--bitrate","120000","--passes","1",
         "--qpa","1","--internal-bitdepth","10","--threads","2",
         "--intraperiod","32","--refreshtype","cra",
         "--additional","GOPSize=32:LookAhead=1","-o",str(bitstream)]
    done=subprocess.run(cmd,env=env,capture_output=True,text=True,timeout=240)
    (folder/(name+".log")).write_text(done.stdout+"\n"+done.stderr)
    if done.returncode or not bitstream.exists() or bitstream.stat().st_size==0:
        raise RuntimeError(f"{name}: VVenC failed, rc={done.returncode}, tail={done.stderr[-1800:]}")
    if action is not None:
        lines=state.read_text().splitlines()
        if len(lines)!=frames:
            raise AssertionError(f"{name}: got {len(lines)} states, expected {frames}")
        for row in lines:
            obs=parse_state(row,allow_proxy=True)
            if not obs.proxy:
                raise AssertionError("Prototype must NOT falsely claim DOT_STATE")
    return sha(bitstream)

def main():
    p=argparse.ArgumentParser()
    p.add_argument("--encoder",type=Path,required=True)
    p.add_argument("--output",type=Path,required=True)
    p.add_argument("--frames",type=int,default=6)
    args=p.parse_args()
    if args.frames<2 or args.output.exists():
        p.error("frames must exceed 1; output must be new")
    args.output.mkdir(parents=True)
    video=args.output/"synthetic.yuv"
    with video.open("wb") as out:
        for frame in range(args.frames):
            luma=bytes((32+(3*x+2*y+frame*19+(x*y//100)%32)%190)
                        for y in range(320) for x in range(480))
            chroma=bytes([128])*(480*320//4)
            out.write(luma)
            out.write(chroma)
            out.write(chroma)
    vanilla=run(args.encoder,video,args.output,"vanilla",frames=args.frames)
    zero=run(args.encoder,video,args.output,"zero",action=0,frames=args.frames)
    plus=run(args.encoder,video,args.output,"plus",action=6,frames=args.frames)
    if vanilla!=zero:
        raise AssertionError("Disabled hook and zero DeltaQP were not bit-identical")
    if plus==vanilla:
        raise AssertionError("Nonzero DeltaQP produced no encoded bitstream change")
    print(f"DOT_CODEC_SMOKE_PASS frames={args.frames} zero_equal=True plus_differs=True")
if __name__=="__main__":
    main()
