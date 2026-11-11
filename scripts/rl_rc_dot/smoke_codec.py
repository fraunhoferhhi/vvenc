#!/usr/bin/env python3
"""Real-encoder smoke gate: hook-off vs zero action vs nonzero action.

Produces its own deterministic YUV420P 8-bit 480x320 sequence. This is not a
BDD100K or detector benchmark; decoder tests remain a separate requirement.
"""
import argparse
import hashlib
import csv
import math
import os
from pathlib import Path
import subprocess
import tempfile
from spec import MAP_ELEMENTS,parse_state,write_action

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_state_pocs(lines,frames):
    observations=[parse_state(row,allow_proxy=True) for row in lines]
    if any(not obs.proxy for obs in observations):
        raise AssertionError("Prototype must NOT falsely claim DOT_STATE")
    pocs=[obs.poc for obs in observations]
    if len(pocs)!=frames or set(pocs)!=set(range(frames)):
        raise AssertionError("Missing, duplicate or unexpected state POC")
    return pocs


def action_values(action,poc):
    if isinstance(action,int):
        return [action]*MAP_ELEMENTS
    return action(poc) if callable(action) else action


def expected_ctu_delta(action,poc,x,y):
    """Area pooling for this smoke profile: 480x320, CTU128, cell16.

    C++ llround uses ties away from zero, unlike Python's round().
    """
    values=action_values(action,poc)
    right,bottom=min(x+128,480),min(y+128,320)
    weighted=area=0
    for row in range(y//16,(bottom-1)//16+1):
        for col in range(x//16,(right-1)//16+1):
            overlap=(min(right,(col+1)*16)-max(x,col*16))*\
                    (min(bottom,(row+1)*16)-max(y,row*16))
            weighted+=overlap*values[row*30+col]
            area+=overlap
    mean=weighted/area
    return math.floor(mean+.5) if mean>=0 else math.ceil(mean-.5)


def verify_final_qp_trace(path,frames,expected_ctus=12,action=None):
    with path.open(newline="") as f:
        records=list(csv.DictReader(f))
    if len(records)!=frames*expected_ctus:
        raise AssertionError(f"QP trace has {len(records)} records instead of {frames*expected_ctus}")
    seen=set()
    values={}
    for r in records:
        poc,ctu=int(r["poc"]),int(r["ctu"])
        if not (0<=poc<frames and 0<=ctu<expected_ctus):
            raise AssertionError(f"Unexpected POC or CTU {(poc,ctu)}")
        if (poc,ctu) in seen:
            raise AssertionError(f"Duplicate POC/CTU {(poc,ctu)}")
        seen.add((poc,ctu))
        x,y=int(r["x"]),int(r["y"])
        if (x,y)!=((ctu%4)*128,(ctu//4)*128):
            raise AssertionError(f"Incorrect CTU coordinates {(poc,ctu,x,y)}")
        if action is not None and int(r["requested_delta"])!=expected_ctu_delta(action,poc,x,y):
            raise AssertionError(f"Requested delta does not match action at {(poc,ctu)}")
        minimum=int(r["min_qp"])
        maximum=int(r["max_qp"])
        mean=float(r["area_mean_qp"])
        if not math.isfinite(mean) or int(r["num_luma_cus"])<1 or minimum>mean+1e-6 or mean>maximum+1e-6:
            raise AssertionError(f"Invalid winning-CU QP statistics {(poc,ctu)}")
        values[(poc,ctu)]=mean
    return values


def verify_spatial_response(zero,spatial,first_poc):
    # Only compare the first policy decision: later frames can have different
    # RC bases because previous nonzero actions changed the bit budget.
    left=[(first_poc,ctu) for ctu in range(12) if ctu%4==0]
    right=[(first_poc,ctu) for ctu in range(12) if ctu%4==3]
    if not any(spatial[k]>zero[k]+1e-6 for k in left):
        raise AssertionError("Positive spatial action did not raise left-region QP")
    if not any(spatial[k]<zero[k]-1e-6 for k in right):
        raise AssertionError("Negative spatial action did not lower right-region QP")


def verify_poc_response(plus,minus,probe_poc):
    if set(plus)!=set(minus):
        raise AssertionError("POC probe traces have different coverage")
    if any(abs(plus[k]-minus[k])>1e-6 for k in plus if k[0]!=probe_poc):
        raise AssertionError("POC probe changed QP before its policy decision")
    if not any(plus[k]>minus[k]+1e-6 for k in plus if k[0]==probe_poc):
        raise AssertionError("Changing the probe POC action did not raise its QP")


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
            # A list requests spatially heterogeneous QP; an integer
            # requests constant QP; a sequence of full maps verifies POC use.
            values=action_values(action,poc)
            write_action(action_dir,poc,values)
        env["VVENC_DOT_ACTION_DIR"]=str(action_dir)
        env["VVENC_DOT_STATE_FILE"]=str(state)
        env["VVENC_DOT_QP_TRACE_FILE"]=str(folder/(name+".final_qp.csv"))
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
        verify_state_pocs(state.read_text().splitlines(),frames)
    stats=verify_final_qp_trace(folder/(name+".final_qp.csv"),frames,action=action) if action is not None else {}
    return sha(bitstream),stats

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
    vanilla,_=run(args.encoder,video,args.output,"vanilla",frames=args.frames)
    zero,zero_qp=run(args.encoder,video,args.output,"zero",action=0,frames=args.frames)
    plus,plus_qp=run(args.encoder,video,args.output,"plus",action=6,frames=args.frames)
    minus,minus_qp=run(args.encoder,video,args.output,"minus",action=-6,frames=args.frames)
    half=[6 if col<15 else -6 for row in range(20) for col in range(30)]
    spatial,spatial_qp=run(args.encoder,video,args.output,"spatial",action=half,frames=args.frames)
    alternate,alternate_qp=run(args.encoder,video,args.output,"alternating",
                  action=lambda poc: [6 if poc%2 else -6]*MAP_ELEMENTS,
                  frames=args.frames)
    if vanilla!=zero:
        raise AssertionError("Disabled hook and zero DeltaQP were not bit-identical")
    if len({vanilla,plus,minus,spatial,alternate})!=5:
        raise AssertionError("Positive/negative/spatial/POC actions not all distinct")
    if not any(plus_qp[k]>minus_qp[k] for k in plus_qp):
        raise AssertionError("Positive delta QP did not raise any actual final-CTU QP")
    order=verify_state_pocs((args.output/"zero.states").read_text().splitlines(),args.frames)
    verify_spatial_response(zero_qp,spatial_qp,order[0])
    # Identical zero-action prefixes isolate the last policy decision from
    # rate-control feedback. Changing just this POC must change its final QP.
    probe=order[-1]
    probe_plus,probe_plus_qp=run(args.encoder,video,args.output,"probe_plus",
        action=lambda poc:[6 if poc==probe else 0]*MAP_ELEMENTS,frames=args.frames)
    probe_minus,probe_minus_qp=run(args.encoder,video,args.output,"probe_minus",
        action=lambda poc:[-6 if poc==probe else 0]*MAP_ELEMENTS,frames=args.frames)
    if probe_plus==probe_minus:
        raise AssertionError("POC probe actions produced identical bitstreams")
    verify_poc_response(probe_plus_qp,probe_minus_qp,probe)
    print(f"DOT_CODEC_SMOKE_PASS frames={args.frames} zero_equal=True "
          "plus_minus_spatial_poc_distinct=True final_qp_trace=True "
          "requested_maps_verified=True spatial_direction=True isolated_poc=True")
if __name__=="__main__":
    main()

