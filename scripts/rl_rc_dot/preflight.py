"""Real VVC encode/decode and applied-QP checks without a training run."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np

from .codec import NativeEpisode
from .data import atomic_json, digest
from .method import FRAME_BYTES, observation


def make_fixture(path, frames):
    y, x = np.mgrid[:320, :480]
    with path.open("wb") as stream:
        for index in range(frames):
            luma = (32 + ((x // 8 + y // 8) % 2) * 40 + ((x + index * 3) % 100)).astype(np.uint8)
            left = 20 + (index * 8) % 300
            luma[120:180, left:left + 70] = 200
            stream.write(luma.tobytes())
            stream.write(bytes([128]) * (480 * 320 // 2))


def decode_audit(folder, ffmpeg, frames, decoder=None):
    decoded = folder / "decoded.yuv"
    if decoder:
        args = [str(decoder), "-b", str(folder / "stream.266"), "-o", str(decoded), "--threads", "2"]
    else:
        args = [str(ffmpeg), "-hide_banner", "-nostdin", "-loglevel", "error", "-y",
                "-i", str(folder / "stream.266"), "-pix_fmt", "yuv420p10le", "-fps_mode", "passthrough",
                "-f", "rawvideo", str(decoded)]
    subprocess.run(args, check=True, timeout=300, stdout=subprocess.DEVNULL)
    if decoded.stat().st_size != frames * FRAME_BYTES * 2:
        raise ValueError("Decoder returned an unexpected number of frames")
    with decoded.open("rb") as stream:
        for poc in range(frames):
            if stream.read(FRAME_BYTES * 2) != (folder / "recon" / f"{poc}.yuv").read_bytes():
                raise ValueError(f"Decoder/native reconstruction differs at POC {poc}")
    return digest(decoded)

def source_poc_audit(source,folder,frames):
    raw=np.memmap(source,mode="r",dtype=np.uint8,shape=(frames,FRAME_BYTES))
    luma=np.asarray(raw[:,:480*320],dtype=np.float64)*4
    for poc in range(frames):
        reconstructed=np.fromfile(folder/"recon"/f"{poc}.yuv",dtype="<u2")[:480*320].astype(np.float64)
        errors=((luma-reconstructed)**2).mean(axis=1)
        if errors[poc]>errors.min()+1e-4:
            raise ValueError(f"Synthetic source/reconstruction POC mismatch at {poc}")
    raw._mmap.close()


def run(encoder, output, ffmpeg, *, frames=32, source=None, threads=2, decoder=None):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    synthetic=source is None
    if source is None:
        source = output / "source.yuv"
        make_fixture(source, frames)
    pattern = np.tile(np.repeat([-8., 8.], 15), (20, 1))
    reports = {}
    first_surfaces={}
    for name, mode in (("anchor", "anchor"), ("zero", "zero"), ("pattern", "live"), ("probe_minus","live"), ("probe_plus","live")):
        with NativeEpisode(encoder, source, output / name, frames, 120, mode=mode,
                           threads=threads, trace_cus=True) as native:
            while native.current is not None:
                observation(native.current, frames)
                action=pattern
                if name.startswith("probe_"):
                    probe=reports["zero"]["decision_pocs"][-1]
                    value=(-8. if name=="probe_minus" else 8.) if native.current["poc"]==probe else 0.
                    action=np.full((20,30),value)
                native.step(action if mode == "live" else None)
            for index,state in enumerate(native.states):
                if not index:continue
                previous=native.posts[index-1]
                g=state["global"]
                if (g[2]!=previous["cumulative_bits"] or not np.allclose(g[5:8],previous["mode_fractions"],atol=1e-8)
                        or not np.allclose(g[8:10],[previous["psnr"],previous["ssim"]],atol=1e-8)):
                    raise ValueError("State statistics differ from the previous completed picture")
            byte_hash = decode_audit(native.output, ffmpeg, frames, decoder)
            if synthetic:source_poc_audit(source,native.output,frames)
            cus = [json.loads(line) for line in (native.output / "cus.jsonl").read_text().splitlines()]
            per_poc = {}
            for poc in range(frames):
                frame_cus = [row for row in cus if row["poc"] == poc]
                if sum(row["w"] * row["h"] for row in frame_cus) != 480 * 320:
                    raise ValueError("Final CU log has missing or duplicated luma area")
                coverage=np.zeros((320,480),dtype=np.uint16)
                for row in frame_cus:coverage[row["y"]:row["y"]+row["h"],row["x"]:row["x"]+row["w"]]+=1
                if not np.all(coverage==1):raise ValueError("Winning luma CUs overlap or leave holes")
                if poc==native.states[0]["poc"]:
                    surface=np.zeros((320,480))
                    for row in frame_cus:surface[row["y"]:row["y"]+row["h"],row["x"]:row["x"]+row["w"]]=row["qp"]
                    first_surfaces[name]=surface
                per_poc[str(poc)] = dict(Counter(str(row["qp"]) for row in frame_cus))
            reports[name] = {"frames": frames, "au_bits": native.bits, "reconstruction_parity": True,
                             "decoded_sha256": byte_hash, "stream_sha256": digest(native.output / "stream.266"),
                             "decision_pocs": [row["poc"] for row in native.states], "final_qps": per_poc,
                             "mean_qps":{str(row["poc"]):row["mean_qp"] for row in native.posts}}
    if reports["anchor"]["stream_sha256"] != reports["zero"]["stream_sha256"]:
        raise ValueError("Zero delta-QP must be bit-exact with the matched native-QPA anchor")
    if reports["anchor"]["final_qps"] != reports["zero"]["final_qps"]:
        raise ValueError("Zero delta-QP must preserve final native QPA CUs")
    if reports["zero"]["stream_sha256"] == reports["pattern"]["stream_sha256"]:
        raise ValueError("The spatial action had no bitstream effect")
    if reports["zero"]["final_qps"] == reports["pattern"]["final_qps"]:
        raise ValueError("The spatial action had no final-CU QP effect")
    if not (first_surfaces["pattern"][:,:200].mean()<first_surfaces["zero"][:,:200].mean()
            and first_surfaces["pattern"][:,280:].mean()>first_surfaces["zero"][:,280:].mean()):
        raise ValueError("Spatial action changed the wrong region/direction")
    minus,plus=reports["probe_minus"]["mean_qps"],reports["probe_plus"]["mean_qps"]
    probe=str(reports["zero"]["decision_pocs"][-1])
    if plus[probe]<=minus[probe] or any(abs(plus[p]-minus[p])>1e-8 for p in plus if p!=probe):
        raise ValueError("Isolated POC action changed the wrong picture or had no effect")
    report = {"schema":"vvenc-rcdot-vvc-adapter/3","gate": "REAL_VVC_STATE_ACTION_AU_DECODE_PASS", "encoder_sha256": digest(encoder),
              "fixture": source.name, "runs": reports}
    atomic_json(output / "PREFLIGHT.json", report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--encoder", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--ffmpeg", default="ffmpeg")
    parser.add_argument("--frames", type=int, default=32)
    parser.add_argument("--decoder", type=Path, help="VVdeC binary, otherwise use FFmpeg's VVC decoder")
    parser.add_argument("--source-yuv", type=Path)
    parser.add_argument("--threads", type=int, default=2)
    args = parser.parse_args()
    result = run(args.encoder, args.output, args.ffmpeg, frames=args.frames, source=args.source_yuv,
                 threads=args.threads, decoder=args.decoder)
    print(result["gate"], args.output / "PREFLIGHT.json")


if __name__ == "__main__":
    main()
