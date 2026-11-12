"""A synchronous frame boundary over the native VVenC event/action bridge."""

import json
import os
from pathlib import Path
import subprocess
import time
import atexit

import numpy as np

from .method import FRAME_BYTES,wire_map


def command(encoder, source, stream, frames, rate_kbps, threads=2):
    return [str(encoder), "-i", str(source), "-s", "480x320", "-r", "30", "-f", str(frames),
            "-c", "yuv420", "--preset", "medium", "--internal-bitdepth", "10", "--bitrate", f"{rate_kbps}k",
            "--passes", "1", "--qpa", "1", "--intraperiod", "32", "--refreshtype", "cra",
            "--sdr", "sdr_709", "--threads", str(threads), "--mtprofile", "0",
            "--additional", "GOPSize=32:LookAhead=1:CTUSize=32:MaxCuDQPSubdiv=2:MaxParallelFrames=0",
            "-o", str(stream)]


class NativeEpisode:
    def __init__(self, encoder, source, output, frames, rate_kbps, *, mode="live", threads=2, timeout=300, trace_cus=False):
        self.output = Path(output).resolve()
        self.output.mkdir(parents=True, exist_ok=False)
        self.actions = self.output / "actions"
        self.recon = self.output / "recon"
        self.actions.mkdir()
        self.recon.mkdir()
        self.frames = frames
        self.timeout = timeout
        self.mode = mode
        self.states = []
        self.posts = []
        self.bits = 0
        self.reader = None
        self.process = None
        self.log = None
        self.current = None
        if Path(source).stat().st_size < frames * FRAME_BYTES:
            raise ValueError("Source does not contain the requested frames")
        env = {key: value for key, value in os.environ.items()
               if not key.startswith(("VVENC_RL_", "VVENC_RCDOT_", "VVENC_DOT_"))}
        env.update(VVENC_RCDOT_MODE=mode, VVENC_RCDOT_STATE_FILE=str(self.output / "events.jsonl"),
                   VVENC_RCDOT_RECON_DIR=str(self.recon), VVENC_RCDOT_ACTION_DIR=str(self.actions))
        if trace_cus:
            env["VVENC_RCDOT_CU_FILE"] = str(self.output / "cus.jsonl")
        self.log = (self.output / "encoder.log").open("wb")
        args = command(Path(encoder).resolve(), Path(source).resolve(), self.output / "stream.266",
                       frames, rate_kbps, threads)
        (self.output / "command.json").write_text(json.dumps(args, indent=2), encoding="utf-8")
        self.process = subprocess.Popen(args, env=env, stdout=self.log, stderr=subprocess.STDOUT)
        atexit.register(self.close)
        try:
            self.current = self._state(self._event())
        except BaseException:
            self.close()
            raise

    def _event(self):
        deadline = time.monotonic() + self.timeout
        path = self.output / "events.jsonl"
        while time.monotonic() < deadline:
            if self.reader is None and path.exists():
                self.reader = path.open(encoding="utf-8")
            if self.reader:
                start = self.reader.tell()
                line = self.reader.readline()
                if line.endswith("\n"):
                    return json.loads(line)
                self.reader.seek(start)
            if self.process.poll() is not None:
                tail = (self.output / "encoder.log").read_text(errors="replace")[-3000:]
                raise RuntimeError(f"Encoder ended before the expected event\n{tail}")
            time.sleep(.005)
        raise TimeoutError(f"No complete encoder event within {self.timeout}s at {path}")

    def _state(self, row):
        if (row.get("schema") != "vvenc-rcdot-vvc-adapter/3" or row.get("kind") != "state" or row.get("step") != len(self.states)
                or type(row.get("poc")) is not int or not 0 <= row["poc"] < self.frames
                or row["poc"] in {r["poc"] for r in self.states}):
            raise ValueError("Repeated, missing or out-of-order decision record")
        previous = self.posts[-1]["poc"] if self.posts else -1
        if row["history_poc"] != previous:
            raise ValueError("State history was not produced before the current decision")
        self.states.append(row)
        return row

    def step(self, delta_map=None):
        if self.current is None:
            raise RuntimeError("Episode has already completed")
        poc = self.current["poc"]
        if self.mode == "live":
            values = wire_map(delta_map)
            if values.shape != (20, 30) or not np.isfinite(values).all() or np.max(np.abs(values)) > 12.00001:
                raise ValueError("Expected a finite 20x30 delta-QP map in [-12,12]")
            target = self.actions / f"{poc}.txt"
            staging = target.with_suffix(".tmp")
            staging.write_text(str(poc) + " " + " ".join(f"{v:.9g}" for v in values.ravel()) + "\n", encoding="ascii")
            staging.replace(target)
        post = self._event()
        if (post.get("kind") != "post" or post.get("poc") != poc
                or post.get("output_step") != len(self.posts) + 1
                or type(post.get("au_bytes")) is not int or post["au_bytes"] <= 0
                or post.get("au_bits") != 8 * post["au_bytes"]):
            raise ValueError("Output AU was not associated with the pending action")
        if (self.recon / f"{poc}.yuv").stat().st_size != FRAME_BYTES * 2:
            raise ValueError("Incomplete native reconstruction")
        fractions=np.asarray(post.get("mode_fractions"),dtype=float)
        if (post.get("area")!=480*320 or fractions.shape!=(3,) or not np.isfinite(fractions).all()
                or (fractions<0).any() or abs(fractions.sum()-1)>1e-8):
            raise ValueError("Final CU mode coverage is missing or invalid")
        if self.mode == "live":
            consumed=np.asarray(post.get("consumed_action"),dtype=float)
            if consumed.shape!=(600,) or not np.array_equal(values.ravel(),consumed):
                raise ValueError("Encoder consumed a different action from the policy request")
        if post.get("cumulative_bits")!=self.bits+post["au_bits"]:
            raise ValueError("Cumulative output bits disagree with the action history")
        if not np.isfinite([post.get("psnr"),post.get("ssim"),post.get("mean_qp")]).all():
            raise ValueError("Nonfinite final-picture statistics")
        self.posts.append(post)
        self.bits += post["au_bits"]
        self.current = None
        if len(self.posts) < self.frames:
            self.current = self._state(self._event())
        else:
            self.process.wait(timeout=self.timeout)
            if self.process.returncode:
                raise RuntimeError("Encoder failed while flushing")
            if self.bits != (self.output / "stream.266").stat().st_size * 8:
                raise ValueError("AU bit accounting differs from the byte size of the bitstream")
            if {row["poc"] for row in self.posts} != set(range(self.frames)):
                raise ValueError("Missing coded POCs")
        return post, self.current

    def close(self):
        atexit.unregister(self.close)
        if self.process and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        if self.reader:
            self.reader.close()
        if self.log:
            self.log.close()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
