"""One PPO transition per encoded picture with the native frame boundary."""

import json
import gzip
import hashlib
from pathlib import Path
import uuid
import shutil

import gymnasium as gym
import numpy as np

from .codec import NativeEpisode
from .method import FRAME_BYTES, action_map, observation, score, auxiliary_supervision_mask, training_reward
from .model import ACTIONS, OBSERVATIONS


class VVenCEnv(gym.Env):
    observation_space = OBSERVATIONS
    action_space = ACTIONS

    def __init__(self, catalog, detector, encoder, output, *, split="train", frame_limit=0, threads=2,
                 fixed_video=None, fixed_rate=None, mode="live", retain_media=False):
        self.catalog, self.detector = catalog, detector
        self.encoder, self.output = Path(encoder), Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        if split not in ("train", "val", "test"):
            raise ValueError("Unknown dataset split")
        self.ids = catalog.lock["splits"][split]
        if fixed_video is not None and fixed_video not in self.ids:
            raise ValueError("Requested video is outside the selected split")
        self.split, self.frame_limit, self.threads = split, frame_limit, threads
        self.fixed_video, self.fixed_rate, self.mode = fixed_video, fixed_rate, mode
        self.retain_media = retain_media
        self.native = None
        self.source = None
        self.trajectory = None
        self.episodes_started=0

    def rng_state(self):
        return {"generator":self.np_random.bit_generator.state,"episodes_started":self.episodes_started}

    def seed_sampler(self,seed):
        self._np_random=np.random.default_rng(seed)

    def suspend_episode(self):
        self._close_episode()

    def restore_rng_state(self,state):
        self.np_random.bit_generator.state=state["generator"]
        self.episodes_started=state["episodes_started"]

    def _close_episode(self):
        completed = self.native is not None and self.native.current is None
        source_path = Path(self.source.filename) if self.source is not None else None
        codec = self.native.output if self.native else None
        if self.native:
            self.native.close()
            self.native = None
        if self.source is not None:
            self.source._mmap.close()
            self.source = None
        if self.trajectory:
            self.trajectory.close()
            self.trajectory = None
        if codec and (codec/"events.jsonl").is_file():
            with (codec/"events.jsonl").open("rb") as src,gzip.open(codec/"events.jsonl.gz","wb") as dst:
                shutil.copyfileobj(src,dst)
            (codec/"events.jsonl").unlink()
        if source_path and not self.retain_media:
            source_path.unlink(missing_ok=True)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self._close_episode()
        self.video_id = self.fixed_video or self.ids[int(self.np_random.integers(len(self.ids)))]
        self.rate = self.fixed_rate or int(self.np_random.integers(50, 201))
        self.episodes_started+=1
        folder = self.output / f"{self.video_id}_{self.rate}_{uuid.uuid4().hex[:8]}"
        folder.mkdir()
        path = folder / "source.yuv"
        self.frames = self.catalog.prepare(self.video_id, path, self.frame_limit)
        self.source = np.memmap(path, mode="r", dtype=np.uint8, shape=(self.frames, FRAME_BYTES))
        self.trajectory = gzip.open(folder / "trajectory.jsonl.gz", "wt", encoding="utf-8")
        self.native = NativeEpisode(self.encoder, path, folder / "codec", self.frames, self.rate,
                                    mode=self.mode, threads=self.threads)
        self.decisions = 0
        self.rewards = []
        self.precisions, self.recalls = [], []
        self.counts = np.zeros(3, dtype=np.int64)
        return observation(self.native.current, self.frames), {"video_id": self.video_id, "rate_kbps": self.rate}

    def step(self, action):
        if self.native is None or self.native.current is None:
            raise RuntimeError("Call reset before taking an action")
        current = self.native.current
        post, next_state = self.native.step(action_map(action))
        poc = post["poc"]
        raw = self.detector(self.source[poc])
        reconstructed_bytes = (self.native.recon / f"{poc}.yuv").read_bytes()
        reconstructed = np.frombuffer(reconstructed_bytes, dtype="<u2")
        reconstructed = np.minimum(255, (reconstructed.astype(np.uint32) + 2) >> 2).astype(np.uint8)
        prediction = self.detector(reconstructed)
        self.decisions += 1
        reward, block, metrics = score(raw, prediction, self.native.bits, self.decisions, self.rate * 1000)
        mask = auxiliary_supervision_mask(raw, prediction)
        info = {"block_reward": block, "block_mask": mask, "paper_reward": reward,
                "video_id": self.video_id, "poc": poc, "rate_kbps": self.rate,
                "consumed_action":np.asarray(post.get("consumed_action",np.zeros(600)),dtype=float).ravel().tolist(),**metrics}
        self.rewards.append(reward)
        self.precisions.append(metrics["precision"])
        self.recalls.append(metrics["recall"])
        self.counts += [metrics["tp"], metrics["fp"], metrics["fn"]]
        row = {"poc": poc, "step": self.decisions, "au_bits": post["au_bits"], "reward": reward,
               "precision": metrics["precision"], "recall": metrics["recall"],
               "tp": metrics["tp"], "fp": metrics["fp"], "fn": metrics["fn"],
               "raw_boxes": raw.tolist(), "reconstructed_boxes": prediction.tolist(),
               "reconstruction_sha256": hashlib.sha256(reconstructed_bytes).hexdigest(),
               "gaussian_action": np.asarray(action).tolist(), "history_poc": current["history_poc"],
               "state_sha256":hashlib.sha256(json.dumps(current,sort_keys=True,allow_nan=False).encode()).hexdigest(),
               "consumed_action":info["consumed_action"],"post":post}
        self.trajectory.write(json.dumps(row, allow_nan=False) + "\n")
        self.trajectory.flush()
        if not self.retain_media:
            (self.native.recon / f"{poc}.yuv").unlink()
            action_file = self.native.actions / f"{poc}.txt"
            if action_file.exists():
                action_file.unlink()
        terminated = next_state is None
        if terminated:
            tp, fp, fn = map(int, self.counts)
            info["episode"] = {"r": float(sum(self.rewards)), "l": self.decisions,
                               "mean_precision": float(np.mean(self.precisions)),
                               "mean_recall": float(np.mean(self.recalls)),
                               "mean_reward": float(np.mean(self.rewards)),
                               "tp": tp, "fp": fp, "fn": fn,
                               "f1": 2 * tp / (2 * tp + fp + fn) if 2 * tp + fp + fn else 0.,
                               "psnr_y": float(np.mean([row["psnr"] for row in self.native.posts])),
                               "mean_qp": float(np.mean([row["mean_qp"] for row in self.native.posts])),
                               "actual_kbps": self.native.bits * 30 / self.frames / 1000}
            obs = {name: np.zeros(space.shape, dtype=np.float32) for name, space in OBSERVATIONS.spaces.items()}
        else:
            obs = observation(next_state, self.frames)
        return obs, training_reward(reward), terminated, False, info

    def close(self):
        self._close_episode()
        self.detector.close()
