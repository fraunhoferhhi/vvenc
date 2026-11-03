"""Strict on-policy checkpoint/update integration gate unit tests."""
import json
import tempfile
from pathlib import Path
import unittest

import torch

from ppo_agent import PPOActorCritic
from verify_real_ppo_update import (
    read_real_rollout, validate_unchanged_collecting_policy,
    one_true_rollout_update, sha256,
)


class TestRealPPOUpdate(unittest.TestCase):
    def make_episode(self, root: Path, n: int = 32,
                     source_checkpoint: Path | None = None):
        torch.manual_seed(7)
        model = PPOActorCritic().eval()
        if source_checkpoint is not None:
            checkpoint_state = torch.load(source_checkpoint, map_location="cpu", weights_only=True)
            model.load_state_dict(checkpoint_state["model_state"], strict=True)
        episode = root / "real_episode"
        episode.mkdir(parents=True)
        joined = []
        for i in range(n):
            gl = torch.rand(1, 8)
            sp = torch.rand(1, 3, 3, 4)
            with torch.no_grad():
                raw, action, lp, val = model.sample(sp, gl)
            joined.append({
                "video_id": "train_only", "poc": i,
                "policy_decision_step": i + 1, "au_output_step": i + 1,
                "global": gl[0].tolist(), "spatial": sp[0].tolist(),
                "raw_action": raw[0].tolist(),
                "discrete_action": action[0].tolist(),
                "log_prob": lp.item(), "value": val.item(),
                "reward": float((i % 5) / 5),
                "next_value": 0.0, "done": i == n - 1,
                "reward_diagnostics": {
                    "video_id": "train_only", "poc": i,
                    "encoded_step": i + 1,
                },
            })
        for i in range(n - 1):
            joined[i]["next_value"] = joined[i + 1]["value"]
        (episode / "joined_reward_ppo.jsonl").write_text(
            "".join(json.dumps(x) + "\n" for x in joined),
            encoding="utf-8",
        )
        (episode / "summary.json").write_text(json.dumps({
            "split": "train", "video_id": "train_only",
            "validation_status": (
                "UPDATED_POLICY_REWARD_DATA_COLLECTION" if source_checkpoint
                else "UNTRAINED_POLICY_REWARD_DATA_COLLECTION"
            ),
            "policy_checkpoint_sha256": sha256(source_checkpoint) if source_checkpoint else None,
            "ppo_policy_training": False, "target_kbps": 120,
            "au_sum_matches_stream": True,
            "frames": n, "baseline_lock_sha256": "unit_test_lock",
        }), encoding="utf-8")
        return episode

    def test_true_collector_parity_and_checkpoint_reload(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            episode = self.make_episode(root)
            checkpoint = root / "checkpoint.pt"
            result = one_true_rollout_update(episode, checkpoint, seed=23)
            self.assertGreater(result["optimizer_steps"], 0)
            self.assertTrue(result["actor_changed"])
            self.assertTrue(result["critic_changed"])
            self.assertTrue(checkpoint.is_file())
            state = torch.load(checkpoint, map_location="cpu", weights_only=True)
            self.assertEqual(state["video_id"], "train_only")
            self.assertEqual(state["schema"], "vvenc-ppo-single-real-rollout-gate/1")
            # A repeated update on the same rollout/checkpoint must be refused.
            with self.assertRaises(FileExistsError):
                one_true_rollout_update(episode, checkpoint, seed=23)

    def test_second_fresh_rollout_resumes_same_actor_and_adam_state(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            first = self.make_episode(root / "episode1")
            ck1 = root / "first.pt"
            one_true_rollout_update(first, ck1, seed=23)
            new_episode = self.make_episode(
                root / "episode2", source_checkpoint=ck1
            )
            ck2 = root / "second.pt"
            with self.assertRaisesRegex(ValueError, "--checkpoint-in is REQUIRED"):
                one_true_rollout_update(new_episode, ck2, seed=24)
            result = one_true_rollout_update(
                new_episode, ck2, seed=24, checkpoint_in=ck1
            )
            self.assertTrue(result["actor_changed"])
            self.assertTrue(result["critic_changed"])
            state2 = torch.load(ck2, map_location="cpu", weights_only=True)
            self.assertEqual(state2["ppo_iteration"], 2)
            self.assertEqual(state2["collector_checkpoint_sha256"], sha256(ck1))
            self.assertNotEqual(sha256(ck1), sha256(ck2))
            self.assertNotEqual(
                state2["rollout_sha256"],
                torch.load(ck1, map_location="cpu", weights_only=True)["rollout_sha256"],
            )

    def test_tampered_action_or_logprob_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            episode = self.make_episode(root)
            _, rows, _ = read_real_rollout(episode)
            torch.manual_seed(7)
            model = PPOActorCritic()
            validate_unchanged_collecting_policy(model, rows)
            broken = [dict(row) for row in rows]
            broken[3]["log_prob"] += 0.5
            with self.assertRaisesRegex(ValueError, "Collector policy mismatch"):
                validate_unchanged_collecting_policy(model, broken)
            broken = [dict(row) for row in rows]
            broken[3]["discrete_action"] = [2] * 12
            with self.assertRaisesRegex(ValueError, "Rounded latent Gaussian"):
                validate_unchanged_collecting_policy(model, broken)


if __name__ == "__main__":
    unittest.main()
