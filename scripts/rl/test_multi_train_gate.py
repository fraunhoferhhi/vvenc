"""CPU-only static verification of bounded Train gate indexing and checkpoint ZIP."""
import json
from pathlib import Path
import tempfile
import unittest
import zipfile

from run_multi_train_gate import parse_train_indices, checkpoint_bundle, sha256, atomic_json


class TestMultiTrainGate(unittest.TestCase):
    def test_train_indices_must_be_unique_and_train_only(self):
        self.assertEqual(parse_train_indices("1,2,3"), [1, 2, 3])
        for bad in ("", "1,1", "-1", "65", "1,2,3,4", "2,foo"):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    parse_train_indices(bad)

    def test_checkpoint_bundle_checks_sha_and_summary(self):
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            ckpt = folder / "ckpt_iter_002.pt"
            ckpt.write_bytes(b"test checkpoint bytes")
            episode = folder / "episode_01_train_01"
            episode.mkdir()
            (episode / "summary.json").write_text(json.dumps({
                "video_id": "train-only", "split": "train",
            }))
            manifest = {
                "checkpoint_files": [
                    {"name": ckpt.name, "sha256": sha256(ckpt), "iteration": 2}
                ],
                "completed": [
                    {"episode_dir": episode.name, "video_id": "train-only"}
                ],
            }
            atomic_json(folder / "progress.json", manifest)
            archive = checkpoint_bundle(folder, manifest)
            with zipfile.ZipFile(archive) as z:
                self.assertEqual(
                    sorted(z.namelist()),
                    sorted(["progress.json", ckpt.name,
                            "episode_01_train_01/summary.json"])
                )
                self.assertEqual(z.read(ckpt.name), ckpt.read_bytes())
            ckpt.write_bytes(b"tampered")
            with self.assertRaisesRegex(RuntimeError, "checksum changed"):
                checkpoint_bundle(folder, manifest)


if __name__ == "__main__":
    unittest.main()
