"""Archived replay must fail closed even though production uses SB3."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import json
import tempfile
import unittest
from unittest.mock import patch
from scripts.rl_rc_dot.train import main

class LegacyGuards(unittest.TestCase):
    def test_nan_and_action_mismatch_rejected_before_files_or_optimizer(self):
        for name,oldp,action,message in [("nan",float("nan"),0,"Nonfinite"),("action",0.,1,"action mismatch")]:
            with self.subTest(name=name),tempfile.TemporaryDirectory() as tmp:
                root=Path(tmp);trajectory=root/"trajectory.jsonl"
                rows=[{"poc":poc,"policy_step":poc+1,"proxy":False,"global":[0.]*16,"local":[[0.]*600]*4,
                       "raw_action":[0.]*35,"discrete_action":[action]*600,"old_logp":oldp,"old_value":0.} for poc in range(2)]
                trajectory.write_text("".join(json.dumps(row)+"\n" for row in rows))
                argv=["train.py","--trajectory",str(trajectory),"--au-csv",str(root/"au.csv"),
                      "--bitstream",str(root/"stream.266"),"--source-detections",str(root/"source.jsonl"),
                      "--decoded-detections",str(root/"decoded.jsonl"),"--output",str(root/"bad.pt"),
                      "--video-id","fixture","--target-bps","120000"]
                with patch.object(sys,"argv",argv),self.assertRaisesRegex(ValueError,message):main()
                self.assertFalse((root/"bad.pt").exists())

if __name__=="__main__":unittest.main()
