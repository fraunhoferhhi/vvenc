"""Immutable BDD100K mini-selection and dataset lock regression tests."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import json
import tempfile
import unittest
from types import SimpleNamespace

from scripts.rl_rc_dot.data import ASSETS, DATASET_SHA, MINI_COUNTS, VideoCatalog, digest, select_locked_ids


class MiniDataset(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock=json.loads((ASSETS/"dataset_lock.json").read_text(encoding="utf-8"))

    def test_unchanged_lock_and_counts_are_stable(self):
        self.assertEqual(digest(ASSETS/"dataset_lock.json"),DATASET_SHA)
        first={key:select_locked_ids(self.lock,key,"mini",20261111) for key in MINI_COUNTS}
        second={key:select_locked_ids(self.lock,key,"mini",20261111) for key in MINI_COUNTS}
        self.assertEqual(first,second)
        self.assertEqual({key:len(v) for key,v in first.items()},MINI_COUNTS)
        self.assertEqual(len(set().union(*(set(v) for v in first.values()))),50)
        for split,ids in first.items():
            self.assertTrue(set(ids)<=set(self.lock["splits"][split]))
            self.assertEqual(select_locked_ids(self.lock,split,"full"),self.lock["splits"][split])
        third={key:select_locked_ids(self.lock,key,"mini",7) for key in MINI_COUNTS}
        self.assertNotEqual(first,third)

    def test_selection_spans_locked_car_box_quartiles(self):
        records={row["video_id"]:row for row in self.lock["video_records"]}
        for split,count in MINI_COUNTS.items():
            originals=self.lock["splits"][split]
            ranked=sorted(originals,key=lambda vid:(records[vid]["car_boxes"],vid))
            chosen=set(select_locked_ids(self.lock,split,"mini"))
            counts=[sum(vid in chosen for vid in ranked[len(ranked)*i//4:len(ranked)*(i+1)//4])
                    for i in range(4)]
            self.assertEqual(counts,[count//4+int(i<count%4) for i in range(4)])

    def test_missing_check_uses_selected_ids_only(self):
        with tempfile.TemporaryDirectory() as root:
            catalog=VideoCatalog(root,mode="mini")
            self.assertEqual(len(catalog.selected["train"]),16)
            self.assertEqual(len(catalog.selected["val"]),4)
            with self.assertRaisesRegex(FileNotFoundError,"Missing locked videos") as error:
                catalog.check_available(("train","val"))
            missing=str(error.exception)
            for vid in catalog.selected["train"]+catalog.selected["val"]:
                self.assertIn(vid,missing)
            # Simulate verified stat sizes without creating massive MOV files.
            for split in ("train","val"):
                for vid in catalog.selected[split]:
                    expected=catalog.records[vid]["size_bytes"]
                    catalog.files[vid]=[SimpleNamespace(stat=lambda size=expected:SimpleNamespace(st_size=size))]
            catalog.check_available(("train","val"))
            with self.assertRaises(FileNotFoundError):
                catalog.check_available(("test",))

    def test_invalid_split_and_mode_rejected(self):
        for mode,split in (("training","train"),("mini","other")):
            with self.assertRaises(ValueError):
                select_locked_ids(self.lock,split,mode)


if __name__=="__main__":
    unittest.main()
