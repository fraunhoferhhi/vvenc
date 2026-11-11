import tempfile
import unittest
from pathlib import Path
from spec import parse_state,write_action,STATE_FIELDS
from reward import bitrate_term,paper_reward
class Contracts(unittest.TestCase):
    def state(self,prefix="DOT_PROXY"):
        return prefix+" 3 "+" ".join(["0.1"]*(STATE_FIELDS-2))
    def test_strict_proxy(self):
        with self.assertRaises(ValueError):parse_state(self.state())
    def test_shapes(self):
        s=parse_state(self.state(),allow_proxy=True)
        self.assertEqual(len(s.globals),16)
        self.assertEqual(list(map(len,s.maps)),[600]*4)
        self.assertFalse(parse_state(self.state("DOT_STATE")).proxy)
    def test_invalid_input(self):
        with self.assertRaises(ValueError):parse_state("DOT_STATE 1 0")
        with self.assertRaises(ValueError):parse_state(self.state().replace("0.1","nan",1),allow_proxy=True)
    def test_atomic_action(self):
        with tempfile.TemporaryDirectory() as d:
            p=write_action(Path(d),3,[2]*600)
            self.assertEqual(len(p.read_text().split()),601)
            with self.assertRaises(FileExistsError):write_action(Path(d),3,[2]*600)
    def test_reward(self):
        self.assertEqual(bitrate_term(120000,30,30,120000),0.)
        self.assertAlmostEqual(paper_reward(.8,120000,30,30,120000),16.)
        self.assertEqual(paper_reward(1.,120000,30,30,120000,has_reference=False),0.)
if __name__=="__main__":unittest.main()
