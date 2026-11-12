"""Check executable notebook configuration against the Mini runner contract."""
import ast
import contextlib
import io
import json
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[2]

class MiniNotebooks(unittest.TestCase):
    def test_all_training_phases_respect_mini_limits(self):
        for name in ('RL_RC_DoT_VVC_Mini_Pilot_Kaggle.ipynb','RL_RC_DoT_VVC_Mini_Trial20480_Kaggle.ipynb'):
            notebook=json.loads((ROOT/'notebooks'/name).read_text(encoding='utf8'))
            source=next(''.join(c['source']) for c in notebook['cells'] if c['cell_type']=='code')
            source=source.split('\nimport torch')[0]
            for phase in ('pilot','trial','full'):
                tree=ast.parse(source)
                for node in tree.body:
                    if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='PHASE' for t in node.targets):
                        node.value=ast.Constant(phase)
                ast.fix_missing_locations(tree)
                with contextlib.redirect_stdout(io.StringIO()):
                    ns={};exec(compile(tree,name,'exec'),ns)
                self.assertEqual(ns['DATASET_MODE'],'mini')
                self.assertEqual(ns['TRAIN_FRAME_LIMIT'],256)
                self.assertLessEqual(ns['VAL_VIDEOS'],4)
                self.assertIn(ns['VAL_FRAMES'],(64,128,256))

    def test_evaluation_notebook_uses_five_rates_and_total_session_clock(self):
        n=json.loads((ROOT/'notebooks/RL_RC_DoT_VVC_Mini_Test30_Kaggle.ipynb').read_text(encoding='utf8'))
        code='\n'.join(''.join(c['source']) for c in n['cells'] if c['cell_type']=='code')
        self.assertIn("TEST_RATES='50,80,120,160,200'",code)
        self.assertIn("'--rates',TEST_RATES",code)
        self.assertIn("'--session-start-utc',str(MINI_EVAL_START_UTC)",code)
        for c in n['cells']:
            if c['cell_type']=='code':ast.parse(''.join(c['source']))

if __name__=='__main__':unittest.main()
