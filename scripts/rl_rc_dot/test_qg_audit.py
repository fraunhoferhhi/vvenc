import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from scripts.rl_rc_dot.qg_audit import pooled_delta,verify_qg_requests
from scripts.rl_rc_dot.codec import NativeEpisode
from scripts.rl_rc_dot.preflight import make_fixture


class QgAudit(unittest.TestCase):
    def row(self):
        return dict(poc=6,intra=False,temporal_layer=3,subdiv=2,x=0,y=0,w=16,h=16,
                    inherited_qp=46,native_qp=45,requested_qp=46,ctu_qp=45,
                    ctu_lambda=100.,rd_lambda=100.*2**(1/3))

    def test_rejects_compounded_parent_delta(self):
        row=self.row();row.update(native_qp=46,requested_qp=47,rd_lambda=100.*2**(2/3))
        with self.assertRaisesRegex(ValueError,'already adjusted'):
            verify_qg_requests([row],{6:np.ones((20,30))})

    def test_zero_child_restores_native_qp_and_lambda(self):
        row=self.row();row.update(requested_qp=45,rd_lambda=100.)
        self.assertEqual(verify_qg_requests([row],{6:np.zeros((20,30))})['zero_child_restorations'],1)
        row['rd_lambda']*=2**(1/3)
        with self.assertRaisesRegex(ValueError,'lambda'):verify_qg_requests([row],{6:np.zeros((20,30))})

    def test_pooling_rounding_edges_and_legal_clipping(self):
        row=self.row()
        self.assertEqual(pooled_delta(np.full((20,30),.5),row),1)
        self.assertEqual(pooled_delta(np.full((20,30),-.5),row),-1)
        row.update(x=464,y=304,w=32,h=32)
        self.assertEqual(pooled_delta(np.ones((20,30)),row),1)
        row.update(native_qp=63,ctu_qp=63,requested_qp=63,rd_lambda=100.)
        verify_qg_requests([row],{6:np.full((20,30),12.)})
        row.update(native_qp=-12,ctu_qp=-12,requested_qp=-12)
        verify_qg_requests([row],{6:np.full((20,30),-12.)})


@unittest.skipUnless(os.environ.get('RCDOT_TEST_ENCODER'),'Set RCDOT_TEST_ENCODER for native QG regressions')
class NativeQgAudit(unittest.TestCase):
    def test_inter_and_intra_qgs_apply_one_delta_and_restore_zero_children(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);source=root/'fixture.yuv';make_fixture(source,8)
            streams={};checks={}
            for name in ('anchor','zero','tiny','plus','minus','cancel'):
                actions={};mode='anchor' if name=='anchor' else 'live'
                with NativeEpisode(os.environ['RCDOT_TEST_ENCODER'],source,root/name,8,120,mode=mode,threads=2,trace_qgs=True) as native:
                    while native.current is not None:
                        poc=native.current['poc'];action=np.zeros((20,30))
                        if name=='tiny':action.fill(.49)
                        elif name=='plus':action.fill(1.)
                        elif name=='minus':action.fill(-1.)
                        elif name=='cancel':action[::2,::2]=4.
                        actions[poc]=action
                        native.step(action if mode=='live' else None)
                streams[name]=(root/name/'stream.266').read_bytes()
                rows=[json.loads(l) for l in (root/name/'qgs.jsonl').read_text().splitlines()]
                if name in ('plus','minus','cancel'):
                    checks[name]=verify_qg_requests(rows,actions)
                    self.assertGreater(checks[name]['inter_child_qgs'],0)
                    self.assertTrue(any(r['intra'] for r in rows))
                else:self.assertEqual(rows,[])
            self.assertEqual(streams['anchor'],streams['zero'])
            self.assertEqual(streams['anchor'],streams['tiny'])
            self.assertNotEqual(streams['plus'],streams['minus'])
            self.assertGreater(checks['cancel']['zero_child_restorations'],0)


if __name__=='__main__':unittest.main()
