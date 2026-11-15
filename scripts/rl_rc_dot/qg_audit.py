"""Audit candidate QG requests, separately from final inferred no-residual QPs."""
import math
import numpy as np


def pooled_delta(action, row):
    action=np.asarray(action,dtype=np.float64)
    if action.shape!=(20,30) or not np.isfinite(action).all() or np.abs(action).max()>12:
        raise ValueError("Invalid QG action map")
    left,top=int(row['x']),int(row['y'])
    right,bottom=min(480,left+int(row['w'])),min(320,top+int(row['h']))
    if not 0<=left<right<=480 or not 0<=top<bottom<=320:
        raise ValueError("Invalid QG geometry")
    total=0.
    for y in range(top//16,(bottom-1)//16+1):
        for x in range(left//16,(right-1)//16+1):
            area=(min(right,(x+1)*16)-max(left,x*16))*(min(bottom,(y+1)*16)-max(top,y*16))
            total+=area*action[y,x]
    value=total/((right-left)*(bottom-top))
    # std::lround rounds half away from zero; numpy.round does not.
    return math.floor(value+.5) if value>=0 else math.ceil(value-.5)


def verify_qg_requests(rows, actions):
    """Check actual hook QP and lambda, including inter ancestor contamination.

    Layer-zero pictures can receive an additional native pumping correction.
    Other inter layers must retain the original CTU QPA base at every depth.
    Final-CU QPs are intentionally not equated with requests: zero-residual
    QGs can legally infer a different QP in the decoder.
    """
    if not rows:raise ValueError("Missing native QG evidence")
    depths=set();inter_children=0;zero_children=0
    for row in rows:
        action=actions[row['poc']]
        expected=max(-12,min(63,row['native_qp']+pooled_delta(action,row)))
        if row['requested_qp']!=expected:
            raise ValueError("QG request differs from one pooled delta")
        if not row['intra'] and row['temporal_layer']>0:
            if row['native_qp']!=row['ctu_qp']:
                raise ValueError("Inter QG inherited an already adjusted native base")
            if row['subdiv']>0:
                inter_children+=1
                zero_children+=int(row['requested_qp']==row['native_qp'] and row['inherited_qp']!=row['native_qp'])
        expected_lambda=row['ctu_lambda']*2.**((row['requested_qp']-row['ctu_qp'])/3.)
        if not math.isfinite(row['rd_lambda']) or not math.isclose(row['rd_lambda'],expected_lambda,rel_tol=1e-11):
            raise ValueError("QG lambda differs from requested QP")
        depths.add(row['subdiv'])
    return {'candidate_qgs':len(rows),'subdivisions':sorted(depths),
            'inter_child_qgs':inter_children,'zero_child_restorations':zero_children}
