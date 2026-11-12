"""VVC state mapping and confidence-ordered Precision reward.

The paper does not release normalization/matching code. These choices are
versioned definitions shared by training, validation and Test.
"""
import math
import numpy as np

W,H,FPS=480,320,30
GRID=(20,30)
FRAME_BYTES=W*H*3//2
SCHEMA="vvenc-rcdot-vvc-adapter/3"
TRAIN_REWARD_SCALE=0.05  # Uniform positive scale; preserves published reward ordering.

def observation(event,frames):
    if event.get("schema")!=SCHEMA or frames<1:raise ValueError("Unknown native schema or invalid frame count")
    g=np.asarray(event["global"],dtype=np.float64)
    maps=np.asarray(event["maps"],dtype=np.float64)
    if g.shape!=(16,) or maps.shape!=(4,600) or not np.isfinite(g).all() or not np.isfinite(maps).all():
        raise ValueError("Expected 16 finite globals and four finite 600-cell maps")
    if not event["poc"]<=event["lookahead_last_poc"]<=min(frames-1,event["poc"]+10):
        raise ValueError("Pre-analysis exceeded the ten-frame lookahead")
    if event["history_poc"]!=-1 and not np.isclose(g[5:8].sum(),1,atol=1e-8):
        raise ValueError("Completed history has no valid mode fractions")
    g[[0,3,4]]/=63
    g[1]/=frames
    g[2]=np.log1p(max(0.,g[2])/100000)
    g[8]/=60
    g[13]=np.tanh(g[13])
    g[14]/=2
    g[15]=np.tanh(g[15]/1500)
    maps[:3]=np.log1p(np.maximum(maps[:3],0))
    maps=(maps-maps.mean(axis=1,keepdims=True))/(maps.std(axis=1,keepdims=True)+1e-6)
    result={"global":g.astype(np.float32),"mb":maps.reshape(4,*GRID).astype(np.float32)}
    if not all(np.isfinite(v).all() for v in result.values()):raise ValueError("State normalization overflow")
    return result

def action_map(action):
    values=np.asarray(action,dtype=np.float64)
    if values.shape!=(35,) or not np.isfinite(values).all():raise ValueError("Expected 35 finite Gaussian actions")
    coarse=12*np.tanh(np.clip(values,-10,10).reshape(5,7))
    ys=np.clip((np.arange(20)+.5)*5/20-.5,0,4)
    xs=np.clip((np.arange(30)+.5)*7/30-.5,0,6)
    across=np.stack([np.interp(xs,np.arange(7),row) for row in coarse])
    return np.stack([np.interp(ys,np.arange(5),column) for column in across.T],axis=1)

def wire_map(delta):
    """Canonical numeric packet sent to the encoder (9 significant digits)."""
    values=np.asarray(delta,dtype=np.float64)
    if values.shape!=(20,30) or not np.isfinite(values).all() or np.abs(values).max()>12:
        raise ValueError("Invalid wire action map")
    return np.asarray([float(f"{v:.9g}") for v in values.ravel()]).reshape(20,30)

def boxes_checked(boxes):
    boxes=np.asarray(boxes,dtype=np.float64)
    if boxes.size==0:return np.empty((0,5),dtype=np.float64)
    if boxes.ndim!=2 or boxes.shape[1]!=5 or not np.isfinite(boxes).all():raise ValueError("Expected finite [x1,y1,x2,y2,confidence]")
    if ((boxes[:,2:4]<=boxes[:,:2]).any() or (boxes[:,:2]<0).any()
            or (boxes[:,[0,2]]>W).any() or (boxes[:,[1,3]]>H).any()
            or (boxes[:,4]<0).any() or (boxes[:,4]>1).any()):raise ValueError("Invalid detection coordinates/confidence")
    return boxes

def detection_targets(raw,reconstructed):
    raw,pred=boxes_checked(raw),boxes_checked(reconstructed)
    available=set(range(len(raw)))
    correct=np.zeros(len(pred),dtype=bool)
    for i in np.argsort(-pred[:,4],kind="stable"):
        best,maximum=None,-1.
        for j in sorted(available):
            lo=np.maximum(pred[i,:2],raw[j,:2]);hi=np.minimum(pred[i,2:4],raw[j,2:4])
            overlap=float(np.prod(np.maximum(0,hi-lo)))
            union=float(np.prod(pred[i,2:4]-pred[i,:2])+np.prod(raw[j,2:4]-raw[j,:2])-overlap)
            iou=overlap/union if union>0 else 0.
            if iou>maximum:best,maximum=j,iou
        if best is not None and maximum>=.5:correct[i]=True;available.remove(best)
    labels=np.zeros(GRID,dtype=np.float32);counts=np.zeros(GRID,dtype=np.int32)
    for box,matched in zip(pred,correct):
        x=min(29,max(0,int(((box[0]+box[2])/2)//16)))
        y=min(19,max(0,int(((box[1]+box[3])/2)//16)))
        labels[y,x]+=matched;counts[y,x]+=1
    np.divide(labels,counts,out=labels,where=counts>0)
    tp=int(correct.sum());fp=len(pred)-tp;fn=len(raw)-tp
    return labels,{"precision":tp/len(pred) if len(pred) else 0.,"recall":tp/len(raw) if len(raw) else 0.,"tp":tp,"fp":fp,"fn":fn}

def auxiliary_supervision_mask(raw,reconstructed):
    """Supervise source/reconstruction object cells, not irrelevant background."""
    mask=np.zeros(GRID,dtype=np.float32)
    for boxes in (boxes_checked(raw),boxes_checked(reconstructed)):
        for box in boxes:
            x=min(29,max(0,int(((box[0]+box[2])/2)//16)))
            y=min(19,max(0,int(((box[1]+box[3])/2)//16)))
            mask[y,x]=1.0
    return mask


def training_reward(paper_reward):
    """Scale the PPO reward while reporting/validating in original paper units."""
    if not math.isfinite(paper_reward):raise ValueError("Nonfinite paper reward")
    return TRAIN_REWARD_SCALE*paper_reward


def score(raw,reconstructed,cumulative_bits,decisions,target_bps):
    if cumulative_bits<=0 or decisions<1 or not math.isfinite(target_bps) or target_bps<=0:raise ValueError("Invalid rate reward inputs")
    block,metrics=detection_targets(raw,reconstructed)
    actual=cumulative_bits*FPS/decisions
    reward=20*metrics["precision"]-abs(math.log(actual/target_bps))
    if not math.isfinite(reward):raise ValueError("Nonfinite task reward")
    return reward,block,{**metrics,"actual_bps":actual}
