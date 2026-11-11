"""Task-aware precision + average-bitrate reward; no fabricated labels."""
import math
def bitrate_term(cumulative_bits:int,steps:int,fps:float,target_bps:float)->float:
    if cumulative_bits<0 or steps<1 or fps<=0 or target_bps<=0:
        raise ValueError("Invalid bitrate inputs")
    actual=max(1.,cumulative_bits*fps/steps)
    return -abs(math.log(actual/target_bps))
def paper_reward(precision:float,cumulative_bits:int,steps:int,fps:float,
                 target_bps:float,task_weight:float=20.,has_reference:bool=True)->float:
    if not math.isfinite(precision) or not 0<=precision<=1 or task_weight<0:
        raise ValueError("Invalid precision/weight")
    return bitrate_term(cumulative_bits,steps,fps,target_bps)+task_weight*(precision if has_reference else 0.)
