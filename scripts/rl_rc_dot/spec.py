"""16+4 RL-RC-DoT state protocol, 480x320, 16x16 local cells.
DOT_PROXY observations are rejected by default: they contain feature proxies.
"""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
import math
import os

GLOBAL_DIM=16
MAP_CHANNELS=4
MAP_HEIGHT=20
MAP_WIDTH=30
MAP_ELEMENTS=600
ACTION_HEIGHT=5
ACTION_WIDTH=7
DELTA_LIMIT=12
STATE_FIELDS=2+GLOBAL_DIM+MAP_CHANNELS*MAP_ELEMENTS

@dataclass(frozen=True)
class Observation:
    poc: int
    globals: tuple[float,...]
    maps: tuple[tuple[float,...],...]
    proxy: bool

def parse_state(line: str, *, allow_proxy: bool=False)->Observation:
    words=line.split()
    if len(words)!=STATE_FIELDS: raise ValueError(f"Expected {STATE_FIELDS} state fields, got {len(words)}")
    if words[0] not in ("DOT_STATE","DOT_PROXY"): raise ValueError("Unrecognized state format")
    proxy=words[0]=="DOT_PROXY"
    if proxy and not allow_proxy: raise ValueError("DOT_PROXY is not a paper-faithful observation")
    poc=int(words[1])
    if poc<0: raise ValueError("Negative POC")
    values=tuple(float(w) for w in words[2:])
    if not all(math.isfinite(x) for x in values):raise ValueError("Nonfinite feature")
    globals_=values[:16]
    maps=tuple(values[16+i*600:16+(i+1)*600] for i in range(4))
    return Observation(poc,globals_,maps,proxy)

def write_action(directory:Path,poc:int,deltas:list[int])->Path:
    if poc<0 or len(deltas)!=600 or any(type(x) is not int or abs(x)>12 for x in deltas):
        raise ValueError("Action requires 600 integer deltas in [-12,12] and valid POC")
    directory.mkdir(parents=True,exist_ok=True)
    path=directory/f"{poc}.txt"
    if path.exists():raise FileExistsError(path)
    temp=directory/f".{poc}.{os.getpid()}.tmp"
    try:
        with temp.open("w",encoding="ascii") as f:
            f.write(str(poc)+" "+" ".join(map(str,deltas))+"\n")
            f.flush()
            os.fsync(f.fileno())
        os.replace(temp,path)
    finally:
        temp.unlink(missing_ok=True)
    return path
