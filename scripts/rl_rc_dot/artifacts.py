"""Generation-based checkpoints, RNG resume and Kaggle-safe verified exports."""
import base64
import hashlib
import json
import os
from pathlib import Path,PurePosixPath
import random
import re
import shutil
import stat
import tempfile
import uuid
import zipfile
import numpy as np
import torch
from .data import atomic_json,digest

RUN_SCHEMA="vvenc-rcdot-run/2"
EXPORT_SCHEMA="vvenc-rcdot-export/2"

def policy_digest(model):
    h=hashlib.sha256()
    for name,value in sorted(model.policy.state_dict().items()):
        a=value.detach().cpu().contiguous().numpy()
        h.update(name.encode());h.update(str(a.dtype).encode());h.update(str(a.shape).encode());h.update(a.tobytes())
    return h.hexdigest()

def capture_rng(env=None):
    nr=np.random.get_state()
    result={"python":random.getstate(),"numpy":[nr[0],nr[1].tolist(),nr[2],nr[3],nr[4]],
            "torch":torch.get_rng_state().tolist(),"env":env.env_method("rng_state") if env is not None else []}
    if torch.cuda.is_initialized():result["cuda"]=[s.tolist() for s in torch.cuda.get_rng_state_all()]
    return result

def restore_rng(state,env=None):
    def tuples(value):return tuple(map(tuples,value)) if isinstance(value,list) else value
    random.setstate(tuples(state["python"]))
    nr=state["numpy"];np.random.set_state((nr[0],np.asarray(nr[1],dtype=np.uint32),nr[2],nr[3],nr[4]))
    torch.set_rng_state(torch.tensor(state["torch"],dtype=torch.uint8))
    if "cuda" in state:
        if len(state["cuda"])!=torch.cuda.device_count():raise ValueError("CUDA RNG device count differs")
        torch.cuda.set_rng_state_all([torch.tensor(s,dtype=torch.uint8) for s in state["cuda"]])
    if env is not None:
        if len(state["env"])!=env.num_envs:raise ValueError("Collector count differs from RNG snapshot")
        for rank,rng in enumerate(state["env"]):env.env_method("restore_rng_state",rng,indices=rank)
        env._reset_seeds() # SB3.load seeds the VecEnv; do not overwrite restored samplers on reset.

def contained(root,relative):
    rel=PurePosixPath(relative)
    if rel.is_absolute() or not rel.parts or any(p in ("..","") or ":" in p for p in rel.parts) or "\\" in relative:
        raise ValueError("Unsafe artifact path")
    root=Path(root).resolve()
    p=(root/Path(*rel.parts)).resolve()
    if not p.is_relative_to(root):raise ValueError("Artifact escapes output directory")
    return p

def read_checkpoint(root,reference):
    directory=contained(root,reference["path"])
    manifest_path=directory/"manifest.json"
    if digest(manifest_path)!=reference["manifest_sha256"]:raise ValueError("Checkpoint manifest changed")
    m=json.loads(manifest_path.read_text())
    for name,sha in m["files"].items():
        if digest(contained(directory,name))!=sha:raise ValueError("Checkpoint evidence changed: "+name)
    if m["schema"]!=RUN_SCHEMA:raise ValueError("Unsupported checkpoint schema")
    return directory,m

def save_checkpoint(model,env,output,contract,*,best=None):
    output=Path(output)
    model.assert_checkpoint_finite()
    old=json.loads((output/"progress.json").read_text()) if (output/"progress.json").exists() else {}
    name=f"step{model.num_timesteps:012d}_update{model.completed_rollouts:08d}_{uuid.uuid4().hex[:8]}"
    folder=output/"checkpoints"/name
    folder.mkdir(parents=True,exist_ok=False)
    model.save(folder/"model.zip")
    with (folder/"model.zip").open("rb+") as f:os.fsync(f.fileno())
    atomic_json(folder/"rng.json",capture_rng(env))
    manifest={"schema":RUN_SCHEMA,"method_sha256":hashlib.sha256(json.dumps(contract,sort_keys=True).encode()).hexdigest(),
              "num_timesteps":model.num_timesteps,"completed_rollouts":model.completed_rollouts,
              "trained_frames":model.trained_frames,"policy_sha256":policy_digest(model),
              "files":{p.name:digest(p) for p in (folder/"model.zip",folder/"rng.json")}}
    atomic_json(folder/"manifest.json",manifest)
    ref={"path":folder.relative_to(output).as_posix(),"manifest_sha256":digest(folder/"manifest.json")}
    progress={"schema":RUN_SCHEMA,"latest":ref,"previous":old.get("latest"),
              "best":best if best is not None else old.get("best"),
              "num_timesteps":model.num_timesteps,"completed_rollouts":model.completed_rollouts,
              "trained_frames":model.trained_frames,"resume":"fresh codec episodes and rollout; restored model/optimizer/RNG"}
    # Publishing this pointer is the commit point. Interrupted generations are
    # never selected by resume and cannot invalidate the previous checkpoint.
    atomic_json(output/"progress.json",progress)
    keep={r["path"] for r in (progress["latest"],progress["previous"],progress["best"]) if r}
    checkpoint_root=(output/"checkpoints").resolve()
    for p in checkpoint_root.iterdir():
        if not p.is_dir() or not re.fullmatch(r"step\d{12}_update\d{8}_[0-9a-f]{8}",p.name):continue
        if p.relative_to(output.resolve()).as_posix() in keep:continue
        resolved=p.resolve()
        if p.is_symlink() or resolved.parent!=checkpoint_root:raise ValueError("Unsafe checkpoint cleanup target")
        shutil.rmtree(resolved)
    return ref

def load_resume(root,contract,env,device="cpu"):
    from .model import AuxPPO,AuxiliaryLabels
    root=Path(root)
    if json.loads((root/"method.json").read_text())!=contract:raise ValueError("Resume method/config/source differs")
    progress=json.loads((root/"progress.json").read_text())
    if progress.get("schema")!=RUN_SCHEMA:raise ValueError("Invalid progress schema")
    for key in ("latest","previous","best"):
        if progress.get(key):read_checkpoint(root,progress[key])
    folder,m=read_checkpoint(root,progress["latest"])
    if any(progress.get(k)!=m[k] for k in ("num_timesteps","completed_rollouts","trained_frames")):
        raise ValueError("Progress counters differ from committed checkpoint")
    expected=hashlib.sha256(json.dumps(contract,sort_keys=True).encode()).hexdigest()
    if m["method_sha256"]!=expected:raise ValueError("Checkpoint method binding differs")
    model=AuxPPO.load(folder/"model.zip",env=AuxiliaryLabels(env),device=device,force_reset=True)
    model.assert_checkpoint_finite()
    if policy_digest(model)!=m["policy_sha256"]:raise ValueError("Loaded policy differs from sealed checkpoint")
    if (model.num_timesteps,model.completed_rollouts,model.trained_frames)!=(m["num_timesteps"],m["completed_rollouts"],m["trained_frames"]):
        raise ValueError("Model counters differ from checkpoint manifest")
    restore_rng(json.loads((folder/"rng.json").read_text()),env)
    return model,progress

def export_run(output,destination,include_evidence=True):
    output=Path(output).resolve();destination=Path(destination).resolve()
    progress=json.loads((output/"progress.json").read_text())
    selected=set()
    for key in ("latest","previous","best"):
        ref=progress.get(key)
        if ref:
            directory,_=read_checkpoint(output,ref)
            selected.update(p for p in directory.rglob("*") if p.is_file())
    for name in ("method.json","progress.json","session.json","history.json","README_RUN.txt","failure.json","preflight/PREFLIGHT.json"):
        p=output/name
        if p.is_file():selected.add(p)
    for name in ("metrics","validations"):
        p=output/name
        if p.exists():selected.update(v for v in p.rglob("*") if v.is_file())
    if include_evidence:
        for name in ("episodes","validation_media"):
            p=output/name
            if p.exists():selected.update(v for v in p.rglob("*") if v.is_file() and v.suffix not in (".tmp",".yuv"))
    files={p.relative_to(output).as_posix():digest(p) for p in sorted(selected)}
    manifest={"schema":EXPORT_SCHEMA,"files":files,"source_schema":RUN_SCHEMA}
    destination.parent.mkdir(parents=True,exist_ok=True)
    staging=destination.with_suffix(destination.suffix+".tmp")
    with zipfile.ZipFile(staging,"w",compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        z.writestr("export_manifest.json",json.dumps(manifest,sort_keys=True))
        for rel in files:z.write(contained(output,rel),rel)
    staging.replace(destination)
    # Preserve the inner ZIP byte-for-byte when Kaggle auto-extracts Inputs.
    safe=destination.with_name(destination.stem+"_KAGGLE_SAFE.zip")
    with zipfile.ZipFile(safe.with_suffix(".tmp"),"w",compression=zipfile.ZIP_STORED) as z:
        z.write(destination,"session_payload.bin")
        z.writestr("payload_sha256.txt",digest(destination)+"\n")
    safe.with_suffix(".tmp").replace(safe)
    return {"export":str(destination),"sha256":digest(destination),"safe_input":str(safe),"files":len(files)}

def _extract_verified(archive,target):
    with zipfile.ZipFile(archive) as z:
        names=z.namelist()
        if len(names)!=len(set(names)):raise ValueError("Duplicate archive paths")
        for item in z.infolist():
            contained(target,item.filename)
            if stat.S_ISLNK(item.external_attr>>16):raise ValueError("Archive symlinks are forbidden")
        if "session_payload.bin" in names:
            with tempfile.NamedTemporaryFile(suffix=".zip",delete=False) as tmp:
                path=Path(tmp.name)
                h=hashlib.sha256()
                with z.open("session_payload.bin") as src:
                    for chunk in iter(lambda:src.read(1024*1024),b""):
                        h.update(chunk);tmp.write(chunk)
            try:
                if h.hexdigest()!=z.read("payload_sha256.txt").decode().strip():raise ValueError("Kaggle-safe payload hash differs")
                return _extract_verified(path,target)
            finally:path.unlink()
        manifest=json.loads(z.read("export_manifest.json"))
        if manifest.get("schema")!=EXPORT_SCHEMA:raise ValueError("Unsupported export schema")
        expected=set(manifest["files"])|{"export_manifest.json"}
        if set(names)!=expected:raise ValueError("Unexpected/missing export files")
        # Verify all bytes before making any files visible to the run.
        for name,sha in manifest["files"].items():
            if hashlib.sha256(z.read(name)).hexdigest()!=sha:raise ValueError("Corrupt export: "+name)
        for name in manifest["files"]:
            if name.startswith(("episodes/","validation_media/")):continue
            p=contained(target,name);p.parent.mkdir(parents=True,exist_ok=True);p.write_bytes(z.read(name))
        return manifest

def import_run(source,output):
    source,output=Path(source),Path(output)
    if output.exists() and any(output.iterdir()):raise FileExistsError("Resume destination must be empty")
    output.mkdir(parents=True,exist_ok=True)
    if source.is_dir():
        payload=source/"session_payload.bin"
        if payload.exists():
            if digest(payload)!=(source/"payload_sha256.txt").read_text().strip():raise ValueError("Safe payload hash differs")
            manifest=_extract_verified(payload,output)
        else:
            manifest=json.loads((source/"export_manifest.json").read_text())
            if manifest.get("schema")!=EXPORT_SCHEMA:raise ValueError("Unsupported extracted export")
            for rel,sha in manifest["files"].items():
                if digest(contained(source,rel))!=sha:raise ValueError("Extracted input changed: "+rel)
            for rel in manifest["files"]:
                if rel.startswith(("episodes/","validation_media/")):continue
                dst=contained(output,rel);dst.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(contained(source,rel),dst)
    else:manifest=_extract_verified(source,output)
    # Evidence from past sessions remains in the user's source export. Keep
    # a sealed index instead of copying cumulative media into every session.
    history={"parent_export_sha256":digest(source) if source.is_file() else None,"parent_files":manifest["files"]}
    atomic_json(output/"history.json",history)
    return history

def export_evaluation(output,destination):
    """Seal completed and partial evaluation evidence for verified pair resume."""
    output,destination=Path(output).resolve(),Path(destination).resolve()
    selected=[p for p in output.rglob("*") if p.is_file() and "exports" not in p.relative_to(output).parts
              and p.suffix not in (".tmp",".yuv") and not p.name.endswith(".decoded.yuv")]
    files={p.relative_to(output).as_posix():digest(p) for p in sorted(selected)}
    manifest={"schema":EXPORT_SCHEMA,"source_schema":"vvenc-rcdot-evaluation/2","files":files}
    destination.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(destination.with_suffix(".tmp"),"w",zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        z.writestr("export_manifest.json",json.dumps(manifest,sort_keys=True))
        for rel in files:z.write(contained(output,rel),rel)
    destination.with_suffix(".tmp").replace(destination)
    safe=destination.with_name(destination.stem+"_KAGGLE_SAFE.zip")
    with zipfile.ZipFile(safe.with_suffix(".tmp"),"w",zipfile.ZIP_STORED) as z:
        z.write(destination,"session_payload.bin");z.writestr("payload_sha256.txt",digest(destination)+"\n")
    safe.with_suffix(".tmp").replace(safe)
    return {"export":str(destination),"safe_input":str(safe),"files":len(files),"sha256":digest(destination)}
