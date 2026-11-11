"""Build the pinned VVenC/VVdeC and frozen YOLO toolchain without replacing CUDA Torch."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import urllib.request
from .data import ASSETS,atomic_json,digest

VVDEC_COMMIT="649f0b2fafee977c998d7e4d674f8b88d952e3a3"

def run(command,root,name,timeout=2400,cwd=None):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    path=root/(name+".log")
    print("SETUP_START",name," ".join(map(str,command)),flush=True)
    with path.open("w") as log:
        process=subprocess.Popen(list(map(str,command)),stdout=log,stderr=subprocess.STDOUT,cwd=cwd)
        start=time.monotonic()
        try:
            while process.poll() is None:
                if time.monotonic()-start>timeout:raise TimeoutError(f"{name} exceeded {timeout}s")
                time.sleep(1)
                if int(time.monotonic()-start)%15==0:print("SETUP_RUNNING",name,int(time.monotonic()-start),"seconds",flush=True)
        finally:
            if process.poll() is None:process.terminate();process.wait()
    if process.returncode:raise RuntimeError(name+" failed:\n"+path.read_text(errors="replace")[-4000:])
    print("SETUP_PASS",name,flush=True)

def pinned_clone(url,path,commit,logs):
    path=Path(path)
    fresh=not (path/".git").exists()
    if fresh:run(["git","clone","--no-checkout","--filter=blob:none",url,path],logs,path.name+"_clone",900)
    actual_url=subprocess.check_output(["git","-C",str(path),"remote","get-url","origin"],text=True).strip()
    if actual_url.removesuffix(".git")!=url.removesuffix(".git"):raise ValueError("Unexpected repository origin")
    if not fresh and subprocess.check_output(["git","-C",str(path),"status","--porcelain","--untracked-files=no"],text=True).strip():
        raise ValueError("Tracked toolchain source modified: "+str(path))
    run(["git","-C",path,"fetch","--no-tags","--depth","1","origin",commit],logs,path.name+"_fetch",900)
    run(["git","-C",path,"checkout","--detach",commit],logs,path.name+"_pin",120)
    if subprocess.check_output(["git","-C",str(path),"rev-parse","HEAD"],text=True).strip()!=commit:raise ValueError("Toolchain pin failed")

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--repo",type=Path,required=True)
    p.add_argument("--source-commit",required=True)
    p.add_argument("--output",type=Path,default=Path("/kaggle/working/rl_rc_dot_tools"))
    p.add_argument("--jobs",type=int,default=2)
    p.add_argument("--install-system",action="store_true")
    a=p.parse_args()
    if a.jobs<1:p.error("jobs must be positive")
    repo=a.repo.resolve();a.output.mkdir(parents=True,exist_ok=True);logs=a.output/"logs"
    if subprocess.check_output(["git","-C",str(repo),"rev-parse","HEAD"],text=True).strip()!=a.source_commit:
        raise ValueError("Source HEAD differs from notebook commit pin")
    if subprocess.check_output(["git","-C",str(repo),"status","--porcelain","--untracked-files=no"],text=True).strip():
        raise ValueError("Tracked source modified before setup")
    if a.install_system:
        if os.name!="posix":raise ValueError("--install-system is intended for Kaggle Linux")
        run(["apt-get","update","-qq"],logs,"apt_update",600)
        run(["apt-get","install","-y","-qq","build-essential","cmake","ninja-build","git","ffmpeg","libswscale-dev","libavutil-dev"],logs,"apt_install",900)
    run([sys.executable,"-m","pip","install","--disable-pip-version-check","-r",repo/"scripts/rl_rc_dot/requirements.txt"],logs,"python_dependencies",900)
    run(["cmake","-S",repo,"-B",repo/"build","-DCMAKE_BUILD_TYPE=Release","-DVVENC_ENABLE_THIRDPARTY_JSON=ON","-DVVENC_ENABLE_INSTALL=OFF"],logs,"vvenc_configure",300)
    run(["cmake","--build",repo/"build","--target","vvencapp","--parallel",a.jobs],logs,"vvenc_build",2400)
    dec=a.output/"vvdec"
    pinned_clone("https://github.com/fraunhoferhhi/vvdec.git",dec,VVDEC_COMMIT,logs)
    run(["cmake","-S",dec,"-B",dec/"build","-DCMAKE_BUILD_TYPE=Release"],logs,"vvdec_configure",300)
    run(["cmake","--build",dec/"build","--target","vvdecapp","--parallel",a.jobs],logs,"vvdec_build",2400)
    lock=json.loads((ASSETS/"baseline_lock.json").read_text())["config"]["detector"]
    yolo=a.output/"yolov5"
    pinned_clone("https://github.com/ultralytics/yolov5.git",yolo,lock["commit"],logs)
    weights=a.output/"yolov5n_v7_locked.pt"
    if not weights.exists():
        temporary=weights.with_suffix(".tmp")
        urllib.request.urlretrieve("https://github.com/ultralytics/yolov5/releases/download/v7.0/yolov5n.pt",temporary)
        if digest(temporary)!=lock["weights_sha256"]:raise ValueError("Downloaded YOLO weight hash differs")
        temporary.replace(weights)
    if digest(weights)!=lock["weights_sha256"]:raise ValueError("Frozen YOLO weights changed")
    result={"source_commit":a.source_commit,"encoder":str(repo/"bin/release-static/vvencapp"),
            "decoder":str(dec/"bin/release-static/vvdecapp"),"yolo_repo":str(yolo),"weights":str(weights)}
    for name in ("encoder","decoder"):result[name+"_sha256"]=digest(result[name])
    atomic_json(a.output/"TOOLCHAIN.json",result)
    print("TOOLCHAIN_READY",json.dumps(result),flush=True)

if __name__=="__main__":main()
