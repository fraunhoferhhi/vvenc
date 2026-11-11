"""Train/validate/resume the declared H.266 adaptation, using SB3 PPO.

Examples and the exact mapping are in README_RL_RC_DOT.md. Test videos are
never accessed by this runner. A session export is generated in finally.
"""
import argparse
from functools import partial
import importlib.metadata
import json
import os
from pathlib import Path
import time
import uuid
import numpy as np
import torch
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import SubprocVecEnv
from .artifacts import (RUN_SCHEMA,export_run,import_run,load_resume,
                        policy_digest,save_checkpoint)
from .data import ASSETS,FrozenDetector,VideoCatalog,atomic_json,digest,source_fingerprint
from .environment import VVenCEnv
from .method import SCHEMA
from .model import build

class BudgetStop(Exception):
    pass

def append_json(path,row):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("a",encoding="utf-8") as f:
        f.write(json.dumps(row,allow_nan=False)+"\n")
        f.flush();os.fsync(f.fileno())

def make_env(spec,rank):
    device=f"cuda:{spec['gpu_ids'][rank%len(spec['gpu_ids'])]}" if spec["gpu_ids"] else "cpu"
    catalog=VideoCatalog(spec["input_root"],spec["ffmpeg"],spec["ffprobe"])
    detector=FrozenDetector(spec["yolo_repo"],spec["weights"],device)
    print("COLLECTOR_READY",json.dumps({"rank":rank,"pid":os.getpid(),
          "cuda_visible_devices":os.environ.get("CUDA_VISIBLE_DEVICES"),
          "detector_device":str(detector.device)}),flush=True)
    return VVenCEnv(catalog,detector,spec["encoder"],Path(spec["output"])/"episodes"/f"worker{rank}",
                    frame_limit=spec["frames"],threads=spec["encoder_threads"])

def validate(model,spec,deadline=float("inf"),detector_factory=FrozenDetector,catalog_factory=VideoCatalog):
    catalog=catalog_factory(spec["input_root"],spec["ffmpeg"],spec["ffprobe"])
    device=f"cuda:{spec['gpu_ids'][0]}" if spec["gpu_ids"] else "cpu"
    detector=detector_factory(spec["yolo_repo"],spec["weights"],device)
    points=[]
    try:
        for video in catalog.lock["splits"]["val"][:spec["validation_videos"]]:
            for rate in spec["validation_rates"]:
                pair={}
                for name,mode in (("anchor","anchor"),("agent","live")):
                    if time.monotonic()>=deadline:raise BudgetStop("Validation interrupted by session budget")
                    env=VVenCEnv(catalog,detector,spec["encoder"],Path(spec["output"])/"validation_media"/str(model.completed_rollouts),
                                split="val",frame_limit=spec["validation_frames"],threads=spec["encoder_threads"],
                                fixed_video=video,fixed_rate=rate,mode=mode)
                    try:
                        obs,_=env.reset(seed=spec["seed"])
                        while True:
                            if time.monotonic()>=deadline:raise BudgetStop("Validation interrupted by session budget")
                            action=np.zeros(35) if mode=="anchor" else model.predict(obs,deterministic=True)[0]
                            obs,_,done,_,info=env.step(action)
                            if done:break
                        pair[name]=info["episode"]
                    finally:env._close_episode()
                points.append({"video_id":video,"rate_kbps":rate,**pair})
    finally:detector.close()
    return {"score":float(np.mean([p["agent"]["mean_reward"] for p in points])),
            "anchor_score":float(np.mean([p["anchor"]["mean_reward"] for p in points])),
            "macro_delta_precision":float(np.mean([p["agent"]["mean_precision"]-p["anchor"]["mean_precision"] for p in points])),
            "points":points,"policy_sha256":policy_digest(model),"completed_rollouts":model.completed_rollouts,
            "num_timesteps":model.num_timesteps,"selection":"macro mean of per-frame training reward on Validation only"}

class SaveAndValidate(BaseCallback):
    def __init__(self,spec,contract,deadline=float("inf"),validator=validate):
        super().__init__()
        self.spec,self.contract,self.deadline=spec,contract,deadline
        self.output=Path(spec["output"]);self.validator=validator;self.saved_update=-1
        p=self.output/"progress.json"
        self.best=json.loads(p.read_text()).get("best") if p.exists() else None
        existing=list((self.output/"validations").glob("*.json")) if (self.output/"validations").exists() else []
        self.last_validation=max((json.loads(p.read_text())["completed_rollouts"] for p in existing),default=0)
        self.last_heartbeat=0
    def save(self):
        ref=save_checkpoint(self.model,self.training_env,self.output,self.contract,best=self.best)
        self.saved_update=self.model.completed_rollouts
        return ref
    def synchronize(self,force_validation=False):
        update=self.model.completed_rollouts
        if update!=self.saved_update:
            ref=self.save()
            if update:
                append_json(self.output/"metrics/updates.jsonl",self.model.last_update_metrics)
                print("PPO_UPDATE_PASS",json.dumps(self.model.last_update_metrics),flush=True)
        else:ref=json.loads((self.output/"progress.json").read_text())["latest"]
        if time.monotonic()>=self.deadline:raise BudgetStop("Soft session budget reached")
        every=self.spec["val_every_updates"]
        due=update>self.last_validation and (force_validation or (every and update%every==0))
        if due:
            # A VVenC process awaiting the next action must not be left paused
            # for a long validation run. Partial training episodes end here;
            # the next rollout starts fresh with the updated policy.
            self.training_env.env_method("suspend_episode")
            result=self.validator(self.model,self.spec,self.deadline)
            name=f"update{update:08d}_{uuid.uuid4().hex[:8]}.json"
            atomic_json(self.output/"validations"/name,result)
            if self.best is None or result["score"]>self.best["score"]:
                self.best={**ref,"score":result["score"],"validation":f"validations/{name}"}
            self.last_validation=update
            self.save()
            print("VALIDATION_PASS",json.dumps({k:v for k,v in result.items() if k!="points"}),flush=True)
            if time.monotonic()<self.deadline:
                self.model._last_obs=self.training_env.reset()
                self.model._last_episode_starts=np.ones(self.training_env.num_envs,dtype=bool)
        if time.monotonic()>=self.deadline:raise BudgetStop("Soft session budget reached")
    def _on_rollout_start(self):
        self.synchronize()
    def _on_step(self):
        if time.monotonic()-self.last_heartbeat>=15:
            print("TRAIN_HEARTBEAT",json.dumps({"num_timesteps":self.num_timesteps,
                  "completed_updates":self.model.completed_rollouts,
                  "seconds_to_soft_stop":max(0,self.deadline-time.monotonic()) if np.isfinite(self.deadline) else None}),flush=True)
            self.last_heartbeat=time.monotonic()
        for info in self.locals.get("infos",[]):
            if "episode" in info:
                append_json(self.output/"metrics/episodes.jsonl",
                    {"num_timesteps":self.num_timesteps,"video_id":info["video_id"],"rate_kbps":info["rate_kbps"],**info["episode"]})
        return time.monotonic()<self.deadline
    def _on_training_end(self):
        self.synchronize(force_validation=True)

def ppo_options(args):
    """Validate and seal policy-update hyperparameters before constructing SB3."""
    if (not np.isfinite(args.ppo_learning_rate) or args.ppo_learning_rate<=0
            or args.ppo_epochs<1 or not np.isfinite(args.ppo_target_kl)
            or args.ppo_target_kl<=0 or not np.isfinite(args.ppo_aux_loss_coef)
            or not 0<=args.ppo_aux_loss_coef<=1):
        raise ValueError("PPO requires finite positive learning rate/target KL, aux coefficient [0,1] and at least one optimization epoch")
    return {"learning_rate":float(args.ppo_learning_rate),
            "n_epochs":int(args.ppo_epochs),"target_kl":float(args.ppo_target_kl),
            "aux_loss_coef":float(args.ppo_aux_loss_coef)}

def contract_for(spec):
    return {"schema":RUN_SCHEMA,"native_schema":SCHEMA,"source_fingerprint":source_fingerprint(),
            "dataset_sha256":digest(ASSETS/"dataset_lock.json"),"baseline_lock_sha256":digest(ASSETS/"baseline_lock.json"),
            "encoder_sha256":digest(spec["encoder"]),"detector_weights_sha256":digest(spec["weights"]),
            "profile":"480x320_30fps_medium_ctu32_dqp2_fpp0_10bit_lookahead10",
            "frames":spec["frames"],"seed":spec["seed"],"workers":spec["workers"],"rollout_steps":spec["rollout_steps"],
            "encoder_threads":spec["encoder_threads"],"algorithm":"SB3 2.7.0 PPO; isolated actor/critic; per-branch gradient clip; bounded auxiliary reward",
            "ppo":spec["ppo"],
            "reward":"20*confidence_ordered_precision-abs(log(prefix_actual_bps/target_bps))",
            "rate_sampling":"uniform integer kbps [50,200]","action":"Gaussian35;clip10;tanh*12;bilinear20x30;native QG pooling",
            "validation":{"videos":spec["validation_videos"],"rates":spec["validation_rates"],"frames":spec["validation_frames"]}}

def parser():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ("input-root","encoder","yolo-repo","weights","output"):p.add_argument("--"+name,type=Path,required=True)
    p.add_argument("--decoder",type=Path,required=True,help="Pinned VVdeC binary for preflight")
    p.add_argument("--mode",choices=("new","resume"),default="new")
    p.add_argument("--resume-from",type=Path)
    p.add_argument("--total-timesteps",type=int,default=20_000_000)
    p.add_argument("--additional-timesteps",type=int,default=0,help="Optional session interaction budget, capped by total-timesteps")
    p.add_argument("--rollout-steps",type=int,default=2048)
    p.add_argument("--ppo-learning-rate",type=float,default=3e-5,help="Conservative post-smoke PPO optimizer learning rate")
    p.add_argument("--ppo-epochs",type=int,default=3,help="Optimizer passes per PPO rollout")
    p.add_argument("--ppo-target-kl",type=float,default=0.05,help="Stop PPO epoch early above 1.5 times this approximate KL")
    p.add_argument("--ppo-aux-loss-coef",type=float,default=0.1,help="Bounded block-reward MSE weight; set 0 for a controlled ablation")
    p.add_argument("--frames",type=int,default=0,help="0 = entire source clip; a limit is an explicitly reduced experiment")
    p.add_argument("--validation-frames",type=int,default=0)
    p.add_argument("--validation-videos",type=int,default=7)
    p.add_argument("--validation-rates",default="50,80,120,160,200")
    p.add_argument("--val-every-updates",type=int,default=25)
    p.add_argument("--workers",type=int,default=2)
    p.add_argument("--gpu-ids",default="0,1")
    p.add_argument("--ppo-device",default="cpu")
    p.add_argument("--encoder-threads",type=int,default=2)
    p.add_argument("--seed",type=int,default=20261111)
    p.add_argument("--max-hours",type=float,default=11.75)
    p.add_argument("--session-start-utc",type=float,help="time.time() at notebook start so setup counts in the time budget")
    p.add_argument("--reserve-minutes",type=float,default=10)
    p.add_argument("--ffmpeg",default="ffmpeg")
    p.add_argument("--ffprobe",default="ffprobe")
    return p

def main():
    start=time.monotonic();wall=time.time()
    p=parser();a=p.parse_args()
    if (a.total_timesteps<1 or a.additional_timesteps<0 or a.rollout_steps<2 or a.workers<1 or a.frames<0 or a.validation_frames<0
            or not 1<=a.validation_videos<=7 or a.encoder_threads<1 or a.val_every_updates<0
            or a.max_hours<0 or a.reserve_minutes<0):p.error("Invalid training budget or configuration")
    if importlib.metadata.version("stable-baselines3")!="2.7.0":raise RuntimeError("Install stable-baselines3==2.7.0")
    torch.set_num_threads(2)
    try:ppo=ppo_options(a)
    except ValueError as error:p.error(str(error))
    spec={k:str(v.resolve()) if isinstance(v,Path) else v for k,v in vars(a).items()}
    spec["ppo"]=ppo
    spec["gpu_ids"]=[v.strip() for v in a.gpu_ids.split(",") if v.strip()]
    spec["validation_rates"]=[int(v) for v in a.validation_rates.split(",")]
    if not spec["validation_rates"] or len(set(spec["validation_rates"]))!=len(spec["validation_rates"]) or any(v not in (50,80,120,160,200) for v in spec["validation_rates"]):
        p.error("Validation rates must be a unique subset of 50,80,120,160,200")
    if spec["gpu_ids"] and (len(set(spec["gpu_ids"]))!=len(spec["gpu_ids"]) or any(not v.isdigit() or int(v)>=torch.cuda.device_count() for v in spec["gpu_ids"])):
        p.error("Requested collector GPUs are unavailable")
    if spec["gpu_ids"] and len(spec["gpu_ids"])<a.workers:p.error("Provide one GPU per collector")
    output=Path(spec["output"])
    if a.resume_from:
        if a.mode!="resume":p.error("--resume-from requires --mode resume")
        import_run(a.resume_from,output)
    output.mkdir(parents=True,exist_ok=True)
    contract=contract_for(spec)
    config=output/"method.json"
    if a.mode=="new" and any(output.iterdir()):raise FileExistsError("New run output is nonempty; use resume")
    if a.mode=="resume" and (not config.exists() or json.loads(config.read_text())!=contract):
        raise ValueError("Resume source/profile/data/training/validation contract differs")
    atomic_json(config,contract)
    atomic_json(output/"session.json",{**spec,"runtime":{"torch":torch.__version__,"sb3":importlib.metadata.version("stable-baselines3")}})
    catalog=VideoCatalog(spec["input_root"],spec["ffmpeg"],spec["ffprobe"])
    catalog.check_available(("train","val"))
    from .pixels import verify_conversion
    print("RGB_PROTOCOL_PASS",json.dumps(verify_conversion(a.ffmpeg)),flush=True)
    seconds=a.max_hours*3600
    elapsed=max(0,wall-a.session_start_utc) if a.session_start_utc else 0
    deadline=start+seconds-elapsed-min(a.reserve_minutes*60,seconds*.1) if seconds else float("inf")
    # Preflight is based on encoder bytes, not a hard-coded binary from a
    # previous Kaggle compiler. Source/policy/dataset remain pinned in method.
    from .preflight import run as native_preflight
    certificate=output/"preflight/PREFLIGHT.json"
    if not certificate.exists() or json.loads(certificate.read_text()).get("encoder_sha256")!=digest(a.encoder):
        preflight_dir=output/("preflight_"+uuid.uuid4().hex[:8])
        result=native_preflight(a.encoder,preflight_dir,a.ffmpeg,frames=32,threads=a.encoder_threads,decoder=a.decoder)
        atomic_json(certificate,result)
    env=None;model=None;callback=None;failed=None
    try:
        if time.monotonic()>=deadline:raise BudgetStop("Setup consumed the session budget")
        env=SubprocVecEnv([partial(make_env,spec,rank) for rank in range(a.workers)],start_method="spawn")
        if a.mode=="resume":model,_=load_resume(output,contract,env,a.ppo_device)
        else:
            model=build(env,a.seed,a.ppo_device,n_steps=a.rollout_steps,verbose=1,**spec["ppo"])
            for rank in range(a.workers):env.env_method("seed_sampler",a.seed+rank,indices=rank)
            env._reset_seeds()
        callback=SaveAndValidate(spec,contract,deadline)
        callback.init_callback(model)
        callback.save() # durable initial/loaded policy before any codec rollout
        remaining=a.total_timesteps-model.num_timesteps
        if a.additional_timesteps:remaining=min(remaining,a.additional_timesteps)
        if remaining>0:model.learn(remaining,reset_num_timesteps=False,callback=callback)
        else:
            print("TRAINING_BUDGET_ALREADY_REACHED",model.num_timesteps,flush=True)
            callback.synchronize(force_validation=True)
    except BudgetStop as error:
        print("SESSION_SOFT_STOP",str(error),flush=True)
        if model is not None and env is not None and callback is not None:callback.save()
    except BaseException as error:
        failed=error
        atomic_json(output/"failure.json",{"type":type(error).__name__,"message":str(error),
                                          "checkpoint_policy":"last committed finite checkpoint only"})
    finally:
        if env is not None:
            try:env.close()
            except (EOFError,BrokenPipeError):
                for child in env.processes:
                    if child.is_alive():child.terminate()
        if (output/"progress.json").exists():
            destination=output/"exports"/f"RL_RC_DOT_VVC_SESSION_{time.time_ns()}.zip"
            print("SESSION_EXPORT",json.dumps(export_run(output,destination)),flush=True)
    if failed is not None:raise failed

if __name__=="__main__":main()
