"""Locked BDD100K source discovery and baseline-matched preprocessing."""

import hashlib
import importlib
import sys
from contextlib import contextmanager
import json
import os
from pathlib import Path
import subprocess

import numpy as np

from .method import FRAME_BYTES

ASSETS = Path(__file__).with_name("assets")
DATASET_SHA="f7a600f501de622f66ace04bb0b12ec5da0dab988cd54732d614f6b16495aaea"


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def atomic_json(path, content):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    staging = path.with_name(path.name + ".tmp")
    with staging.open("w",encoding="utf-8") as f:
        f.write(json.dumps(content,indent=2,allow_nan=False)+"\n")
        f.flush();os.fsync(f.fileno())
    staging.replace(path)


def source_fingerprint():
    root = Path(__file__).resolve().parents[2]
    value = hashlib.sha256()
    value.update((root/"CMakeLists.txt").read_bytes())
    for directory in ("source", "include", "scripts/rl_rc_dot", "cmake"):
        for path in sorted((root / directory).rglob("*")):
            if directory=="scripts/rl_rc_dot" and (path.name.startswith("test_") or path.name in ("evaluate.py","setup_kaggle.py","preflight.py")):
                continue # QA/report-only changes do not invalidate trained weights.
            if path.is_file() and (path.suffix in (".h", ".cpp", ".py", ".json", ".cmake", ".in") or path.name=="CMakeLists.txt"):
                value.update(path.relative_to(root).as_posix().encode())
                value.update(path.read_bytes())
    return value.hexdigest()


class VideoCatalog:
    def __init__(self, input_root, ffmpeg="ffmpeg", ffprobe="ffprobe"):
        if digest(ASSETS/"dataset_lock.json")!=DATASET_SHA:raise ValueError("Bundled split lock changed")
        self.lock = json.loads((ASSETS / "dataset_lock.json").read_text(encoding="utf-8"))
        self.records = {row["video_id"]: row for row in self.lock["video_records"]}
        self.files = {}
        self.verified = set()
        for path in Path(input_root).rglob("*.mov"):
            if path.stem in self.records:
                self.files.setdefault(path.stem, []).append(path)
        self.ffmpeg, self.ffprobe = ffmpeg, ffprobe

    def check_available(self,splits=("train","val")):
        missing=[vid for split in splits for vid in self.lock["splits"][split]
                 if not any(p.stat().st_size==self.records[vid]["size_bytes"] for p in self.files.get(vid,[]))]
        if missing:raise FileNotFoundError("Missing locked videos: "+", ".join(missing))

    def source(self, video_id):
        row = self.records[video_id]
        for path in self.files.get(video_id, []):
            if path in self.verified:
                return path
            if path.stat().st_size == row["size_bytes"] and digest(path) == row["sha256"]:
                self.verified.add(path)
                return path
        raise FileNotFoundError(f"Locked MOV absent or hash mismatch for {video_id}")

    def prepare(self, video_id, output, frame_limit=0):
        source = self.source(video_id)
        metadata = subprocess.check_output([self.ffprobe, "-v", "error", "-select_streams", "v:0",
                    "-show_entries", "stream=color_range,color_space,color_transfer,color_primaries", "-of", "json", str(source)])
        stream = json.loads(metadata)["streams"][0]
        matrix = stream.get("color_space", "bt709")
        primaries = stream.get("color_primaries", "bt709")
        transfer = stream.get("color_transfer", "bt709")
        range_ = stream.get("color_range", "tv")
        values = [matrix, primaries, transfer, range_]
        fallback = ["bt709", "bt709", "bt709", "tv"]
        values = [default if value in (None, "unknown", "unspecified", "reserved") else value
                  for value, default in zip(values, fallback)]
        matrix, primaries, transfer, range_ = values
        allowed=[{"bt709":"bt709","bt470bg":"bt470bg","smpte170m":"smpte170m","smpte240m":"smpte240m","bt2020nc":"bt2020nc"},
                 {"bt709":"bt709","bt470bg":"bt470bg","smpte170m":"smpte170m","smpte240m":"smpte240m","bt2020":"bt2020","bt470m":"bt470m"},
                 {"bt709":"bt709","bt470bg":"bt470bg","bt470m":"bt470m","smpte170m":"smpte170m","smpte240m":"smpte240m","iec61966-2-1":"srgb","srgb":"srgb","bt2020-10":"bt2020-10","bt2020-12":"bt2020-12"},
                 {"tv":"tv","pc":"pc"}]
        if any(v not in choices for v,choices in zip(values,allowed)):raise ValueError("Unsupported source color metadata")
        matrix,primaries,transfer,range_=[choices[v] for v,choices in zip(values,allowed)]
        filter_ = (f"fps=30,colorspace=ispace={matrix}:iprimaries={primaries}:itrc={transfer}:irange={range_}:"
                   "space=bt709:primaries=bt709:trc=bt709:range=tv:format=yuv420p,"
                   "scale=480:320:flags=bicubic,format=yuv420p")
        args = [self.ffmpeg, "-hide_banner", "-nostdin", "-loglevel", "error", "-y", "-threads", "1",
                "-i", str(source), "-an", "-sn", "-vf", filter_, "-pix_fmt", "yuv420p", "-fps_mode", "passthrough"]
        if frame_limit:
            args += ["-frames:v", str(frame_limit)]
        args += ["-f", "rawvideo", str(output)]
        subprocess.run(args, stdout=subprocess.DEVNULL, check=True, timeout=900)
        size = Path(output).stat().st_size
        if not size or size % FRAME_BYTES:
            raise ValueError("Preprocessed clip has an invalid YUV420p byte size")
        atomic_json(Path(output).with_suffix(".preprocess.json"),{"video_id":video_id,
                    "source_sha256":self.records[video_id]["sha256"],"color_metadata":stream,
                    "resolved_input_color":dict(matrix=matrix,primaries=primaries,transfer=transfer,range=range_),
                    "filter":filter_,"frames":size//FRAME_BYTES,"yuv_sha256":digest(output)})
        return size // FRAME_BYTES


@contextmanager
def detector_device(repo, device):
    """Load/fuse on the explicit logical GPU without YOLO remasking CUDA.

    The pinned select_device remasks the process then returns cuda:0. That
    is unsafe after SB3/another library has initialized CUDA.
    """
    import torch
    target = torch.device(device)
    if target.type == "cuda":
        index = 0 if target.index is None else target.index
        if not torch.cuda.is_available() or index >= torch.cuda.device_count():
            raise ValueError(f"Detector GPU unavailable: {target}")
        target = torch.device("cuda", index)
        torch.cuda.set_device(target)
    elif target.type != "cpu":
        raise ValueError("Detector supports explicit CPU or CUDA devices only")
    sys.path.insert(0, str(Path(repo).resolve()))
    helper = original = None
    try:
        helper = importlib.import_module("utils.torch_utils")
        if Path(helper.__file__).resolve() != (Path(repo)/"utils/torch_utils.py").resolve():
            raise ValueError("A different YOLO utils module is already imported")
        original = helper.select_device
        helper.select_device = lambda *args, **kwargs: target
        yield target
    finally:
        if original is not None:
            helper.select_device = original
        sys.path.pop(0)


class FrozenDetector:
    def __init__(self, repo, weights, device="cuda:0"):
        import inspect
        import torch

        self.device = device
        lock = json.loads((ASSETS / "baseline_lock.json").read_text(encoding="utf-8"))["config"]["detector"]
        actual = subprocess.check_output(["git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()
        if actual != lock["commit"] or digest(weights) != lock["weights_sha256"]:
            raise ValueError("Detector source or weights differ from the frozen baseline")
        if subprocess.check_output(["git","-C",str(repo),"status","--porcelain","--untracked-files=no"],text=True).strip():
            raise ValueError("Tracked YOLO source has local modifications")
        torch.set_num_threads(1)
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        original = torch.load
        try:
            if "weights_only" in inspect.signature(original).parameters:
                def trusted_load(*args, **kwargs):
                    kwargs.setdefault("weights_only", False)
                    return original(*args, **kwargs)
                torch.load = trusted_load
            with detector_device(repo, device) as target:
                self.model = torch.hub.load(str(repo), "custom", path=str(weights), source="local", device=target, autoshape=True)
        finally:
            torch.load = original
        self.model.eval()
        if next(self.model.parameters()).dtype!=torch.float32:raise ValueError("Frozen detector must use FP32")
        if next(self.model.parameters()).device!=target:raise ValueError("Frozen detector loaded onto the wrong device")
        self.device=str(target)
        for parameter in self.model.parameters():
            parameter.requires_grad_(False)
        self.model.conf, self.model.iou, self.model.classes = .25, .45, [2]
        self.model.max_det, self.model.amp = 300, False
        from .pixels import RGB709
        self.color = RGB709()

    def __call__(self, yuv8):
        import torch
        rgb = self.color(yuv8)
        with torch.inference_mode():
            detections = self.model([rgb], size=640).xyxy[0].detach().cpu().numpy()
        boxes=detections[detections[:,5]==2,:5]
        # The locked baseline rounds recorded detector coordinates to 5 decimals.
        return np.asarray(np.round(boxes,5),dtype=np.float64)

    def close(self):
        self.color.close()
