"""Access to an exported sequence (see export_sequence.py) and to the YAML configuration."""
import csv
from pathlib import Path

import cv2
import yaml

from controller.quality import QualityParams
from controller.scheduler import LevelSettings, SchedulerParams
from degradation.profiles import Profile

REPO = Path(__file__).resolve().parents[1]


class Sequence:
    def __init__(self, path):
        self.path = Path(path)
        self.name = self.path.name
        self.camera = yaml.safe_load((self.path / "camera.yaml").read_text())
        self.frames = list(csv.DictReader(open(self.path / "frames.csv")))
        self.stamps = [float(r["stamp"]) for r in self.frames]
        self.has_thermal = "thermal" in self.frames[0]

    def __len__(self):
        return len(self.frames)

    @property
    def duration(self):
        return self.stamps[-1] - self.stamps[0]

    def rgb(self, i):
        return cv2.imread(str(self.path / self.frames[i]["rgb"]), cv2.IMREAD_COLOR)

    def depth(self, i):
        return cv2.imread(str(self.path / self.frames[i]["depth"]), cv2.IMREAD_UNCHANGED)

    def thermal_raw(self, i):
        return cv2.imread(str(self.path / self.frames[i]["thermal"]), cv2.IMREAD_UNCHANGED)


def load_yaml(name):
    return yaml.safe_load((REPO / "config" / name).read_text())


def load_profile(name, seq: Sequence) -> Profile:
    """Profile times are fractions of the sequence duration, measured from its first frame."""
    spec = load_yaml("profiles.yaml")[name]
    if spec["kind"] == "none":
        return Profile(name, "none")
    t0, dur = seq.stamps[0], seq.duration
    return Profile(name, spec["kind"], onset_s=t0 + spec["onset_frac"] * dur, ramp_s=spec["ramp_s"],
                   hold_s=spec["hold_frac"] * dur, peak=spec["peak"], seed=spec["seed"])


def quality_params(params: dict, calibration: dict) -> QualityParams:
    q = params["quality"]
    return QualityParams(alpha=q["alpha"], beta=q["beta"], gamma=q["gamma"], v_ref=dict(calibration["v_ref"]))


def scheduler_params(params: dict) -> SchedulerParams:
    s = dict(params["scheduler"])
    levels = tuple(LevelSettings(n_features=l["n_features"], min_keyframe_interval=l["min_keyframe_interval"],
                                 local_ba=l["local_ba"], loop_closure=l["loop_closure"],
                                 local_ba_every=l.get("local_ba_every", 1), frame_stride=l.get("frame_stride", 1))
                   for l in s.pop("levels"))
    return SchedulerParams(levels=levels, **s)
