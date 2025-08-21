from __future__ import annotations
from dataclasses import dataclass

@dataclass
class ControllerSpec:
    name: str
    locomotion: str  # 'wheeled' | 'legged' | 'humanoid' | 'unknown'

class BaseController:
    spec = ControllerSpec(name="base", locomotion="unknown")

    def __init__(self, stage, art_root: str, feats):
        self.stage = stage
        self.art_root = art_root
        self.feats = feats
        self.dt = 1.0 / 60.0
        self.vx = 0.0
        self.yaw_rate = 0.0

    def start(self, vx: float = 0.0, yaw_rate: float = 0.0, hz: float = 60.0):
        self.vx = float(vx)
        self.yaw_rate = float(yaw_rate)
        self.dt = 1.0 / float(hz)

    def step(self):
        pass  # override

    def shutdown(self):
        pass  # override
