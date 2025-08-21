from .base import BaseController, ControllerSpec
from locomotion.wheeled_controller import dc_drive_wheels  # you already have this file

class WheeledDiffController(BaseController):
    spec = ControllerSpec(name="wheeled-dc", locomotion="wheeled")

    def __init__(self, stage, art_root: str, feats):
        super().__init__(stage, art_root, feats)
        self._name_to_path = {p.split("/")[-1].lower(): p for p in (feats.drive_joint_paths or [])}
        self._omega_cmd = 0.0

    def start(self, vx: float = 0.0, yaw_rate: float = 0.0, hz: float = 60.0):
        super().start(vx, yaw_rate, hz)
        r = self.feats.wheel_radius_m or 0.0
        self._omega_cmd = (self.vx / r) if r > 0 else 0.0
        # one-shot command (target velocity is persistent)
        dc_drive_wheels(self.art_root, self._omega_cmd, self._name_to_path)

    def step(self):
        # nothing to do per-timestep; DC keeps the targets applied
        pass

    @property
    def omega_cmd(self) -> float:
        return self._omega_cmd
