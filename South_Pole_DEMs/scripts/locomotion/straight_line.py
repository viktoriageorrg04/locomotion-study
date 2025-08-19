# South_Pole_DEMs/scripts/locomotion/straight_line.py
import json
import numpy as np

from isaacsim.core.api import SimulationContext
from isaacsim.core.prims import SingleArticulation

# ArticulationAction import path (new API), with a fallback for older builds.
try:
    from isaacsim.core.api.controllers.articulation_controller import ArticulationAction
except Exception:
    from omni.isaac.core.controllers import ArticulationAction  # older Isaac

# ---------- Metrics helper ----------
class BaselineMetrics:
    def __init__(self, art: SingleArticulation, vx_cmd: float | None = None):
        self.art = art
        self.p0, _ = art.get_world_pose()
        self.energy = 0.0
        self._vx_cmd = float(vx_cmd) if vx_cmd is not None else None

    def _efforts(self):
        for attr in ("get_applied_joint_efforts", "get_joint_efforts", "get_net_joint_torques"):
            if hasattr(self.art, attr):
                v = getattr(self.art, attr)()
                if v is not None:
                    return np.asarray(v, dtype=np.float32)
        return None

    def step(self, dt: float):
        try:
            qd = self.art.get_joint_velocities()
            tau = self._efforts()
        except Exception:
            qd, tau = None, None
        if qd is not None and tau is not None:
            qd = np.asarray(qd, dtype=np.float32)
            tau = np.asarray(tau, dtype=np.float32)
            n = min(qd.shape[-1], tau.shape[-1])
            self.energy += float(np.sum(np.abs(qd[..., :n] * tau[..., :n]))) * float(dt)

    def summary(self, t_elapsed: float):
        p, _ = self.art.get_world_pose()
        disp = float(np.linalg.norm((np.array(p[:2]) - np.array(self.p0[:2]))))
        avg_speed = disp / max(t_elapsed, 1e-6)
        epm = (self.energy / max(disp, 1e-3)) if disp > 1e-3 else float("inf")
        slip = None
        if self._vx_cmd is not None and self._vx_cmd > 1e-6:
            slip = max(0.0, 1.0 - (disp / (self._vx_cmd * t_elapsed + 1e-6)))
        return {
            "elapsed_s": float(t_elapsed),
            "distance_m": disp,
            "avg_speed_mps": avg_speed,
            "energy_J": float(self.energy),
            "energy_per_m_Jpm": float(epm),
            "slip_ratio_est": (None if slip is None else float(slip)),
        }

# ---------- Generic runner for controllers that return ArticulationAction ----------
def run_with_articulation_action_controller(
    prim_path: str,
    controller_ctor,
    cmd=(0.5, 0.0, 0.0),
    duration: float = 10.0,
    hz: float = 60.0,
):
    sim = SimulationContext()
    sim.set_simulation_dt(1.0 / hz)
    sim.play()

    robot = SingleArticulation(prim_path=prim_path)
    robot.initialize()

    ctrl = controller_ctor(robot)
    metrics = BaselineMetrics(robot, vx_cmd=cmd[0])

    dt = 1.0 / hz
    steps = int(round(duration * hz))
    u = np.array(cmd, dtype=np.float32)

    for _ in range(steps):
        try:
            action = ctrl.forward(command=u)   # modern signature
        except TypeError:
            action = ctrl.forward(*u.tolist()) # older signature

        if not isinstance(action, ArticulationAction):
            raise TypeError("Controller must return ArticulationAction")
        robot.apply_action(action)

        metrics.step(dt)
        sim.step(render=True)

    sim.pause()
    return metrics.summary(duration)

# ---------- Differential drive (Jackal) ----------
def run_diff_drive(
    prim_path: str,
    left_wheels=None,            # <— allow None
    right_wheels=None,           # <— allow None
    vx: float = 0.5,
    yaw_rate: float = 0.0,
    duration: float = 10.0,
    hz: float = 60.0,
    wheel_radius: float = 0.098,
    wheel_base: float = 0.33,
):
    """Works with both old (tuple) and new (ArticulationAction) DifferentialController APIs."""
    from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

    sim = SimulationContext()
    sim.set_simulation_dt(1.0 / hz)
    sim.play()

    robot = SingleArticulation(prim_path=prim_path)
    robot.initialize()

    names = list(robot.dof_names)
    lower = [n.lower() for n in names]

    # Auto-detect by substring if not provided
    if left_wheels is None:
        L_idx = np.array([i for i, n in enumerate(lower) if ("left" in n and "wheel" in n)], dtype=np.int64)
    else:
        kws = [kw.lower() for kw in left_wheels]
        L_idx = np.array([i for i, n in enumerate(lower) if any(kw in n for kw in kws)], dtype=np.int64)

    if right_wheels is None:
        R_idx = np.array([i for i, n in enumerate(lower) if ("right" in n and "wheel" in n)], dtype=np.int64)
    else:
        kws = [kw.lower() for kw in right_wheels]
        R_idx = np.array([i for i, n in enumerate(lower) if any(kw in n for kw in kws)], dtype=np.int64)

    if L_idx.size == 0 or R_idx.size == 0:
        raise RuntimeError(f"Could not find wheel DOFs. Found names={names}")

    ctrl = DifferentialController(name="diff", wheel_radius=wheel_radius, wheel_base=wheel_base)
    metrics = BaselineMetrics(robot, vx_cmd=vx)

    dt = 1.0 / hz
    steps = int(round(duration * hz))
    cmd = np.array([vx, yaw_rate], dtype=np.float32)

    for _ in range(steps):
        res = ctrl.forward(command=cmd)

        if isinstance(res, ArticulationAction):
            # If controller didn’t set indices, expand its 2-element velocity to our wheel DOFs.
            jv = getattr(res, "joint_velocities", None)
            ji = getattr(res, "joint_indices", None)
            if jv is not None and (ji is None or len(ji) == 0):
                flat = np.array(jv, dtype=np.float32).reshape(-1)
                # Expect 2 entries: [wL, wR]
                if flat.size >= 2:
                    vels = np.zeros(robot.num_dof, dtype=np.float32)
                    vels[L_idx] = float(flat[0])
                    vels[R_idx] = float(flat[1])
                    robot.apply_action(ArticulationAction(joint_velocities=vels))
                else:
                    # Fallback: just apply what we got
                    robot.apply_action(res)
            else:
                robot.apply_action(res)
        else:
            # Old API: (wL, wR)
            wL, wR = res
            vels = np.zeros(robot.num_dof, dtype=np.float32)
            vels[L_idx] = float(wL)
            vels[R_idx] = float(wR)
            robot.apply_action(ArticulationAction(joint_velocities=vels))

        metrics.step(dt)
        sim.step(render=True)

    sim.pause()
    return metrics.summary(duration)

# ---------- Helper to reuse “keyboard” controllers at fixed velocity ----------
class ConstantVelocityWrapper:
    """Feed a fixed [vx, vy, yaw] to any teleop/keyboard controller every step."""
    def __init__(self, inner_controller, fixed_cmd=(0.5, 0.0, 0.0)):
        self.inner = inner_controller
        self.fixed = np.array(fixed_cmd, dtype=np.float32)

    def forward(self, command=None):
        u = self.fixed if command is None else np.asarray(command, dtype=np.float32)
        try:
            return self.inner.forward(command=u)  # modern signature
        except TypeError:
            return self.inner.forward(*u.tolist())  # legacy
