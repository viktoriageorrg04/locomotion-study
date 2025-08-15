# drive_diff.py — reusable differential-drive loop for any rover
from curses.ascii import ctrl
from typing import Callable, Tuple, Optional
import numpy as np
from pxr import Usd
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.wheeled_robots.controllers import DifferentialController
import omni.timeline

def _ensure_simctx():
    """Create a SimulationContext if the new API expects one."""
    try:
        # New namespace (4.5)
        from isaacsim.core.api.simulation_context import SimulationContext
    except Exception:
        # Back-compat (older examples)
        from omni.isaac.core import SimulationContext
    # Constructing it is enough for SingleArticulation to get a simulation_view.
    # It will use the existing /World/physicsScene you already authored.
    return SimulationContext()

def _find_articulation_prim(stage: Usd.Stage, root_hint: str) -> str:
    """Resolve the actual articulation prim path given a holder/root hint."""
    # Try the hint itself
    if Articulation(prim_path=root_hint).is_valid():
        return root_hint
    # Common holder/payload pattern
    p = f"{root_hint}/payload"
    if Articulation(prim_path=p).is_valid():
        return p
    # Fallback: scan one level down for a valid Articulation
    root = stage.GetPrimAtPath(root_hint)
    if root:
        for child in root.GetChildren():
            path = child.GetPath().pathString
            if Articulation(prim_path=path).is_valid():
                return path
    # Last resort: assume hint
    return root_hint

def _group_wheels(art, left_keys, right_keys):
    # 4.5+ exposes a .dof_names property; older builds had get_dof_names()
    try:
        names = list(art.dof_names)
    except AttributeError:
        names = list(art.get_dof_names())

    lower = [n.lower() for n in names]

    def _indices(keys):
        if isinstance(keys, (tuple, list)):
            return [i for i, n in enumerate(lower) if any(k.lower() in n for k in keys)]
        else:
            k = str(keys).lower()
            return [i for i, n in enumerate(lower) if k in n]

    L_idx = _indices(left_keys)
    R_idx = _indices(right_keys)

    if not L_idx or not R_idx:
        print("[drive_diff] Available DOFs:", names)
        raise RuntimeError(f"Could not find wheels with keys L={left_keys}, R={right_keys}")

    return L_idx, R_idx

def run(
    stage: Usd.Stage,
    prim_hint: str = "/World/jackal",
    seconds: Optional[float] = 8.0,
    hz: float = 60.0,
    wheel_radius: float = 0.098,
    wheel_base: float = 0.375,
    left_key: str = "left",
    right_key: str = "right",
    command_fn: Optional[Callable[[float], Tuple[float, float]]] = None,
):
    """
    Drive a differential rover already spawned in THIS process.
    - stage: current USD stage (from your launcher)
    - prim_hint: holder or articulation prim path (e.g. /World/jackal or /World/jackal/payload)
    - seconds: None to run indefinitely under your app loop; otherwise run ~seconds then return
    - hz: control rate
    - wheel_radius/base: kinematics
    - left_key/right_key: substrings to classify wheel DOFs (adjust per rover)
    - command_fn(t)->(v,w): m/s and rad/s. If None, uses a small demo.
    """
    artic_path = _find_articulation_prim(stage, prim_hint)
    robot = Articulation(articulation_prim_path := artic_path)
    _ensure_simctx()
    robot.initialize()

    L_idx, R_idx = _group_wheels(robot, left_key, right_key)
    L_idx = np.array(L_idx, dtype=int)
    R_idx = np.array(R_idx, dtype=int)

    if L_idx.size == 0 or R_idx.size == 0:
        try:
            print("[drive_diff] DOFs:", list(robot.dof_names))
        except AttributeError:
            print("[drive_diff] DOFs:", list(robot.get_dof_names()))
        raise RuntimeError(f"Could not find wheels L={left_key} R={right_key}")

    print("[drive_diff] L_idx:", L_idx, "R_idx:", R_idx)

    try:
        dof_names = list(robot.dof_names)
    except Exception:
        dof_names = list(robot.get_dof_names())
    num_dof = len(dof_names)

    ctrl = DifferentialController(
        name="diff",
        wheel_radius=wheel_radius,
        wheel_base=wheel_base,
        max_linear_speed=5.0,
        max_angular_speed=6.0,
    )

    def demo_cmd(t: float) -> Tuple[float, float]:
        if t < 2.0:   return 1.0, 0.0     # forward
        if t < 4.0:   return 0.0, 0.8     # spin
        return 0.8, 0.6                   # circle

    cmd = command_fn or demo_cmd

    # Make sure the timeline is playing so physics advances
    tl = omni.timeline.get_timeline_interface()
    if not tl.is_playing():
        tl.play()

    dt = 1.0 / float(hz)
    steps = int(seconds * hz) if seconds is not None else None
    t = 0.0
    i = 0
    print(f"[drive_diff] Driving {articulation_prim_path} (L DOFs={L_idx.tolist()}, R DOFs={R_idx.tolist()})")

    # Control loop (drop-in replacement)
    while True:
        # command: (v [m/s], w [rad/s])
        v, w = cmd(t)
        cmd_arr = np.array([v, w], dtype=np.float32)

        res = ctrl.forward(command=cmd_arr)   # 4.5: ArticulationAction, older: (wl, wr)

        # Unify to (wl, wr) wheel angular velocities [rad/s]
        if isinstance(res, ArticulationAction):
            jv = np.array(res.joint_velocities).ravel()
            if jv.size >= 2:
                wl, wr = float(jv[0]), float(jv[1])
            else:
                # Fallback: compute from kinematics
                wl = (v - 0.5 * wheel_base * w) / wheel_radius
                wr = (v + 0.5 * wheel_base * w) / wheel_radius
        elif isinstance(res, (tuple, list, np.ndarray)) and len(res) == 2:
            wl, wr = float(res[0]), float(res[1])
        else:
            # Unknown shape → compute directly
            wl = (v - 0.5 * wheel_base * w) / wheel_radius
            wr = (v + 0.5 * wheel_base * w) / wheel_radius

        # Map L/R wheel speeds to ALL wheel joints (e.g., 2 left, 2 right)
        try:
            dof_names = list(robot.dof_names)
        except AttributeError:
            dof_names = list(robot.get_dof_names())
        num_dof = len(dof_names)

        vels = np.zeros(num_dof, dtype=np.float32)
        vels[L_idx] = wl
        vels[R_idx] = wr

        action = ArticulationAction(joint_velocities=vels)
        robot.apply_action(action)

        yield dt  # tell the caller how long to wait

        t += dt
        i += 1
        if steps is not None and i >= steps:
            break

    print("[drive_diff] Done.")
