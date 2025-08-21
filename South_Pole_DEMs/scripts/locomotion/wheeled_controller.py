"""
Wheeled rover controller helpers extracted from launch_isaac.py
- Robust wheel sign inference based on joint axis in world frame
- dynamic_control-based differential drive command
Designed to be imported by generic Isaac scene launchers.
"""
from pxr import Usd, UsdGeom, UsdPhysics, Gf
import omni
from omni.isaac.dynamic_control import _dynamic_control as dc

__all__ = ["wheel_sign_for_joint", "dc_drive_wheels"]

def wheel_sign_for_joint(stage, joint_path: str) -> float:
    """Return wheel sign from actual joint axis (robust to mirroring)."""
    jprim = stage.GetPrimAtPath(joint_path)

    # 1) Determine revolute axis in local joint frame
    axis_tok = None
    try:
        if jprim.IsA(UsdPhysics.RevoluteJoint):
            axis_tok = UsdPhysics.RevoluteJoint(jprim).GetAxisAttr().Get()
        elif jprim.HasAPI(UsdPhysics.Joint):
            axis_tok = UsdPhysics.Joint(jprim).GetAxisAttr().Get()
    except Exception:
        axis_tok = None

    axis_local = Gf.Vec3d(0, 1, 0)  # default Y
    if axis_tok:
        t = str(axis_tok).upper()
        if t == "X":
            axis_local = Gf.Vec3d(1, 0, 0)
        elif t == "Y":
            axis_local = Gf.Vec3d(0, 1, 0)
        elif t == "Z":
            axis_local = Gf.Vec3d(0, 0, 1)

    # 2) Convert to world and use Y direction for sign
    try:
        M = UsdGeom.Xformable(jprim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        axis_world = (M.TransformDir(axis_local)).GetNormalized()
        return 1.0 if axis_world[1] >= 0.0 else -1.0
    except Exception:
        return 1.0  # safe default

def dc_drive_wheels(prim_for_ctrl, omega_cmd: float, name_to_path: dict) -> int:
    dcif = dc.acquire_dynamic_control_interface()
    art = dcif.get_articulation(prim_for_ctrl)
    assert art, f"Dynamic Control can't find articulation at {prim_for_ctrl}"

    stage = omni.usd.get_context().get_stage()

    # Build (dof_name, sign) pairs from the mapping if provided; else fall back to Jackal names under the same root.
    if name_to_path:
        items = [(p.rsplit("/", 1)[-1], wheel_sign_for_joint(stage, p)) for p in name_to_path.values()]
    else:
        base = prim_for_ctrl
        wheel_joint_paths = [
            f"{base}/front_left_wheel_joint",
            f"{base}/front_right_wheel_joint",
            f"{base}/rear_left_wheel_joint",
            f"{base}/rear_right_wheel_joint",
        ]
        items = [(p.rsplit("/", 1)[-1], wheel_sign_for_joint(stage, p)) for p in wheel_joint_paths]

    ok = 0
    for dof_name, s in items:
        dof = dcif.find_articulation_dof(art, dof_name)   # (art_handle, dof_name) — correct signature
        if dof:
            dcif.set_dof_velocity_target(dof, s * omega_cmd)
            ok += 1
        else:
            print(f"[dc] WARN: DOF not found for '{dof_name}' under {prim_for_ctrl}")

    print(f"[dc] commanded per-wheel ±ω on {ok}/{len(items)} wheel DOFs (|ω|={abs(omega_cmd):.3f})")
    return ok
