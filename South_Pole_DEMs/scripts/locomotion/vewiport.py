# locomotion/viewport.py
from pxr import Usd, UsdGeom, Gf

def get_or_create_camera(stage, path="/World/Camera"):
    prim = stage.GetPrimAtPath(path)
    if not prim:
        UsdGeom.Camera.Define(stage, path)
        prim = stage.GetPrimAtPath(path)
    return prim

def attach_birdseye(stage, target_path: str, height: float = 20.0, parent: bool = True):
    cam = get_or_create_camera(stage)
    tgt_prim = stage.GetPrimAtPath(target_path)
    if not tgt_prim:
        return cam.GetPath()
    tgt_xf = UsdGeom.Xformable(tgt_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    tgt_pos = tgt_xf.ExtractTranslation()
    cam_xf = Gf.Matrix4d(1.0)
    cam_xf.SetTranslate(Gf.Vec3d(tgt_pos[0], tgt_pos[1], tgt_pos[2] + height))
    # Look straight down
    rot_down = Gf.Rotation(Gf.Vec3d(1,0,0), -90).GetMatrix()
    cam_xf = rot_down * cam_xf
    UsdGeom.Xformable(cam).AddTransformOp().Set(cam_xf)
    if parent:
        stage.GetEditTarget()
        Usd.PrimRange.ReversePostOrder(tgt_prim)
        cam.GetParent().GetChildren()
        # Just reparent via xformOps parenting in USD: keep the path fixed and bind via Follow (if you prefer)
    return cam.GetPath()
