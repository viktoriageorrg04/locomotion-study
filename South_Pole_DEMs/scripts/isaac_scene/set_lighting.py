# scripts/isaac_scene/set_lighting.py
from __future__ import annotations
import math
from pxr import Usd, UsdGeom, UsdLux, Gf

def _deg2rad(d: float) -> float:
    return d * math.pi / 180.0

def _dir_from_angles(elev_deg: float, az_deg: float) -> Gf.Vec3d:
    # Z-up. Elevation=0 on horizon. Az=0 -> +X, 90 -> +Y. Light points downward.
    e = _deg2rad(elev_deg); a = _deg2rad(az_deg)
    c = math.cos(e)
    return Gf.Vec3d(c*math.cos(a), c*math.sin(a), -math.sin(e))

def _quat_from_vectors(v_from: Gf.Vec3d, v_to: Gf.Vec3d) -> Gf.Quatf:
    # robust quaternion that rotates v_from into v_to
    a = v_from.GetNormalized()
    b = v_to.GetNormalized()
    cross = Gf.Cross(a, b)
    dot   = Gf.Dot(a, b)

    if dot > 1.0 - 1e-8:  # same direction
        return Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0))
    if dot < -1.0 + 1e-8:  # opposite direction
        # pick any axis perpendicular to a
        axis = Gf.Cross(a, Gf.Vec3d(1, 0, 0))
        if axis.GetLength() < 1e-6:
            axis = Gf.Cross(a, Gf.Vec3d(0, 1, 0))
        axis = axis.GetNormalized()
        return Gf.Quatf(0.0, Gf.Vec3f(axis[0], axis[1], axis[2]))  # 180° rotation

    s = math.sqrt((1.0 + dot) * 2.0)
    invs = 1.0 / s
    qx, qy, qz = (cross[0]*invs, cross[1]*invs, cross[2]*invs)
    qw = s * 0.5
    return Gf.Quatf(float(qw), Gf.Vec3f(float(qx), float(qy), float(qz)))

def ensure_lunar_lighting(
    stage: Usd.Stage,
    scope_path: str = "/World/Lights",
    sun_elev: float = 15.0,
    sun_az: float = 45.0,
    sun_angle_deg: float = 0.53,
    sun_intensity: float = 15000.0,
    sun_exposure: float = 3.0,
    black_sky: bool = True,
    add_fill: bool = True,
    fill_intensity: float = 0.03,
    **kwargs,  # accepts elevation_deg / azimuth_deg / fill_ratio / add_debug_sphere
) -> bool:
    if "elevation_deg" in kwargs: sun_elev = float(kwargs["elevation_deg"])
    if "azimuth_deg"   in kwargs: sun_az   = float(kwargs["azimuth_deg"])
    if "fill_ratio"    in kwargs: fill_intensity = float(kwargs["fill_ratio"])
    add_debug_sphere = bool(kwargs.get("add_debug_sphere", False))

    scope = UsdGeom.Scope.Define(stage, scope_path)

    # Sun
    sun = UsdLux.DistantLight.Define(stage, f"{scope_path}/Sun")
    sun.CreateIntensityAttr(float(sun_intensity))
    sun.CreateExposureAttr(float(sun_exposure))
    sun.CreateAngleAttr(float(sun_angle_deg))

    desired_dir = _dir_from_angles(sun_elev, sun_az)
    q = _quat_from_vectors(Gf.Vec3d(0, 0, -1), desired_dir)  # rotate -Z to desired_dir
    UsdGeom.Xformable(sun).AddOrientOp().Set(q)

    # shadows (use ShadowAPI in 4.5)
    UsdLux.ShadowAPI.Apply(sun.GetPrim()).CreateShadowEnableAttr(True)

    print(f"[lighting] Sun angle={sun_angle_deg:.2f}°, elev={sun_elev:.2f}°, az={sun_az:.2f}°, "
          f"dir=({desired_dir[0]:.6f}, {desired_dir[1]:.6f}, {desired_dir[2]:.6f})")
    print(f"[lighting] Sun intensity={sun_intensity}, exposure={sun_exposure:.2f} | "
          f"fill={add_fill} ({fill_intensity:.3f}) | black_sky={black_sky}")

    # Fill
    if add_fill and fill_intensity > 0.0:
        fill = UsdLux.DistantLight.Define(stage, f"{scope_path}/Fill")
        fill.CreateIntensityAttr(float(sun_intensity) * float(fill_intensity))
        fill.CreateExposureAttr(float(sun_exposure))
        fill.CreateAngleAttr(2.0)
        qf = _quat_from_vectors(Gf.Vec3d(0, 0, -1), -desired_dir)
        UsdGeom.Xformable(fill).AddOrientOp().Set(qf)
        UsdLux.ShadowAPI.Apply(fill.GetPrim()).CreateShadowEnableAttr(False)

    # Debug sphere
    if add_debug_sphere:
        dbg = UsdLux.SphereLight.Define(stage, f"{scope_path}/DebugSphere")
        dbg.CreateRadiusAttr(5.0)
        dbg.CreateIntensityAttr(200000.0)
        dbg.CreateExposureAttr(2.0)
        UsdGeom.XformCommonAPI(dbg).SetTranslate(Gf.Vec3d(0.0, 0.0, 200.0))
        print("[lighting] Debug sphere ON")

    return True
