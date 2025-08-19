# scripts/isaac_scene/set_lighting.py
from __future__ import annotations
import math

import carb
import omni.usd
from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf

# Stage helpers

def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()

def ensure_scope(path: str):
    st = get_stage()
    if not st.GetPrimAtPath(path).IsValid():
        st.DefinePrim(path, "Scope")

def remove_prim_if_exists(path: str) -> bool:
    st = get_stage()
    prim = st.GetPrimAtPath(path)
    if prim.IsValid():
        st.RemovePrim(path)
        print(f"[lighting] Removed prim: {path}")
        return True
    return False


# Viewport/default light rig

def disable_viewport_light_rig():
    """Delete /OmniKit_Viewport_LightRig so ONLY stage lights are active."""
    removed = remove_prim_if_exists("/OmniKit_Viewport_LightRig")
    if not removed:
        print("[lighting] No /OmniKit_Viewport_LightRig prim found (already using Stage Lights?)")


# Renderer settings (identical behavior to launch_isaac.py)

def configure_renderer_settings(args):
    s = carb.settings.get_settings()
    if args.renderer.lower() == "pathtracing":
        s.set("/rtx/pathtracing/spp", int(args.pt_spp))
        s.set("/rtx/pathtracing/maxBounces", int(args.pt_max_bounces))
        s.set("/rtx/pathtracing/clampSpp", 0)

        s.set("/rtx/pathtracing/accumulation/enable", True)
        s.set("/rtx/pathtracing/accumulation/maxFrames", 512)
        s.set("/rtx/pathtracing/accumulation/resetOnCameraCut", True)

        s.set("/rtx/pathtracing/denoiser/enable", True)
        s.set("/rtx/pathtracing/denoiser/inputType", "float4")
        s.set("/rtx/pathtracing/denoiser/blendFactor", 0.2)

        s.set("/rtx/pathtracing/fireflyFilter/enable", True)
        s.set("/rtx/pathtracing/fireflyFilter/threshold", 10.0)

        s.set("/rtx/pathtracing/colorClamp/enable", True)
        s.set("/rtx/pathtracing/colorClamp/value", 10.0)

        s.set("/rtx/pathtracing/samplingFilter", "BlackmanHarris")

    elif args.renderer.lower() == "raytracedlighting":
        s.set("/rtx/raytracing/spp", 32)
        s.set("/rtx/raytracing/maxBounces", 6)

        s.set("/rtx/raytracing/accumulation/enable", True)
        s.set("/rtx/raytracing/accumulation/maxFrames", 64)
        s.set("/rtx/raytracing/accumulation/resetOnCameraCut", True)

        s.set("/rtx/raytracing/denoiser/enable", True)
        s.set("/rtx/raytracing/denoiser/inputType", "float4")
        s.set("/rtx/raytracing/denoiser/blendFactor", 0.2)

        s.set("/rtx/raytracing/fireflyFilter/enable", True)
        s.set("/rtx/raytracing/fireflyFilter/threshold", 10.0)

        s.set("/rtx/raytracing/colorClamp/enable", True)
        s.set("/rtx/raytracing/colorClamp/value", 10.0)

        s.set("/rtx/raytracing/samplingFilter", "BlackmanHarris")
        s.set("/rtx/sceneDb/meshlights/forceDisable", False)

        # Keep tonemapper on, but stop it from “helping”
        for k, v in [
            ("/rtx/post/tonemapper/enable", True),
            ("/rtx/post/tonemapper/enableAutoExposure", False),
            ("/rtx/post/tonemapper/autoExposure", False),
            ("/rtx/post/tonemapper/localExposure/enable", False),
            ("/rtx/post/tonemapper/exposureCompensation", -1.5),
        ]:
            try:
                s.set(k, v)
            except Exception:
                pass


# Environment / sky

def disable_dome_and_skylights():
    st = get_stage()
    domes = [p for p in st.Traverse() if p.GetTypeName() in ("DomeLight", "SkydomeLight", "DomeLight:Light")]
    if not domes:
        print("[lighting] No Dome/Skydome lights found.")
        return
    for d in domes:
        try:
            light = UsdLux.DomeLight(d)
            if light:
                light.CreateIntensityAttr().Set(0.0)
                light.CreateExposureAttr().Set(-20.0)
                print(f"[lighting] Disabled dome light: {d.GetPath()}")
        except Exception as e:
            print(f"[lighting][WARN] Could not modify dome {d.GetPath()}: {e}")


# Fake ambient fill

def add_fill_light(fill_ratio: float, base_intensity: float):
    """Create a very dim dome fill to fake ground bounce."""
    if fill_ratio <= 0.0:
        return
    st = get_stage()
    ensure_scope("/World/Lights")
    fill = UsdLux.DomeLight.Define(st, Sdf.Path("/World/Lights/Fill"))
    intensity = float(base_intensity) * float(fill_ratio) or (50000.0 * float(fill_ratio))
    fill.CreateIntensityAttr().Set(intensity)
    fill.CreateExposureAttr().Set(0.0)
    fill.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    fill.CreateTextureFileAttr().Clear()
    fill.CreateNormalizeAttr().Set(True)
    print(f"[lighting] Fill dome added: ratio={fill_ratio:.4f}, intensity≈{intensity:.2f}")


# Sun light (distant)

def set_sun(az_deg: float, el_deg: float, intensity: float, exposure: float):
    st = get_stage()
    ensure_scope("/World/Lights")
    sun_prim_path = Sdf.Path("/World/Lights/Sun")
    if not st.GetPrimAtPath(sun_prim_path).IsValid():
        st.DefinePrim(sun_prim_path, "DistantLight")
        print(f"[lighting] Created DistantLight at {sun_prim_path}")

    sun = UsdLux.DistantLight(st.GetPrimAtPath(sun_prim_path))

    sun.CreateIntensityAttr().Set(float(intensity))
    sun.CreateExposureAttr().Set(float(exposure))
    sun.CreateAngleAttr().Set(0.53)  # solar full angle ~0.53°

    # Aim -Z to desired direction via quaternion
    elev = math.radians(el_deg)
    az   = math.radians(az_deg)
    dirv = Gf.Vec3d(math.cos(elev)*math.cos(az),
                    math.cos(elev)*math.sin(az),
                   -math.sin(elev))

    xf = UsdGeom.Xformable(sun.GetPrim())
    try:
        xf.ClearXformOpOrder()
    except Exception:
        xf.SetXformOpOrder([])

    a = Gf.Vec3d(0, 0, -1).GetNormalized()
    b = dirv.GetNormalized()
    c = Gf.Cross(a, b); d = Gf.Dot(a, b)
    if d > 1.0 - 1e-8:
        q = Gf.Quatf(1.0, Gf.Vec3f(0, 0, 0))
    elif d < -1.0 + 1e-8:
        axis = Gf.Cross(a, Gf.Vec3d(1,0,0))
        if axis.GetLength() < 1e-6:
            axis = Gf.Cross(a, Gf.Vec3d(0,1,0))
        axis = axis.GetNormalized()
        q = Gf.Quatf(0.0, Gf.Vec3f(axis[0], axis[1], axis[2]))
    else:
        s = math.sqrt((1.0 + d) * 2.0); invs = 1.0 / s
        q = Gf.Quatf(s*0.5, Gf.Vec3f(c[0]*invs, c[1]*invs, c[2]*invs))
    xf.AddOrientOp().Set(q)

    print(f"[lighting] Sun set: az={az_deg:.1f}°, el={el_deg:.1f}°, intensity={intensity:.1f}, exposure={exposure:.1f}")


# One-call setup (wrapper used by launch_isaac.py)

def setup_daylight(args):
    """Mirror the working lighting flow from launch_isaac.py using argparse args."""
    set_sun(args.sun_az, args.sun_el, args.sun_intensity, args.sun_exposure)

    if getattr(args, "viewport", "stage") == "stage":
        disable_viewport_light_rig()
    else:
        print("[lighting] Using DEFAULT viewport light rig (not lunar realistic).")

    # Keep behavior identical to the working code: always kill domes/skydomes
    disable_dome_and_skylights()

    effective_sun_I = float(args.sun_intensity) * (2.0 ** float(args.sun_exposure))
    add_fill_light(getattr(args, "fill_ratio", 0.0), effective_sun_I)


__all__ = [
    "configure_renderer_settings",
    "setup_daylight",
    "disable_viewport_light_rig",
    "disable_dome_and_skylights",
    "add_fill_light",
    "set_sun",
]
