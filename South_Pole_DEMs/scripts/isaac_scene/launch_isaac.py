# launch_isaac.py
from __future__ import annotations

import argparse
import math
from pathlib import Path

from isaacsim import SimulationApp

# -------------------------------
# CLI
# -------------------------------

ap = argparse.ArgumentParser("Open USD terrain in Isaac Sim, optionally bind regolith, set lunar lighting.")
ap.add_argument("--usd", required=True, help="Path to a USD/USDA stage to open")
ap.add_argument("--renderer", default="RayTracedLighting", help="E.g. 'RayTracedLighting', 'PathTracing'")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--viewport", choices=["default", "stage"], default="stage",
                help="default = Omniverse default viewport lights on; stage = only lights authored on the stage")
ap.add_argument("--black-sky", action="store_true", help="Disable any Dome/Sky lights for a moon-like sky")

# Sun parameters
ap.add_argument("--sun-az", type=float, default=120.0, help="Azimuth in degrees")
ap.add_argument("--sun-el", type=float, default=8.0, help="Elevation in degrees above horizon")
ap.add_argument("--sun-intensity", type=float, default=3000.0, help="UsdLux intensity (unitless multiplier)")
ap.add_argument("--sun-exposure", type=float, default=0.01, help="UsdLux exposure (EV offset)")

ap.add_argument("--fill-ratio", type=float, default=0.0,
                help="0..1 fraction of sun intensity used for a dim ambient fill (0 = off)")

# Material knobs (applied to /World/Terrain if not skipped)
ap.add_argument("--skip-material", action="store_true",
                help="Do NOT rebind material on /World/Terrain (preserve the USD's original material).")

# Path Tracing quality helpers (safe defaults; ignored on RTL)
ap.add_argument("--pt-spp", type=int, default=128, help="Path Tracing samples per pixel")
ap.add_argument("--pt-max-bounces", type=int, default=6, help="Path Tracing max bounces")

# Spawn assets
ap.add_argument("--asset-usd", default="", help="Path to any robot/asset USD to spawn")
ap.add_argument("--asset-name", default="asset", help="Name under /World")
ap.add_argument("--asset-x", type=float, default=0.0)
ap.add_argument("--asset-y", type=float, default=0.0)
ap.add_argument("--asset-yaw", type=float, default=0.0)
ap.add_argument("--drop-margin", type=float, default=0.00, help="Meters to drop above terrain")
ap.add_argument("--moon", action="store_true", help="Use 1.62 m/s^2 gravity (lunar)")
ap.add_argument("--asset-center", action="store_true",
                help="Place asset at the center of /World/Terrain")

args = ap.parse_args()
print(f"[launch_isaac] Args: {args}")

simulation_app = SimulationApp({"renderer": args.renderer, "headless": args.headless})

# Import after SimulationApp
import carb
import omni.usd
from pxr import Usd, UsdGeom, UsdLux, Gf, Sdf

# -------------------------------
# Helpers
# -------------------------------

def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()

def open_stage(path: str):
    ctx = omni.usd.get_context()
    ctx.open_stage(path)
    st = ctx.get_stage()
    st.SetTimeCodesPerSecond(60)
    print(f"[launch_isaac] Opened stage: {path}")

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

# -------------------------------
# Viewport/default light rig
# -------------------------------

def disable_viewport_light_rig():
    """Delete /OmniKit_Viewport_LightRig so ONLY stage lights are active."""
    removed = remove_prim_if_exists("/OmniKit_Viewport_LightRig")
    if not removed:
        print("[lighting] No /OmniKit_Viewport_LightRig prim found (already using Stage Lights?)")

# -------------------------------
# Renderer settings
# -------------------------------

def configure_renderer_settings(args):
    s = carb.settings.get_settings()
    if args.renderer.lower() == "pathtracing":
        s.set("/rtx/pathtracing/spp", int(args.pt_spp))
        s.set("/rtx/pathtracing/maxBounces", int(args.pt_max_bounces))
        s.set("/rtx/pathtracing/clampSpp", 0)  # stop the default 64-SPP clamp

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
            ("/rtx/post/tonemapper/enableAutoExposure", False),  # 4.5 name
            ("/rtx/post/tonemapper/autoExposure", False),        # older name; harmless if unknown
            ("/rtx/post/tonemapper/localExposure/enable", False),
            ("/rtx/post/tonemapper/exposureCompensation", -1.5),
        ]:
            try: s.set(k, v)
            except Exception: pass

# -------------------------------
# Environment / sky
# -------------------------------

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

# -------------------------------
# Fake ambient light
# -------------------------------

def add_fill_light(fill_ratio: float, base_intensity: float):
    """Create a very dim dome fill to fake ground bounce."""
    if fill_ratio <= 0.0:
        return
    st = get_stage()
    ensure_scope("/World/Lights")
    fill = UsdLux.DomeLight.Define(st, Sdf.Path("/World/Lights/Fill"))
    # Use a minimum intensity if base_intensity is zero
    intensity = float(base_intensity) * float(fill_ratio)
    if intensity == 0.0:
        intensity = 50000.0 * float(fill_ratio)  # fallback value
    fill.CreateIntensityAttr().Set(intensity)
    fill.CreateExposureAttr().Set(0.0)
    fill.CreateColorAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))
    fill.CreateTextureFileAttr().Clear()
    fill.CreateNormalizeAttr().Set(True)
    print(f"[lighting] Fill dome added: ratio={fill_ratio:.4f}, intensity≈{intensity:.2f}")

# -------------------------------
# Sun light (distant)
# -------------------------------

def set_sun(az_deg: float, el_deg: float, intensity: float, exposure: float):
    st = get_stage()
    ensure_scope("/World/Lights")
    sun_prim_path = Sdf.Path("/World/Lights/Sun")
    if not st.GetPrimAtPath(sun_prim_path).IsValid():
        st.DefinePrim(sun_prim_path, "DistantLight")
        print(f"[lighting] Created DistantLight at {sun_prim_path}")

    sun = UsdLux.DistantLight(st.GetPrimAtPath(sun_prim_path))

    sun.CreateIntensityAttr().Set(float(intensity))
    # sun.CreateExposureAttr().Set(0.0)
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

    print(f"[launch_isaac] Sun set: az={az_deg:.1f}°, el={el_deg:.1f}°, intensity={intensity:.1f}, exposure={exposure:.1f}")

# -------------------------------
# Frame camera on terrain (best-effort; version-safe)
# -------------------------------

def frame_view_on_prim(prim_path: str) -> bool:
    try:
        # Try the older viewport window API
        from omni.kit.viewport.utility import get_active_viewport_window
        vpw = get_active_viewport_window()
        if vpw:
            import omni.kit.commands
            omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[prim_path], expand_in_stage=True)
            if hasattr(vpw, "frame_selection"):
                vpw.frame_selection()
                return True
    except Exception as e:
        print(f"[frame][WARN] {e}")
    return False

# -------------------------------
# Open file and configure
# -------------------------------

from pathlib import Path
import sys

_THIS = Path(__file__).resolve()
_SCRIPT_DIR = _THIS.parent.parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from helper.terrain_debug import print_terrain_elevation, debug_asset_vs_terrain

stage_path = str(Path(args.usd).expanduser().resolve())
open_stage(stage_path)

# Author lightning on the loaded stage
set_sun(args.sun_az, args.sun_el, args.sun_intensity, args.sun_exposure)
if args.viewport == "stage":
    disable_viewport_light_rig()
else:
    print("[lighting] Using DEFAULT viewport light rig (not lunar realistic).")
disable_dome_and_skylights()
effective_sun_I = args.sun_intensity * (2.0 ** args.sun_exposure)  # exposure into intensity
add_fill_light(args.fill_ratio, effective_sun_I)

# Path tracing quality
configure_renderer_settings(args)

# Material on /World/Terrain if present (unless user wants to preserve original)
if not args.skip_material:
    # Use set_regolith.py for material binding
    from set_regolith import make_and_bind_regolith

    root = Path(__file__).parent.parent.parent  # Points to South_Pole_DEMs
    tex_dir = root / "assets" / "textures" / "regolith"
    mesh_path = "/World/Terrain"

    stage = get_stage()
    ok = make_and_bind_regolith(
        stage,
        mesh_path,
        tex_dir=str(tex_dir),
        tile_meters=2.0,
        verbose=True
    )
    print(f"[launch_isaac] make_and_bind_regolith returned: {ok}")
else:
    print("[material] --skip-material set: preserving original material on /World/Terrain")

framed = frame_view_on_prim("/World/Terrain")
print(f"[launch_isaac] View framed: {framed}")
print("[launch_isaac] Ready.")

from spawn_rover import ensure_physics_scene, apply_terrain_physics, spawn_asset, world_range_for_path, snap_to_ground

stage = get_stage()

print_terrain_elevation(stage, "/World/Terrain")

ensure_physics_scene(stage, moon=args.moon)
apply_terrain_physics(stage, "/World/Terrain")

spawn_x, spawn_y = args.asset_x, args.asset_y
if args.asset_center:
    r = world_range_for_path(stage, "/World/Terrain")
    if r:
        spawn_x = 0.5 * (float(r.GetMin()[0]) + float(r.GetMax()[0]))
        spawn_y = 0.5 * (float(r.GetMin()[1]) + float(r.GetMax()[1]))
        print(f"[spawn] --asset-center requested -> using center ({spawn_x:.3f}, {spawn_y:.3f})")

prim_path = spawn_asset(
    stage,
    usd_path=args.asset_usd,
    name=args.asset_name,
    x=spawn_x, y=spawn_y, yaw_deg=args.asset_yaw,
    drop_margin=args.drop_margin,
    terrain_path="/World/Terrain",
    orient_to_slope=True,
)

from isaacsim.core.api import SimulationContext
sim = SimulationContext()
sim.set_simulation_dt(1.0/60.0)
sim.play()

# small warmup so the referenced USD is fully ready
for _ in range(30):
    sim.step(render=False)

from locomotion import drive_diff

drive = drive_diff.run(
    stage,
    prim_hint=f"/World/{args.asset_name}",  # e.g. /World/jackal  (will auto-resolve /payload)
    seconds=None,                           # run until you close the app
    hz=60.0,
    wheel_radius=0.098,                     # Jackal-ish
    wheel_base=0.375,                       # Jackal-ish
    # left_key="left", right_key="right",   # override for other robots if names differ
)

# main loop: tick the driver and the sim
while simulation_app.is_running():
    try:
        next(drive)     # advance one control step
    except StopIteration:
        break
    sim.step(render=True)

simulation_app.close()
