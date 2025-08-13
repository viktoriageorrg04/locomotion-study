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
ap.add_argument("--sun-exposure", type=float, default=0.0, help="UsdLux exposure (EV offset)")

# Material knobs (applied to /World/Terrain if not skipped)
ap.add_argument("--skip-material", action="store_true",
                help="Do NOT rebind material on /World/Terrain (preserve the USD's original material).")
ap.add_argument("--roughness", type=float, default=0.9, help="Force material roughness (0-1). If <0, leave as-authored")
ap.add_argument("--metallic", type=float, default=0.0, help="Force material metallic (0-1). If <0, leave as-authored")
ap.add_argument("--normal-scale", type=float, default=2.0, help="Normal map scale if a normal is used")
ap.add_argument("--use-ao", action="store_true", help="If an AO/ORD map exists, wire its R channel to occlusion.")
ap.add_argument("--albedo", type=float, default=0.12, help="Fallback diffuse value if no albedo texture is found")

# Path Tracing quality helpers (safe defaults; ignored on RTL)
ap.add_argument("--pt-spp", type=int, default=128, help="Path Tracing samples per pixel")
ap.add_argument("--pt-max-bounces", type=int, default=6, help="Path Tracing max bounces")

ap.add_argument("--no-normal", action="store_true", help="Disable normal map even if present")

args = ap.parse_args()
print(f"[launch_isaac] Args: {args}")

simulation_app = SimulationApp({"renderer": args.renderer, "headless": args.headless})

# Import after SimulationApp
import carb
import omni.usd
from pxr import Usd, UsdGeom, UsdLux, UsdShade, Gf, Sdf

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
# Path Tracing tuning
# -------------------------------

def configure_path_tracing():
    if args.renderer.lower() != "pathtracing":
        return
    s = carb.settings.get_settings()
    # More SPP and sane bounces help reduce speckling at low sun angles
    s.set("/rtx/pathtracing/spp", int(args.pt_spp))
    s.set("/rtx/pathtracing/maxBounces", int(args.pt_max_bounces))
    # A little safety: ensure mesh lights aren’t forced off/on unexpectedly
    s.set("/rtx/sceneDb/meshlights/forceDisable", False)

# -------------------------------
# Material (safe & correct wiring)
# -------------------------------

def force_preview_surface_on_prim(prim_path: str, albedo_tex: Path|None=None,
                                  normal_tex: Path|None=None, ao_tex: Path|None=None):
    st = get_stage()
    mesh = st.GetPrimAtPath(prim_path)
    if not mesh.IsValid():
        print(f"[material][WARN] Prim not found: {prim_path}")
        return False

    ensure_scope("/World/Looks")
    mat_path = Sdf.Path("/World/Looks/Regolith")
    if not st.GetPrimAtPath(mat_path).IsValid():
        st.DefinePrim(mat_path, "Material")
    material = UsdShade.Material.Define(st, mat_path)
    shader = UsdShade.Shader.Define(st, mat_path.AppendPath("Preview"))
    shader.CreateIdAttr("UsdPreviewSurface")

    # metallic/roughness
    if args.metallic >= 0:
        shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(float(args.metallic))
    if args.roughness >= 0:
        shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(args.roughness))

    # ST primvar
    st_reader = UsdShade.Shader.Define(st, mat_path.AppendPath("PrimvarST"))
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_out = st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    # Albedo / diffuse
    if albedo_tex and Path(albedo_tex).exists():
        try:
            albedo_node = UsdShade.Shader.Define(st, mat_path.AppendPath("AlbedoTex"))
            albedo_node.CreateIdAttr("UsdUVTexture")
            albedo_node.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(albedo_tex))
            albedo_node.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            albedo_node.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)\
                  .ConnectToSource(albedo_node.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
        except Exception as e:
            print(f"[material][ERROR] Failed to load albedo texture: {e}")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(args.albedo, args.albedo, args.albedo)
            )
            print("[material] Fallback: using flat diffuse based on lunar albedo")
    else:
        shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
            Gf.Vec3f(args.albedo, args.albedo, args.albedo)
        )
        print("[material] No albedo texture; using flat diffuse based on lunar albedo")

    # Normal map (correct: UsdNormalMap). No duplicate/ direct RGB hookup.
    if normal_tex and Path(normal_tex).exists():
        try:
            ntex = UsdShade.Shader.Define(st, mat_path.AppendPath("NormalTex"))
            ntex.CreateIdAttr("UsdUVTexture")
            ntex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(normal_tex))
            ntex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
            ntex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)

            nmap = UsdShade.Shader.Define(st, mat_path.AppendPath("NormalMap"))
            nmap.CreateIdAttr("UsdNormalMap")
            nmap.CreateInput("in", Sdf.ValueTypeNames.Float3)\
                .ConnectToSource(ntex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
            nmap.CreateInput("scale", Sdf.ValueTypeNames.Float).Set(float(args.normal_scale))

            shader.CreateInput("normal", Sdf.ValueTypeNames.Normal3f)\
                .ConnectToSource(ntex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
        except Exception as e:
            print(f"[material][ERROR] Failed to load normal map: {e}")
    else:
        print("[material] No normal texture provided")

    # Optional AO (R channel of AO/ORD map) to 'occlusion'
    if args.use_ao and ao_tex and Path(ao_tex).exists():
        aot = UsdShade.Shader.Define(st, mat_path.AppendPath("AO_Tex"))
        aot.CreateIdAttr("UsdUVTexture")
        aot.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(ao_tex))
        aot.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
        aot.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
        shader.CreateInput("occlusion", Sdf.ValueTypeNames.Float)\
            .ConnectToSource(aot.CreateOutput("r", Sdf.ValueTypeNames.Float))

    # Bind
    surface_output = material.CreateSurfaceOutput()
    shader_surface_output = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    surface_output.ConnectToSource(shader_surface_output)
    UsdShade.MaterialBindingAPI(mesh).Bind(material)
    print(f"[launch_isaac] Bound PreviewSurface to {prim_path} (roughness={args.roughness}, metallic={args.metallic})")
    return True

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

stage_path = str(Path(args.usd).expanduser().resolve())
open_stage(stage_path)

# Lighting mode
if args.viewport == "stage":
    disable_viewport_light_rig()
else:
    print("[lighting] Using DEFAULT viewport light rig (not lunar realistic).")

if args.black_sky:
    disable_dome_and_skylights()

# South-pole: low sun; use exposure to control brightness
set_sun(args.sun_az, args.sun_el, args.sun_intensity, args.sun_exposure)

# Path tracing quality
configure_path_tracing()

# Material on /World/Terrain if present (unless user wants to preserve original)
if not args.skip_material:
    root = Path(stage_path).parent
    albedo = root / "assets" / "textures" / "regolith" / "T_Dry_Sand_combined_equal_4K_D.png"
    normal = root / "assets" / "textures" / "regolith" / "T_Dry_Sand_combined_equal_4K_N.png"
    # Try to find an AO/ORD map next to them
    ao     = root / "assets" / "textures" / "regolith" / "T_Dry_Sand_combined_equal_4K_ORD.png"

    print(f"[set_regolith] Albedo: file={albedo} exists={albedo.exists()}")
    print(f"[set_regolith] Normal: file={normal} exists={normal.exists()}")
    if args.use_ao:
        print(f"[set_regolith] AO/ORD: file={ao} exists={ao.exists()}")

    _ = force_preview_surface_on_prim(
        "/World/Terrain",
        albedo_tex=albedo if albedo.exists() else None,
        normal_tex=None if args.no_normal else (normal if normal.exists() else None),
        ao_tex=ao if (args.use_ao and ao.exists()) else None
    )
    print(f"[launch_isaac] Material summary: "
          f"Albedo={'YES' if albedo.exists() else 'NO'}, "
          f"Normal={'YES' if (not args.no_normal and normal.exists()) else 'NO'}, "
          f"AO={'YES' if (args.use_ao and ao.exists()) else 'NO'}")
else:
    print("[material] --skip-material set: preserving original material on /World/Terrain")

framed = frame_view_on_prim("/World/Terrain")
print(f"[launch_isaac] View framed: {framed}")
print("[launch_isaac] Ready.")

# -------------------------------
# Sim loop
# -------------------------------

while simulation_app.is_running():
    simulation_app.update()

simulation_app.close()
