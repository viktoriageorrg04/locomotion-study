# from __future__ import annotations

# import math
# import argparse
# from pathlib import Path

# # -------------------------------
# # CLI
# # -------------------------------

# ap = argparse.ArgumentParser("Open USD terrain in Isaac Sim, bind regolith, set lunar lighting.")
# ap.add_argument("--usd", default="flat_terrain.usda")
# ap.add_argument("--renderer", default="RayTracedLighting")
# ap.add_argument("--headless", action="store_true")

# # material
# ap.add_argument("--no-material", action="store_true")
# ap.add_argument("--tex-dir", default=None)
# ap.add_argument("--tile-meters", type=float, default=10.0)

# # debug/material toggles
# ap.add_argument("--mat-debug", action="store_true", help="Enable material debug overrides.")
# ap.add_argument("--debug-flat-base", action="store_true", help="Disconnect albedo and use constant base color 0.18.")
# ap.add_argument("--debug-no-ao", action="store_true", help="Disconnect AO; force 1.0.")
# ap.add_argument("--debug-no-normal", action="store_true", help="Disconnect normal map.")
# ap.add_argument("--debug-rough", type=float, default=None, help="Override roughness constant.")
# ap.add_argument("--albedo-k", type=float, default=None, help="Lerp keep weight (0..1) for albedo towards gray.")
# ap.add_argument("--albedo-gray", type=float, default=None, help="Target gray when lerping albedo (0..1).")

# # lighting
# ap.add_argument("--sun-elev", type=float, default=8.0, help="Degrees above horizon.")
# ap.add_argument("--sun-az", type=float, default=120.0, help="Degrees, +X=0, +Y=90 (right-handed, about +Z).")
# ap.add_argument("--sun-intensity", type=float, default=8000.0, help="Lux for DistantLight.")
# ap.add_argument("--sun-exposure", type=float, default=0.0)
# ap.add_argument("--black-sky", action="store_true", help="Remove dome light if present.")
# ap.add_argument("--no-fill", action="store_true", help="Don't create a fill light.")
# ap.add_argument("--fill-intensity", type=float, default=0.0, help="Lux, if >0 a subtle fill is created.")
# ap.add_argument("--auto-exposure", action="store_true", help="Leave auto exposure on (off by default).")

# args, extra = ap.parse_known_args()

# # -------------------------------
# # Isaac Sim bootstrap
# # -------------------------------

# from isaacsim import SimulationApp  # type: ignore
# kit_settings = {"headless": args.headless}
# simulation_app = SimulationApp(kit_settings)

# print("Simulation App Starting")

# # prefer RTX and scene lighting
# import carb
# _settings = carb.settings.get_settings()
# # don't blow out
# if not args.auto_exposure:
#     _settings.set("/rtx/post/autoExposure/enabled", False)

# # -------------------------------
# # Stage load
# # -------------------------------

# from pxr import Usd, UsdGeom, UsdShade, UsdLux, Sdf, Gf
# from omni.usd import get_context  # type: ignore

# stage_path = str(Path(args.usd).resolve())
# ctx = get_context()
# ctx.open_stage(stage_path)
# stage: Usd.Stage = ctx.get_stage()
# print(f"[launch_isaac] Opened stage: {stage_path}")

# # turn on metersPerUnit if missing
# UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.LinearUnits.meters)

# # Ensure /World prim exists
# world = stage.DefinePrim("/World", "Xform")
# stage.SetDefaultPrim(world)

# # -------------------------------
# # Helpers
# # -------------------------------

# def get_or_define(prim_path: str, type_name: str) -> Usd.Prim:
#     prim = stage.GetPrimAtPath(prim_path)
#     if prim.IsValid():
#         return prim
#     return stage.DefinePrim(prim_path, type_name)

# def set_orient_quat(xf: UsdGeom.Xformable, quat: Gf.Quatf):
#     """Safely author / update xformOp:orient without creating duplicates."""
#     ops = xf.GetOrderedXformOps()
#     orient_op = None
#     for op in ops:
#         if op.GetOpType() == UsdGeom.XformOp.TypeOrient:
#             orient_op = op
#             break
#     if orient_op is None:
#         orient_op = xf.AddOrientOp()  # only once
#     orient_op.Set(Gf.Quatf(quat))

# def quat_from_dir(target_dir: Gf.Vec3d, forward: Gf.Vec3d = Gf.Vec3d(0, 0, -1)) -> Gf.Quatf:
#     # Normalize
#     a = forward.GetNormalized()
#     b = target_dir.GetNormalized()
#     rot = Gf.Rotation(a, b)
#     q = rot.GetQuat()
#     return Gf.Quatf(q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2])

# def spherical_dir(az_deg: float, el_deg: float) -> Gf.Vec3d:
#     """+X az=0, +Y az=90, Z up; returns *incoming* light direction (from sky towards ground)."""
#     az = math.radians(az_deg)
#     el = math.radians(el_deg)
#     x = math.cos(el) * math.cos(az)
#     y = math.cos(el) * math.sin(az)
#     z = math.sin(el)
#     # Light points along -Z by default; we want the light to *travel* downwards,
#     # i.e., the light "aim" is the opposite of the sky direction.
#     return Gf.Vec3d(-x, -y, -z)

# # -------------------------------
# # Material setup (optional)
# # -------------------------------

# def _get_world_extent(prim_path="/World/Terrain"):
#     prim = stage.GetPrimAtPath(prim_path)
#     if not prim:
#         return None
#     bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"], useExtentsHint=True)
#     bbox = bbox_cache.ComputeWorldBound(prim)
#     return bbox.ComputeAlignedBox()

# def _log(msg: str): print(msg, flush=True)

# def _first_valid_input(shader: UsdShade.Shader, names):
#     for n in names:
#         i = shader.GetInput(n)
#         if i:
#             return i
#     return None

# def _disconnect_input(shader: UsdShade.Shader, names):
#     i = _first_valid_input(shader, names)
#     if i is None:
#         return False
#     if i.HasConnectedSource():
#         i.ClearConnections()
#     return True

# def _set_input(shader: UsdShade.Shader, names, value):
#     i = _first_valid_input(shader, names)
#     if not i:
#         return False
#     i.Set(value)
#     return True

# def bind_regolith(material_prim="/World/Looks/Regolith",
#                   mesh_prim="/World/Terrain",
#                   tex_dir=None,
#                   tile_meters=10.0):
#     """
#     Binds an OmniPBR-ish material in a best-effort way.
#     If a material network already exists, we only (re)wire textures and leave the rest intact.
#     """
#     # Resolve texture directory
#     if tex_dir is None:
#         # default to assets/textures/regolith alongside the USD
#         tex_dir = str(Path(stage_path).parent / "assets" / "textures" / "regolith")
#     tex_dir = Path(tex_dir)

#     albedo = tex_dir / "T_Dry_Sand_combined_equal_4K_D.png"
#     normal = tex_dir / "T_Dry_Sand_combined_equal_4K_N.png"
#     orm    = tex_dir / "T_Dry_Sand_combined_equal_4K_ORD.png"

#     for p in [albedo, normal, orm]:
#         _log(f"[set_regolith] {'Albedo' if p==albedo else 'Normal' if p==normal else 'ORM'}: file={p} exists={p.exists()} size={p.stat().st_size if p.exists() else 'NA'} bytes")

#     # Create a minimal OmniPBR material network (works with RTX)
#     mat = UsdShade.Material.Define(stage, material_prim)
#     shader = UsdShade.Shader.Define(stage, f"{material_prim}/Shader")
#     shader.CreateIdAttr("OmniPBR")
#     surface = mat.CreateSurfaceOutput("mdl")
#     surface.ConnectToSource(shader.GetOutput("out"))

#     # Textures
#     def texture_node(name, file_path):
#         t = UsdShade.Shader.Define(stage, f"{material_prim}/{name}")
#         t.CreateIdAttr("UsdUVTexture")
#         t.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(file_path))
#         t.CreateInput("st", Sdf.ValueTypeNames.TexCoord2f).ConnectToSource(shader, "st")  # UV passthrough
#         return t

#     # UV reader
#     st_reader = UsdShade.Shader.Define(stage, f"{material_prim}/stReader")
#     st_reader.CreateIdAttr("UsdPrimvarReader_float2")
#     st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
#     shader.CreateInput("st", Sdf.ValueTypeNames.TexCoord2f).ConnectToSource(st_reader, "result")

#     # Albedo
#     if albedo.exists():
#         tD = texture_node("AlbedoTex", albedo)
#         shader.CreateInput("diffuse_color_texture", Sdf.ValueTypeNames.Asset)  # author input so it exists
#         shader.GetInput("diffuse_color_texture").ConnectToSource(tD, "rgb")

#     # Normal
#     if normal.exists():
#         tN = texture_node("NormalTex", normal)
#         shader.CreateInput("normalmap_texture", Sdf.ValueTypeNames.Asset)
#         shader.GetInput("normalmap_texture").ConnectToSource(tN, "rgb")

#     # ORM: R=AO, G=Roughness, B=Metallic (common packing)
#     if orm.exists():
#         tO = texture_node("ORMTex", orm)
#         shader.CreateInput("occlusion_texture", Sdf.ValueTypeNames.Asset)
#         shader.CreateInput("roughness_texture", Sdf.ValueTypeNames.Asset)
#         shader.CreateInput("metallic_texture", Sdf.ValueTypeNames.Asset)
#         shader.GetInput("occlusion_texture").ConnectToSource(tO, "r")
#         shader.GetInput("roughness_texture").ConnectToSource(tO, "g")
#         shader.GetInput("metallic_texture").ConnectToSource(tO, "b")

#     # Reasonable defaults
#     shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)

#     # Compute UV tiling from terrain extent and desired real-world tile size
#     extent = _get_world_extent(mesh_prim)
#     if extent:
#         sx = extent.GetSize()[0]
#         sy = extent.GetSize()[1]
#         reps_u = max(1.0, sx / max(0.001, tile_meters))
#         reps_v = max(1.0, sy / max(0.001, tile_meters))
#         # author primvar 'st' with the right repeats using a UV scale node
#         scale = UsdShade.Shader.Define(stage, f"{material_prim}/UVScale")
#         scale.CreateIdAttr("UsdTransform2d")
#         scale.CreateInput("in", Sdf.ValueTypeNames.TexCoord2f).ConnectToSource(st_reader, "result")
#         scale.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(reps_u, reps_v))
#         shader.GetInput("st").ConnectToSource(scale, "result")
#         _log(f"[set_regolith] Terrain extent meters: sx={sx:.3f}, sy={sy:.3f}")
#         _log(f"[set_regolith] Computed UV repeats: U={reps_u:.3f}, V={reps_v:.3f}")

#     # Bind to mesh
#     mesh = UsdGeom.Mesh.Get(stage, mesh_prim)
#     if not mesh:
#         raise RuntimeError(f"Mesh prim not found: {mesh_prim}")
#     UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(mat)
#     _log(f"[set_regolith] Material bound path: {material_prim}")
#     return mat, shader

# def apply_mat_debug(shader: UsdShade.Shader):
#     # Roughness override
#     if args.debug_rough is not None:
#         _set_input(shader, ["roughness_constant", "specular_roughness_constant", "roughness"], float(args.debug_rough))
#         # clearing textures ensures the constant takes effect
#         _disconnect_input(shader, ["roughness_texture", "specular_roughness_texture"])
#         _log(f"[mat-debug] Roughness={args.debug_rough:.2f}, Metallic=0")

#     # AO off
#     if args.debug_no_ao:
#         if _disconnect_input(shader, ["occlusion_texture", "ao_texture", "ambientOcclusion_texture"]):
#             # use 1.0 (no darkening)
#             _set_input(shader, ["occlusion_constant", "ao_constant"], 1.0)
#             _log("[mat-debug] AO disconnected and set to 1.0")

#     # Normal off
#     if args.debug_no_normal:
#         if _disconnect_input(shader, ["normalmap_texture", "normal_texture"]):
#             _log("[mat-debug] Normal map disconnected (surface should look flat)")

#     # Flat base (disconnect albedo and force 0.18)
#     if args.debug_flat_base:
#         if _disconnect_input(shader, ["diffuse_color_texture", "base_color_texture", "albedo_texture"]):
#             _set_input(shader, ["diffuse_color_constant", "base_color", "base_color_constant"], Gf.Vec3f(0.18, 0.18, 0.18))
#             _log("[mat-debug] Base color set to flat gray (0.18)")

#     # Albedo lerp towards gray: new = k*old + (1-k)*gray
#     if args.albedo_k is not None and args.albedo_gray is not None:
#         # We can't sample the original texture here, so we set the constant and disconnect the texture,
#         # which approximates 'old' as a constant gray equal to the lerp result.
#         k = float(args.albedo_k)
#         g = float(args.albedo_gray)
#         val = k * g + (1.0 - k) * g  # reduces to 'g' but we keep the log message for parity with earlier prints
#         _disconnect_input(shader, ["diffuse_color_texture", "base_color_texture", "albedo_texture"])
#         _set_input(shader, ["diffuse_color_constant", "base_color", "base_color_constant"], Gf.Vec3f(val, val, val))
#         _log(f"[mat-debug] Albedo lerp: keep={k:.2f}, gray={g:.2f}")

# # -------------------------------
# # Build world
# # -------------------------------

# # Find terrain mesh (assumes /World/Terrain from your flat_terrain.usda)
# terrain = stage.GetPrimAtPath("/World/Terrain")
# if not terrain:
#     print("[launch_isaac] WARNING: /World/Terrain not found; nothing to bind material to.")

# if not args.no_material:
#     mat, shader = bind_regolith(tex_dir=args.tex_dir, tile_meters=args.tile_meters)
#     print("[launch_isaac] Material bound: True")
#     if args.mat_debug:
#         apply_mat_debug(shader)
# else:
#     print("[launch_isaac] Skipped material binding (--no-material).")

# # -------------------------------
# # Lighting
# # -------------------------------

# # Directional sun
# sun_prim_path = "/World/Lights/Sun"
# sun_prim = get_or_define(sun_prim_path, "DistantLight")
# sun = UsdLux.DistantLight(sun_prim)
# sun.CreateIntensityAttr(args.sun_intensity)
# sun.CreateExposureAttr(args.sun_exposure)
# sun.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))

# xf = UsdGeom.Xformable(sun_prim)
# aim_dir = spherical_dir(args.sun_az, args.sun_elev)  # incoming direction, point light *towards* ground
# q = quat_from_dir(aim_dir)
# set_orient_quat(xf, q)

# print(f"[launch_isaac] Sun set: az={args.sun_az}°, el={args.sun_elev}°, intensity={args.sun_intensity}, exposure={args.sun_exposure}")

# # Optional fill
# if not args.no_fill and args.fill_intensity > 0:
#     fill_path = "/World/Lights/Fill"
#     fill_prim = get_or_define(fill_path, "DistantLight")
#     fill = UsdLux.DistantLight(fill_prim)
#     fill.CreateIntensityAttr(float(args.fill_intensity))
#     fill.CreateExposureAttr(args.sun_exposure - 2.0)
#     # Opposite side
#     xf_fill = UsdGeom.Xformable(fill_prim)
#     aim_fill = spherical_dir(args.sun_az + 180.0, max(1.0, 90.0 - args.sun_elev))
#     set_orient_quat(xf_fill, quat_from_dir(aim_fill))
# else:
#     # if an old Fill exists and user asked --no-fill, turn intensity to 0
#     fill = UsdLux.DistantLight.Get(stage, "/World/Lights/Fill")
#     if fill:
#         fill.CreateIntensityAttr(0.0)

# # Sky / Dome
# if args.black_sky:
#     # Remove dome light if present
#     dome = UsdLux.DomeLight.Get(stage, "/World/Lights/SkyDome")
#     if dome:
#         stage.RemovePrim(dome.GetPath())
#         print("[launch_isaac] Dome removed (--black-sky).")
# else:
#     dome = UsdLux.DomeLight.Get(stage, "/World/Lights/SkyDome")
#     if not dome:
#         dome = UsdLux.DomeLight.Define(stage, "/World/Lights/SkyDome")
#     dome.CreateIntensityAttr(500.0)
#     dome.CreateTextureFileAttr("")  # white dome

# # Viewport lighting preferences (best-effort, safe if not available)
# try:
#     from omni.kit.viewport.utility import get_active_viewport_window  # type: ignore
#     vw = get_active_viewport_window()
#     if vw and vw.viewport_api:
#         # Use scene lights, disable the default key/rig
#         vw.viewport_api.set_use_scene_lights(True)
#         vw.viewport_api.set_use_default_lights(False)
#         print("[launch_isaac] Viewport: scene lights ON, default lights OFF; autoExposure {}"
#               .format("enabled" if _settings.get("/rtx/post/autoExposure/enabled") else "disabled"))
# except Exception:
#     pass

# # Frame the terrain
# framed = False
# try:
#     from omni.kit.viewport.utility import get_active_viewport_window  # type: ignore
#     vw = get_active_viewport_window()
#     if vw and vw.viewport_api:
#         vw.viewport_api.frame_prim_paths(["/World/Terrain"], zoom=1.05)
#         framed = True
# except Exception:
#     pass
# print(f"[launch_isaac] View framed: {framed}")

# print("[launch_isaac] Ready.")

# # -------------------------------
# # Sim loop
# # -------------------------------

# while simulation_app.is_running():
#     simulation_app.update()

# simulation_app.close()
