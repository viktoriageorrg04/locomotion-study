#!/usr/bin/env python3
from __future__ import annotations

import os
from typing import Optional, Tuple

from pxr import Usd, UsdGeom, UsdShade, Gf, Sdf, Vt


def dbg(*args):
    print("[set_regolith]", *args)


def _project_root_from_stage(stage: Usd.Stage) -> str:
    layer_path = stage.GetRootLayer().realPath or stage.GetRootLayer().identifier
    return os.path.abspath(os.path.dirname(layer_path))


def _auto_texture_dir(stage: Usd.Stage) -> Optional[str]:
    root = _project_root_from_stage(stage)
    for c in [
        os.path.join(root, "assets", "textures", "regolith"),
        os.path.join(root, "assets", "regolith"),
        os.path.join(root, "textures", "regolith"),
        os.path.join(root, "regolith"),
    ]:
        if os.path.isdir(c):
            return c
    return None


def _mesh_xy_extent_and_count(mesh_prim: Usd.Prim) -> Tuple[Tuple[float, float], Tuple[float, float], int]:
    mesh = UsdGeom.Mesh(mesh_prim)
    pts = mesh.GetPointsAttr().Get() or []
    n = len(pts)
    if n == 0:
        ext = mesh.GetExtentAttr().Get()
        if not ext:
            return (0.0, 0.0), (0.0, 0.0), 0
        mn, mx = ext[0], ext[1]
        return (mn[0], mx[0]), (mn[1], mx[1]), 0
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs), max(xs)), (min(ys), max(ys)), n


def _author_st_primvar(mesh_prim: Usd.Prim, repeats_uv):
    mesh = UsdGeom.Mesh(mesh_prim)
    (xmin, xmax), (ymin, ymax), n = _mesh_xy_extent_and_count(mesh_prim)
    if n == 0:
        dbg("WARN: Could not read mesh points; skipping UV authoring.")
        return

    xr = max(xmax - xmin, 1e-6)
    yr = max(ymax - ymin, 1e-6)

    pts = mesh.GetPointsAttr().Get()
    uvs = Vt.Vec2fArray(n)
    for i, p in enumerate(pts):
        u = float((p[0] - xmin) / xr)
        v = 1.0 - float((p[1] - ymin) / yr)  # flip V (north-up)
        uvs[i] = Gf.Vec2f(u, v)

    st = UsdGeom.PrimvarsAPI(mesh_prim).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    st.Set(uvs)
    dbg(f"Authored 'st' primvar: interpolation=vertex, count={n}, s:[0..1] t:[1..0]")


def make_and_bind_regolith(stage: Usd.Stage, mesh_path: str,
                           tex_dir: Optional[str] = None,
                           tile_meters: float = 10.0,
                           verbose: bool = True) -> bool:
    mesh_prim = stage.GetPrimAtPath(mesh_path)
    if not mesh_prim or not mesh_prim.IsA(UsdGeom.Mesh):
        dbg(f"ERROR: Prim at {mesh_path} is not a Mesh.")
        return False

    mesh = UsdGeom.Mesh(mesh_prim)
    ext = mesh.GetExtentAttr().Get()
    if not ext:
        dbg("ERROR: Mesh has no extent; cannot compute tiling.")
        return False
    mn, mx = ext[0], ext[1]
    sx, sy, sz = float(mx[0] - mn[0]), float(mx[1] - mn[1]), float(mx[2] - mn[2])
    dbg(f"Terrain extent meters: sx={sx:.3f}, sy={sy:.3f}, sz={sz:.3f}")

    tile = max(float(tile_meters), 1e-6)
    rep_u, rep_v = sx / tile, sy / tile
    dbg(f"Computed UV repeats: U={rep_u:.3f}, V={rep_v:.3f}")

    _author_st_primvar(mesh_prim, (rep_u, rep_v))

    if not tex_dir:
        tex_dir = _auto_texture_dir(stage) or ""
    if not tex_dir:
        dbg("ERROR: Could not locate a regolith texture folder.")
        return False
    dbg(f"Using texture directory: {tex_dir}")

    f_alb = os.path.join(tex_dir, "T_Dry_Sand_combined_equal_4K_D.png")
    f_nrm = os.path.join(tex_dir, "T_Dry_Sand_combined_equal_4K_N.png")
    f_orm = os.path.join(tex_dir, "T_Dry_Sand_combined_equal_4K_ORD.png")

    def _stat(p, label, cs):
        ok = os.path.isfile(p)
        sz = os.path.getsize(p) if ok else 0
        dbg(f"{label}: file={p} exists={ok} size={sz} bytes (colorspace={cs})")
        return ok

    ok_alb = _stat(f_alb, "Albedo", "sRGB")
    ok_nrm = _stat(f_nrm, "Normal", "raw")
    ok_orm = _stat(f_orm, "ORM", "raw")

    # Nodes
    UsdGeom.Scope.Define(stage, "/World/Looks")
    mat = UsdShade.Material.Define(stage, "/World/Looks/Regolith")
    pbr = UsdShade.Shader.Define(stage, "/World/Looks/Regolith/PBR")
    pbr.CreateIdAttr("UsdPreviewSurface")

    st_reader = UsdShade.Shader.Define(stage, "/World/Looks/Regolith/stReader")
    st_reader.CreateIdAttr("UsdPrimvarReader_float2")
    st_reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    st_out = st_reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    st_xform = UsdShade.Shader.Define(stage, "/World/Looks/Regolith/UVXform")
    st_xform.CreateIdAttr("UsdTransform2d")
    st_xform.CreateInput("in", Sdf.ValueTypeNames.Float2).ConnectToSource(st_out)
    st_xform.CreateInput("scale", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(float(rep_u), float(rep_v)))
    st_xform.CreateInput("translation", Sdf.ValueTypeNames.Float2).Set(Gf.Vec2f(0.0, 0.0))
    st_xform_out = st_xform.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    alb = UsdShade.Shader.Define(stage, "/World/Looks/Regolith/AlbedoTex")
    alb.CreateIdAttr("UsdUVTexture")
    alb.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f_alb))
    alb.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_xform_out)
    alb.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    alb.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    alb.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    alb_rgb = alb.GetOutput("rgb") or alb.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    nrm = UsdShade.Shader.Define(stage, "/World/Looks/Regolith/NormalTex")
    nrm.CreateIdAttr("UsdUVTexture")
    nrm.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f_nrm))
    nrm.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_xform_out)
    nrm.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    nrm.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    nrm.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
    nrm_rgb = nrm.GetOutput("rgb") or nrm.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    orm = UsdShade.Shader.Define(stage, "/World/Looks/Regolith/ORMTex")
    orm.CreateIdAttr("UsdUVTexture")
    orm.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(f_orm))
    orm.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st_xform_out)
    orm.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    orm.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    orm.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("raw")
    orm_r = orm.GetOutput("r") or orm.CreateOutput("r", Sdf.ValueTypeNames.Float)
    orm_g = orm.GetOutput("g") or orm.CreateOutput("g", Sdf.ValueTypeNames.Float)
    # orm_b = orm.GetOutput("b") or orm.CreateOutput("b", Sdf.ValueTypeNames.Float)  # unused

    pbr_dc    = pbr.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)
    pbr_nrm   = pbr.CreateInput("normal",       Sdf.ValueTypeNames.Float3)
    pbr_rough = pbr.CreateInput("roughness",    Sdf.ValueTypeNames.Float)
    pbr_metal = pbr.CreateInput("metallic",     Sdf.ValueTypeNames.Float)
    pbr_occ   = pbr.CreateInput("occlusion",    Sdf.ValueTypeNames.Float)

    pbr_dc.ConnectToSource(alb_rgb)
    pbr_nrm.ConnectToSource(nrm_rgb)
    pbr_rough.ConnectToSource(orm_g)
    pbr_occ.ConnectToSource(orm_r)
    pbr_metal.Set(0.0)

    pbr_surface_out = pbr.GetOutput("surface") or pbr.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat.CreateSurfaceOutput().ConnectToSource(pbr_surface_out)

    UsdShade.MaterialBindingAPI(mesh_prim).Bind(mat)

    if verbose:
        dbg("Verify connections:")
        def _dump(inp): 
            if inp.HasConnectedSource():
                infos = inp.GetConnectedSources()
                for info in infos:
                    try:
                        dbg(f"   {inp.GetBaseName()} -> {info.source.GetPath()}.{info.sourceName}")
                    except Exception:
                        dbg(f"   {inp.GetBaseName()} -> <connected>")
            else:
                dbg(f"   {inp.GetBaseName()} -> <none> (value={inp.Get()})")
        for nm in ("diffuseColor","normal","roughness","metallic","occlusion"):
            _dump(pbr.GetInput(nm))

    if not (ok_alb and ok_nrm and ok_orm):
        dbg("WARNING: One or more texture files are missing. Expect flat gray or odd shading until fixed.")

    dbg("Material bound path:", UsdShade.MaterialBindingAPI(mesh_prim).GetDirectBinding().GetMaterialPath())
    dbg("Done.")
    return True
