# terrain_debug.py
from pxr import UsdGeom
from spawn_rover import world_range_for_path

def _print_stage_units(stage):
    mpu = UsdGeom.GetStageMetersPerUnit(stage)
    up  = UsdGeom.GetStageUpAxis(stage)
    print(f"[units] metersPerUnit={mpu:g}  (1 USD unit = {mpu} m), upAxis={up}")

def _print_bounds(stage, prim_path):
    r = world_range_for_path(stage, prim_path)
    if not r:
        print(f"[bounds] {prim_path}: <no bounds>")
        return None
    mn, mx = r.GetMin(), r.GetMax()
    dz = float(mx[2] - mn[2])
    print(f"[bounds] {prim_path}: "
          f"min=({mn[0]:.3f},{mn[1]:.3f},{mn[2]:.3f})  "
          f"max=({mx[0]:.3f},{mx[1]:.3f},{mx[2]:.3f})  ΔZ={dz:.3f} m")
    return float(mn[2]), float(mx[2])

def print_terrain_elevation(stage, terrain_path="/World/Terrain"):
    """Print units and terrain min/max elevation (world space)."""
    _print_stage_units(stage)
    return _print_bounds(stage, terrain_path)

def debug_asset_vs_terrain(stage, asset_path, terrain_path="/World/Terrain"):
    """Print asset and terrain bounds + their vertical offset."""
    _print_stage_units(stage)
    terr = _print_bounds(stage, terrain_path)
    asset = _print_bounds(stage, asset_path)
    if terr and asset:
        terr_min, terr_max = terr
        asset_min, asset_max = asset
        print(f"[compare] asset_minZ - terrain_maxZ = {asset_min - terr_max:+.3f} m  "
              f"(>0 means hovering, <0 means interpenetrating)")
