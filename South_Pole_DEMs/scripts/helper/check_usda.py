#!/usr/bin/env python3
import sys
from pxr import Usd, UsdGeom, Gf

def find_heightfield_or_mesh(stage):
    for prim in stage.Traverse():
        if prim.GetTypeName() == "HeightField":
            return prim
        if prim.GetTypeName() == "Mesh":
            return prim
    return None

def get_elevations(prim):
    if prim.GetTypeName() == "HeightField":
        hf = UsdGeom.HeightField(prim)
        arr = hf.GetHeightAttr().Get()
        return arr.flatten() if hasattr(arr, "flatten") else arr
    elif prim.GetTypeName() == "Mesh":
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        return [p[2] for p in points]  # Z values
    return []

def main(path):
    stage = Usd.Stage.Open(path)
    prim = find_heightfield_or_mesh(stage)
    if not prim:
        print("No HeightField or Mesh found in USD file.")
        return
    elevations = get_elevations(prim)
    if not elevations:
        print("No elevation data found.")
        return
    vmin, vmax = min(elevations), max(elevations)
    vrng = vmax - vmin
    print(f"Elevation stats for {prim.GetPath()}:")
    print(f"  min: {vmin:.3f} m")
    print(f"  max: {vmax:.3f} m")
    print(f"  range: {vrng:.3f} m")
    print(f"  count: {len(elevations)} values")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: check_usd_elevation.py <file.usda/usd>")
        sys.exit(1)
    main(sys.argv[1])