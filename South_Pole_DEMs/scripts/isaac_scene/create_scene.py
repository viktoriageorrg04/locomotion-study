#!/usr/bin/env python3
"""
Create a terrain USD(.usda) from an *_enhanced.tif height patch.

- Reads the TIF with tifffile, Pillow, or imageio (first available).
- Authors a /World/Terrain UsdGeom.Mesh with quads.
- XY is centered at (0,0) so --asset-center lands your robot in the middle.
- Z can be 'relative' (subtract min), 'absolute' (use as-is), or 'center' (subtract mean).
- Use --stride >1 to decimate for huge tiles (memory saver).
"""

import sys, argparse
from pathlib import Path

import numpy as np

# ---- flexible TIF loader (no rasterio dependency) ----
def load_tif(path: Path) -> np.ndarray:
    last_err = None
    for loader in ("tifffile", "PIL", "imageio"):
        try:
            if loader == "tifffile":
                import tifffile as tiff
                arr = tiff.imread(str(path))
            elif loader == "PIL":
                from PIL import Image
                with Image.open(str(path)) as im:
                    arr = np.array(im)
            else:
                import imageio.v3 as iio
                arr = iio.imread(str(path))
            if arr.ndim == 3:
                arr = arr[..., 0]  # take first channel if RGB(A)
            return arr.astype(np.float32)
        except Exception as e:
            last_err = e
    raise RuntimeError(f"Unable to read TIF '{path}': {last_err}")

def main():
    ap = argparse.ArgumentParser("Create terrain USD from *_enhanced.tif")
    ap.add_argument("--tif", required=True, help="Path to *_enhanced.tif")
    ap.add_argument("--out", required=True, help="Output USD/USDA path, e.g. South_Pole_DEMs/flat_terrain.usda")
    ap.add_argument("--mpp", type=float, default=5.0, help="meters-per-pixel (defaults to 5.0)")
    ap.add_argument("--stride", type=int, default=1, help="take every Nth pixel to decimate (1 = full res)")
    ap.add_argument("--z-mode", choices=["relative","absolute","center"], default="relative",
                    help="relative: subtract min; absolute: as-is; center: subtract mean")
    ap.add_argument("--z-scale", type=float, default=1.0, help="scale Z after mode handling")
    ap.add_argument("--z-offset", type=float, default=0.0, help="add constant meters to Z after scaling")
    args = ap.parse_args()

    tif_path = Path(args.tif).resolve()
    out_path = Path(args.out).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if not tif_path.exists():
        print(f"[create_scene] ERROR: TIF not found: {tif_path}", file=sys.stderr)
        return 2

    # ---- load heightmap ----
    h = load_tif(tif_path)
    if args.stride > 1:
        h = h[::args.stride, ::args.stride]
        mpp = args.mpp * args.stride
    else:
        mpp = args.mpp

    # Z handling
    if args.z_mode == "relative":
        h = h - np.nanmin(h)
    elif args.z_mode == "center":
        h = h - float(np.nanmean(h))
    # else 'absolute' leaves as-is

    h = h * args.z_scale + args.z_offset
    h = np.nan_to_num(h, copy=False)

    ny, nx = h.shape
    print(f"[create_scene] size: {nx} x {ny} pixels | mpp={mpp} | verts={nx*ny:,} | quads={(nx-1)*(ny-1):,}")

    # ---- build grid vertices (center XY at 0,0) ----
    xs = (np.arange(nx, dtype=np.float32) - (nx - 1) * 0.5) * mpp
    ys = (np.arange(ny, dtype=np.float32) - (ny - 1) * 0.5) * mpp
    X, Y = np.meshgrid(xs, ys)
    V = np.stack([X, Y, h], axis=-1).reshape(-1, 3)  # (N,3)

    # faces as quads
    i = np.arange(nx - 1, dtype=np.int64)
    j = np.arange(ny - 1, dtype=np.int64)
    J, I = np.meshgrid(j, i, indexing="ij")
    v00 = J * nx + I
    v10 = v00 + 1
    v11 = v00 + 1 + nx
    v01 = v00 + nx
    quads = np.stack([v00, v10, v11, v01], axis=-1).reshape(-1)
    counts = np.full((nx - 1) * (ny - 1), 4, dtype=np.int64)

    # ---- author USD ----
    sim_app = None
    try:
        from pxr import Usd, UsdGeom, Sdf, Gf
    except ModuleNotFoundError:
        from isaacsim.simulation_app import SimulationApp
        sim_app = SimulationApp({"headless": True, "noWindow": True})
        from pxr import Usd, UsdGeom, Sdf, Gf

    # choose ASCII if extension is .usda
    stage = Usd.Stage.CreateNew(str(out_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    world = UsdGeom.Xform.Define(stage, "/World")
    terrain = UsdGeom.Mesh.Define(stage, "/World/Terrain")
    terrain.CreatePointsAttr([Gf.Vec3f(float(x), float(y), float(z)) for (x, y, z) in V.tolist()])
    terrain.CreateFaceVertexCountsAttr(counts.tolist())
    terrain.CreateFaceVertexIndicesAttr(quads.tolist())
    terrain.CreateDoubleSidedAttr(True)

    # nice-to-have: extent (bbox)
    min_xyz = V.min(axis=0).tolist()
    max_xyz = V.max(axis=0).tolist()
    min_xyz = [float(min_xyz[0]), float(min_xyz[1]), float(min_xyz[2])]
    max_xyz = [float(max_xyz[0]), float(max_xyz[1]), float(max_xyz[2])]
    terrain.CreateExtentAttr([Gf.Vec3f(*min_xyz), Gf.Vec3f(*max_xyz)])

    # name nodes
    world_prim = world.GetPrim()
    stage.SetDefaultPrim(world_prim)
    stage.GetRootLayer().Save()

    print(f"[create_scene] Wrote: {out_path}")
    print("[create_scene] Prim: /World/Terrain (Mesh, quads)")
    return 0

if __name__ == "__main__":
    sys.exit(main())

    if sim_app is not None:
        sim_app.close()
        