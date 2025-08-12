#!/usr/bin/env python3
from __future__ import annotations
import os, argparse
import numpy as np
import rasterio
from pxr import Usd, UsdGeom, Gf

try:
    from PIL import Image
    HAVE_PIL = True
except Exception:
    HAVE_PIL = False


def read_dem(path: str):
    if path.lower().endswith(".npy"):
        dem = np.load(path).astype(np.float32)
        xres = yres = 1.0
        print(f"[DEBUG] Loaded NPY heightmap: shape={dem.shape}, dtype=float32, assuming 1.0 m/px")
    else:
        with rasterio.open(path) as src:
            dem = src.read(1).astype(np.float32)
            xr, yr = src.res  # yr may be negative in GeoTIFFs
            print(f"[DEBUG] Rasterio read: shape={dem.shape}, dtype=float32, raw pixel size=({xr},{yr})")
            xres, yres = float(abs(xr)), float(abs(yr))
    return dem, xres, yres


def grid_indices(w: int, h: int):
    counts, indices = [], []
    for y in range(h - 1):
        row = y * w
        nxt = (y + 1) * w
        for x in range(w - 1):
            a = row + x; b = a + 1; c = nxt + x; d = c + 1
            counts.extend([3, 3])
            indices.extend([a, c, b,  b, c, d])
    return counts, indices


def compute_vertex_normals(z2d: np.ndarray, xres: float, yres: float):
    # gradients wrt meters
    gy, gx = np.gradient(z2d.astype(np.float32), yres, xres)
    nx = -gx; ny = -gy; nz = np.ones_like(z2d, dtype=np.float32)
    n = np.stack([nx, ny, nz], axis=-1)
    # normalize
    l = np.linalg.norm(n, axis=-1, keepdims=True)
    n = n / np.maximum(l, 1e-12)
    return n.reshape(-1, 3).astype(np.float32)


def build_points(dem: np.ndarray, xres: float, yres: float,
                 z_mode: str, z_exag: float, center: bool,
                 stride: int, flip_y: bool):
    H, W = dem.shape
    nan_cnt = int(np.isnan(dem).sum())
    print(f"[DEBUG] DEM nan count: {nan_cnt}")

    # stride/decimate
    if stride > 1:
        dem = dem[::stride, ::stride]
        xres *= stride; yres *= stride
        H, W = dem.shape
        print(f"[DEBUG] After stride={stride}: shape={H}x{W}, pixel size=({xres},{yres})")

    # fill NaNs with local min (conservative)
    if nan_cnt:
        vmin = float(np.nanmin(dem))
        dem = np.where(np.isfinite(dem), dem, vmin)
        print(f"[DEBUG] Filled NaNs with {vmin:.3f}")

    mn, mx = float(dem.min()), float(dem.max())
    rng = mx - mn
    print(f"[DEBUG] RAW Z stats (meters): min={mn:.3f}, max={mx:.3f}, range={rng:.3f}")

    # Z transform
    if z_mode == "relative":
        base = mn
        z = dem - base
        z_info = f"relative (base={base:.3f})"
    elif z_mode == "absolute":
        z = dem.copy()
        z_info = "absolute"
    else:
        z = (dem - mn) / max(rng, 1e-6)
        z_info = "normalize [0..1]"

    z *= float(z_exag)
    zmn, zmx = float(z.min()), float(z.max())
    print(f"[DEBUG] FINAL Z after {z_info}, exag={z_exag}: min={zmn:.3f}, max={zmx:.3f}, range={zmx-zmn:.3f}")

    # XY grid in meters
    xs = np.arange(W, dtype=np.float32) * xres
    ys = np.arange(H, dtype=np.float32) * yres
    X, Y = np.meshgrid(xs, ys)

    if center:
        X -= xs.mean(); Y -= ys.mean()
        print("[DEBUG] Centered XY about origin.")
    if flip_y:
        Y = -Y
        print("[DEBUG] Flipped Y (north-up visual).")

    pts = np.stack([X, Y, z], axis=-1).reshape(-1, 3).astype(np.float32)

    # more diagnostics
    aabb_min, aabb_max = pts.min(axis=0), pts.max(axis=0)
    print(f"[DEBUG] Points AABB:")
    print(f"      min = [{aabb_min[0]:.3f}, {aabb_min[1]:.3f}, {aabb_min[2]:.3f}]")
    print(f"      max = [{aabb_max[0]:.3f}, {aabb_max[1]:.3f}, {aabb_max[2]:.3f}]")
    print(f"      size= [{(aabb_max-aabb_min)[0]:.3f}, {(aabb_max-aabb_min)[1]:.3f}, {(aabb_max-aabb_min)[2]:.3f}] (meters)")

    # quicklook image to sanity-check contrast
    if HAVE_PIL:
        try:
            q = ((dem - mn)/max(rng, 1e-6) * 255).astype(np.uint8)
            Image.fromarray(q).save("/tmp/height_debug.png")
            print("[DEBUG] Wrote quicklook: /tmp/height_debug.png")
        except Exception as e:
            print(f"[DEBUG] Quicklook save failed: {e}")

    # return Z as 2D for normals
    return pts, (W, H), (xres, yres), (mn, mx), z


def make_mesh(stage, mesh_path, pts, size_wh, normals=None):
    W, H = size_wh
    counts, indices = grid_indices(W, H)
    mesh = UsdGeom.Mesh.Define(stage, mesh_path)
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)

    # Vec3f wants Python floats, not numpy scalars
    pts_list = [Gf.Vec3f(float(p[0]), float(p[1]), float(p[2])) for p in pts]
    mesh.CreatePointsAttr(pts_list)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)

    if normals is not None:
        n_list = [Gf.Vec3f(float(n[0]), float(n[1]), float(n[2])) for n in normals]
        mesh.CreateNormalsAttr(n_list)
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)

    xyz_min = Gf.Vec3f(*(float(x) for x in pts.min(axis=0)))
    xyz_max = Gf.Vec3f(*(float(x) for x in pts.max(axis=0)))
    mesh.CreateExtentAttr([xyz_min, xyz_max])
    mesh.CreateDoubleSidedAttr(True)
    return mesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tif", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--z-mode", choices=["relative", "absolute", "normalize"], default="relative")
    ap.add_argument("--z-exag", type=float, default=1.0)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--no-center", action="store_true")
    ap.add_argument("--flip-y", action="store_true")
    args = ap.parse_args()

    dem, xres, yres = read_dem(args.tif)
    H, W = dem.shape
    pts, (W2, H2), (xr, yr), (mn, mx), z2d = build_points(
        dem, xres, yres,
        z_mode=args.z_mode, z_exag=args.z_exag,
        center=not args.no_center, stride=max(1, args.stride),
        flip_y=args.flip_y
    )

    normals = compute_vertex_normals(z2d, xr, yr)

    out_path = os.path.abspath(args.out)
    out_dir  = os.path.dirname(out_path) or "."
    os.makedirs(out_dir, exist_ok=True)

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    world = UsdGeom.Xform.Define(stage, "/World")
    make_mesh(stage, "/World/Terrain", pts, (W2, H2), normals=normals)
    stage.SetDefaultPrim(world.GetPrim())
    stage.GetRootLayer().Save()

    tile_w = (W2 - 1) * xr
    tile_h = (H2 - 1) * yr
    print(f"[create_scene] Tile XY = {tile_w:.3f}m x {tile_h:.3f}m; mesh = {W2}x{H2} verts")
    print(f"[create_scene] Wrote {out_path}")


if __name__ == "__main__":
    main()
