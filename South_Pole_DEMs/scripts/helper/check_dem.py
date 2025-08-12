#!/usr/bin/env python3
import sys, os, math
import numpy as np
import rasterio

def georef_info(path):
    with rasterio.open(path) as ds:
        px = abs(ds.transform.a) if ds.transform is not None else float("nan")
        crs = ds.crs.to_string() if ds.crs else "None"
        sz = (ds.width, ds.height)
        return px, crs, sz

def core_stats(arr):
    m = np.isfinite(arr)
    vals = arr[m]
    vmin, vmax = float(vals.min()), float(vals.max())
    vrng = vmax - vmin
    return vmin, vmax, vrng, m

def print_core(label, vmin, vmax, vrng):
    print(f"{label}:")
    print(f"  min/max: {vmin:.3f} {vmax:.3f} (meters)")
    print(f"  vertical range: {vrng:.3f} m")
    print(f"  mapping (norm→m): Z ≈ norm * ({vrng:.3f}) + {vmin:.3f}")

def slope_deg(z, px=5.0):
    gy, gx = np.gradient(z, px, px)
    return np.degrees(np.arctan(np.hypot(gx, gy)))

enh_path = sys.argv[1]
orig_path = enh_path.replace("_enhanced", "")

# Georef peek (from enhanced)
px, crs, sz = georef_info(enh_path)
print(f"FILE: {enh_path}")
print(f"  size: {sz[0]}x{sz[1]} px  |  pixel size: {px:.6g} m  |  CRS: {crs}")
if not math.isfinite(px) or abs(px - 5.0) > 1e-3:
    print("  [WARN] pixel size not ~5 m or missing transform")

# Load enhanced
with rasterio.open(enh_path) as e:
    E = e.read(1).astype(np.float32)

if os.path.exists(orig_path):
    with rasterio.open(orig_path) as o:
        O = o.read(1).astype(np.float32)
else:
    O = None
    print("Original DEM not found for this patch.")

# Basic stats
evmin, evmax, evrng, Emask = core_stats(E)
print_core("ENHANCED", evmin, evmax, evrng)

if O is not None:
    ovmin, ovmax, ovrng, Omask = core_stats(O)
    print_core("ORIGINAL", ovmin, ovmax, ovrng)

    # Deltas
    M = np.isfinite(O) & np.isfinite(E)
    d = (E - O)[M]
    mean_d = float(d.mean()) if d.size else float("nan")
    rmse = float(np.sqrt((d**2).mean())) if d.size else float("nan")
    pct = (rmse / (ovrng + 1e-9)) * 100.0 if math.isfinite(rmse) else float("nan")
    print(f"Δ vs original: mean={mean_d:.3f} m, RMSE={rmse:.3f} m ({pct:.1f}% of range)")

    # Slopes (use georef pixel size if valid)
    px_use = px if math.isfinite(px) and px > 0 else 5.0
    s0, s1 = slope_deg(O, px_use), slope_deg(E, px_use)
    p50_0, p50_1 = np.nanpercentile(s0, 50), np.nanpercentile(s1, 50)
    p90_0, p90_1 = np.nanpercentile(s0, 90), np.nanpercentile(s1, 90)
    inc50 = (p50_1 - p50_0) / max(1e-9, p50_0) * 100.0 if p50_0 else float("inf")
    inc90 = (p90_1 - p90_0) / max(1e-9, p90_0) * 100.0 if p90_0 else float("inf")
    print(f"Slope p50: {p50_0:.2f} → {p50_1:.2f} deg ({inc50:+.1f}%)")
    print(f"Slope p90: {p90_0:.2f} → {p90_1:.2f} deg ({inc90:+.1f}%)")

    # NoData & clipping (check enhanced against original bounds)
    nodata_orig = int((~np.isfinite(O)).sum())
    nodata_enh  = int((~np.isfinite(E)).sum())
    print(f"NoData count: orig={nodata_orig}, enh={nodata_enh}")
    clip_lo = int((E <= ovmin + 1e-6).sum())
    clip_hi = int((E >= ovmax - 1e-6).sum())
    tot = E.size
    print(f"Clipping @orig bounds: low={clip_lo} ({clip_lo/tot*100:.3f}%), "
          f"high={clip_hi} ({clip_hi/tot*100:.3f}%)")
