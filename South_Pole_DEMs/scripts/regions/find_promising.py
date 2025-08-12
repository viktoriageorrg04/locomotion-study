import os
import json
import argparse
import numpy as np


def parse_args():
    script_dir  = os.path.dirname(__file__)
    default_dem = os.path.abspath(os.path.join(
        script_dir, "..", "..", "dem_processed",
        "ldem_87s_5mpp", "dem.npy"))

    p = argparse.ArgumentParser(
        description="Find broadly‐level patches by masking on slope, ranking on elevation range")
    p.add_argument("--dem",       default=default_dem,
                   help="Path to DEM .npy")
    p.add_argument("--patch_w",   type=int, default=256,
                   help="Patch width in pixels")
    p.add_argument("--patch_h",   type=int, default=256,
                   help="Patch height in pixels")
    p.add_argument("--stride_x",  type=int, default=None,
                   help="Horizontal stride; defaults to patch_w")
    p.add_argument("--stride_y",  type=int, default=None,
                   help="Vertical stride; defaults to patch_h")
    p.add_argument("--mask_slope",type=float, default=1.5,
                   help="Max allowed cell‐to‐cell slope (m/pixel)")
    p.add_argument("--top_n",     type=int,   default=5,
                   help="How many top patches to report")
    p.add_argument("--output",    default="regions.json",
                   help="Output JSON for candidate patches")
    p.add_argument("--auto", action="store_true",
                   help="Auto-tune mask_slope to minimize elevation range")
    p.add_argument("--slope_max", type=float, default=3.0,
                   help="Upper bound on mask_slope for auto‐tune search")
    p.add_argument("--max_range", type=float, default=100.0,
                    help="Max allowed elevation range per patch (m)")

    args = p.parse_args()
    args.stride_x = args.stride_x or args.patch_w
    args.stride_y = args.stride_y or args.patch_h
    print(f"[DEBUG] Parsed args: {args}")
    return args


def compute_max_slope(patch):
    # neighbor diffs
    dx = np.abs(np.diff(patch, axis=1))
    dy = np.abs(np.diff(patch, axis=0))
    # pad back
    dx = np.pad(dx, ((0,0),(0,1)), constant_values=0)
    dy = np.pad(dy, ((0,1),(0,0)), constant_values=0)
    slope = np.maximum(dx, dy)
    return float(slope.max())


def evaluate_range(dem, args, mask_slope):
    """Return the smallest elev_range for any patch passing mask_slope."""
    h,w = dem.shape
    best_range = float("inf")
    for yi in range(0, h - args.patch_h + 1, args.stride_y):
        for xi in range(0, w - args.patch_w + 1, args.stride_x):
            patch = dem[yi:yi+args.patch_h, xi:xi+args.patch_w]
            if np.isnan(patch).any():
                continue
            if compute_max_slope(patch) > mask_slope:
                continue
            mn, mx = float(np.nanmin(patch)), float(np.nanmax(patch))
            rng = mx - mn
            if rng < best_range:
                best_range = rng
    return best_range if best_range < float("inf") else None


def main(args):
    # 1) load DEM
    dem = np.load(args.dem)
    print(f"[DEBUG] Loaded DEM shape={dem.shape}, dtype={dem.dtype}")

    h, w = dem.shape
    candidates = []

    # 2) scan every patch
    for yi in range(0, h - args.patch_h + 1, args.stride_y):
        for xi in range(0, w - args.patch_w + 1, args.stride_x):
            patch = dem[yi:yi+args.patch_h, xi:xi+args.patch_w]
            if np.isnan(patch).any():
                continue

            # mask by slope
            max_s = compute_max_slope(patch)
            if max_s > args.mask_slope:
                continue

            # rank by elevation range + compute slope‐roughness
            mn, mx = float(np.nanmin(patch)), float(np.nanmax(patch))
            if mx - mn > args.max_range:
                continue
            # compute cell‐to‐cell slope array
            dx = np.abs(np.diff(patch, axis=1))
            dy = np.abs(np.diff(patch, axis=0))
            slope_arr = np.maximum(
                np.pad(dx, ((0,0),(0,1)), constant_values=0),
                np.pad(dy, ((0,1),(0,0)), constant_values=0),
            )
            roughness = float(np.std(slope_arr))
            candidates.append({
                "x": xi, "y": yi,
                "roughness":   roughness,
                "elev_range":  mx - mn,
                "max_slope":   max_s
            })

    print(f"[DEBUG] Kept {len(candidates)} patches with slope ≤ {args.mask_slope}")

    if not candidates:
        print(f"[ERROR] No patches found with slope ≤ {args.mask_slope}.")
        print("Try increasing --mask_slope, widening your stride, or using --auto")
        return

    # 3) sort and pick top_n by smallest range
    best = sorted(candidates,
                  key=lambda r: (r["roughness"], r["elev_range"])
                 )[: args.top_n]

    # 4) write JSON and print best patch
    with open(args.output, "w") as fp:
        json.dump(best, fp, indent=2)

    b = best[0]
    print(f"[BEST PATCH @({b['x']},{b['y']})] elev_range={b['elev_range']:.3f}, max_slope={b['max_slope']:.3f}")
    print(f"[DEBUG] Written top {len(best)} patches to {args.output}")


def refine_auto_tune_topn(args, slope_steps=21):
    """
    1D grid‐search on mask_slope to minimize the average elev_range
    of the top-N flattest patches.
    """
    dem = np.load(args.dem)
    h,w = dem.shape

    best_score = float("inf")
    best_s     = None

    slopes = np.linspace(0.0, args.slope_max, slope_steps)

    for s in slopes:
        rngs = []
        # collect all ranges for patches passing slope mask
        for yi in range(0, h - args.patch_h + 1, args.stride_y):
            for xi in range(0, w - args.patch_w + 1, args.stride_x):
                patch = dem[yi:yi+args.patch_h, xi:xi+args.patch_w]
                if np.isnan(patch).any():
                    continue
                if compute_max_slope(patch) > s:
                    continue
                mn, mx = float(np.nanmin(patch)), float(np.nanmax(patch))
                rngs.append(mx - mn)

        if len(rngs) < args.top_n:
            continue

        rngs.sort()
        score = sum(rngs[: args.top_n]) / args.top_n

        if score < best_score:
            best_score, best_s = score, s

    if best_s is None:
        print("[WARN] no slope threshold yielded ≥ top_n patches – please relax slope_max or stride.")
        return

    print(f"[TUNED] mask_slope={best_s:.3f} → avg_top{args.top_n}_range={best_score:.3f}m")
    args.mask_slope = best_s
    main(args)


# in your __main__ block, replace refine_auto_tune(args) with:
if __name__ == "__main__":
    args = parse_args()
    if args.auto:
        refine_auto_tune_topn(args)
    else:
        main(args)