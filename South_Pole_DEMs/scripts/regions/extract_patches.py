import os
import argparse
import json
import numpy as np
from PIL import Image

def parse_args():
    p = argparse.ArgumentParser(
        description="Extract one or many patches from a DEM .npy as 32-bit PNGs")
    script_dir = os.path.dirname(__file__)
    default_dem = os.path.abspath(os.path.join(
        script_dir, "..", "..", "dem_processed", "ldem_87s_5mpp", "dem.npy"))
    p.add_argument("--dem", default=default_dem,
                   help="Path to the DEM .npy file")
    grp = p.add_mutually_exclusive_group(required=False)
    grp.add_argument("--regions", help="JSON file containing an array of {x,y} objects")
    grp.add_argument("--x", type=int, help="X offset for a single patch")
    p.add_argument("--y", type=int, help="Y offset (required with --x)")
    p.add_argument("--width",  type=int, default=256, help="Patch width in pixels")
    p.add_argument("--height", type=int, default=256, help="Patch height in pixels")
    p.add_argument("--output", default="blender/patch_default.png",
                   help="Output PNG path for single patch")
    p.add_argument("--output_dir", default="blender", 
                   help="Directory to write patches when using --regions")

    args = p.parse_args()

    # auto-load regions.json if neither --regions nor --x was given
    if args.regions is None and args.x is None:
        default_regions = os.path.join(os.getcwd(), "regions.json")
        if os.path.exists(default_regions):
            args.regions = default_regions
        else:
            p.error("one of --regions or --x is required (or drop regions.json in project root)")

    if args.x is not None and args.y is None:
        p.error("--y is required when --x is used")

    return args


def extract_and_save(dem, x, y, w, h, outpath, bit_depth=32, keep_meters=True):
    patch = dem[y:y+h, x:x+w].astype(np.float32)
    if not np.isfinite(patch).any():
        raise ValueError("Empty or NaN patch")
    # Fill NaNs with local median (don’t change scale)
    med = float(np.nanmedian(patch[np.isfinite(patch)]))
    patch = np.where(np.isfinite(patch), patch, med)

    os.makedirs(os.path.dirname(outpath) or ".", exist_ok=True)

    if keep_meters and bit_depth == 32:
        # Write float32 TIFF **without normalization**
        try:
            import tifffile
        except ImportError:
            raise RuntimeError("pip install tifffile")
        out_tif = os.path.splitext(outpath)[0] + ".tif"
        tifffile.imwrite(out_tif, patch, dtype=np.float32)
        print(f"[OK] wrote meters (float32) → {out_tif}")
        return

    # Otherwise (e.g., for Blender PNG), normalize to 16-bit *and* save metadata
    vmin, vmax = float(np.nanmin(patch)), float(np.nanmax(patch))
    z01 = (patch - vmin) / max(1e-12, (vmax - vmin))
    z16 = np.clip(np.round(z01 * 65535.0), 0, 65535).astype(np.uint16)
    out_png = os.path.splitext(outpath)[0] + ".png"
    Image.fromarray(z16, mode="I;16").save(out_png)

    # optional sidecar meta so you can recover meters later:
    with open(os.path.splitext(out_png)[0] + ".json", "w") as f:
        json.dump({"min_m": vmin, "max_m": vmax}, f, indent=2)
    print(f"[OK] wrote normalized PNG + meta → {out_png}") 


def main():
    args = parse_args()
    print(f"[DEBUG] Parsed args: {args}")
    dem = np.load(args.dem)
    print(f"[DEBUG] Loaded DEM shape={dem.shape}, dtype={dem.dtype}")

    if args.regions:
        regions = json.load(open(args.regions))

        for r in regions:
            out = os.path.join(
                args.output_dir,
                f"x{r['x']}y{r['y']}.png"
            )
            extract_and_save(
                dem, r['x'], r['y'],
                args.width, args.height,
                out
            )


if __name__ == "__main__":
    main()