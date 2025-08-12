#!/usr/bin/env python3
import os, re, glob, argparse
import rasterio as rio
from rasterio.windows import Window
from rasterio.windows import transform as win_transform

parser = argparse.ArgumentParser(description="Fix georeferencing for DEM patches.")
parser.add_argument("--full_dem", required=True, help="Path to full DEM GeoTIFF")
parser.add_argument("--patch_dir", required=True, help="Directory containing patch TIFFs")
args = parser.parse_args()

FULL = os.path.abspath(args.full_dem)
PATCH_DIR = os.path.abspath(args.patch_dir)
rx = re.compile(r"x(\d+)y(\d+)", re.I)

with rio.open(FULL) as src:
    t0, crs = src.transform, src.crs
    for path in glob.glob(os.path.join(PATCH_DIR, "*.tif")):
        m = rx.search(os.path.basename(path))
        if not m: 
            continue
        x, y = map(int, m.groups())
        with rio.open(path, "r+") as dst:
            w, h = dst.width, dst.height
            win = Window(x, y, w, h)
            dst.transform = win_transform(win, t0)
            dst.crs = crs
            print("fixed:", path)
            
print("done.")