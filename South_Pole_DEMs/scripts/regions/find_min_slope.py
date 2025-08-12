import os, sys
import numpy as np
import os, sys, argparse, numpy as np

# insert project root into PYTHONPATH so we can import scripts.regions.find_promising
root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
if root not in sys.path:
    sys.path.insert(0, root)

from scripts.regions.find_promising import compute_max_slope

def compute_min_slope(dem_path: str, patch_size: int) -> float:
    dem = np.load(dem_path)
    h, w = dem.shape
    min_s = float("inf")
    for yi in range(0, h - patch_size + 1, patch_size):
        for xi in range(0, w - patch_size + 1, patch_size):
            s = compute_max_slope(dem[yi:yi+patch_size, xi:xi+patch_size])
            min_s = min(min_s, s)
    return min_s

if __name__ == "__main__":
    p = argparse.ArgumentParser(description="Compute min slope over DEM tiles")
    p.add_argument("-d", "--dem_path", required=True,
                   help="Path to dem.npy file")
    p.add_argument("-p", "--patch_size", type=int, default=256,
                   help="Square tile size")
    args = p.parse_args()

    result = compute_min_slope(args.dem_path, args.patch_size)
    name = os.path.basename(os.path.dirname(args.dem_path))
    print(f"{name}: minimum mask_slope = {result:.3f}")