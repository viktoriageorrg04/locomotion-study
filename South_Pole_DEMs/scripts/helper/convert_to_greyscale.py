"""
Convert NumPy height‐map into a grayscale image and use a Displace modifier
a. Export .npy into a 16-bit PNG (so you don’t lose precision)
b. In Blender, add a subdivided plane (resolution matching DEM)
c. Add a Displace modifier → Texture → “New” → Texture tab → set Type to “Image or Movie” and open heightmap.png
d. Tweak Strength and Midlevel until the terrain looks right
e. Scale the plane in X/Y/Z to real‐world dimensions (use your YAML for cell size and vertical units)
"""

import numpy as np
from PIL import Image

# Load NumPy heightmap and inspect
data = np.load("dem_processsed/ldem_87s_5mpp/dem.npy")
print(f"Loaded data: shape={data.shape}, dtype={data.dtype}")
nan_count = np.count_nonzero(np.isnan(data))
print(f"NaN count: {nan_count}")
if nan_count > 0:
    # replace NaNs with zeros or a small value
    data = np.nan_to_num(data, nan=0)
    print("Replaced NaNs with 0 for normalization.")

# normalize into 0–65535 range
min_val = data.min()
max_val = data.max()
print(f"Data min: {min_val}, max: {max_val}")
if max_val == min_val:
    raise ValueError("Data has constant value; cannot normalize heightmap.")
norm = ((data - min_val)/(max_val - min_val)*65535).astype(np.uint16)
Image.fromarray(norm).save("blender/heightmap.png")
