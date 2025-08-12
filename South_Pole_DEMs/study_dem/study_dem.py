import os
import rasterio
from PIL import Image
import numpy as np
from osgeo import gdal

def load_dem(file_path):
    print("Loading...")

    # Open the DEM file
    with rasterio.open(file_path) as dem:
        # Extract dimensions
        width = dem.width
        height = dem.height
        resolution = dem.res
        bounds = dem.bounds
        crs = dem.crs

        # Read mask (0 = nodata, 255 = data)
        mask = dem.read_masks(1)
        unique, counts = np.unique(mask, return_counts=True)
        print(f"Mask values: {dict(zip(unique.tolist(), counts.tolist()))}")

        print(f"Width: {width}, Height: {height}")
        print(f"Resolution: {resolution}")
        print(f"Bounds: {bounds}")
        # print(f"CRS: {crs}")

        # dtype and bit-depth
        dtype = dem.profile['dtype']
        bits = np.dtype(dtype).itemsize * 8
        print(f"Rasterio dtype: {dtype} → {bits}-bit")


if __name__ == '__main__':
    # Determine project root
    script_dir = os.path.dirname(__file__)
    project_root = os.path.abspath(os.path.join(script_dir, '..'))

    # Absolute path to your TIFF in preview/
    tif_name = 'ldem_87s_5mpp.tif'
    preview_tif = os.path.join(project_root, tif_name)

    # 1) Inspect with rasterio
    load_dem(preview_tif)

    # 2) Inspect with GDAL
    print("Loading via GDAL...")
    ds = gdal.Open(preview_tif)
    if ds is None:
        raise FileNotFoundError(f"Could not open {preview_tif}")
    band = ds.GetRasterBand(1)
    print("GDAL NoDataValue =", band.GetNoDataValue())


"""
TO DOs:
1. Preprocessing
2. Metadata Generation
3. Organizing Files
4. Rendering
"""