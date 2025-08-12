# #!/usr/bin/env python3
# import os, argparse
# import numpy as np
# from scipy.ndimage import gaussian_filter
# import rasterio
# from noise import pnoise2

# def generate_fbm_noise(shape, scale, octaves, persistence, lacunarity, x_off=0, y_off=0):
#     h, w = shape
#     noise = np.zeros((h, w), np.float32)
#     for i in range(h):
#         for j in range(w):
#             x = (j + x_off) / scale
#             y = (i + y_off) / scale
#             val, amp, freq = 0.0, 1.0, 1.0
#             max_amp = 0.0
#             for _ in range(octaves):
#                 val    += amp * pnoise2(x * freq, y * freq)
#                 max_amp += amp
#                 amp    *= persistence
#                 freq   *= lacunarity
#             noise[i, j] = val / max_amp
#     return noise


# def main():
#     p = argparse.ArgumentParser()
#     p.add_argument("-i","--input",      required=True)
#     p.add_argument("-o","--output",     required=True)
#     p.add_argument("--target_res",      type=float, default=1.0)
#     p.add_argument("--noise_scale",     type=float, default=5.0)
#     p.add_argument("--noise_amp",       type=float, default=0.1)
#     p.add_argument("--octaves",         type=int,   default=5)
#     p.add_argument("--persistence",     type=float, default=0.6)
#     p.add_argument("--lacunarity",      type=float, default=2.0)
#     p.add_argument("--smooth_sigma",    type=float, default=1.0)
#     args = p.parse_args()

#     # 1. Read source DEM
#     if args.input.lower().endswith(".npy"):
#         dem = np.load(args.input)
#         src_transform = src_crs = None
#     else:
#         with rasterio.open(args.input) as src:
#             dem = src.read(1)
#         src_transform = src_crs = None

#     # 2. Upsample to target resolution
#     from scipy.ndimage import zoom
#     dem_hr = zoom(dem, args.target_res, order=3)

#     # 3. Generate and blend fBM noise
#     noise = generate_fbm_noise(
#         dem_hr.shape,
#         scale=args.noise_scale,
#         octaves=args.octaves,
#         persistence=args.persistence,
#         lacunarity=args.lacunarity
#     )
#     elev_range = dem_hr.max() - dem_hr.min()
#     dem_hr += noise * elev_range * args.noise_amp

#     # 4. Optional Gaussian smoothing
#     if args.smooth_sigma > 0:
#         dem_hr = gaussian_filter(dem_hr, sigma=args.smooth_sigma)

#     # 5. Write output: .npy or GeoTIFF
#     os.makedirs(os.path.dirname(args.output), exist_ok=True)
#     if args.output.lower().endswith(".npy"):
#         np.save(args.output, dem_hr)
#         print(f"Saved NumPy array to {args.output}")
#     else:
#         # ensure writer knows the raster size
#         h, w = dem_hr.shape
#         kwargs = dict(
#             driver   = "GTiff",
#             dtype    = "float32",
#             count    = 1,
#             compress = "deflate",
#             height   = h,
#             width    = w,
#         )
#         if src_transform is not None:
#             kwargs.update(transform=src_transform, crs=src_crs)
#         with rasterio.open(args.output, "w", **kwargs) as dst:
#             dst.write(dem_hr, 1)
#         print(f"Saved GeoTIFF to {args.output}")


# if __name__ == "__main__":
#     import sys, os, numpy as _np

#     # when no args are passed, run a built-in smoke test
#     if len(sys.argv) == 1:
#         base = os.path.dirname(__file__)
#         test_patch = os.path.abspath(
#             os.path.join(base, "..", "dem_processed",
#                          "ldem_87s_5mpp", "patches", "x5632y31488.tif")
#         )
#         test_out = os.path.abspath(
#             os.path.join(base, "..", "dem_processed",
#                          "ldem_87s_5mpp", "patches", "x5632y31488_test.npy")
#         )
#         sys.argv = [
#             sys.argv[0],
#             "-i", test_patch,
#             "-o", test_out,
#             "--target_res", "1",
#             "--noise_scale", "1000",
#             "--noise_amp", "0.05",
#             "--octaves", "5",
#             "--persistence", "0.6",
#             "--lacunarity", "2.0",
#             "--smooth_sigma", "1.0",
#         ]
#         print(f"Running built-in test: {test_patch} → {test_out}")

#     main()

#     # after a smoke test, print basic stats and produce a test GeoTIFF
#     if len(sys.argv) > 1 and sys.argv[1] == "-i":
#         # load and report
#         arr = _np.load(test_out, allow_pickle=True)
#         print(f"Test .npy: shape={arr.shape}, min={arr.min():.3f}, max={arr.max():.3f}")

#         # write out a GeoTIFF version
#         import rasterio
#         from pathlib import Path
#         test_tif = str(Path(test_out).with_suffix(".tif"))
#         with rasterio.open(test_patch) as src:
#             meta = src.meta.copy()
#         meta.update(dtype=rasterio.float32, count=1)
#         with rasterio.open(test_tif, "w", **meta) as dst:
#             dst.write(arr, 1)
#         print(f"Test GeoTIFF saved to {test_tif}")

#!/usr/bin/env python3
# enhance_resolution_5m.py
# Keep 5 m/px; upsample -> add fBM-like detail -> (optional smooth) -> downsample back.
import argparse, os
import numpy as np
import rasterio
from rasterio.enums import Resampling
from scipy.ndimage import zoom, gaussian_filter

def spectral_fbm(shape, beta=1.8, rng=None):
    """
    FFT-based 1/f^beta noise (tile-sized, zero-mean, unit-std).
    Produces fractal-looking detail without per-pixel Python loops.
    """
    rng = np.random.default_rng(None if rng is None else rng)
    h, w = shape
    # random phases, symmetric spectrum
    phase = rng.random((h, w)) * 2 * np.pi
    re = np.cos(phase)
    im = np.sin(phase)
    spec = re + 1j * im

    # radial frequency
    fy = np.fft.fftfreq(h)
    fx = np.fft.fftfreq(w)
    fx, fy = np.meshgrid(fx, fy, indexing="xy")
    f = np.sqrt(fx * fx + fy * fy)
    f[0, 0] = 1e-6  # avoid div-by-zero at DC

    # target power ~ 1/f^beta  => amplitude ~ 1/f^(beta/2)
    amp = 1.0 / np.power(f, beta * 0.5)
    spec *= amp

    noise = np.fft.ifft2(spec).real
    # normalize to zero-mean, unit-std
    noise -= noise.mean()
    std = noise.std()
    if std > 0:
        noise /= std
    # clip for stability
    noise = np.clip(noise, -5.0, 5.0)
    return noise.astype(np.float32)

def main():
    p = argparse.ArgumentParser(description="Add sub-pixel fractal detail but keep 5 m/px.")
    p.add_argument("-i", "--input", required=True, help="Input GeoTIFF (5 m/px)")
    p.add_argument("-o", "--output", required=True, help="Output GeoTIFF (same size/res as input)")
    p.add_argument("--upsample_factor", type=int, default=4, help="Internal upsample factor (e.g., 2, 4, 8)")
    p.add_argument("--noise_amp", type=float, default=0.04,
                   help="Noise amplitude as fraction of elevation dynamic range (e.g., 0.02–0.06)")
    p.add_argument("--beta", type=float, default=1.8, help="Fractal slope (1/f^beta). 1.5–2.0 is a good range.")
    p.add_argument("--smooth_sigma", type=float, default=0.8,
                   help="Gaussian sigma in *upsampled* pixels (0 disables smoothing)")
    p.add_argument("--seed", type=int, default=None, help="Random seed for reproducibility")
    args = p.parse_args()

    with rasterio.open(args.input) as src:
        profile = src.profile.copy()
        nodata = src.nodata
        data = src.read(1, masked=True).astype(np.float32)  # masked array if nodata exists

    # basic stats (robust range)
    valid = np.ma.masked_invalid(data).compressed()
    if valid.size == 0:
        raise RuntimeError("No valid pixels in input DEM.")
    p1, p99 = np.percentile(valid, [1, 99])
    elev_range = max(1e-6, float(p99 - p1))
    orig_min = float(valid.min())
    orig_max = float(valid.max())

    # prepare arrays (fill masked with nearest valid before zoom to avoid artifacts)
    if isinstance(data, np.ma.MaskedArray) and data.mask.any():
        # simple fill with median; for large nodata regions consider inpainting
        fill_val = float(np.median(valid))
        base = data.filled(fill_val)
        mask = data.mask.astype(np.uint8)
    else:
        base = np.asarray(data)
        mask = None

    k = max(1, int(args.upsample_factor))
    if k == 1:
        hi = base.copy()
    else:
        hi = zoom(base, k, order=3)

    # add spectral fBM noise at upsampled resolution
    rng = np.random.default_rng(args.seed)
    noise = spectral_fbm(hi.shape, beta=args.beta, rng=rng)
    amp = args.noise_amp * elev_range
    hi = hi + amp * noise

    # optional light smoothing at upsampled scale
    if args.smooth_sigma > 0:
        hi = gaussian_filter(hi, sigma=args.smooth_sigma)

    # downsample back to original size (exactly)
    if k != 1:
        out = zoom(hi, 1.0 / k, order=3)
    else:
        out = hi

    # clamp to original min/max to avoid unrealistic extremes
    out = np.clip(out, orig_min, orig_max).astype(np.float32)

    # restore nodata mask, if any
    if mask is not None:
        # downsample mask with nearest (order=0) to keep it binary-ish
        if k != 1:
            mask_lo = zoom(mask, 1.0, order=0)  # mask remained original size
        else:
            mask_lo = mask
        out = np.ma.array(out, mask=mask_lo).filled(nodata if nodata is not None else np.nan)
        if nodata is None:
            # if no explicit nodata, keep array as-is (nan allowed); otherwise set nodata:
            pass

    # write GeoTIFF with original georeferencing
    profile.update(dtype=rasterio.float32, count=1, compress="deflate")
    with rasterio.open(args.output, "w", **profile) as dst:
        dst.write(out, 1)

    print(f"Saved: {args.output}")
    print(f"Kept 5 m/px, size {profile['width']}x{profile['height']} px,"
          f" CRS={profile.get('crs')}, transform preserved.")

if __name__ == "__main__":
    main()
