#!/usr/bin/env bash
set -euo pipefail

# If you're already inside the env, you can comment these to avoid the libmamba warning.
# set +u
# eval "$(conda shell.bash hook)"
# conda activate SouthPoleDEMs
# set -u

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMS_DIR="$ROOT/dem_processed"
FULL_TIF="$ROOT/ldem_87s_5mpp.tif"

for demdir in "$DEMS_DIR"/*; do
  [ -d "$demdir" ] || continue
  name=$(basename "$demdir")
  dem_np="$demdir/dem.npy"
  echo "==== Processing $name ===="

  echo "-- min slope --"
  python3 "$ROOT/scripts/regions/find_min_slope.py" \
    --dem_path "$dem_np" --patch_size 256

  echo "-- find promising --"
  regions_json="$demdir/regions.json"
  python3 "$ROOT/scripts/regions/find_promising.py" \
    --dem "$dem_np" --auto --output "$regions_json"

  echo "-- extract patches (float32 TIFF, no georef yet) --"
  outdir="$demdir/patches"
  python3 "$ROOT/scripts/regions/extract_patches.py" \
    --dem "$dem_np" \
    --regions "$regions_json" \
    --width 256 --height 256 \
    --output_dir "$outdir"

  echo "-- stamp georeferencing onto patches (5 m/px + CRS) --"
  python3 "$ROOT/scripts/regions/fix_patches_georef.py" \
    --full_dem "$FULL_TIF" \
    --patch_dir "$outdir"

  echo "-- enhance patches (keeps georef) --"
  for tif in "$outdir"/*.tif; do
    # skip already-enhanced files in case of re-runs
    [[ "$tif" == *_enhanced.tif ]] && continue
    tif_enh="${tif%.*}_enhanced.tif"
    echo "→ enhancing $tif → $tif_enh"
    python3 "$ROOT/scripts/regions/enhance_resolution.py" \
      -i "$tif" \
      -o "$tif_enh" \
      --upsample_factor 2 \
      --noise_amp 0.010 \
      --beta 2.0 \
      --smooth_sigma 2.0 \
      --seed 42
  done

  echo "-- sanity check (enhanced vs original patch) --"
  for enh in "$outdir"/*_enhanced.tif; do
    python3 "$ROOT/scripts/helper/check_dem.py" "$enh"
  done

  echo "Done processing $name"
done
