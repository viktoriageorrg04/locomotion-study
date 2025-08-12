#!/bin/bash
set -euo pipefail
shopt -s nullglob

echo "Extracting DEMs..."

# Determine project root directory (parent of this scripts folder)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CWD="$(dirname "$SCRIPT_DIR")"
echo "Project root directory: $CWD"
cd "$CWD"

DEMS_PATH="$CWD/preview"
TGT_PATH="$CWD/dem_processed"

echo "Creating temporary DEMs path: $DEMS_PATH"
mkdir -p "$DEMS_PATH"
echo "Creating target DEMs path: $TGT_PATH"
mkdir -p "$TGT_PATH"

# Move all .tif from base dir into preview/
echo "Copying .tif files from $CWD → $DEMS_PATH"
for f in "$CWD"/*.tif; do
  [ -e "$f" ] || continue
  echo " → cp $f -> $DEMS_PATH/"
  cp "$f" "$DEMS_PATH/"
done

# now only need to look in preview/
tif_files=( "$DEMS_PATH"/*.tif )
if [ ${#tif_files[@]} -eq 0 ]; then
  echo "No .tif in $DEMS_PATH – exiting."
  exit 0
fi

echo "Reading DEMs info..."
for dem in "$DEMS_PATH"/*.tif; do
    echo "Processing DEM: $dem"
    dem_basename="$(basename "$dem" .tif)"
    dem_name_no_ext="$dem_basename"
    echo "[DEBUG] dem_name_no_ext='$dem_name_no_ext'"

    gdalinfo "$dem" > "$DEMS_PATH/${dem_name_no_ext}.xml"
    echo "[DEBUG] gdalinfo exit code=$?"

    python3 scripts/preprocessing/info.py \
      --info_path "$DEMS_PATH/${dem_name_no_ext}.xml" \
      --output_dir "$DEMS_PATH" \
      --output_name "$dem_name_no_ext"

    python3 scripts/preprocessing/preprocess.py \
      --dem_path "${dem}" \
      --output_dir "$DEMS_PATH" \
      --output_name "$dem_name_no_ext"
done
echo "Finished extracting DEMs."

echo "Moving processed DEMs to $TGT_PATH…"
for dem in "$DEMS_PATH"/*.npy; do
  [ -e "$dem" ] || continue
  base="$(basename "${dem%.*}")"
  echo " → moving $base"
  mkdir -p "$TGT_PATH/$base"
  mv "$dem" "$TGT_PATH/$base/dem.npy"
  if [ -e "$DEMS_PATH/$base.yaml" ]; then
    mv "$DEMS_PATH/$base.yaml" "$TGT_PATH/$base/dem.yaml"
  fi
done

echo "Cleaning up preview directory"
rm -rf "$DEMS_PATH"
echo "Done."
