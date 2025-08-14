#!/bin/bash
set -e

DEM_PATCH_DIR="South_Pole_DEMs/dem_processed/ldem_87s_5mpp/patches"
CREATE_SCENE="South_Pole_DEMs/scripts/isaac_scene/create_scene.py"
LAUNCH_ISAAC="South_Pole_DEMs/scripts/isaac_scene/launch_isaac.py"
PYTHON="python"  # or your env path

echo "Available DEM patches:"
select tif in "$DEM_PATCH_DIR"/*_enhanced.tif; do
    if [ -n "$tif" ]; then
        echo "Creating flat_terrain.usda from $tif..."
        $PYTHON "$CREATE_SCENE" --tif "$tif" --out "South_Pole_DEMs/flat_terrain.usda" --z-mode relative --z-exag 1.0
        echo "Launching Isaac Sim with flat_terrain.usda..."
        $PYTHON "$LAUNCH_ISAAC" --usd "South_Pole_DEMs/flat_terrain.usda" --viewport stage --black-sky --pt-spp 128 --pt-max-bounces 6
        break
    else
        echo "Invalid selection."
    fi
done
