#!/usr/bin/env bash
set -euo pipefail

# Pretty tracing (set TRACE=1 to see every command)
if [[ "${TRACE:-0}" == "1" ]]; then
  export PS4='+ [${BASH_SOURCE##*/}:${LINENO}] '
  set -x
fi

log(){ echo "[launch_sim] $*"; }
die(){ echo "[launch_sim][ERROR] $*" >&2; exit 1; }

# -----------------------------
# Isaac-capable Python detector
# -----------------------------
AUTO_CONDA_ENV="${AUTO_CONDA_ENV:-env_isaacsim}"

_has_omni() {
  "$1" - <<'PY' >/dev/null 2>&1 || return 1
try:
    import omni, carb  # noqa: F401
except Exception:
    raise SystemExit(1)
PY
}

_pick_env_python() {
  local env_name="$1"
  [[ -z "$env_name" ]] && return 1

  for CBASE in "$HOME/anaconda3" "$HOME/miniconda3" "$HOME/mambaforge" "$HOME/miniforge3" "/opt/conda"; do
    if [[ -x "$CBASE/envs/$env_name/bin/python" ]]; then
      if _has_omni "$CBASE/envs/$env_name/bin/python"; then
        echo "$CBASE/envs/$env_name/bin/python"; return 0
      fi
    fi
  done

  # As a last resort, try to activate conda env into this shell (best-effort)
  for CBASE in "$HOME/anaconda3" "$HOME/miniconda3" "$HOME/mambaforge" "$HOME/miniforge3" "/opt/conda"; do
    [[ -f "$CBASE/etc/profile.d/conda.sh" ]] || continue
    # shellcheck disable=SC1090
    . "$CBASE/etc/profile.d/conda.sh"
    if command -v conda >/dev/null 2>&1; then
      conda activate "$env_name" 2>/dev/null || true
      command -v python >/dev/null 2>&1 && _has_omni python && { echo python; return 0; }
    fi
    break
  done
  return 1
}

_detect_isaac_python() {
  # 1) Respect explicit override
  if [[ -n "${ISAAC_PYTHON:-}" && -x "${ISAAC_PYTHON:-}" ]] && _has_omni "$ISAAC_PYTHON"; then
    echo "$ISAAC_PYTHON"; return
  fi
  # 2) Try preferred conda env
  if PY=$(_pick_env_python "$AUTO_CONDA_ENV"); then
    echo "$PY"; return
  fi
  # 3) Current python
  if command -v python >/dev/null 2>&1 && _has_omni python; then
    echo python; return
  fi
  # 4) Isaac install folder(s)
  if [[ -d "$HOME/.local/share/ov/pkg" ]]; then
    while IFS= read -r d; do
      [[ -x "$d/python.sh" ]] && _has_omni "$d/python.sh" && { echo "$d/python.sh"; return; }
    done < <(ls -1d "$HOME/.local/share/ov/pkg"/isaac-sim-* 2>/dev/null | sort -Vr)
  fi
  [[ -x "/opt/isaac-sim/python.sh" ]] && _has_omni "/opt/isaac-sim/python.sh" && { echo "/opt/isaac-sim/python.sh"; return; }
  # 5) Fallback (may not work)
  echo python
}

PYTHON="$(_detect_isaac_python)"
log "Using Python: $PYTHON"
log "which python => $(command -v "${PYTHON%% *}" || true)"
log "PYTHONPATH=${PYTHONPATH:-<unset>}"
log "CONDA_PREFIX=${CONDA_PREFIX:-<unset>}"
log "VIRTUAL_ENV=${VIRTUAL_ENV:-<unset>}"

if ! _has_omni "$PYTHON"; then
  log "WARNING: '$PYTHON' could not import omni/carb. Isaac might fail to launch."
fi

# -----------------------------
# Paths (absolute)
# -----------------------------
ROOT="$(pwd)"
CREATE_SCENE="$(realpath South_Pole_DEMs/scripts/isaac_scene/create_scene.py)"
LAUNCH_ISAAC="$(realpath South_Pole_DEMs/scripts/isaac_scene/launch_isaac.py)"
USD_OUT_DEFAULT="$(realpath South_Pole_DEMs/flat_terrain.usda)"
DEM_PATCH_DIR="$(realpath South_Pole_DEMs/dem_processed/ldem_87s_5mpp/patches)"

log "Paths:"
log "  CREATE_SCENE = $CREATE_SCENE"
log "  LAUNCH_ISAAC  = $LAUNCH_ISAAC"
log "  USD_OUT_DEFAULT = $USD_OUT_DEFAULT"
log "  DEM_PATCH_DIR = $DEM_PATCH_DIR"

[[ -f "$LAUNCH_ISAAC" ]]  || die "Missing file: $LAUNCH_ISAAC"
[[ -d "$DEM_PATCH_DIR" ]] || die "Missing dir: $DEM_PATCH_DIR"

# -----------------------------
# Render / viewport defaults
# -----------------------------
RENDERER="${RENDERER:-RayTracedLighting}"  # or PathTracing
VIEWPORT="${VIEWPORT:-stage}"
BLACK_SKY="${BLACK_SKY:-1}"   # 1 = enabled
HEADLESS="${HEADLESS:-0}"     # 1 = headless
PT_SPP="${PT_SPP:-128}"
PT_MAX_BOUNCES="${PT_MAX_BOUNCES:-6}"

# -----------------------------
# Robot + pose defaults
# -----------------------------
ASSET_USD="${ASSET_USD:-omniverse://localhost/NVIDIA/Assets/Isaac/4.5/Isaac/Robots/Clearpath/Jackal/jackal.usd}"
ASSET_NAME="${ASSET_NAME:-jackal}"
ASSET_CENTER="${ASSET_CENTER:-1}"  # 1 = place at DEM center
ASSET_X="${ASSET_X:-0.0}"
ASSET_Y="${ASSET_Y:-0.0}"
ASSET_YAW="${ASSET_YAW:-0.0}"
DROP_MARGIN="${DROP_MARGIN:-0.0}"
MOON="${MOON:-0}"

# -----------------------------
# Motion / sim params
# -----------------------------
VX="${VX:-0.5}"
YAW_RATE="${YAW_RATE:-0.0}"
DURATION="${DURATION:-20}"
HZ="${HZ:-60}"
WHEEL_RADIUS="${WHEEL_RADIUS:-0.098}"
WHEEL_BASE="${WHEEL_BASE:-0.375}"

# -----------------------------
# Decide USD source
# -----------------------------
# If you set USD_IN, we’ll skip scene creation and use that USD.
USD_IN="${USD_IN:-}"
USD_OUT="$USD_OUT_DEFAULT"

# If CREATE_SCENE is missing (overwritten/removed), we must have USD_IN
if [[ ! -f "$CREATE_SCENE" && -z "$USD_IN" ]]; then
  die "create_scene.py missing. Set USD_IN to an existing USD (e.g. South_Pole_DEMs/usd_patches/*.usda) or restore create_scene.py."
fi

# -----------------------------
# Probe create_scene CLI (guardrail)
# -----------------------------
if [[ -z "$USD_IN" && -f "$CREATE_SCENE" ]]; then
  log "Probing create_scene.py CLI..."
  HELP_OUT="$("$PYTHON" "$CREATE_SCENE" -h 2>&1 || true)"
  printf '%s\n' "$HELP_OUT" | sed 's/^/[create_scene -h] /' | head -n 20 || true

  if ! grep -q -- "--tif" <<<"$HELP_OUT"; then
    log "FATAL: create_scene.py help does NOT mention --tif/--out."
    log "It likely contains the argparse from launch_isaac.py (expects --usd/--asset-usd)."
    log "Head of create_scene.py for confirmation:"
    head -n 20 "$CREATE_SCENE" | sed 's/^/[HEAD create_scene] /' || true
    log "Please restore create_scene.py (or set USD_IN to a ready-made USD to bypass)."
    exit 42
  fi
fi

# -----------------------------
# Pick a DEM patch, possibly map to prebuilt USD
# -----------------------------
if [[ -z "$USD_IN" ]]; then
  echo "Available DEM patches:"
  select TIF in "$DEM_PATCH_DIR"/*_enhanced.tif; do
    [[ -n "${TIF:-}" ]] || { echo "Invalid selection."; continue; }
    TIF="$(realpath "$TIF")"
    break
  done

  # If there is a prebuilt USD for this patch, you may force it with USD_IN
  PATCH_NAME="$(basename "${TIF%_enhanced.tif}")_enhanced.usda"
  PREBUILT="South_Pole_DEMs/usd_patches/$PATCH_NAME"
  if [[ -f "$PREBUILT" ]]; then
    log "Prebuilt USD found: $PREBUILT (set USD_IN to use it and skip creation)"
  fi

  # Allow skipping creation
  if (( ${SKIP_CREATE:-0} )); then
    if [[ -f "$PREBUILT" ]]; then
      USD_IN="$(realpath "$PREBUILT")"
      log "SKIP_CREATE=1 -> using prebuilt USD: $USD_IN"
    else
      die "SKIP_CREATE=1 was set but no prebuilt USD found for $TIF (expected $PREBUILT)."
    fi
  fi
fi

# -----------------------------
# Create USD (unless USD_IN supplied)
# -----------------------------
if [[ -z "$USD_IN" ]]; then
  echo "Creating $USD_OUT from $TIF..."
  {
    set -x
    "$PYTHON" "$CREATE_SCENE" \
      --tif "$TIF" \
      --out "$USD_OUT" \
      --z-mode relative \
      --z-exag 1.0
    set +x
  } 2>&1 | tee /tmp/create_scene.log

  if [[ ! -s "$USD_OUT" ]]; then
    echo
    log "===== create_scene FAILED or produced empty USD ====="
    log "Command used: $PYTHON $CREATE_SCENE --tif \"$TIF\" --out \"$USD_OUT\" --z-mode relative --z-exag 1.0"
    log "Log tail:"
    tail -n 80 /tmp/create_scene.log || true
    die "USD was not created at $USD_OUT (create_scene error)."
  fi
else
  USD_OUT="$(realpath "$USD_IN")"
  [[ -f "$USD_OUT" ]] || die "USD_IN does not exist: $USD_OUT"
  log "Using USD_IN: $USD_OUT"
fi

# -----------------------------
# Launch Isaac
# -----------------------------
log "Launching Isaac Sim with:"
log "  --usd        $USD_OUT"
log "  --asset-usd  $ASSET_USD"
log "  --renderer   $RENDERER | --viewport $VIEWPORT | --black-sky $BLACK_SKY | --headless $HEADLESS"
log "  motion: vx=$VX yaw_rate=$YAW_RATE dur=$DURATION hz=$HZ"
log "  robot: wheel_radius=$WHEEL_RADIUS wheel_base=$WHEEL_BASE"

cmd=( "$PYTHON" "$LAUNCH_ISAAC"
  --usd "$USD_OUT"
  --renderer "$RENDERER"
  --viewport "$VIEWPORT"
  --pt-spp "$PT_SPP" --pt-max-bounces "$PT_MAX_BOUNCES"
  --asset-usd "$ASSET_USD"
  --asset-name "$ASSET_NAME"
  --asset-x "$ASSET_X" --asset-y "$ASSET_Y" --asset-yaw "$ASSET_YAW"
  --drop-margin "$DROP_MARGIN"
  --vx "$VX" --yaw-rate "$YAW_RATE" --duration "$DURATION"
  --hz "$HZ" --wheel-radius "$WHEEL_RADIUS" --wheel-base "$WHEEL_BASE"
)
(( BLACK_SKY ))    && cmd+=( --black-sky )
(( HEADLESS ))     && cmd+=( --headless )
(( ASSET_CENTER )) && cmd+=( --asset-center )
(( MOON ))         && cmd+=( --moon )

printf '[launch_cmd] '; printf '%q ' "${cmd[@]}"; echo
exec "${cmd[@]}"
