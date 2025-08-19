from __future__ import annotations

import argparse
from pathlib import Path
import sys

from isaacsim import SimulationApp

# Add project root (…/South_Pole_DEMs/scripts) to import path
_THIS = Path(__file__).resolve()
_SCRIPT_DIR = _THIS.parent.parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

ap = argparse.ArgumentParser("Open USD terrain in Isaac Sim, optionally bind regolith, set lunar lighting.")
ap.add_argument("--usd", required=True, help="Path to a USD/USDA stage to open")
ap.add_argument("--renderer", default="RayTracedLighting", help="E.g. 'RayTracedLighting', 'PathTracing'")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--viewport", choices=["default", "stage"], default="stage",
                help="default = Omniverse default viewport lights on; stage = only lights authored on the stage")
ap.add_argument("--black-sky", action="store_true", help="Disable any Dome/Sky lights for a moon-like sky")

# Sun parameters
ap.add_argument("--sun-az", type=float, default=120.0, help="Azimuth in degrees")
ap.add_argument("--sun-el", type=float, default=8.0, help="Elevation in degrees above horizon")
ap.add_argument("--sun-intensity", type=float, default=3000.0, help="UsdLux intensity (unitless multiplier)")
ap.add_argument("--sun-exposure", type=float, default=0.01, help="UsdLux exposure (EV offset)")

ap.add_argument("--fill-ratio", type=float, default=0.0,
                help="0..1 fraction of sun intensity used for a dim ambient fill (0 = off)")

# Material knobs (applied to /World/Terrain if not skipped)
ap.add_argument("--skip-material", action="store_true",
                help="Do NOT rebind material on /World/Terrain (preserve the USD's original material).")

# Path Tracing quality helpers (safe defaults; ignored on RTL)
ap.add_argument("--pt-spp", type=int, default=128, help="Path Tracing samples per pixel")
ap.add_argument("--pt-max-bounces", type=int, default=6, help="Path Tracing max bounces")

# Spawn assets
ap.add_argument("--asset-usd", default="", help="Path to any robot/asset USD to spawn")
ap.add_argument("--asset-name", default="asset", help="Name under /World")
ap.add_argument("--asset-x", type=float, default=0.0)
ap.add_argument("--asset-y", type=float, default=0.0)
ap.add_argument("--asset-yaw", type=float, default=0.0)
ap.add_argument("--drop-margin", type=float, default=0.00, help="Meters to drop above terrain")
ap.add_argument("--moon", action="store_true", help="Use 1.62 m/s^2 gravity (lunar)")
ap.add_argument("--asset-center", action="store_true",
                help="Place asset at the center of /World/Terrain")

# Straight-line locomotion knobs
ap.add_argument("--vx", type=float, default=0.5, help="Forward speed (m/s)")
ap.add_argument("--yaw-rate", type=float, default=0.0, help="Yaw rate (rad/s); 0 = straight line")
ap.add_argument("--duration", type=float, default=20.0, help="How long to drive (s)")
ap.add_argument("--hz", type=float, default=60.0, help="Control/physics rate (Hz)")
# ap.add_argument("--wheel-radius", type=float, default=0.098, help="Wheel radius (m)")
# ap.add_argument("--wheel-base", type=float, default=0.375, help="Wheel base (m)")

# Constraints
ap.add_argument("--dust-mu-scale", type=float, default=1.0, help="Terrain friction scale (μd & μs)")
ap.add_argument("--robot-kg", type=float, default=0.0, help="Extra mass added to base link (robot)")
ap.add_argument("--payload-kg", type=float, default=0.0, help="Extra mass added to base link (payload)")
ap.add_argument("--payload-offset", type=float, nargs=3, default=(0.0,0.0,0.0), help="(unused for now) payload CoM offset")
ap.add_argument("--energy-model", choices=["proxy","measured"], default="proxy", help="Energy accounting model")
ap.add_argument("--require-los", action="store_true", help="Abort if LOS check fails (stubbed)")
ap.add_argument("--battery-kJ", type=float, default=0.0,
                help="If >0, estimate max range on flat terrain with this energy budget.")

args = ap.parse_args()
print(f"[launch_isaac] Args: {args}")

simulation_app = SimulationApp({"renderer": args.renderer, "headless": args.headless})


from locomotion.constraints import (
    ConstraintConfig, apply_constraints, los_ok, world_translation,
    distance_from, get_gravity_mps2, proxy_energy_J, rover_features, cost_of_transport, total_articulation_mass_kg, 
    effective_rover_mass_kg, body_upright_tilt_deg, static_proxy_metrics, get_terrain_friction, slip_ratio_estimate
)

import omni.usd
from pxr import Usd, UsdPhysics, PhysxSchema

from set_lighting import setup_daylight, configure_renderer_settings
from helper.terrain_debug import print_terrain_elevation
from set_regolith import make_and_bind_regolith  # only used if not --skip-material
from spawn_rover import (
    ensure_physics_scene,
    apply_terrain_physics,
    spawn_asset,
    world_range_for_path,
)

from isaacsim.core.api import SimulationContext
from locomotion import straight_line as SL
from omni.kit.viewport.utility import get_active_viewport_window
import omni.kit.commands

# -------------------------------
# Helpers
# -------------------------------

def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()

def open_stage(path: str):
    ctx = omni.usd.get_context()
    ctx.open_stage(path)
    st = ctx.get_stage()
    st.SetTimeCodesPerSecond(60)
    print(f"[launch_isaac] Opened stage: {path}")

# -------------------------------
# Extract rover features, friction, gravity, and effective mass
# -------------------------------

def auto_extract(stage, art_root, cfg):
    """Read rover features, friction, gravity, and effective mass straight from USD + cfg."""
    feats = rover_features(stage, art_root)
    mu_s, mu_d = get_terrain_friction(stage, "/World/Terrain")
    g = get_gravity_mps2(stage)
    m_eff = effective_rover_mass_kg(stage, art_root, cfg)
    print(
        "[features] mass={:.3f} kg, wheel_r={} m, wheelbase={} m, drives={}, ω_target={} rad/s, ω_max={} rad/s".format(
            feats.total_mass_kg,
            (None if feats.wheel_radius_m is None else f"{feats.wheel_radius_m:.9f}"),
            (None if feats.wheel_base_m   is None else f"{feats.wheel_base_m:.9f}"),
            len(feats.drive_joint_paths),
            ("n/a" if feats.drive_target_omega is None else f"{feats.drive_target_omega:.3f}"),
            ("n/a" if feats.drive_max_omega    is None else f"{feats.drive_max_omega:.3f}"),
        )
    )
    return feats, mu_s, mu_d, g, m_eff

def inject_proxy_energy_metrics(stage, art_root, cfg, dist_m, avg_v_mps):
    """Compute energy/CoT/slip with the proxy model (no hard-coded params)."""
    proxy = static_proxy_metrics(
        stage=stage, art_root=art_root, cfg=cfg, distance_m=dist_m, avg_v_mps=avg_v_mps
    )
    # Pretty print one-liner for J/m & CoT
    jpm = proxy["energy_per_m_Jpm"]
    print(f"[energy][proxy] d={dist_m:.2f} m, μd={proxy['mu_d']:.2f}, m={proxy['mass_eff_kg']:.2f} kg "
          f"-> E≈{proxy['energy_J']:.1f} J, J/m={jpm:.2f}, CoT={proxy['cot']:.3f}")
    return proxy

class TiltSampler:
    """Collect max & RMS tilt (upright deviation) while sim runs."""
    def __init__(self, stage, body_prim):
        self.stage = stage
        self.body  = body_prim
        self.max_deg = 0.0
        self.sum_sq  = 0.0
        self.n       = 0

    def sample(self):
        t = body_upright_tilt_deg(self.stage, self.body)
        if t is None: return
        if t > self.max_deg: self.max_deg = t
        self.sum_sq += t*t
        self.n += 1

    def results(self):
        rms = (self.sum_sq / self.n) ** 0.5 if self.n > 0 else None
        return {"tilt_max_deg": self.max_deg, "tilt_rms_deg": rms}

# -------------------------------
# Frame camera on terrain (best-effort; version-safe)
# -------------------------------

def frame_view_on_prim(prim_path: str) -> bool:
    try:
        vpw = get_active_viewport_window()
        if vpw:
            omni.kit.commands.execute("SelectPrims", old_selected_paths=[], new_selected_paths=[prim_path], expand_in_stage=True)
            if hasattr(vpw, "frame_selection"):
                vpw.frame_selection()
                return True
    except Exception as e:
        print(f"[frame][WARN] {e}")
    return False

# Articulation root finder
def find_articulation_root_path(root_path: str) -> str | None:
    st = get_stage()
    root = st.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        print(f"[articulation] Prim not found: {root_path}")
        return None

    # check the prim itself
    try:
        if root.HasAPI(UsdPhysics.ArticulationRootAPI):
            return root_path
    except Exception:
        pass

    # correct traversal: iterate the subtree rooted at `root`
    for p in Usd.PrimRange(root):
        try:
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                return p.GetPath().pathString
        except Exception:
            pass
    return None

# -------------------------------
# Open file and configure
# -------------------------------

stage_path = str(Path(args.usd).expanduser().resolve())
open_stage(stage_path)

setup_daylight(args)
configure_renderer_settings(args)
stage = get_stage()

# Material on /World/Terrain if present (unless user wants to preserve original)
if not args.skip_material:
    root = Path(__file__).parent.parent.parent  # Points to South_Pole_DEMs
    tex_dir = root / "assets" / "textures" / "regolith"
    mesh_path = "/World/Terrain"

    ok = make_and_bind_regolith(
        stage,
        mesh_path,
        tex_dir=str(tex_dir),
        tile_meters=2.0,
        verbose=True,
    )
    print(f"[launch_isaac] make_and_bind_regolith returned: {ok}")
else:
    print("[material] --skip-material set: preserving original material on /World/Terrain")

framed = frame_view_on_prim("/World/Terrain")
print(f"[launch_isaac] View framed: {framed}")
print("[launch_isaac] Ready.")

print_terrain_elevation(stage, "/World/Terrain")

ensure_physics_scene(stage, moon=args.moon)
apply_terrain_physics(stage, "/World/Terrain")

# -------------------------------
# Spawn robot
# -------------------------------
spawn_x, spawn_y = args.asset_x, args.asset_y
if args.asset_center:
    r = world_range_for_path(stage, "/World/Terrain")
    if r:
        spawn_x = 0.5 * (float(r.GetMin()[0]) + float(r.GetMax()[0]))
        spawn_y = 0.5 * (float(r.GetMin()[1]) + float(r.GetMax()[1]))
        print(f"[spawn] --asset-center requested -> using center ({spawn_x:.3f}, {spawn_y:.3f})")

prim_path = spawn_asset(
    stage,
    usd_path=args.asset_usd,
    name=args.asset_name,
    x=spawn_x,
    y=spawn_y,
    yaw_deg=args.asset_yaw,
    drop_margin=args.drop_margin,
    terrain_path="/World/Terrain",
    orient_to_slope=True,
)

# -------------------------------
# Warm up sim
# -------------------------------
sim = SimulationContext()
sim.set_simulation_dt(1.0 / args.hz)
sim.play()

# small warmup so the referenced USD is fully ready
for _ in range(30):
    sim.step(render=False)

# Choose control prim (articulation root if present)
prim_for_ctrl = find_articulation_root_path(prim_path) or prim_path
print(f"[launch_isaac] Control prim: {prim_for_ctrl}")

# -------------------------------
# Pull rover features straight from USD
# -------------------------------
feats = rover_features(stage, prim_for_ctrl)
print(
    "[features]"
    f" mass={feats.total_mass_kg:.3f} kg,"
    f" wheel_r={feats.wheel_radius_m if feats.wheel_radius_m else 'n/a'} m,"
    f" wheelbase={feats.wheel_base_m if feats.wheel_base_m else 'n/a'} m,"
    f" drives={len(feats.drive_joint_paths)},"
    f" ω_target={feats.drive_target_omega if feats.drive_target_omega else 'n/a'} rad/s,"
    f" ω_max={feats.drive_max_omega if feats.drive_max_omega else 'n/a'} rad/s"
)

wheel_radius = feats.wheel_radius_m or 0.0
wheel_base   = feats.wheel_base_m or 0.0

# -------------------------------
# Constraints (friction scaling + extra masses)
# -------------------------------
cfg = ConstraintConfig(
    dust_mu_scale=args.dust_mu_scale,
    robot_kg=args.robot_kg,
    payload_kg=args.payload_kg,
    payload_offset_xyz=tuple(args.payload_offset),
    check_los=args.require_los,
    energy_model=args.energy_model,
)
apply_constraints(stage, prim_for_ctrl, cfg)

if args.require_los:
    if not los_ok(stage, prim_for_ctrl, "/World/Terrain"):
        print("[constraints] LOS requirement failed — aborting.")
        simulation_app.close()
        sys.exit(3)

# -------------------------------
# Drive straight for args.duration
# -------------------------------
start_pos = world_translation(stage, prim_for_ctrl)
tilts: list[float] = []
ts = TiltSampler(stage, prim_for_ctrl)

def _set_joint_target_velocity(joint_path: str, omega: float) -> bool:
    """Attempt to set an angular drive target velocity on a joint."""
    try:
        jprim = stage.GetPrimAtPath(joint_path)
        # Use the USD Physics Drive API (angular drive)
        drv = UsdPhysics.DriveAPI(jprim, "angular")
        if not drv:
            # If no drive authored, apply one so we can command velocity.
            drv = UsdPhysics.DriveAPI.Apply(jprim, "angular")
        attr = drv.GetTargetVelocityAttr()
        if not attr:
            attr = drv.CreateTargetVelocityAttr()
        attr.Set(float(omega))
        # Give it some damping/stiffness if missing (prevents oscillations)
        if not drv.GetDampingAttr().Get():
            drv.CreateDampingAttr().Set(50.0)
        if not drv.GetStiffnessAttr().Get():
            drv.CreateStiffnessAttr().Set(0.0)
        return True
    except Exception as e:
        print(f"[drive][WARN] unable to set target velocity on {joint_path}: {e}")
        return False

def _zero_all_drives():
    for jp in getattr(feats, "drive_joint_paths", []) or []:
        _set_joint_target_velocity(jp, 0.0)

# Try a proper straight-line controller if available
ran_controller = False
try:
    if hasattr(SL, "run"):
        # Expected signature in our repo; safely try it
        run_out = SL.run(
            stage=stage,
            art_root=prim_for_ctrl,
            vx=args.vx,
            yaw_rate=args.yaw_rate,
            duration_s=args.duration,
            hz=args.hz,
            tilt_cb=ts.sample,  # will be called each step
        )
        ran_controller = True
        # allow downstream to see any metrics the controller produced
        metrics = run_out if isinstance(run_out, dict) else {}
except Exception as e:
    print(f"[straight_line][WARN] SL.run() not available or failed: {e}")

if not ran_controller:
    # Fallback: command same wheel omega to all drive joints (straight line)
    omega = (args.vx / max(1e-6, wheel_radius)) if wheel_radius > 0 else 0.0
    ok_count = 0
    for jp in getattr(feats, "drive_joint_paths", []) or []:
        if _set_joint_target_velocity(jp, omega):
            ok_count += 1
    print(f"[drive] commanded ω={omega:.3f} rad/s on {ok_count}/{len(feats.drive_joint_paths)} drive joints")

    # Step the sim for the requested duration
    steps = max(1, int(args.duration * args.hz))
    for _ in range(steps):
        ts.sample()
        sim.step(render=not args.headless)

    # Stop wheels
    _zero_all_drives()

# -------------------------------
# Post-run quick metrics (proxy)
# -------------------------------
metrics = locals().get("metrics", {}) or {}

d = distance_from(stage, prim_for_ctrl, start_pos)
g = get_gravity_mps2(stage, default_g=(1.62 if args.moon else 9.81))
m_eff = effective_rover_mass_kg(stage, prim_for_ctrl, cfg)
_, mu_d = get_terrain_friction(stage, "/World/Terrain")

E = proxy_energy_J(distance_m=d, mass_kg=m_eff, mu=mu_d, g=g)
Jpm = (E / d) if d > 1e-9 else None
cot = cost_of_transport(E_J=E, mass_kg=m_eff, g=g, distance_m=d)

avg_v = (
    metrics.get("avg_speed_mps")
    if isinstance(metrics, dict) and "avg_speed_mps" in metrics
    else (args.vx if getattr(args, "vx", None) is not None else (d / max(1e-6, args.duration)))
)

omega_for_slip = feats.drive_target_omega if feats.drive_target_omega is not None else (
    (avg_v / max(1e-6, wheel_radius)) if wheel_radius > 0 else None
)
slip_i = slip_ratio_estimate(avg_v, wheel_radius, omega_for_slip)

tilt_stats = ts.results()
tilt_max = tilt_stats["tilt_max_deg"]
tilt_rms = tilt_stats["tilt_rms_deg"]

Jpm_str = f"{Jpm:.2f}" if Jpm is not None else "n/a"
cot_str = f"{cot:.3f}" if cot is not None else "n/a"
print(
    f"[energy][proxy] d={d:.2f} m, μd={mu_d:.2f}, m={m_eff:.2f} kg"
    f" -> E≈{E:.1f} J, J/m={Jpm_str}, CoT={cot_str}"
)

if getattr(args, "battery_kJ", 0.0) and Jpm is not None and Jpm > 0:
    est_range_m = (args.battery_kJ * 1000.0) / Jpm
    print(f"[range][est] With {args.battery_kJ:.1f} kJ budget: ~{est_range_m:.1f} m")

if isinstance(metrics, dict):
    metrics.update({
        "elapsed_s": metrics.get("elapsed_s", args.duration),
        "distance_m": d,
        "avg_speed_mps": avg_v,
        "energy_J": E,
        "energy_per_m_Jpm": (Jpm or 0.0),
        "cot": cot,
        "slip_ratio_est": slip_i,
        "tilt_max_deg": tilt_max,
        "tilt_rms_deg": tilt_rms,
    })

print("[straight_line] metrics:", metrics)
