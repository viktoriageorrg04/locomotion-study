from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

from isaacsim import SimulationApp

# Add project root
_THIS = Path(__file__).resolve()
_SCRIPT_DIR = _THIS.parent.parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

ap = argparse.ArgumentParser("Open USD terrain in Isaac Sim, optionally bind regolith, set lunar lighting.")

ap.add_argument('--controller', choices=['auto','dc','none','base-twist'], default='auto',
               help='Select controller: auto (default), dc (wheeled), none, or base-twist proxy')
ap.add_argument("--usd", required=True, help="Path to a USD/USDA stage to open")
ap.add_argument("--renderer", default="RayTracedLighting", help="E.g. 'RayTracedLighting', 'PathTracing'")
ap.add_argument("--headless", action="store_true")
ap.add_argument("--viewport", choices=["default", "stage"], default="stage",
                help="default = Omniverse default viewport lights on; stage = only lights authored on the stage")
ap.add_argument("--real-time", action="store_true",
                    help="Throttle stepping so wall time matches simulated time.")
ap.add_argument("--black-sky", action="store_true", help="Disable any Dome/Sky lights for a moon-like sky")

# Sun parameters
ap.add_argument("--sun-az", type=float, default=120.0, help="Azimuth in degrees")
ap.add_argument("--sun-el", type=float, default=8.0, help="Elevation in degrees above horizon")
ap.add_argument("--sun-intensity", type=float, default=3000.0, help="UsdLux intensity (unitless multiplier)")
ap.add_argument("--sun-exposure", type=float, default=0.01, help="UsdLux exposure (EV offset)")

ap.add_argument("--fill-ratio", type=float, default=0.0,
                help="0..1 fraction of sun intensity used for a dim ambient fill (0 = off)")

# Birds-eye view camera
ap.add_argument("--birdseye", action="store_true",
                help="Create a top-down camera and switch the viewport to it.")
ap.add_argument("--cam-height", type=float, default=8.0,
                help="Bird's-eye camera height above the robot (meters).")
ap.add_argument("--cam-parent", action="store_true",
                help="Parent camera under the robot (follows + rotates with robot). "
                         "If omitted, camera follows in world frame (no pitch/roll).")


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
    ConstraintConfig, apply_constraints, world_translation,
    distance_from, get_gravity_mps2, proxy_energy_J, rover_features, cost_of_transport,
    effective_rover_mass_kg, body_upright_tilt_deg, get_terrain_friction, slip_ratio_estimate
)

import omni.usd
from locomotion.controllers.registry import pick_controller
from pxr import Usd, UsdPhysics, UsdGeom, Gf

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
from omni.kit.viewport.utility import get_active_viewport, get_active_viewport_window
import omni.kit.commands
from omni.isaac.dynamic_control import _dynamic_control as dc

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

def _step_rt(sim, step_idx: int, t0: float, dt: float, render: bool):
    sim.step(render=render)
    if args.real_time:
        target = t0 + (step_idx + 1) * dt
        while True:
            remaining = target - time.perf_counter()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.002))

# -------------------------------
# Extract rover features, friction, gravity, and effective mass
# -------------------------------

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

def _get_active_viewport():
    """Return a viewport API object (not the window) across Kit versions."""
    # Preferred: grab the active viewport window then take .viewport_api
    try:
        win = get_active_viewport_window()
        if win and hasattr(win, "viewport_api"):
            return win.viewport_api
    except Exception:
        pass

    # Fallback: some builds return a viewport object directly
    try:
        vp = get_active_viewport()
        return getattr(vp, "viewport_api", vp)
    except Exception:
        pass

    # Last resort: import get_viewport_interface lazily if present
    try:
        from omni.kit.viewport.utility import get_viewport_interface as _gvi
        return _gvi().get_viewport_window(None).viewport_api
    except Exception:
        return None

def frame_view_on_prim(path: str) -> bool:
    vp = _get_active_viewport()
    if not vp:
        print("[frame] No active viewport yet")
        return False

    # Select the prim (handle both old/new Kit APIs)
    try:
        # Newer API
        omni.kit.commands.execute(
            "SelectPrims",
            old_selected_paths=[],
            new_selected_paths=[path],
            expand_in_stage=True,
        )
    except TypeError:
        # Older API
        omni.kit.commands.execute(
            "SelectPrims",
            old_selection=[],
            new_selection=[path],
            expand_in_stage=True,
        )
    except Exception as e:
        print(f"[frame] selection failed: {e}")
        return False

    # Frame it (handle version differences)
    if hasattr(vp, "frame_selection"):
        vp.frame_selection()
    elif hasattr(vp, "new_frame_selection"):
        vp.new_frame_selection()
    return True

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

dt = 1.0 / args.hz
sim = SimulationContext(physics_dt=dt, rendering_dt=dt)
sim.play()

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

# after: prim_path = spawn_asset(...)
def find_articulation_root_path(stage, root_path):
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return None
    hits = []
    try:
        if root.HasAPI(UsdPhysics.ArticulationRootAPI):
            hits.append(root_path)
    except Exception:
        pass
    for p in Usd.PrimRange(root):
        try:
            if p.HasAPI(UsdPhysics.ArticulationRootAPI):
                hits.append(p.GetPath().pathString)
        except Exception:
            pass
    if not hits:
        return None
    hits.sort(key=lambda s: s.count("/"))
    return hits[0]

prim_for_ctrl = find_articulation_root_path(stage, prim_path) or prim_path
print(f"[launch_isaac] Control prim: {prim_for_ctrl}")

for _ in range(5):
    sim.step(render=not args.headless)

# If DC can't see an articulation here, try parent or the asset root.
try:
    dcif = dc.acquire_dynamic_control_interface()
    if not dcif.get_articulation(prim_for_ctrl):
        from pathlib import Path as _P
        for try_path in [str(_P(prim_for_ctrl).parent).replace("\\","/"), prim_path]:
            if dcif.get_articulation(try_path):
                prim_for_ctrl = try_path
                print(f"[launch_isaac] Switched control prim to articulation root: {prim_for_ctrl}")
                break
except Exception:
    pass

# -------------------------------
# Camera view
# -------------------------------

def create_birdseye_camera(stage, follow_path, height, parent):
    if parent:
        cam_path = f"{follow_path}/TopDownCam"
    else:
        cam_path = "/World/TopDownCam"

    cam = UsdGeom.Camera.Define(stage, cam_path)

    # Clear any existing xform ops, then place above the robot.
    xform = UsdGeom.Xformable(cam)
    for op in xform.GetOrderedXformOps():
        xform.RemoveXformOp(op)
    # Camera looks along -Z by default; placing it at +Z above the robot makes it look down.
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height))
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 5000.0))

    # If we want it to move with the robot automatically, parent it now.
    if parent:
        cam_prim = stage.GetPrimAtPath(cam_path)
        # (Already authored under follow_path by using the composed path above.)
    else:
        # Not parented: we'll update its world translation each step.
        pass

    # Switch the viewport to this camera if possible
    try:
        vp = _get_active_viewport()
        if vp:
            try:
                vp.set_active_camera(cam_path)                      # string path works in newer builds
            except Exception:
                vp.set_active_camera(stage.GetPrimAtPath(cam_path)) # older builds want a prim

            # Frame the camera prim via the viewport API when available
            try:
                omni.kit.commands.execute(
                    "SelectPrims", old_selected_paths=[], new_selected_paths=[cam_path], expand_in_stage=True
                )
            except Exception:
                pass
            if hasattr(vp, "frame_selection"):
                vp.frame_selection()
        else:
            print("[camera] No active viewport API")
    except Exception as e:
        print(f"[camera] Could not set active viewport camera: {e}")
    return cam_path

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

# Camera after we know the moving body
cam_path = None
if args.birdseye:
    _follow = feats.base_link_path or prim_for_ctrl
    cam_path = create_birdseye_camera(stage, _follow, args.cam_height, args.cam_parent)

# -------------------------------
# Constraints (friction scaling + extra masses)
# -------------------------------

cfg = ConstraintConfig(
    dust_mu_scale=args.dust_mu_scale,
    robot_kg=args.robot_kg,
    payload_kg=args.payload_kg,
    payload_offset_xyz=tuple(args.payload_offset),
    energy_model=args.energy_model,
)
apply_constraints(stage, prim_for_ctrl, cfg)

# -------------------------------
# Pick controller
# -------------------------------

metrics = {}

# Use the base link (moves in world) for telemetry, not the articulation root
body_path = (feats.base_link_path or prim_for_ctrl)
start_pos = world_translation(stage, body_path)
ts = TiltSampler(stage, body_path)

# Pick and run the appropriate controller for this robot
ctrl = pick_controller(stage, prim_for_ctrl, feats, preferred=args.controller)
ctrl.start(vx=args.vx, yaw_rate=args.yaw_rate, hz=args.hz)

steps = max(1, int(args.duration * args.hz))
t0 = time.perf_counter()
for i in range(steps):
    ts.sample()
    ctrl.step()
    _step_rt(sim, i, t0, dt, render=not args.headless)
ctrl.shutdown()

metrics["elapsed_s"] = steps * dt
metrics["controller"] = getattr(ctrl.spec, "name", "unknown")
metrics["commanded_vx_mps"] = float(args.vx)
# For wheeled DC, keep omega for slip calculations (optional)
if hasattr(ctrl, "omega_cmd"):
    metrics["commanded_omega_rad_s"] = ctrl.omega_cmd

# -------------------------------
# Post-run quick metrics (proxy + slip)
# -------------------------------

d = distance_from(stage, body_path, start_pos)
elapsed_s = metrics.get("elapsed_s", args.duration)
avg_v_obs = (d / max(1e-6, elapsed_s))

g = get_gravity_mps2(stage, default_g=(1.62 if args.moon else 9.81))
m_eff = effective_rover_mass_kg(stage, prim_for_ctrl, cfg)
_, mu_d = get_terrain_friction(stage, "/World/Terrain")

E = proxy_energy_J(distance_m=d, mass_kg=m_eff, mu=mu_d, g=g)
Jpm = (E / d) if d > 1e-9 else None
cot = cost_of_transport(E_J=E, mass_kg=m_eff, g=g, distance_m=d)

slip_i = None
wheel_r = feats.wheel_radius_m or 0.0
omega_for_slip = metrics.get("commanded_omega_rad_s", None)
if wheel_r > 0.0 and omega_for_slip is not None:
    slip_i = slip_ratio_estimate(avg_v_obs, wheel_r, omega_for_slip)

tilt_stats = ts.results()
Jpm_str = f"{Jpm:.2f}" if Jpm is not None else "n/a"
cot_str = f"{cot:.3f}" if cot is not None else "n/a"
slip_str = f"{slip_i:.3f}" if slip_i is not None else "n/a"
print(f"[energy][proxy] d={d:.2f} m,  avg_v={avg_v_obs:.2f} mps, μd={mu_d:.2f}, m={m_eff:.2f} kg -> E≈{E:.1f} J, J/m={Jpm_str}, CoT={cot_str}, slip_ratio={slip_str}")

metrics.update({
    "elapsed_s": elapsed_s,
    "distance_m": d,
    "avg_speed_mps": avg_v_obs,
    "energy_J": E,
    "energy_per_m_Jpm": (Jpm or 0.0),
    "cot": cot,
    "slip_ratio_est": (slip_i if slip_i is not None else 0.0),
    "tilt_max_deg": tilt_stats["tilt_max_deg"],
    "tilt_rms_deg": tilt_stats["tilt_rms_deg"],
})
