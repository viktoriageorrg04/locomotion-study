from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

# --- bootstrap Kit BEFORE any omni/pxr imports ---
from isaacsim import SimulationApp

_THIS = Path(__file__).resolve()
_SCRIPT_DIR = _THIS.parent.parent  # South_Pole_DEMs/scripts
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

# ---CLI -----------------------------------------------------------------------

def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser("Open USD terrain in Isaac Sim; spawn asset; run simple drive + metrics.")
    # Core
    ap.add_argument("--usd", required=True, help="Path to a USD/USDA stage to open")
    ap.add_argument("--renderer", default="RayTracedLighting", help="e.g. RayTracedLighting, PathTracing")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--viewport", choices=["default", "stage"], default="stage",
                    help="default: Kit defaults on; stage: only lights authored on the stage")
    ap.add_argument("--real-time", action="store_true", help="Throttle stepping to wall time")
    ap.add_argument("--black-sky", action="store_true", help="Disable Dome/Sky lights")

    # Sun
    ap.add_argument("--sun-az", type=float, default=120.0, help="Azimuth (deg)")
    ap.add_argument("--sun-el", type=float, default=8.0, help="Elevation (deg above horizon)")
    ap.add_argument("--sun-intensity", type=float, default=3000.0, help="UsdLux intensity multiplier")
    ap.add_argument("--sun-exposure", type=float, default=0.01, help="UsdLux exposure (EV)")

    ap.add_argument("--fill-ratio", type=float, default=0.0,
                    help="0..1 fraction of sun intensity for dim ambient fill (0=off)")

    # Camera
    ap.add_argument("--birdseye", action="store_true", help="Create top-down camera and switch viewport")
    ap.add_argument("--cam-height", type=float, default=8.0, help="Bird’s-eye height (m)")
    ap.add_argument("--cam-parent", action="store_true", help="Parent camera to robot")

    # Materials
    ap.add_argument("--skip-material", action="store_true",
                    help="Do NOT rebind material on /World/Terrain")

    # Path Tracing (ignored on RTL)
    ap.add_argument("--pt-spp", type=int, default=128, help="Path Tracing samples per pixel")
    ap.add_argument("--pt-max-bounces", type=int, default=6, help="Path Tracing max bounces")

    # Asset
    ap.add_argument("--asset-usd", default="", help="USD path (file or omniverse://) to spawn")
    ap.add_argument("--asset-name", default="asset", help="Name under /World")
    ap.add_argument("--asset-x", type=float, default=0.0)
    ap.add_argument("--asset-y", type=float, default=0.0)
    ap.add_argument("--asset-yaw", type=float, default=0.0)
    ap.add_argument("--drop-margin", type=float, default=0.0, help="Drop above terrain (m)")
    ap.add_argument("--moon", action="store_true", help="Use 1.62 m/s^2 gravity (lunar)")
    ap.add_argument("--asset-center", action="store_true", help="Place at /World/Terrain center")

    # Controller + run
    ap.add_argument('--controller', choices=['auto','dc','none','base-twist'], default='auto',
                    help='auto (default), dc (wheeled), none, base-twist proxy')
    ap.add_argument("--vx", type=float, default=0.5, help="Forward speed (m/s)")
    ap.add_argument("--yaw-rate", type=float, default=0.0, help="Yaw rate (rad/s)")
    ap.add_argument("--duration", type=float, default=20.0, help="Duration (s)")
    ap.add_argument("--hz", type=float, default=60.0, help="Control/physics rate (Hz)")

    # Constraints
    ap.add_argument("--dust-mu-scale", type=float, default=1.0, help="Scale μd & μs")
    ap.add_argument("--robot-kg", type=float, default=0.0, help="Extra mass on base link")
    ap.add_argument("--payload-kg", type=float, default=0.0, help="Extra mass (payload)")
    ap.add_argument("--payload-offset", type=float, nargs=3, default=(0.0,0.0,0.0), help="Payload CoM offset")
    ap.add_argument("--energy-model", choices=["proxy","measured"], default="proxy", help="Energy accounting model")
    ap.add_argument("--require-los", action="store_true", help="Abort if LOS check fails (stubbed)")
    ap.add_argument("--battery-kJ", type=float, default=0.0, help="If >0, estimate max range.")

    # Output
    ap.add_argument("--metrics-json", default="", help="Write metrics dict to this path as JSON")
    args = ap.parse_args()
    print(f"[launch_isaac] Args: {args}")
    return args

# --- Start Kit now that we know headless/renderer; THEN import omni/pxr/etc ---

args = parse_args()
simulation_app = SimulationApp({"renderer": args.renderer, "headless": args.headless})

# Heavy imports (safe after SimulationApp)
import omni.usd
import omni.kit.commands
from omni.kit.viewport.utility import get_active_viewport, get_active_viewport_window
from pxr import Usd, UsdPhysics, UsdGeom, Gf
from isaacsim.core.api import SimulationContext
from omni.isaac.dynamic_control import _dynamic_control as dc

# Local helpers (your modules)
from locomotion.constraints import (
    ConstraintConfig, apply_constraints, world_translation,
    distance_from, get_gravity_mps2, proxy_energy_J, rover_features, cost_of_transport,
    effective_rover_mass_kg, body_upright_tilt_deg, get_terrain_friction,
    xcom_margin_metric, effective_contact_mu, TiltSampler, ForwardSpeedSampler, slip_metrics,
)
from set_lighting import setup_daylight, configure_renderer_settings
from helper.terrain_debug import print_terrain_elevation
from set_regolith import make_and_bind_regolith
from spawn_rover import ensure_physics_scene, apply_terrain_physics, spawn_asset, world_range_for_path

# --- Utilities ----------------------------------------------------------------

def get_stage() -> Usd.Stage:
    return omni.usd.get_context().get_stage()

def open_stage(path: str) -> Usd.Stage:
    ctx = omni.usd.get_context()
    ctx.open_stage(path)
    st = ctx.get_stage()
    st.SetTimeCodesPerSecond(60)
    print(f"[launch_isaac] Opened stage: {path}")
    return st

def _get_active_viewport():
    """Return a viewport API object across Kit versions (or None)."""
    try:
        win = get_active_viewport_window()
        if win and hasattr(win, "viewport_api"):
            return win.viewport_api
    except Exception:
        pass
    try:
        vp = get_active_viewport()
        return getattr(vp, "viewport_api", vp)
    except Exception:
        pass
    try:
        from omni.kit.viewport.utility import get_viewport_interface as _gvi
        return _gvi().get_viewport_window(None).viewport_api
    except Exception:
        return None

def frame_view_on_prim(path: str) -> bool:
    if args.headless:
        return False
    vp = _get_active_viewport()
    if not vp:
        print("[frame] No active viewport yet")
        return False
    try:
        omni.kit.commands.execute(
            "SelectPrims",
            old_selected_paths=[],
            new_selected_paths=[path],
            expand_in_stage=True,
        )
    except Exception as e:
        print(f"[frame] selection failed: {e}")
        return False
    if hasattr(vp, "frame_selection"):
        vp.frame_selection()
    elif hasattr(vp, "new_frame_selection"):
        vp.new_frame_selection()
    return True

def _step_rt(sim: SimulationContext, step_idx: int, t0: float, dt: float, render: bool):
    sim.step(render=render)
    if args.real_time:
        target = t0 + (step_idx + 1) * dt
        while True:
            remaining = target - time.perf_counter()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.002))

def create_birdseye_camera(stage: Usd.Stage, follow_path: str, height: float, parent: bool) -> Optional[str]:
    if args.headless:
        return None
    cam_path = f"{follow_path}/TopDownCam" if parent else "/World/TopDownCam"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    xform = UsdGeom.Xformable(cam)
    for op in xform.GetOrderedXformOps():
        xform.RemoveXformOp(op)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, height))
    cam.CreateClippingRangeAttr().Set(Gf.Vec2f(0.1, 5000.0))

    try:
        vp = _get_active_viewport()
        if vp:
            try:
                vp.set_active_camera(cam_path)
            except Exception:
                vp.set_active_camera(stage.GetPrimAtPath(cam_path))
            try:
                omni.kit.commands.execute(
                    "SelectPrims", old_selected_paths=[], new_selected_paths=[cam_path], expand_in_stage=True
                )
            except Exception:
                pass
            if hasattr(vp, "frame_selection"):
                vp.frame_selection()
    except Exception as e:
        print(f"[camera] Could not set active viewport camera: {e}")
    return cam_path

def find_articulation_root_path(stage: Usd.Stage, root_path: str) -> Optional[str]:
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

# --- Scene setup --------------------------------------------------------------

def setup_scene_and_physics(stage: Usd.Stage):
    setup_daylight(args)
    configure_renderer_settings(args)
    if not args.skip_material:
        root = _THIS.parent.parent.parent  # South_Pole_DEMs/
        tex_dir = root / "assets" / "textures" / "regolith"
        ok = make_and_bind_regolith(stage, "/World/Terrain", tex_dir=str(tex_dir), tile_meters=2.0, verbose=True)
        print(f"[launch_isaac] make_and_bind_regolith returned: {ok}")
    else:
        print("[material] --skip-material: preserving original material on /World/Terrain")

    framed = frame_view_on_prim("/World/Terrain")
    print(f"[launch_isaac] View framed: {framed}")
    print_terrain_elevation(stage, "/World/Terrain")

    ensure_physics_scene(stage, moon=args.moon)
    apply_terrain_physics(stage, "/World/Terrain")

# --- Run ----------------------------------------------------------------------

def main() -> int:
    # Open stage
    stage_path = str(Path(args.usd).expanduser().resolve())
    stage = open_stage(stage_path)
    setup_scene_and_physics(stage)

    # Simulation context
    dt = 1.0 / float(args.hz)
    sim = SimulationContext(physics_dt=dt, rendering_dt=dt)
    sim.play()

    # Spawn asset
    spawn_x, spawn_y = args.asset_x, args.asset_y
    if args.asset_center:
        r = world_range_for_path(stage, "/World/Terrain")
        if r:
            spawn_x = 0.5 * (float(r.GetMin()[0]) + float(r.GetMax()[0]))
            spawn_y = 0.5 * (float(r.GetMin()[1]) + float(r.GetMax()[1]))
            print(f"[spawn] --asset-center -> ({spawn_x:.3f}, {spawn_y:.3f})")

    prim_path = spawn_asset(
        stage,
        usd_path=args.asset_usd,
        name=args.asset_name,
        x=spawn_x, y=spawn_y,
        yaw_deg=args.asset_yaw,
        drop_margin=args.drop_margin,
        terrain_path="/World/Terrain",
        orient_to_slope=True,
    )

    # Controller target prim
    prim_for_ctrl = find_articulation_root_path(stage, prim_path) or prim_path
    print(f"[launch_isaac] Control prim: {prim_for_ctrl}")

    # Let USD settle a few frames
    for _ in range(5):
        sim.step(render=not args.headless)

    # Try to switch to the real articulation root if DC sees it elsewhere
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

    # Camera (after features)
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
    if args.birdseye:
        _follow = feats.base_link_path or prim_for_ctrl
        _ = create_birdseye_camera(stage, _follow, args.cam_height, args.cam_parent)

    # Constraints
    cfg = ConstraintConfig(
        dust_mu_scale=args.dust_mu_scale,
        robot_kg=args.robot_kg,
        payload_kg=args.payload_kg,
        payload_offset_xyz=tuple(args.payload_offset),
        energy_model=args.energy_model,
    )
    apply_constraints(stage, prim_for_ctrl, cfg)

    # Telemetry helpers
    body_path = (feats.base_link_path or prim_for_ctrl)
    start_pos = world_translation(stage, body_path)
    ts = TiltSampler(stage, body_path)
    fs = ForwardSpeedSampler(stage, body_path, dt)
    prev_pos = world_translation(stage, body_path)
    track_len = 0.0

    # Settle
    for _ in range(int(0.5 * args.hz)):
        ts.sample(); fs.sample()
        sim.step(render=not args.headless)

    # Pick controller
    from locomotion.controllers.registry import pick_controller
    ctrl = pick_controller(stage, prim_for_ctrl, feats, preferred=args.controller)
    ctrl.start(vx=args.vx, yaw_rate=args.yaw_rate, hz=args.hz)

    # Run
    steps = max(1, int(args.duration * args.hz))
    t0 = time.perf_counter()
    for i in range(steps):
        cur_pos = world_translation(stage, body_path)
        if prev_pos is not None and cur_pos is not None:
            track_len += float((cur_pos - prev_pos).GetLength())
        prev_pos = cur_pos

        ts.sample(); fs.sample()
        ctrl.step()
        _step_rt(sim, i, t0, dt, render=not args.headless)
    ctrl.shutdown()

    # Metrics
    elapsed_s = steps * dt
    d_net   = distance_from(stage, body_path, start_pos)       # straight-line
    d_track = max(track_len, d_net)                            # arc length if available
    avg_v   = d_track / max(1e-6, elapsed_s)
    v_fwd   = fs.results()["v_forward_avg_mps"]

    g = get_gravity_mps2(stage, default_g=(1.62 if args.moon else 9.81))
    if args.moon and abs(g - 1.62) > 0.05:
        print(f"[warn] Stage reports g≈{g:.2f} but --moon set; overriding metrics to 1.62.")
        g = 1.62

    m_eff = effective_rover_mass_kg(stage, prim_for_ctrl, cfg)
    mu_eff = effective_contact_mu(stage)
    xcom_m, xinfo = xcom_margin_metric(stage, prim_for_ctrl, v_fwd, g=g)
    print("[xcom-debug]", xinfo)

    E = proxy_energy_J(distance_m=d_track, mass_kg=m_eff, mu=mu_eff, g=g)
    Jpm = (E / max(d_track, 1e-9))
    cot = cost_of_transport(E_J=E, mass_kg=m_eff, g=g, distance_m=d_track)

    wheel_r = feats.wheel_radius_m or 0.0
    omega_ref = None
    if wheel_r > 0.0 and feats.drive_target_omega:
        omega_ref = float(feats.drive_target_omega)
    if omega_ref is None and wheel_r > 0.0 and args.vx is not None:
        omega_ref = float(args.vx) / float(wheel_r)
    slip_long, _ = slip_metrics(v_fwd, args.vx, wheel_r, omega_ref)

    slip_arc = None
    if d_track > 1e-6:
        slip_arc = max(0.0, min(1.0, 1.0 - (d_net / d_track)))

    tilt_stats = ts.results()

    metrics = {
        "elapsed_s": elapsed_s,
        "distance_m": d_track,
        "avg_speed_mps": avg_v,
        "energy_J": E,
        "energy_per_m_Jpm": Jpm,
        "cot": cot,
        "xcom_margin_m": xcom_m,
        "v_forward_avg_mps": v_fwd,
        "tilt_max_deg": tilt_stats.get("tilt_max_deg"),
        "tilt_rms_deg": tilt_stats.get("tilt_rms_deg"),
        "mass_eff_kg": m_eff,
        "gravity_mps2": g,
        "mu_eff": mu_eff,
        "slip_longitudinal": slip_long,
        "slip_arc_vs_net": slip_arc,
        "controller": getattr(ctrl.spec, "name", "unknown"),
        "commanded_vx_mps": float(args.vx),
    }

    print(
        f"[energy][proxy] d_net={d_net:.2f} m, d_track={d_track:.2f} m, "
        f"avg_v={avg_v:.2f} mps, v_fwd={v_fwd:.2f} mps, μeff={mu_eff:.2f}, "
        f"m={m_eff:.2f} kg, g={g:.2f} -> E≈{E:.1f} J, J/m={Jpm:.2f}, CoT={cot:.3f}, "
        f"xCOM={xcom_m:.3f} m, slip_long={('%.1f%%'%(slip_long*100)) if slip_long is not None else 'n/a'}, "
        f"slip_arc={('%.1f%%'%(slip_arc*100)) if slip_arc is not None else 'n/a'}"
    )

    if args.metrics_json:
        outp = Path(args.metrics_json).expanduser().resolve()
        outp.parent.mkdir(parents=True, exist_ok=True)
        with outp.open("w") as f:
            json.dump(metrics, f, indent=2)
        print(f"[metrics] wrote {outp}")

    # Clean shutdown
    try:
        sim.stop()
    finally:
        simulation_app.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
