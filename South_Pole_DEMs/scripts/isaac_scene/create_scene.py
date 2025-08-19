# #!/usr/bin/env python3
# import argparse
# import sys
# from pathlib import Path

# # ---------- CLI ----------
# def parse_args():
#     p = argparse.ArgumentParser("Launch terrain + spawn wheeled robot + run straight-line baseline")
#     # scene
#     p.add_argument("--usd", required=True, help="Terrain USD to open")
#     p.add_argument("--renderer", default="RayTracedLighting", choices=["RayTracedLighting","PathTracing"])
#     p.add_argument("--headless", action="store_true")
#     p.add_argument("--viewport", choices=["stage","camera"], default="stage")

#     # lighting
#     p.add_argument("--black-sky", action="store_true")
#     p.add_argument("--sun-az", type=float, default=120.0)
#     p.add_argument("--sun-el", type=float, default=8.0)
#     p.add_argument("--sun-intensity", type=float, default=3000.0)
#     p.add_argument("--sun-exposure", type=float, default=0.01)
#     p.add_argument("--fill-ratio", type=float, default=0.0)
#     p.add_argument("--pt-spp", type=int, default=128)
#     p.add_argument("--pt-max-bounces", type=int, default=6)
#     p.add_argument("--skip-material", action="store_true")

#     # robot spawn
#     p.add_argument("--asset-usd", required=True, help="Robot USD path (or omniverse URL)")
#     p.add_argument("--asset-name", default="rover")
#     p.add_argument("--asset-x", type=float, default=0.0)
#     p.add_argument("--asset-y", type=float, default=0.0)
#     p.add_argument("--asset-yaw", type=float, default=0.0)
#     p.add_argument("--asset-center", action="store_true", help="Spawn at terrain center XY")
#     p.add_argument("--drop-margin", type=float, default=0.02)
#     p.add_argument("--moon", action="store_true", help="Use Moon gravity/material")

#     # baseline motion (diff drive straight line)
#     p.add_argument("--vx", type=float, default=0.4)
#     p.add_argument("--yaw-rate", type=float, default=0.0)
#     p.add_argument("--duration", type=float, default=10.0)
#     p.add_argument("--hz", type=float, default=60.0)
#     p.add_argument("--wheel-radius", type=float, default=0.098)
#     p.add_argument("--wheel-base", type=float, default=0.33)

#     # constraints + output
#     p.add_argument("--out-root", default="runs")
#     p.add_argument("--dust-mu-scale", type=float, default=1.0)
#     p.add_argument("--payload-kg", type=float, default=0.0)
#     p.add_argument("--payload-offset", type=str, default="0,0,0")
#     p.add_argument("--energy-model", choices=["proxy","measured"], default="proxy")
#     p.add_argument("--require-los", action="store_true")
#     return p.parse_args()


# def main():
#     args = parse_args()
#     print("[launch_isaac] Args:", args)

#     # ---------- Isaac bootstrap (no omni imports before this) ----------
#     from isaacsim.simulation_app import SimulationApp
#     simulation_app = SimulationApp({"renderer": args.renderer, "headless": args.headless})

#     # project import paths so we can import sibling/helper modules
#     _THIS = Path(__file__).resolve()
#     _SCRIPTS = _THIS.parent.parent   # .../scripts
#     if str(_SCRIPTS) not in sys.path:
#         sys.path.insert(0, str(_SCRIPTS))

#     # now safe to import isaac/pxr + your helpers
#     import omni.usd
#     from pxr import Usd
#     # helpers in this folder
#     from create_scene import open_stage, get_stage, frame_view_on_prim  # uses omni.usd context
#     from set_lighting import (configure_renderer_settings,
#                               disable_viewport_light_rig,
#                               disable_dome_and_skylights,
#                               set_sun,
#                               add_fill_light)
#     from set_regolith import make_and_bind_regolith
#     from spawn_rover import ensure_physics_scene, apply_terrain_physics, spawn_asset

#     # locomotion + logging (your utils)
#     from locomotion.straight_line import run_diff_drive  # baseline runner (diff)  :contentReference[oaicite:2]{index=2}
#     from locomotion.logging import ensure_run_dir, write_jsonl, write_csv_header_if_needed  # :contentReference[oaicite:3]{index=3}
#     from locomotion.constraints import ConstraintConfig, apply_constraints  # your constraints hook

#     # ---------- Open stage ----------
#     stage_path = str(Path(args.usd).expanduser().resolve())
#     open_stage(stage_path)
#     stage: Usd.Stage = get_stage()
#     stage.SetTimeCodesPerSecond(60)
#     print(f"[launch_isaac] Opened stage: {stage_path}")

#     # ---------- Lighting ----------
#     configure_renderer_settings(args)
#     if args.viewport == "stage":
#         disable_viewport_light_rig()
#     disable_dome_and_skylights(black_sky=args.black_sky)
#     set_sun(az_deg=args.sun_az, el_deg=args.sun_el,
#             intensity=args.sun_intensity, exposure=args.sun_exposure)
#     if args.fill_ratio > 0.0:
#         add_fill_light(ratio=args.fill_ratio)

#     # ---------- Terrain material/physics ----------
#     if not args.skip_material:
#         ok = make_and_bind_regolith()
#         print(f"[launch_isaac] make_and_bind_regolith returned: {ok}")
#     ensure_physics_scene(moon=args.moon)
#     apply_terrain_physics("/World/Terrain", moon=args.moon)

#     # ---------- Spawn robot ----------
#     spawn_x = 0.0 if args.asset_center else args.asset_x
#     spawn_y = 0.0 if args.asset_center else args.asset_y
#     prim_path = spawn_asset(
#         stage,
#         usd_path=args.asset_usd,
#         name=args.asset_name,
#         x=spawn_x, y=spawn_y, yaw_deg=args.asset_yaw,
#         drop_margin=args.drop_margin,
#         terrain_path="/World/Terrain",
#         orient_to_slope=True,
#     )
#     print(f"[launch_isaac] Spawned asset at {prim_path}")

#     frame_view_on_prim("/World/Terrain")  # best-effort

#     # ---------- Constraints ----------
#     ox, oy, oz = map(float, args.payload_offset.split(","))
#     cfg = ConstraintConfig(
#         dust_mu_scale=args.dust_mu_scale,
#         payload_kg=args.payload_kg,
#         payload_offset_xyz=(ox, oy, oz),
#         check_los=args.require_los,
#         energy_model=args.energy_model,
#     )
#     apply_constraints(simulation_app, stage, prim_path, cfg)

#     # ---------- Sim warmup ----------
#     from isaacsim.core.api import SimulationContext
#     sim = SimulationContext()
#     sim.set_simulation_dt(1.0 / args.hz)
#     sim.play()
#     for _ in range(30):  # let referenced USDs settle
#         sim.step(render=False)

#     # ---------- Auto-pick wheel joints ----------
#     from isaacsim.core.prims import SingleArticulation
#     art = SingleArticulation(prim_path=prim_path)
#     art.initialize()
#     name_map = {n.lower(): n for n in art.dof_names}
#     lows = list(name_map.keys())

#     def pick(side):
#         # collect joints that look like wheel/drive on this side
#         h = [name_map[k] for k in lows if side in k and ("wheel" in k or "drive" in k)]
#         return h

#     left_wheels = pick("left")
#     right_wheels = pick("right")
#     if not left_wheels or not right_wheels:
#         # fallback example (Clearpath Jackal)
#         left_wheels  = ["front_left_wheel_joint", "rear_left_wheel_joint"]
#         right_wheels = ["front_right_wheel_joint", "rear_right_wheel_joint"]

#     print("[launch_isaac] DOFs:", list(art.dof_names))
#     print("[launch_isaac] Wheels L:", left_wheels, "R:", right_wheels)

#     # ---------- Baseline: straight line diff-drive ----------
#     metrics = run_diff_drive(
#         prim_path=prim_path,
#         left_wheels=left_wheels,
#         right_wheels=right_wheels,
#         vx=args.vx,
#         yaw_rate=args.yaw_rate,
#         duration=args.duration,
#         hz=args.hz,
#         wheel_radius=args.wheel_radius,
#         wheel_base=args.wheel_base,
#     )
#     if metrics is not None:
#         print("[baseline metrics]", metrics)

#     # ---------- Log ----------
#     run_dir = ensure_run_dir(args.out_root)
#     jsonl = run_dir / "metrics.jsonl"
#     csv = run_dir / "metrics.csv"
#     # stable superset header for csv
#     fieldnames = ["run_id","robot","vx_cmd","duration_s",
#                   "elapsed_s","distance_m","avg_speed_mps",
#                   "energy_J","energy_per_m_Jpm","slip_ratio_est","success"]
#     f, w = write_csv_header_if_needed(csv, fieldnames)
#     rec = {"run_id": run_dir.name, "robot": args.asset_name,
#            "vx_cmd": args.vx, "duration_s": args.duration}
#     if isinstance(metrics, dict):
#         rec.update(metrics)
#     write_jsonl(jsonl, rec); w.writerow(rec); f.close()
#     print(f"[metrics] saved to {run_dir}")

#     # ---------- Shutdown ----------
#     simulation_app.close()


# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
import argparse
from pathlib import Path
import shutil
import sys

def parse_args():
    p = argparse.ArgumentParser("Create terrain USD from a DEM patch (stub: copies from usd_patches)")
    p.add_argument("--tif", required=True, help="Path to the *_enhanced.tif patch")
    p.add_argument("--out", required=True, help="Output USD path to write")
    p.add_argument("--z-mode", default="relative", help="Ignored (kept for compatibility)")
    p.add_argument("--z-exag", type=float, default=1.0, help="Ignored (kept for compatibility)")
    return p.parse_args()

def main():
    args = parse_args()
    tif = Path(args.tif).resolve()
    out_usd = Path(args.out).resolve()

    if not tif.exists():
        print(f"[create_scene] ERROR: TIF does not exist: {tif}", file=sys.stderr)
        return 2

    # repo root: South_Pole_DEMs/
    # This file = South_Pole_DEMs/scripts/isaac_scene/create_scene.py
    # parents[0]=isaac_scene, [1]=scripts, [2]=South_Pole_DEMs
    try:
        repo_root = Path(__file__).resolve().parents[2]
    except Exception:
        repo_root = Path(__file__).resolve().parent.parent.parent

    usd_patches = repo_root / "usd_patches"
    candidate = usd_patches / (tif.stem + ".usda")

    print(f"[create_scene] TIF: {tif}")
    print(f"[create_scene] Repo root: {repo_root}")
    print(f"[create_scene] Looking for USD: {candidate}")

    if not candidate.exists():
        print(f"[create_scene] ERROR: No matching USD found at {candidate}", file=sys.stderr)
        print(f"[create_scene] Available USDs:", file=sys.stderr)
        for u in sorted(usd_patches.glob("*.usda")):
            print(f"  - {u}", file=sys.stderr)
        return 3

    out_usd.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(candidate, out_usd)
    print(f"[create_scene] Wrote: {out_usd}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
