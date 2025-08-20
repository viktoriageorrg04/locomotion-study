# South_Pole_DEMs/scripts/locomotion/constraints.py

from dataclasses import dataclass
from typing import Tuple, Optional, List
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf, Sdf

# Config

@dataclass
class ConstraintConfig:
    dust_mu_scale: float = 1.0               # multiply terrain dynamic/static friction
    robot_kg: float = 0.0                    # extra mass added to base (robot shell, sensors, etc.)
    payload_kg: float = 0.0                  # extra payload mass
    payload_offset_xyz: Tuple[float,float,float] = (0.0, 0.0, 0.0)
    check_los: bool = False                  # TODO: path-of-sight check (stub)
    energy_model: str = "proxy"              # "proxy" | "measured" (measured = out-of-scope here)

# Terrain friction

def _resolve_physics_material(stage: Usd.Stage, terrain_path: str = "/World/Terrain"):
    """Return (mat_prim, usd_api, physx_api) for terrain's bound physics material (robust to USD vs PhysX)."""
    terr = stage.GetPrimAtPath(terrain_path)
    if not terr or not terr.IsValid():
        return None, None, None

    mat_prim = terr
    rel = None
    try:
        rel = terr.GetRelationship("physics:material:binding")
    except Exception:
        pass
    if rel:
        targets = []
        try:
            rel.GetTargets(targets)
        except Exception:
            targets = []
        if targets:
            p = stage.GetPrimAtPath(targets[0])
            if p and p.IsValid():
                mat_prim = p

    usd_api = None
    physx_api = None
    try:
        if mat_prim.HasAPI(UsdPhysics.MaterialAPI):
            usd_api = UsdPhysics.MaterialAPI.Get(stage, mat_prim.GetPath())
    except Exception:
        pass
    try:
        if mat_prim.HasAPI(PhysxSchema.PhysxMaterialAPI):
            physx_api = PhysxSchema.PhysxMaterialAPI.Get(stage, mat_prim.GetPath())
    except Exception:
        pass

    return mat_prim, usd_api, physx_api


def _read_mu(usd_api, physx_api) -> Tuple[Optional[float], Optional[float]]:
    stat = dyn = None
    if usd_api:
        try:
            stat = usd_api.GetStaticFrictionAttr().Get()
            dyn  = usd_api.GetDynamicFrictionAttr().Get()
        except Exception:
            pass
    if physx_api:
        try:
            if stat is None:
                stat = physx_api.GetStaticFrictionAttr().Get()
            if dyn is None:
                dyn  = physx_api.GetDynamicFrictionAttr().Get()
        except Exception:
            pass
    return stat, dyn


def _write_mu(usd_api, physx_api, stat: float, dyn: float) -> bool:
    ok = False
    if usd_api:
        try:
            usd_api.GetStaticFrictionAttr().Set(float(stat))
            usd_api.GetDynamicFrictionAttr().Set(float(dyn))
            ok = True
        except Exception:
            pass
    if physx_api:
        try:
            physx_api.GetStaticFrictionAttr().Set(float(stat))
            physx_api.GetDynamicFrictionAttr().Set(float(dyn))
            ok = True
        except Exception:
            pass
    return ok


def get_terrain_friction(stage: Usd.Stage, terrain_path: str = "/World/Terrain") -> Tuple[float, float]:
    _, usd_api, physx_api = _resolve_physics_material(stage, terrain_path)
    stat, dyn = _read_mu(usd_api, physx_api)
    if stat is None: stat = 0.8
    if dyn  is None: dyn  = 0.6
    return float(stat), float(dyn)


def scale_terrain_friction(stage: Usd.Stage, scale: float, terrain_path: str = "/World/Terrain") -> bool:
    if abs(scale - 1.0) < 1e-6:
        return True

    mat_prim, usd_api, physx_api = _resolve_physics_material(stage, terrain_path)
    if mat_prim is None:
        print("[constraints] No terrain prim/material to scale.")
        return False

    stat0, dyn0 = _read_mu(usd_api, physx_api)
    if stat0 is None: stat0 = 0.8
    if dyn0  is None: dyn0  = 0.6

    stat_new = float(stat0) * float(scale)
    dyn_new  = float(dyn0)  * float(scale)
    ok = _write_mu(usd_api, physx_api, stat_new, dyn_new)
    api_kind = "USD+PhysX" if (usd_api and physx_api) else ("USD" if usd_api else ("PhysX" if physx_api else "none"))
    print(f"[constraints] ({api_kind}) Friction μd {dyn0:.3f}->{dyn_new:.3f}, μs {stat0:.3f}->{stat_new:.3f}")

    if not ok:
        # Fallback: author a USD Physics material directly on the terrain prim.
        terr = stage.GetPrimAtPath(terrain_path)
        try:
            usd2 = UsdPhysics.MaterialAPI.Apply(terr)
            usd2.GetStaticFrictionAttr().Set(stat_new)
            usd2.GetDynamicFrictionAttr().Set(dyn_new)
            print("[constraints] Authored USD Physics material on terrain as fallback.")
            return True
        except Exception:
            return False
    return True

def set_wheel_friction(stage, art_root: str, mu_s=1.5, mu_d=1.3):
    """
    Bind a physics material (via USD/PhysX *API schemas*) to all wheel collision
    shapes under `art_root` (e.g. /World/jackal/payload).
    """
    wheel_mat_path = "/World/Looks/WheelMaterial"

    # Create a container prim and author friction via APIs (no typed UsdPhysics.Material needed)
    mat_prim = stage.GetPrimAtPath(wheel_mat_path)
    if not mat_prim or not mat_prim.IsValid():
        mat_prim = UsdGeom.Scope.Define(stage, wheel_mat_path).GetPrim()

    # USD Physics friction
    usd_mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
    (usd_mat.GetStaticFrictionAttr()  or usd_mat.CreateStaticFrictionAttr()).Set(float(mu_s))
    (usd_mat.GetDynamicFrictionAttr() or usd_mat.CreateDynamicFrictionAttr()).Set(float(mu_d))

    # PhysX combine mode: multiply (keeps traction if one side is low)
    try:
        px_mat = PhysxSchema.PhysxMaterialAPI.Apply(mat_prim)
        (px_mat.GetFrictionCombineModeAttr() or px_mat.CreateFrictionCombineModeAttr()).Set("multiply")
    except Exception:
        pass

    def _bind(prim):
        try:
            UsdPhysics.MaterialBindingAPI.Apply(prim).Bind(mat_prim)
        except Exception:
            pass

    # Bind to colliders under anything that looks like a wheel link
    for prim in stage.Traverse():
        if art_root in prim.GetPath().pathString and "wheel" in prim.GetName().lower():
            for child in prim.GetChildren():
                n = child.GetName().lower()
                if "collision" in n or "collisions" in n:
                    for sub in child.GetChildren():
                        if sub.IsA(UsdGeom.Xform) or sub.IsA(UsdGeom.Gprim) or sub.IsA(UsdGeom.Mesh):
                            _bind(sub)

# Mass / payload

def _find_base_link(stage: Usd.Stage, art_root: str) -> Optional[Usd.Prim]:
    """Heuristic: prefer a descendant with 'base' in name that has MassAPI; else first MassAPI Xform."""
    art = stage.GetPrimAtPath(art_root)
    if not art or not art.IsValid():
        return None
    best = None
    for p in Usd.PrimRange(art):
        if not p.IsA(UsdGeom.Xform):
            continue
        try:
            if p.HasAPI(UsdPhysics.MassAPI):
                if "base" in p.GetName().lower():
                    return p
                if best is None:
                    best = p
        except Exception:
            pass
    return best


def add_payload_mass(stage: Usd.Stage, art_root: str, payload_kg: float) -> bool:
    """Increase the base link mass by payload_kg. If no mass is authored yet, author one."""
    if payload_kg <= 0.0:
        return True
    base = _find_base_link(stage, art_root)
    if base is None:
        print(f"[constraints][WARN] Could not find a base link under {art_root} to add payload.")
        return False
    try:
        mapi = UsdPhysics.MassAPI.Apply(base) if not base.HasAPI(UsdPhysics.MassAPI) else UsdPhysics.MassAPI(base)
        cur = mapi.GetMassAttr().Get() or 0.0
        mapi.GetMassAttr().Set(float(cur) + float(payload_kg))
        print(f"[constraints] Added payload mass: +{payload_kg:.3f} kg on {base.GetPath().pathString} (now {cur+payload_kg:.3f} kg).")
        return True
    except Exception as e:
        print(f"[constraints][ERROR] add_payload_mass failed: {e}")
        return False


def total_articulation_mass_kg(stage: Usd.Stage, art_root: str) -> float:
    """Sum all authored MassAPI.mass under the articulation root."""
    root = stage.GetPrimAtPath(art_root)
    if not root or not root.IsValid():
        return 0.0
    total = 0.0
    for p in Usd.PrimRange(root):
        try:
            if p.HasAPI(UsdPhysics.MassAPI):
                m = UsdPhysics.MassAPI(p).GetMassAttr().Get()
                if m is not None:
                    total += float(m)
        except Exception:
            pass
    return total


def effective_rover_mass_kg(stage: Usd.Stage, art_root: str, cfg: ConstraintConfig) -> float:
    """Mass used for energy proxy: authored link masses + robot_kg + payload_kg."""
    return total_articulation_mass_kg(stage, art_root) + float(cfg.robot_kg) + float(cfg.payload_kg)

# LOS (stub)

def los_ok(stage: Usd.Stage, src_path: str, dst_path: str) -> bool:
    # TODO: implement a real ray cast using omni.physx query or RTX ray queries
    return True

# Pose & energy

def world_translation(stage: Usd.Stage, prim_path: str) -> Optional[Gf.Vec3d]:
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    xf = UsdGeom.Xformable(prim)
    try:
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    except Exception:
        return None
    return Gf.Vec3d(m.ExtractTranslation())


def distance_from(stage: Usd.Stage, prim_path: str, start_pos: Optional[Gf.Vec3d]) -> float:
    if start_pos is None:
        return 0.0
    end = world_translation(stage, prim_path)
    if end is None:
        return 0.0
    return float((end - start_pos).GetLength())


def get_gravity_mps2(stage: Usd.Stage, default_g: float = 9.81) -> float:
    """Read gravity magnitude from the first UsdPhysics.Scene found, else fallback."""
    try:
        for p in Usd.PrimRange(stage.GetPseudoRoot()):
            if p.IsA(UsdPhysics.Scene):
                sc = UsdPhysics.Scene(p)
                mag = None
                try:
                    mag = sc.GetGravityMagnitudeAttr().Get()
                except Exception:
                    mag = None
                if mag is not None and float(mag) > 0:
                    return float(mag)
    except Exception:
        pass
    return float(default_g)


def proxy_energy_J(distance_m: float, mass_kg: float, mu: float, g: float) -> float:
    """Rolling/sliding-loss proxy: E ≈ μ * m * g * d"""
    return float(mu) * float(mass_kg) * float(g) * float(distance_m)

# Rover feature extraction (USD)

@dataclass
class RoverFeatures:
    base_link_path: Optional[str]
    total_mass_kg: float
    wheel_radius_m: Optional[float]
    wheel_base_m: Optional[float]
    drive_joint_paths: List[str]
    drive_target_omega: Optional[float]   # rad/s (authored target if present)
    drive_max_omega: Optional[float]      # rad/s (authored limit if present)


def _prim_local_extent_radius(prim: Usd.Prim) -> Optional[float]:
    """Approximate radius from local AABB (useful for meshes)."""
    try:
        mesh = UsdGeom.Mesh(prim)
        if mesh:
            ext = mesh.GetExtentAttr().Get()
            if ext and len(ext) == 2:
                mn, mx = ext
                rx = 0.5 * (mx[0] - mn[0])
                ry = 0.5 * (mx[1] - mn[1])
                return float(max(abs(rx), abs(ry)))
    except Exception:
        pass
    return None


def _estimate_wheels(stage: Usd.Stage, art_root: str) -> Tuple[List[Usd.Prim], Optional[float], Optional[float]]:
    """Find wheel-like prims, estimate radius & wheelbase from geometry/positions."""
    root = stage.GetPrimAtPath(art_root)
    if not root or not root.IsValid():
        return [], None, None

    wheels: List[Usd.Prim] = []
    for p in Usd.PrimRange(root):
        name = p.GetName().lower()
        try:
            if "wheel" in name or p.IsA(UsdGeom.Cylinder):
                wheels.append(p)
        except Exception:
            pass

    radii = []
    positions = []
    for w in wheels:
        r = None
        try:
            cyl = UsdGeom.Cylinder(w)
            if cyl:
                r = cyl.GetRadiusAttr().Get()
        except Exception:
            pass
        if r is None:
            r = _prim_local_extent_radius(w)
        if r is not None and r > 0:
            radii.append(float(r))
        try:
            xf = UsdGeom.Xformable(w).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            positions.append(Gf.Vec3d(xf.ExtractTranslation()))
        except Exception:
            pass

    wheel_radius = float(sorted(radii)[len(radii)//2]) if radii else None  # median for robustness

    wheel_base = None
    if len(positions) >= 2:
        xs = [p[0] for p in positions]
        ys = [p[1] for p in positions]
        spread_x = max(xs) - min(xs)
        spread_y = max(ys) - min(ys)
        axis = 0 if spread_x >= spread_y else 1
        coords = xs if axis == 0 else ys
        wheel_base = float(max(coords) - min(coords))

    return wheels, wheel_radius, wheel_base


def _find_drive_joints(stage: Usd.Stage, art_root: str) -> Tuple[List[str], Optional[float], Optional[float]]:
    """
    Collect joints that have a DriveAPI (typical for wheel motors). Return:
      - paths of those joints
      - a representative authored target ω (rad/s) if present
      - a representative authored max ω (rad/s) if present
    Also scans raw attribute names containing 'targetVelocity'/'maxVelocity' to be schema-agnostic.
    """
    root = stage.GetPrimAtPath(art_root)
    if not root or not root.IsValid():
        return [], None, None

    joint_paths: List[str] = []
    target_candidates: List[float] = []
    max_candidates: List[float] = []

    for p in Usd.PrimRange(root):
        try:
            has_joint = p.IsA(UsdPhysics.RevoluteJoint) or p.IsA(UsdPhysics.PrismaticJoint) or p.HasAPI(UsdPhysics.Joint)
            if not has_joint:
                continue

            # If DriveAPI exists, read its attributes
            if p.HasAPI(UsdPhysics.DriveAPI):
                joint_paths.append(p.GetPath().pathString)
                dapi = UsdPhysics.DriveAPI(p)
                tv = None
                try:
                    tv = dapi.GetTargetVelocityAttr().Get()
                except Exception:
                    tv = None
                if tv is not None:
                    target_candidates.append(float(tv))

                # Some schemas have no explicit maxVelocity on DriveAPI; try PhysX or raw attrs.
                mv = None
                try:
                    # PhysX drive?
                    if p.HasAPI(PhysxSchema.PhysxDriveAPI):
                        mv = PhysxSchema.PhysxDriveAPI(p).GetMaxVelocityAttr().Get()
                except Exception:
                    mv = None
                if mv is not None:
                    max_candidates.append(float(mv))

            # Schema-agnostic fallback: scan attribute names
            for attr in p.GetAttributes():
                n = attr.GetName().lower()
                if "targetvelocity" in n:
                    val = attr.Get()
                    if isinstance(val, (int, float)):
                        target_candidates.append(float(val))
                        joint_paths.append(p.GetPath().pathString)
                if "maxvelocity" in n:
                    val = attr.Get()
                    if isinstance(val, (int, float)):
                        max_candidates.append(float(val))
                        joint_paths.append(p.GetPath().pathString)
        except Exception:
            pass

    # Deduplicate paths while preserving order
    seen = set()
    joint_paths = [j for j in joint_paths if not (j in seen or seen.add(j))]

    tgt = (sorted(target_candidates)[len(target_candidates)//2] if target_candidates else None)  # median
    vmax = (sorted(max_candidates)[len(max_candidates)//2] if max_candidates else None)

    return joint_paths, tgt, vmax


def rover_features(stage: Usd.Stage, art_root: str) -> 'RoverFeatures':
    base = _find_base_link(stage, art_root)
    total_mass = total_articulation_mass_kg(stage, art_root)
    _, wheel_r, wheelbase = _estimate_wheels(stage, art_root)
    drive_paths, tgt, vmax = _find_drive_joints(stage, art_root)
    return RoverFeatures(
        base_link_path=base.GetPath().pathString if base else None,
        total_mass_kg=float(total_mass),
        wheel_radius_m=wheel_r,
        wheel_base_m=wheelbase,
        drive_joint_paths=drive_paths,
        drive_target_omega=tgt,
        drive_max_omega=vmax,
    )

# Metrics helpers

def cost_of_transport(E_J: float, mass_kg: float, g: float, distance_m: float) -> Optional[float]:
    if mass_kg <= 0 or g <= 0 or distance_m <= 0 or E_J is None:
        return None
    return float(E_J) / (float(mass_kg) * float(g) * float(distance_m))


def slip_ratio_estimate(avg_v_mps: float, wheel_radius_m: Optional[float], omega_rad_s: Optional[float]) -> Optional[float]:
    """i = (r*ω - v) / (r*ω), clipped to [0,1]; returns None if not computable."""
    if wheel_radius_m is None or omega_rad_s is None or wheel_radius_m <= 0:
        return None
    ideal = wheel_radius_m * omega_rad_s
    if ideal <= 1e-6:
        return None
    return max(0.0, min(1.0, (ideal - max(0.0, avg_v_mps)) / ideal))


def body_upright_tilt_deg(stage: Usd.Stage, prim_path: str) -> Optional[float]:
    """Angle between body +Z and world +Z, in degrees (0 = upright)."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    try:
        xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # transform body Z into world, then angle to world Z
        z_world_from_bodyZ = (xf.TransformDir(Gf.Vec3d(0, 0, 1))).GetNormalized()
        cosang = max(-1.0, min(1.0, float(Gf.Dot(z_world_from_bodyZ, Gf.Vec3d(0, 0, 1)))))
        return float(Gf.RadToDeg(Gf.Acos(cosang)))
    except Exception:
        return None

# “Static” proxy metrics

def static_proxy_metrics(stage: Usd.Stage, art_root: str, cfg: ConstraintConfig, distance_m: float, avg_v_mps: float):
    """
    Quick metrics using authored USD + proxy energy model (no runtime sensors):
      - μd from terrain, g from physics scene, m from USD + cfg masses
      - Energy per meter J/m = μd * m * g  (CoT == μd)
      - Slip estimate if a target ω is authored: i = (r*ω - v)/(r*ω)
    """
    mu_s, mu_d = get_terrain_friction(stage, "/World/Terrain")
    g = get_gravity_mps2(stage)
    m_eff = effective_rover_mass_kg(stage, art_root, cfg)
    feats = rover_features(stage, art_root)

    E_J = proxy_energy_J(distance_m=distance_m, mass_kg=m_eff, mu=mu_d, g=g) if distance_m > 0 else 0.0
    J_per_m = mu_d * m_eff * g
    cot = mu_d
    slip_i = slip_ratio_estimate(avg_v_mps, feats.wheel_radius_m, feats.drive_target_omega)

    return {
        "mu_d": mu_d,
        "mu_s": mu_s,
        "g_mps2": g,
        "mass_eff_kg": m_eff,
        "features": feats,
        "energy_J": E_J,
        "energy_per_m_Jpm": J_per_m,
        "cot": cot,
        "slip_ratio_est": slip_i,
    }

# Main entry

def apply_constraints(stage: Usd.Stage, art_root: str, cfg: ConstraintConfig):
    # Dust/friction
    scale_terrain_friction(stage, cfg.dust_mu_scale, "/World/Terrain")
    # Add masses
    add_payload_mass(stage, art_root, cfg.robot_kg)
    add_payload_mass(stage, art_root, cfg.payload_kg)

    # NEW: give wheels decent friction so they actually grip
    try:
        set_wheel_friction(stage, art_root, mu_s=1.5, mu_d=1.3)
        print("[constraints] Wheel friction bound (μs=1.5, μd=1.3) on wheel colliders.")
    except Exception as e:
        print(f"[constraints][WARN] wheel friction binding failed: {e}")
