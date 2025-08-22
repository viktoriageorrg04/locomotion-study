# South_Pole_DEMs/scripts/locomotion/constraints.py

from dataclasses import dataclass
from typing import Tuple, Optional, List
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, Gf

# Config

@dataclass
class ConstraintConfig:
    dust_mu_scale: float = 1.0               # multiply terrain dynamic/static friction
    robot_kg: float = 0.0                    # extra mass added to base (robot shell, sensors, etc.)
    payload_kg: float = 0.0                  # extra payload mass
    payload_offset_xyz: Tuple[float,float,float] = (0.0, 0.0, 0.0)
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

def get_wheel_friction(stage: Usd.Stage, wheel_mat_path: str = "/World/Looks/WheelMaterial"):
    """Return (μs, μd, combine_mode_str) from the wheel material if present."""
    prim = stage.GetPrimAtPath(wheel_mat_path)
    if not prim or not prim.IsValid():
        return None, None, None
    usd_api = UsdPhysics.MaterialAPI.Get(stage, prim.GetPath()) if prim.HasAPI(UsdPhysics.MaterialAPI) else None
    physx_api = PhysxSchema.PhysxMaterialAPI.Get(stage, prim.GetPath()) if prim.HasAPI(PhysxSchema.PhysxMaterialAPI) else None

    stat, dyn = _read_mu(usd_api, physx_api)
    combine = None
    try:
        if physx_api and physx_api.GetFrictionCombineModeAttr():
            combine = physx_api.GetFrictionCombineModeAttr().Get()
    except Exception:
        pass
    return (float(stat) if stat is not None else None,
            float(dyn) if dyn is not None else None,
            (combine or "").lower() or None)

def effective_contact_mu(stage: Usd.Stage) -> float:
    """PhysX-style effective μd using terrain and wheel material with combine mode."""
    _, terr_mu_d = get_terrain_friction(stage, "/World/Terrain")
    w_mu_s, w_mu_d, combine = get_wheel_friction(stage)
    if w_mu_d is None:
        return float(terr_mu_d)
    combine = (combine or "min").lower()
    if combine == "multiply":
        return float(terr_mu_d) * float(w_mu_d)
    if combine in ("min",):
        return min(float(terr_mu_d), float(w_mu_d))
    if combine in ("max",):
        return max(float(terr_mu_d), float(w_mu_d))
    if combine in ("average", "avg", "mean"):
        return 0.5 * (float(terr_mu_d) + float(w_mu_d))
    # Fallback: conservative
    return min(float(terr_mu_d), float(w_mu_d))

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

def get_gravity_mps2(stage, default_g: float = 9.81) -> float:
    """Return gravity magnitude (m/s^2) from the stage. Falls back to default_g."""
    try:
        from pxr import UsdPhysics, PhysxSchema, Gf

        # Try PhysX gravity vector first (most reliable during runtime)
        prim = stage.GetPrimAtPath("/World/physicsScene")
        if prim:
            api = PhysxSchema.PhysxSceneAPI(prim)
            if api:
                g_vec = api.GetGravityAttr().Get()
                if g_vec is not None:
                    return float(Gf.Vec3f(g_vec).GetLength())

        # Fallback: USD Physics gravity magnitude attribute
        ps = UsdPhysics.Scene.Get(stage, "/World/physicsScene")
        if ps:
            mag_attr = ps.GetGravityMagnitudeAttr()
            if mag_attr and mag_attr.HasAuthoredValue():
                mag = mag_attr.Get()
                if mag is not None:
                    return abs(float(mag))
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

class TiltSampler:
    """Collect max & RMS tilt (upright deviation) while sim runs."""
    def __init__(self, stage, body_prim: str):
        self.stage = stage
        self.body  = body_prim
        self.max_deg = 0.0
        self.sum_sq  = 0.0
        self.n       = 0

    def sample(self):
        t = body_upright_tilt_deg(self.stage, self.body)
        if t is None:
            return
        if t > self.max_deg:
            self.max_deg = t
        self.sum_sq += t * t
        self.n += 1

    def results(self):
        rms = (self.sum_sq / self.n) ** 0.5 if self.n > 0 else None
        return {"tilt_max_deg": self.max_deg, "tilt_rms_deg": rms}


class ForwardSpeedSampler:
    """Estimate forward speed by projecting world velocity onto the body +X axis."""
    def __init__(self, stage, body_path: str, dt: float):
        self.stage = stage
        self.body_path = body_path
        self.dt = dt
        self.prev_pos = world_translation(stage, body_path)
        self.sum = 0.0
        self.n = 0

    def sample(self):
        cur_pos = world_translation(self.stage, self.body_path)
        if self.prev_pos is None or cur_pos is None:
            self.prev_pos = cur_pos
            return
        try:
            xf = UsdGeom.Xformable(self.stage.GetPrimAtPath(self.body_path)).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            fwd = xf.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0)).GetNormalized()
        except Exception:
            fwd = Gf.Vec3d(1.0, 0.0, 0.0)
        v_vec = (cur_pos - self.prev_pos) * (1.0 / max(self.dt, 1e-6))
        from pxr import Gf as _Gf
        v_fwd = float(_Gf.Dot(v_vec, fwd))
        self.sum += v_fwd
        self.n += 1
        self.prev_pos = cur_pos

    def results(self):
        return {"v_forward_avg_mps": (self.sum / self.n) if self.n > 0 else 0.0}

def slip_metrics(v_forward_mps: float, vx_cmd_mps: float, wheel_r_m: float, omega_target_rad_s: Optional[float] = None):
    """
    Return (slip_long_0to1_or_None, v_ref_mps).
    v_ref = r*omega_target (if provided) else commanded vx.
    """
    v_ref = None
    if wheel_r_m and omega_target_rad_s is not None:
        v_ref = float(wheel_r_m) * float(omega_target_rad_s)
    elif vx_cmd_mps is not None:
        v_ref = float(vx_cmd_mps)
    if v_ref is None or abs(v_ref) < 1e-6:
        return None, v_ref
    slip_long = max(0.0, min(1.0, (v_ref - float(v_forward_mps)) / max(abs(v_ref), 1e-6)))
    return slip_long, v_ref

def _convex_hull_xy(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    # Andrew’s monotone chain; returns CCW hull
    pts = sorted(set(points))
    if len(pts) <= 1:
        return pts
    def cross(o, a, b): return (a[0]-o[0])*(b[1]-o[1]) - (a[1]-o[1])*(b[0]-o[0])
    lower = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0: lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0: upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]

def _point_in_convex_polygon(p: Tuple[float,float], hull: List[Tuple[float,float]]) -> bool:
    if len(hull) < 3: return False
    sign = None
    for i in range(len(hull)):
        a, b = hull[i], hull[(i+1) % len(hull)]
        cross = (b[0]-a[0])*(p[1]-a[1]) - (b[1]-a[1])*(p[0]-a[0])
        if cross == 0:  # on edge
            continue
        s = cross > 0
        if sign is None: sign = s
        elif s != sign: return False
    return True

def _point_to_segment_distance(p: Tuple[float,float], a: Tuple[float,float], b: Tuple[float,float]) -> float:
    import math
    px, py = p; ax, ay = a; bx, by = b
    abx, aby = bx-ax, by-ay
    apx, apy = px-ax, py-ay
    ab2 = abx*abx + aby*aby
    t = 0.0 if ab2 == 0 else max(0.0, min(1.0, (apx*abx + apy*aby) / ab2))
    cx, cy = ax + t*abx, ay + t*aby
    return math.hypot(px - cx, py - cy)

def _point_to_polygon_distance(p: Tuple[float,float], hull: List[Tuple[float,float]]) -> float:
    if len(hull) == 0: return float('inf')
    if len(hull) == 1:
        import math
        return math.hypot(p[0]-hull[0][0], p[1]-hull[0][1])
    if len(hull) == 2: return _point_to_segment_distance(p, hull[0], hull[1])
    dmin = float('inf')
    for i in range(len(hull)):
        dmin = min(dmin, _point_to_segment_distance(p, hull[i], hull[(i+1) % len(hull)]))
    return dmin

def estimate_support_polygon_xy(stage: Usd.Stage, art_root: str) -> List[Tuple[float,float]]:
    """
    Collect likely ground-contact link XY under art_root: wheels or feet.
    Fallback = base aligned-bbox footprint. Returns convex hull (CCW).
    """
    prim_root = stage.GetPrimAtPath(art_root)
    pts: List[Tuple[float,float]] = []
    if prim_root and prim_root.IsValid():
        tcode = Usd.TimeCode.Default()
        # wheels / feet / toes / ankles
        for prim in stage.Traverse():
            path = prim.GetPath().pathString
            if not path.startswith(art_root): continue
            n = prim.GetName().lower()
            if any(k in n for k in ("wheel", "foot", "toe", "ankle")):
                try:
                    xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(tcode)
                    tr = xf.ExtractTranslation()
                    pts.append((float(tr[0]), float(tr[1])))
                except Exception:
                    pass
        if len(pts) < 3:
            try:
                bbox = UsdGeom.BBoxCache(tcode, ["default"]).ComputeWorldBound(prim_root)
                aabb = bbox.ComputeAlignedBox()
                mn, mx = aabb.GetMin(), aabb.GetMax()
                pts = [(float(mn[0]), float(mn[1])),
                       (float(mx[0]), float(mn[1])),
                       (float(mx[0]), float(mx[1])),
                       (float(mn[0]), float(mx[1]))]
            except Exception:
                pts = []
    return _convex_hull_xy(pts) if pts else pts

def world_center_of_mass(stage: Usd.Stage, art_root: str) -> Tuple[Optional[Gf.Vec3d], float]:
    """
    Mass-weighted world CoM (x,y,z) using authored UsdPhysics.MassAPI.mass on links.
    Falls back to the articulation root transform if masses are missing.
    """
    root = stage.GetPrimAtPath(art_root)
    if not root or not root.IsValid():
        return None, 0.0
    tcode = Usd.TimeCode.Default()
    m_sum = 0.0
    p_sum = Gf.Vec3d(0.0, 0.0, 0.0)
    for p in Usd.PrimRange(root):
        try:
            if p.HasAPI(UsdPhysics.MassAPI):
                m = UsdPhysics.MassAPI(p).GetMassAttr().Get()
                if m is None or m <= 0:
                    continue
                xf = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(tcode)
                tr = xf.ExtractTranslation()
                p_sum += Gf.Vec3d(tr) * float(m)
                m_sum += float(m)
        except Exception:
            pass
    if m_sum > 0.0:
        return (p_sum / m_sum), m_sum
    # Fallback: use articulation root pose
    try:
        xf = UsdGeom.Xformable(root).ComputeLocalToWorldTransform(tcode)
        return Gf.Vec3d(xf.ExtractTranslation()), 0.0
    except Exception:
        return None, 0.0

def xcom_margin_metric(stage: Usd.Stage, art_root: str, v_mps: float, g: float = 9.81):
    """
    Capture-point (Extrapolated-CoM) margin [m]:
      margin = d_in − v/ω0, with ω0 = sqrt(g / z_com)
      d_in = shortest distance from CoM projection to support polygon edge (>=0 if inside).
    Uses mass-weighted world CoM for z_com and (x,y).
    Returns (margin_m, info_dict).
    """
    com, _m = world_center_of_mass(stage, art_root)
    if com is None:
        return 0.0, {"error": "CoM not available"}
    z_com = max(0.05, float(com[2]))
    pxy = (float(com[0]), float(com[1]))

    omega0 = (g / z_com) ** 0.5
    r = max(0.0, float(v_mps)) / max(1e-6, omega0)

    hull = estimate_support_polygon_xy(stage, art_root)
    if not hull:
        return -r, {"r_xcom": r, "z_com": z_com, "d_in": 0.0, "inside": False, "n_support": 0}

    inside = _point_in_convex_polygon(pxy, hull)
    d_in = _point_to_polygon_distance(pxy, hull)
    margin = (d_in if inside else -d_in) - r
    return margin, {"r_xcom": r, "z_com": z_com, "d_in": d_in, "inside": inside, "n_support": len(hull)}

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
    xcom_m, _xinfo = xcom_margin_metric(stage, art_root, avg_v_mps, g=g)

    return {
        "distance_m": distance_m,
        "avg_v_mps": avg_v_mps,
        "energy_J": E_J,
        "energy_per_m_Jpm": J_per_m,
        "CoT": cot,
        "xcom_margin_m": float(xcom_m),
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
