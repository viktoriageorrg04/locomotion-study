# spawn_rover.py
import math
from pxr import Usd, UsdGeom, UsdPhysics, Gf, Sdf
import numpy as np

# ---------- USD mesh-based ground sampler (robust) ----------
_GRID_CACHE = {}  # keyed by (stage.GetRootLayer().identifier, terrain_path)

def _terrain_grid_cache(stage, terrain_path):
    key = (stage.GetRootLayer().identifier, terrain_path)
    if key in _GRID_CACHE:
        return _GRID_CACHE[key]

    prim = stage.GetPrimAtPath(terrain_path)
    if not prim or not prim.IsA(UsdGeom.Mesh):
        return None

    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get() or []
    if not pts:
        return None

    # Points are in local space of the mesh
    P = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
    xs = np.unique(P[:, 0])
    ys = np.unique(P[:, 1])
    W, H = len(xs), len(ys)
    if W * H != len(P):
        # Not a perfect grid (unexpected for our writer) -> give up
        return None

    # Sort and reshape Z onto (H, W) with row-major layout matching create_scene
    xs.sort()
    ys.sort()
    # Map each point to indices — since grid is regular, we can do a fast index
    x_to_i = {float(v): i for i, v in enumerate(xs)}
    y_to_j = {float(v): j for j, v in enumerate(ys)}

    Z = np.empty((H, W), dtype=np.float32)
    for x, y, z in P:
        i = x_to_i[float(x)]
        j = y_to_j[float(y)]
        Z[j, i] = z

    cache = {
        "prim": prim,
        "xs": xs,              # ascending
        "ys": ys,              # ascending
        "Z": Z,                # shape (H, W)
    }
    _GRID_CACHE[key] = cache
    return cache


def world_range_for_path(stage, prim_path: str):
    """
    Return a Gf.Range3d (min/max) for the given prim in WORLD space,
    or None if the prim doesn't exist.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        print(f"[bounds][ERR] prim not found: {prim_path}")
        return None
    try:
        bbox_cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            includedPurposes=[UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            useExtentsHint=True,
        )
        bbox = bbox_cache.ComputeWorldBound(prim)
        return bbox.ComputeAlignedRange()
    except Exception as e:
        print(f"[bounds][ERR] failed for {prim_path}: {e}")
        return None


def _world_to_local_xy(prim, x_w, y_w):
    """Transform a world (x,y) into the mesh's local (x,y)."""
    cache = UsdGeom.XformCache()
    M = cache.GetLocalToWorldTransform(prim).GetInverse()
    pw = Gf.Vec3d(float(x_w), float(y_w), 0.0)
    pl = M.Transform(pw)
    return float(pl[0]), float(pl[1])


def _bilinear(xs, ys, Z, x, y):
    """Bilinear sample with edge clamping."""
    # find i s.t. xs[i] <= x <= xs[i+1]
    i = int(np.clip(np.searchsorted(xs, x) - 1, 0, len(xs) - 2))
    j = int(np.clip(np.searchsorted(ys, y) - 1, 0, len(ys) - 2))

    x0, x1 = xs[i], xs[i + 1]
    y0, y1 = ys[j], ys[j + 1]
    # avoid divide-by-zero in degenerate grids
    tx = 0.0 if x1 == x0 else (x - x0) / (x1 - x0)
    ty = 0.0 if y1 == y0 else (y - y0) / (y1 - y0)
    # Z layout is [j,i] = Y-first, then X
    z00 = Z[j, i]
    z10 = Z[j, i + 1]
    z01 = Z[j + 1, i]
    z11 = Z[j + 1, i + 1]
    return float((1 - tx) * (1 - ty) * z00 +
                 tx * (1 - ty) * z10 +
                 (1 - tx) * ty * z01 +
                 tx * ty * z11)


def _sample_height_from_mesh(stage, terrain_path, x_world, y_world):
    """Return ground Z at (x_world, y_world) by sampling the terrain mesh."""
    cache = _terrain_grid_cache(stage, terrain_path)
    if not cache:
        return None
    prim = cache["prim"]
    xs, ys, Z = cache["xs"], cache["ys"], cache["Z"]
    x_local, y_local = _world_to_local_xy(prim, x_world, y_world)

    # Clamp query inside the grid
    xq = float(np.clip(x_local, xs[0], xs[-1]))
    yq = float(np.clip(y_local, ys[0], ys[-1]))
    return _bilinear(xs, ys, Z, xq, yq)


# ---------- unified ground-height API (mesh first, then ray/physx) ----------
def local_ground_height(stage, x, y, terrain_path="/World/Terrain",
                        use_raycast=True, use_physx=True):
    """Robust ground Z at (x,y): try mesh sampling, then raycast, then physX."""
    # 1) USD mesh sampling (robust, no sim needed)
    z = _sample_height_from_mesh(stage, terrain_path, x, y)
    if z is not None:
        # print(f"[ground] mesh z={z:.3f} at ({x:.3f},{y:.3f})")
        return z

    # 2) Viewport raycast (optional)
    if use_raycast:
        try:
            from omni.kit.raycast.query import ray_cast
            hit = ray_cast(origin=Gf.Vec3f(x, y, 1e5), direction=Gf.Vec3f(0, 0, -1), max_distance=2e5, first_hit=True)
            if hit and hit.get("hits"):
                return float(hit["hits"][0]["point"][2])
        except Exception:
            pass

    # 3) PhysX
    if use_physx:
        try:
            import omni.physx as physx
            sq = physx.get_physx_scene_query_interface()
            hit = sq.raycast_any((x, y, 1e5), (0.0, 0.0, -1.0), 2e5)
            if hit["hit"]:
                return float(hit["position"][2])
        except Exception:
            pass

    return None

# --- Physics scene + terrain collider/material + spawn/snap helpers ---
def ensure_physics_scene(stage, moon=False):
    """Create /World/physicsScene with Earth or Moon gravity."""
    scene = UsdPhysics.Scene.Define(stage, Sdf.Path("/World/physicsScene"))
    scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
    scene.CreateGravityMagnitudeAttr().Set(1.62 if moon else 9.81)
    # Ensure stage units/up-axis are sane (won’t overwrite if already set)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    try:
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    except AttributeError:
        # fallback for very old USD builds
        UsdGeom.SetMetersPerUnit(stage, 1.0)
    print(f"[physics] Scene ready. gravity={('Moon','Earth')[not moon]} ({scene.GetGravityMagnitudeAttr().Get():.2f} m/s^2)")
    return scene

def apply_terrain_physics(stage, terrain_path="/World/Terrain",
                          mu_static=0.8, mu_dynamic=0.6, restitution=0.0):
    """Make the terrain collide + bind a physics material (MaterialAPI-based)."""
    prim = stage.GetPrimAtPath(terrain_path)
    if not prim:
        print(f"[terrain_physics][ERR] missing prim: {terrain_path}")
        return False

    # Ensure collider
    try:
        UsdPhysics.CollisionAPI.Apply(prim)
    except Exception as e:
        print(f"[terrain_physics][WARN] CollisionAPI apply failed: {e}")

    # Create a prim to hold the physics material API (typed schema not available here)
    mat_path = Sdf.Path("/World/PhysicsMaterial/Terrain")
    mat_prim = stage.GetPrimAtPath(mat_path)
    if not mat_prim:
        # 'Scope' is fine; any prim type can carry an API schema
        mat_prim = stage.DefinePrim(mat_path, "Scope")

    # Apply MaterialAPI and set properties
    pmat = UsdPhysics.MaterialAPI.Apply(mat_prim)
    pmat.CreateStaticFrictionAttr().Set(float(mu_static))
    pmat.CreateDynamicFrictionAttr().Set(float(mu_dynamic))
    pmat.CreateRestitutionAttr().Set(float(restitution))

    # Bind the material to the terrain via the standard relationship
    rel = prim.CreateRelationship("physics:material:binding", False)
    rel.SetTargets([mat_prim.GetPath()])

    print(f"[terrain_physics] Collider+material bound to {terrain_path} via 'physics:material:binding' "
          f"(μs={mu_static}, μd={mu_dynamic}, e={restitution}).")
    return True

def terrain_top_z(stage, terrain_path="/World/Terrain"):
    r = world_range_for_path(stage, terrain_path)
    return float(r.GetMax()[2]) if r else None

def _ground_normal(stage, terrain_path, x_world, y_world):
    cache = _terrain_grid_cache(stage, terrain_path)
    if not cache:
        return Gf.Vec3d(0, 0, 1)
    xs, ys, Z = cache["xs"], cache["ys"], cache["Z"]

    # cell indices
    i = int(np.clip(np.searchsorted(xs, _world_to_local_xy(cache["prim"], x_world, y_world)[0]) - 1, 0, len(xs)-2))
    j = int(np.clip(np.searchsorted(ys, _world_to_local_xy(cache["prim"], x_world, y_world)[1]) - 1, 0, len(ys)-2))
    x0, x1 = xs[i], xs[i+1];  dx = float(x1 - x0) or 1.0
    y0, y1 = ys[j], ys[j+1];  dy = float(y1 - y0) or 1.0
    z00, z10 = float(Z[j, i]),     float(Z[j, i+1])
    z01, z11 = float(Z[j+1, i]),   float(Z[j+1, i+1])

    # bilinear gradient inside the cell (take center of cell, ty=tx=0.5)
    dzdx = ((z10 - z00) * 0.5 + (z11 - z01) * 0.5) / dx
    dzdy = ((z01 - z00) * 0.5 + (z11 - z10) * 0.5) / dy
    n_local = Gf.Vec3d(-dzdx, -dzdy, 1.0).GetNormalized()

    # to world
    M = UsdGeom.XformCache().GetLocalToWorldTransform(cache["prim"])
    n_world = M.TransformDir(n_local)
    return Gf.Vec3d(n_world[0], n_world[1], n_world[2]).GetNormalized()

def _quat_from_a_to_b(a: Gf.Vec3d, b: Gf.Vec3d) -> Gf.Quatf:
    a = a.GetNormalized(); b = b.GetNormalized()
    c = Gf.Cross(a, b); d = Gf.Dot(a, b)
    if d > 1.0 - 1e-8:  return Gf.Quatf(1.0, Gf.Vec3f(0,0,0))
    if d < -1.0 + 1e-8:
        axis = Gf.Cross(a, Gf.Vec3d(1,0,0))
        if axis.GetLength() < 1e-6: axis = Gf.Cross(a, Gf.Vec3d(0,1,0))
        axis = axis.GetNormalized()
        return Gf.Quatf(0.0, Gf.Vec3f(axis[0], axis[1], axis[2]))
    s = math.sqrt((1.0 + d) * 2.0); invs = 1.0 / s
    return Gf.Quatf(s*0.5, Gf.Vec3f(c[0]*invs, c[1]*invs, c[2]*invs))

def _set_holder_trs(stage, holder_prim, x, y, yaw_deg, tz, terrain_path=None, orient_to_slope=False):
    xf = UsdGeom.Xformable(holder_prim)
    try: xf.ClearXformOpOrder()
    except Exception: xf.SetXformOpOrder([])

    # translate first
    xf.AddTranslateOp().Set(Gf.Vec3d(float(x), float(y), float(tz)))

    # orientation: optionally tilt so +Z matches ground normal, then add yaw around that up axis
    q = Gf.Quatf(1.0, Gf.Vec3f(0,0,0))
    if orient_to_slope and terrain_path:
        n = _ground_normal(stage, terrain_path, x, y)
        q_align = _quat_from_a_to_b(Gf.Vec3d(0,0,1), n)
        # yaw around the aligned up axis
        yaw = math.radians(float(yaw_deg))
        q_yaw  = Gf.Quatf(math.cos(yaw/2.0), Gf.Vec3f(n[0], n[1], n[2]) * math.sin(yaw/2.0))
        q = q_yaw * q_align
    else:
        # yaw only (about Z)
        q = Gf.Quatf(math.cos(math.radians(yaw_deg)/2.0), Gf.Vec3f(0,0,1) * math.sin(math.radians(yaw_deg)/2.0))

    xf.AddOrientOp().Set(q)

def spawn_asset(stage, usd_path, name="asset", x=0.0, y=0.0, yaw_deg=0.0,
                drop_margin=0.01, terrain_path="/World/Terrain", orient_to_slope=False):
    """Reference an asset under /World/<name>/payload and place its parent at local ground+margin."""
    holder_path  = f"/World/{name}"
    payload_path = f"{holder_path}/payload"

    # Clean existing
    if stage.GetPrimAtPath(holder_path):
        stage.RemovePrim(holder_path)

    # Holder (clean Xform for placement) + child payload (carries the reference)
    holder  = stage.DefinePrim(holder_path,  "Xform")
    payload = stage.DefinePrim(payload_path, "Xform")
    try:
        payload.GetReferences().AddReference(usd_path)
    except Exception as e:
        print(f"[spawn_asset][ERR] AddReference failed for {usd_path}: {e}")
        return None

    # Ensure the holder has a simple, writable xform stack
    xf_holder = UsdGeom.Xformable(holder)
    try:
        xf_holder.ClearXformOpOrder()
    except Exception:
        xf_holder.SetXformOpOrder([])

    # Local ground height at (x,y)
    terr_z = local_ground_height(stage, x, y, terrain_path=terrain_path)
    if terr_z is None:
        terr_z = terrain_top_z(stage, terrain_path) or 0.0

    # bbox BEFORE translate (on the holder so it includes the payload)
    r0 = world_range_for_path(stage, holder_path)
    minz_before = float(r0.GetMin()[2]) if r0 else 0.0

    # Move so bottom sits at ground + margin
    dz = (terr_z + float(drop_margin)) - minz_before
    _set_holder_trs(stage, holder, x, y, yaw_deg, tz=dz,
                    terrain_path=terrain_path, orient_to_slope=orient_to_slope)

    # Logs
    r1 = world_range_for_path(stage, holder_path)
    new_minz = float(r1.GetMin()[2]) if r1 else (terr_z + float(drop_margin))
    print(f"[spawn_asset] terr_z={terr_z:.3f}  minz_before={minz_before:.3f}  dz={dz:.3f}  "
          f"new_minz~={new_minz:.3f}  holder_T=({x:.3f},{y:.3f},{dz:.3f})")
    print(f"[spawn_asset] Spawned '{usd_path}' as {holder_path} at (x={x:.2f}, y={y:.2f}), "
          f"yaw={yaw_deg:.1f}°, drop={drop_margin:.2f} m.")
    return holder_path

def snap_to_ground(stage, prim_path, terrain_path="/World/Terrain", drop_margin=0.01):
    """Recompute bottom Z vs. local ground and adjust only in Z."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        print(f"[snap_to_ground][ERR] prim not found: {prim_path}")
        return False
    xf = UsdGeom.XformCommonAPI(prim)
    t, rdeg, s, p, rot_order = xf.GetXformVectors(Usd.TimeCode.Default())
    tx, ty, tz = float(t[0]), float(t[1]), float(t[2])

    # Recompute ground at (tx,ty)
    terr_z = local_ground_height(stage, tx, ty, terrain_path=terrain_path)
    if terr_z is None:
        terr_z = terrain_top_z(stage, terrain_path) or 0.0

    r = world_range_for_path(stage, prim_path)
    minz_before = float(r.GetMin()[2]) if r else tz
    dz = (terr_z + float(drop_margin)) - minz_before
    xf.SetTranslate(Gf.Vec3d(tx, ty, tz + dz))

    r2 = world_range_for_path(stage, prim_path)
    new_minz = float(r2.GetMin()[2]) if r2 else terr_z + float(drop_margin)
    print(f"[snap_to_ground] terr_z={terr_z:.3f}  minz_before={minz_before:.3f}  dz={dz:.3f}  "
          f"new_minz~={new_minz:.3f}  holder_T=({tx:.3f},{ty:.3f},{tz+dz:.3f})")
    return True
