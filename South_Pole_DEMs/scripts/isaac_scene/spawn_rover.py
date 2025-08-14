from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf, Sdf

# ---------- physics helpers you already have ----------
def ensure_physics_scene(stage, moon=False):
    scene = UsdPhysics.Scene.Get(stage, "/World/physicsScene")
    if not scene:
        scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(1.62 if moon else 9.81)
    return scene

def apply_terrain_physics(stage, path="/World/Terrain",
                          static_friction=0.8, dynamic_friction=0.6, restitution=0.0):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsA(UsdGeom.Mesh):
        print(f"[terrain_physics] '{path}' not found or not a Mesh.")
        return False

    # Make the terrain a triangle-mesh collider (no convex hull)
    UsdPhysics.CollisionAPI.Apply(prim)
    # If MeshCollisionAPI exists, use it to set approximation="none"
    if hasattr(UsdPhysics, "MeshCollisionAPI"):
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr().Set("none")
    else:
        # Fallback for older schemas
        prim.CreateAttribute("physics:approximation", Usd.Tokens.token).Set("none")

    # Create a USD *shading* Material and attach Physics material attributes to it
    mat = UsdShade.Material.Define(stage, "/World/Physics/RegolithMaterial")
    mapi = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
    mapi.CreateStaticFrictionAttr(static_friction)
    mapi.CreateDynamicFrictionAttr(dynamic_friction)
    mapi.CreateRestitutionAttr(restitution)

    # Bind physics material by relationship (works on 4.5)
    rel = prim.GetRelationship("physics:material:binding")
    if not rel:
        rel = prim.CreateRelationship("physics:material:binding")
    rel.SetTargets([mat.GetPrim().GetPath()])

    print(f"[terrain_physics] Collider+material bound to {path} via 'physics:material:binding' "
          f"(μs={static_friction}, μd={dynamic_friction}, e={restitution}).")
    return True


# ---------- robust bounds helpers ----------
def _bbox_cache():
    # Include default + render purposes; no extentsHint so we get true world bounds
    return UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                             [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
                             useExtentsHint=False)

def world_range_for_path(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        return None
    cache = _bbox_cache()
    bbox = cache.ComputeWorldBound(prim)
    return bbox.ComputeAlignedBox()  # Gf.Range3d

def terrain_top_z(stage, path="/World/Terrain"):
    r = world_range_for_path(stage, path)
    return float(r.GetMax()[2]) if r is not None else 0.0

# ---------- generic spawning ----------
def _unique_child_path(stage, parent="/World", base_name="Asset"):
    # /World/Asset, /World/Asset_1, /World/Asset_2, ...
    i = 0
    while True:
        name = base_name if i == 0 else f"{base_name}_{i}"
        p = Sdf.Path(parent).AppendChild(name)
        if not stage.GetPrimAtPath(p).IsValid():
            return p.pathString
        i += 1

def spawn_asset(stage, usd_path, x=0.0, y=0.0, yaw_deg=0.0,
                name="asset", parent="/World", drop_margin=0.20,
                terrain_path="/World/Terrain"):
    """
    Reference any USD under /World/<name>, place it at (x,y) and lift so that
    the asset's bottom sits just above the terrain (drop_margin meters).
    """
    if not usd_path:
        print("[spawn_asset] No USD path provided.")
        return None

    # Create a holder Xform and reference the asset into it
    prim_path = _unique_child_path(stage, parent=parent, base_name=name)
    # after creating the holder and adding the reference
    holder = stage.DefinePrim(prim_path, "Xform")
    holder.GetReferences().ClearReferences()
    holder.GetReferences().AddReference(usd_path)

    # NEW: ensure the holder has "common" xform ops (avoid incompatible-xformable warning)
    xformable = UsdGeom.Xformable(holder)
    try:
        xformable.ClearXformOpOrder()
    except Exception:
        xformable.SetXformOpOrder([])

    xf = UsdGeom.XformCommonAPI(holder)
    terrain_z = terrain_top_z(stage, terrain_path)
    z_guess = terrain_z + drop_margin
    print(f"[spawn_asset] terrain_top_z={terrain_z:.3f}, drop_margin={drop_margin:.3f}, initial_z_guess={z_guess:.3f}")
    xf.SetTranslate(Gf.Vec3d(x, y, z_guess))
    xf.SetRotate((0.0, 0.0, yaw_deg))

    # After composition, compute world bounds of the referenced asset to align bottom to terrain
    r = world_range_for_path(stage, prim_path)
    if r:
        minz = float(r.GetMin()[2])
        print(f"[spawn_asset] asset world_range minz={minz:.3f}")
        terr_z = terrain_top_z(stage, terrain_path)
        # Move the holder so the asset's bottom is at terrain_z + drop_margin
        dz = (terr_z + drop_margin) - minz
        
        print(f"[spawn_asset] dz to align bottom={dz:.3f}")
        if abs(dz) > 1e-6:
            xf.SetTranslate(Gf.Vec3d(x, y, z_guess + dz))
            print(f"[spawn_asset] final_z={z_guess + dz:.3f}")

    print(f"[spawn_asset] Spawned '{usd_path}' as {prim_path} "
          f"at (x={x:.2f}, y={y:.2f}), yaw={yaw_deg:.1f}° with drop={drop_margin} m.")
    return holder
