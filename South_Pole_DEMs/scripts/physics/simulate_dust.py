import bpy
import numpy as np
import mathutils

DUST_SOURCE_NAME = "Wheel"
_last_source_pos = None

class DustSimulator:
    def __init__(self, max_particles=100000, v_emit_thresh=1.0, dt=1/240,
                 voxel_size=0.1, influence_radius=1.0, seed=0):
        self.rng = np.random.default_rng(seed)

        print(f"[DustSimulator] init: max={max_particles}, v_thresh={v_emit_thresh}, dt={dt}, seed={seed}")

        self.max_particles = max_particles
        self.v_thresh = v_emit_thresh
        self.dt = dt
        self.infl_rad = influence_radius

        # state buffers for positions, velocities, and lifetimes
        self.pos  = np.zeros((0,3), dtype=float)
        self.vel  = np.zeros((0,3), dtype=float)
        self.life = np.zeros((0,),   dtype=float)

    def emit(self, contact_pts, contact_vel):
        """
        Emit new dust particles at contact points if their velocities exceeds a threshold.
        contact_pts: Nx3 array of contact point positions
        contact_vel: N array of velocities at those points
        """
        mask = contact_vel > self.v_thresh
        print(f"[emit] {len(contact_pts)} contact_pts, {len(contact_pts[mask])} above v_thresh")
        new_p = contact_pts[mask]
        n = len(new_p)
        if n == 0:
            return
        new_v = np.vstack((
            self.rng.uniform(-0.5,0.5,size=n),
            np.full(n, 2.0),
            self.rng.uniform(-0.5,0.5,size=n),
        )).T
        new_l = np.full(n, 5.0)
        self.pos  = np.vstack((self.pos,  new_p))[-self.max_particles:]
        self.vel  = np.vstack((self.vel,  new_v))[-self.max_particles:]
        self.life = np.hstack((self.life, new_l))[-self.max_particles:]

    def integrate(self):
        """
        Simple Euler integration step for dust particles.
        Applies a constant downward acceleration (lunar gravity) and updates positions and decrementing life.
        """
        print(f"[integrate] before: pos_count={len(self.pos)}")
        self.vel[:,2] -= 1.62 * self.dt
        self.pos += self.vel * self.dt
        self.life -= self.dt
        print(f"[integrate] after: sample_pos={self.pos[:1] if len(self.pos)>0 else 'none'}")

    def cleanup(self, bounds=None):
        """
        Remove particles that have expired or are outside the specified bounds.
        If bounds is None, only remove particles that are dead.
        """
        print(f"[cleanup] before: total={len(self.life)}")
        alive = self.life > 0
        if bounds is not None:
            inside = np.all((self.pos >= bounds[0]) & (self.pos <= bounds[1]), axis=1)
            alive &= inside
        self.pos  = self.pos[alive]
        self.vel  = self.vel[alive]
        self.life = self.life[alive]
        print(f"[cleanup] after: alive={len(self.life)}")

# ── set up simulator and Blender mesh ──
sim = DustSimulator(max_particles=20000, v_emit_thresh=1.0, dt=1/120, seed=42)
mesh = bpy.data.meshes.new("DustParticles")
obj  = bpy.data.objects.new("DustParticles", mesh)
bpy.context.collection.objects.link(obj)

def update_dust(scene):
    global _last_source_pos

    # 1) fetch the source object by name
    src = bpy.data.objects.get(DUST_SOURCE_NAME)
    if src is None:
        print(f">>> update_dust: no object called '{DUST_SOURCE_NAME}'")
        return

    # 2) get world-space position and compute approximate downward velocity
    pos = src.matrix_world.to_translation()
    if _last_source_pos is None:
        vel_vec = mathutils.Vector((0,0,0))
    else:
        vel_vec = (pos - _last_source_pos) / sim.dt
    _last_source_pos = pos.copy()

    # 3) ray-cast down from just above the highest point of the source object
    # get all eight corners of local bounding box in world space
    world_corners = [src.matrix_world @ mathutils.Vector(c) for c in src.bound_box]
    # find the maximum Z
    top_z = max(v.z for v in world_corners)
    # cast from a little above that
    origin    = mathutils.Vector((pos.x, pos.y, top_z + 0.1))
    direction = mathutils.Vector((0,0,-1))
    terrain   = bpy.data.objects.get("Terrain")
    deps      = bpy.context.evaluated_depsgraph_get()
    hit, pt, _, _, _ = terrain.evaluated_get(deps).ray_cast(origin, direction)
    if not hit:
        return

    # 4) build contact_pts & contact_vel arrays
    contact_pts = np.array([pt])
    # take only the downward component as positive speed
    contact_vel = np.array([max(0.0, -vel_vec.z)])

    print(f"[update_dust] Frame {scene.frame_current}, {DUST_SOURCE_NAME}.vel.z={vel_vec.z:.3f}")
    sim.emit(contact_pts, contact_vel)
    sim.integrate()
    sim.cleanup(bounds=np.array([[-10,-10,0],[10,10,10]]))

    # 5) rebuild your mesh points
    coords = [tuple(v) for v in sim.pos]
    mesh.clear_geometry()
    mesh.from_pydata(coords, [], [])
    mesh.update()

# re-register handler
bpy.app.handlers.frame_change_pre.clear()
bpy.app.handlers.frame_change_pre.append(update_dust)