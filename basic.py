"""
Basic Imperial-style yo-yo, parametric, for 3D printing.

Two halves — lid and shell are ONE piece each. No electronics cavity.
The two halves screw together via a male/female threaded axle.

Run in Blender:  Scripting tab -> open this file -> Run Script
or headless:     blender --background --python basic.py

Exports THREE STLs:
    half_male.stl        - dome half with hollow male threaded axle stub
    half_female.stl      - dome half with hollow female threaded socket
    axle_double_male.stl - standalone axle, threaded on BOTH ends, for
                           joining two half_female shells into a yo-yo
                           with no printed male half.

Assembly (male/female pair): screw the male stub into the female socket.
The smooth axle section spans the string gap between the two inner faces.
String wraps the smooth section. No glue, no extra hardware.

Assembly (double-female pair): screw axle_double_male into two
half_female shells, one end each. Same smooth-section string wrap as
above.

Print both halves crown-down (flat face on bed).
"""

import bpy, bmesh, math, os

# ----------------------------------------------------------------------------
# PARAMETERS  (all millimetres)
# ----------------------------------------------------------------------------
OUTER_DIAMETER = 60.0     # whole yo-yo diameter
HALF_WIDTH     = 15.0     # axial thickness of ONE half
FLAT_DIAMETER  = 45.0     # diameter of the flat crown (print base)

STRING_GAP     = 4.0      # total gap between the two assembled inner faces
RIM_FILLET     = 2.0      # radius of the round-over at the outer rim (string area)
CROWN_FILLET   = 5.0      # radius of the round-over at the flat crown edge (slightly
                           # more than RIM_FILLET so the crown edge feels less sharp)

# --- hollow threaded axle ---
AXLE_BORE      = 6.0      # centre bore diameter
THREAD_MAJOR_D = 10.0     # thread crest diameter
THREAD_DEPTH   = 0.9      # thread depth (major->minor on radius)
THREAD_PITCH   = 2.0      # mm per turn
THREAD_CLEAR   = 0.35     # radial clearance added to female tap
THREAD_ENGAGE  = 6.0      # threaded engagement length
SMOOTH_LEN     = STRING_GAP   # smooth string-contact section = gap width
FEM_TUBE_OD    = 14.0     # OD of the female socket tube

THREAD_SEGS    = 96
THREAD_LPP     = 24
SEGMENTS       = 128

OUTPUT_DIR = os.path.expanduser("~/Documents/Code/yoyo")

# ----------------------------------------------------------------------------
# derived
# ----------------------------------------------------------------------------
R        = OUTER_DIAMETER / 2.0
FLAT_R   = FLAT_DIAMETER / 2.0
TH_MAJ_R = THREAD_MAJOR_D / 2.0
TH_MIN_R = (THREAD_MAJOR_D - 2.0 * THREAD_DEPTH) / 2.0
C = HALF_WIDTH / math.sqrt(1.0 - (FLAT_R / R) ** 2)


def r_outer(z):
    """Outer dome radius at height z (0 = crown, HALF_WIDTH = inner face)."""
    z = min(max(z, 0.0), HALF_WIDTH)
    return R * math.sqrt(max(1.0 - ((HALF_WIDTH - z) / C) ** 2, 0.0))


def crown_fillet_arc(fillet_r, n_arc=12):
    """Round-over profile points for the flat-crown / dome corner at (FLAT_R, 0).
    The flat crown is horizontal there; the dome wall leaves the corner at its
    analytic tangent slope. A circle of radius fillet_r tangent to both lines
    replaces the sharp corner. Returns (p1, [interior arc pts], p2) where p1
    sits on the flat crown and p2 sits on the dome wall, both (r, z) tuples."""
    slope = R * R * HALF_WIDTH / (C * C * FLAT_R)   # dr/dz of the dome at z=0
    e1 = (-1.0, 0.0)                                 # along the flat, back toward axis
    norm2 = math.hypot(slope, 1.0)
    e2 = (slope / norm2, 1.0 / norm2)                # along the dome, away from crown
    theta = math.acos(max(-1.0, min(1.0, -e2[0])))   # interior angle at the corner
    t = fillet_r / math.tan(theta / 2.0)
    bx, bz = e1[0] + e2[0], e1[1] + e2[1]
    bn = math.hypot(bx, bz)
    bx, bz = bx / bn, bz / bn
    cdist = fillet_r / math.sin(theta / 2.0)
    center = (FLAT_R + bx * cdist, bz * cdist)
    p1 = (FLAT_R + e1[0] * t, e1[1] * t)
    p2 = (FLAT_R + e2[0] * t, e2[1] * t)
    a1 = math.atan2(p1[1] - center[1], p1[0] - center[0])
    a2 = math.atan2(p2[1] - center[1], p2[0] - center[0])
    diff = (a2 - a1 + math.pi) % (2.0 * math.pi) - math.pi
    pts = [(center[0] + fillet_r * math.cos(a1 + diff * k / n_arc),
            center[1] + fillet_r * math.sin(a1 + diff * k / n_arc))
           for k in range(1, n_arc)]
    return p1, pts, p2


# ----------------------------------------------------------------------------
# scene helpers
# ----------------------------------------------------------------------------
def reset_scene():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()
    for block in (bpy.data.meshes, bpy.data.objects):
        for b in list(block):
            try:
                block.remove(b)
            except Exception:
                pass
    sc = bpy.context.scene.unit_settings
    sc.system = 'METRIC'
    sc.length_unit = 'MILLIMETERS'


def new_mesh_object(name, verts, faces):
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.scene.collection.objects.link(obj)
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    return obj


def revolve(name, profile, segs=SEGMENTS):
    """Solid of revolution from a CLOSED (r, z) profile. Watertight."""
    eps = 1e-6
    angles = [2.0 * math.pi * k / segs for k in range(segs)]
    verts, ring = [], []
    for r, z in profile:
        if r < eps:
            ring.append(('axis', len(verts)))
            verts.append((0.0, 0.0, z))
        else:
            ring.append(('ring', len(verts)))
            for a in angles:
                verts.append((r * math.cos(a), r * math.sin(a), z))
    faces = []
    n = len(profile)
    for i in range(n):
        j = (i + 1) % n
        ti, bi = ring[i]
        tj, bj = ring[j]
        if ti == 'axis' and tj == 'axis':
            continue
        if ti == 'ring' and tj == 'ring':
            for k in range(segs):
                k2 = (k + 1) % segs
                faces.append((bi + k, bi + k2, bj + k2, bj + k))
        elif ti == 'axis':
            for k in range(segs):
                k2 = (k + 1) % segs
                faces.append((bi, bj + k, bj + k2))
        else:
            for k in range(segs):
                k2 = (k + 1) % segs
                faces.append((bi + k2, bi + k, bj))
    return new_mesh_object(name, verts, faces)


def add_cylinder(name, dia, z0, z1, x=0.0, y=0.0, segs=96):
    h = z1 - z0
    bpy.ops.mesh.primitive_cylinder_add(vertices=segs, radius=dia / 2.0,
                                         depth=h, location=(x, y, z0 + h / 2.0))
    obj = bpy.context.active_object
    obj.name = name
    return obj


def thread_r(phase, rmin, rmaj):
    """Trapezoidal thread profile — flat-flanked for FDM printability."""
    if phase < 0.25:
        return rmin + (rmaj - rmin) * (phase / 0.25)
    if phase < 0.5:
        return rmaj
    if phase < 0.75:
        return rmaj - (rmaj - rmin) * ((phase - 0.5) / 0.25)
    return rmin


def threaded_solid(name, rmin, rmaj, z0, z1, pitch,
                   segs=THREAD_SEGS, lpp=THREAD_LPP):
    """Watertight helical threaded solid. Use as male thread or boolean tap."""
    length = z1 - z0
    nz = max(2, int(round(length / pitch * lpp)) + 1)
    verts = []
    for j in range(nz):
        z = z0 + length * j / (nz - 1)
        for i in range(segs):
            a = 2.0 * math.pi * i / segs
            phase = ((z - z0) / pitch - i / segs) % 1.0
            r = thread_r(phase, rmin, rmaj)
            verts.append((r * math.cos(a), r * math.sin(a), z))
    c_bot = len(verts); verts.append((0.0, 0.0, z0))
    c_top = len(verts); verts.append((0.0, 0.0, z1))

    def idx(j, i):
        return j * segs + (i % segs)

    faces = []
    for j in range(nz - 1):
        for i in range(segs):
            faces.append((idx(j, i), idx(j, i + 1), idx(j + 1, i + 1), idx(j + 1, i)))
    for i in range(segs):
        faces.append((c_bot, idx(0, i + 1), idx(0, i)))
    for i in range(segs):
        faces.append((c_top, idx(nz - 1, i), idx(nz - 1, i + 1)))
    return new_mesh_object(name, verts, faces)


def boolean(obj, cutter, op):
    mod = obj.modifiers.new(name="bool", type='BOOLEAN')
    mod.operation = op
    mod.solver = 'EXACT'
    mod.object = cutter
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def export_stl(obj, filepath):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj
    try:
        bpy.ops.wm.stl_export(filepath=filepath, export_selected_objects=True,
                              global_scale=1.0, apply_modifiers=True)
    except Exception:
        bpy.ops.export_mesh.stl(filepath=filepath, use_selection=True,
                                global_scale=1.0)


# ----------------------------------------------------------------------------
# dome — the shared outer shell body for both halves
# ----------------------------------------------------------------------------
def build_dome(name):
    """Solid dome half with rounded edges at both the crown and the string gap.
    Crown at z=0, inner face at z=HALF_WIDTH. Print crown-down.
    A fillet arc rounds the flat-crown/dome corner at z=0 (CROWN_FILLET), the
    dome runs up to z = HALF_WIDTH - RIM_FILLET, then a quarter-circle arc
    rounds the outer rim into the inner face, eliminating both sharp edges."""
    steps = 48
    p1, crown_arc_pts, p2 = crown_fillet_arc(CROWN_FILLET)
    prof = [(0.0, 0.0), p1] + crown_arc_pts + [p2]
    # main dome from the crown fillet's tangent point up to the rim fillet tangent point
    z_start = p2[1]
    z_end = HALF_WIDTH - RIM_FILLET
    for s in range(1, steps + 1):
        z = z_start + (z_end - z_start) * s / steps
        prof.append((r_outer(z), z))
    # quarter-circle fillet: rounds the outer rim from dome into inner face
    # arc centre is inset RIM_FILLET from both the outer wall and the inner face
    arc_cx = R - RIM_FILLET
    arc_cz = HALF_WIDTH - RIM_FILLET
    n_arc = 12
    for k in range(n_arc + 1):
        angle = math.pi / 2.0 * k / n_arc   # 0 -> pi/2
        prof.append((arc_cx + RIM_FILLET * math.cos(angle),
                     arc_cz + RIM_FILLET * math.sin(angle)))
    # flat inner face: from the fillet end to the axis
    prof.append((0.0, HALF_WIDTH))
    return revolve(name, prof)


# ----------------------------------------------------------------------------
# half_male  — dome + male axle stub (smooth section + external thread + bore)
# ----------------------------------------------------------------------------
def build_half_male():
    half = build_dome("half_male")
    face_z    = HALF_WIDTH
    smooth_z1 = face_z + SMOOTH_LEN       # end of smooth / start of threads
    tip_z     = smooth_z1 + THREAD_ENGAGE # thread tip

    smooth = add_cylinder("axsmooth", THREAD_MAJOR_D, face_z, smooth_z1,
                          segs=THREAD_SEGS)
    boolean(half, smooth, 'UNION')

    th = threaded_solid("axthread", TH_MIN_R, TH_MAJ_R,
                        smooth_z1, tip_z, THREAD_PITCH)
    boolean(half, th, 'UNION')

    return half


# ----------------------------------------------------------------------------
# half_female — dome + female socket tube (internal thread + through bore)
# ----------------------------------------------------------------------------
def build_half_female():
    half = build_dome("half_female")
    face_z      = HALF_WIDTH
    tube_bottom = face_z - THREAD_ENGAGE - 1.5   # tube extends into the dome

    tube = add_cylinder("femtube", FEM_TUBE_OD, tube_bottom, face_z + 0.01,
                        segs=THREAD_SEGS)
    boolean(half, tube, 'UNION')

    tap = threaded_solid("femtap",
                         TH_MIN_R + THREAD_CLEAR, TH_MAJ_R + THREAD_CLEAR,
                         face_z - THREAD_ENGAGE, face_z + 0.2, THREAD_PITCH)
    boolean(half, tap, 'DIFFERENCE')

    bore = add_cylinder("fembore", AXLE_BORE, tube_bottom - 0.2, face_z + 0.2,
                        segs=THREAD_SEGS)
    boolean(half, bore, 'DIFFERENCE')

    return half


# ----------------------------------------------------------------------------
# axle_double_male — standalone axle, male thread on BOTH ends, for joining
# two half_female shells (no printed male dome half needed)
# ----------------------------------------------------------------------------
def build_axle_double_male():
    """Symmetric double-ended threaded stud. Smooth centre section (same
    diameter as the male stub's smooth section) spans the string gap
    between two assembled half_female inner faces; each end then has an
    independent THREAD_ENGAGE-long male thread that screws into a
    half_female socket. Both ends use the same thread hand — like a
    threaded-rod coupler, each shell is spun independently onto its own
    end, so no reversed thread is needed."""
    smooth_z0 = -SMOOTH_LEN / 2.0
    smooth_z1 = SMOOTH_LEN / 2.0

    axle = add_cylinder("axle_double_male", THREAD_MAJOR_D, smooth_z0, smooth_z1,
                        segs=THREAD_SEGS)

    th_pos = threaded_solid("axthread_pos", TH_MIN_R, TH_MAJ_R,
                            smooth_z1, smooth_z1 + THREAD_ENGAGE, THREAD_PITCH)
    boolean(axle, th_pos, 'UNION')

    th_neg = threaded_solid("axthread_neg", TH_MIN_R, TH_MAJ_R,
                            smooth_z0 - THREAD_ENGAGE, smooth_z0, THREAD_PITCH)
    boolean(axle, th_neg, 'UNION')

    return axle


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------
def main():
    reset_scene()

    male = build_half_male()
    male.location = (0.0, 0.0, 0.0)

    female = build_half_female()
    female.location = (70.0, 0.0, 0.0)

    axle = build_axle_double_male()
    axle.location = (140.0, 0.0, 0.0)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    male.location = (0.0, 0.0, 0.0)
    export_stl(male, os.path.join(OUTPUT_DIR, "half_male.stl"))
    male.location = (0.0, 0.0, 0.0)

    female.location = (0.0, 0.0, 0.0)
    export_stl(female, os.path.join(OUTPUT_DIR, "half_female.stl"))
    female.location = (70.0, 0.0, 0.0)

    axle.location = (0.0, 0.0, 0.0)
    export_stl(axle, os.path.join(OUTPUT_DIR, "axle_double_male.stl"))
    axle.location = (140.0, 0.0, 0.0)

    print("=" * 60)
    print("Wrote half_male.stl, half_female.stl, axle_double_male.stl to:", OUTPUT_DIR)
    print("  PRINT: 1x half_male, 1x half_female  (both crown-down)")
    print("  OR:    2x half_female + 1x axle_double_male")
    print("  yo-yo diameter : %.1f mm" % OUTER_DIAMETER)
    print("  half width     : %.1f mm  (total %.1f mm)" % (HALF_WIDTH, 2 * HALF_WIDTH))
    print("  flat crown dia : %.1f mm" % FLAT_DIAMETER)
    print("  string gap     : %.1f mm" % STRING_GAP)
    print("  axle           : major %.1f mm, pitch %.1f mm, engage %.1f mm"
          % (THREAD_MAJOR_D, THREAD_PITCH, THREAD_ENGAGE))
    print("  bore           : %.1f mm (string/wire passage)" % AXLE_BORE)
    print("=" * 60)


if __name__ == "__main__":
    main()
