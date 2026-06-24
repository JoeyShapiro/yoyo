"""
Imperial-style yo-yo, parametric, for 3D printing with an electronics cavity.

Run in Blender:  Scripting tab -> open this file -> Run Script
or headless:     blender --background --python yoyo_imperial.py

It builds THREE parts and exports them as STL (millimetres):
    shell.stl       - hollow dome half with screw bosses (NO centre axle)
    lid_male.stl    - inner plate with a HOLLOW THREADED AXLE STUB (male)
    lid_female.stl  - inner plate with a HOLLOW THREADED SOCKET (female)

Print: 2x shell, 1x lid_male, 1x lid_female. Assembly:
    1. drop electronics into each shell cavity
    2. screw lid_male onto one shell, lid_female onto the other
       (3x screws each, from the gap side into the shell bosses)
    3. run wires up through the hollow axle, then screw the two halves
       together: the male stub threads into the female socket. No glue.
       The string wraps the smooth section of the stub that spans the gap.

The axle is HOLLOW (centre bore) so wires/cabling pass straight from one
shell cavity, through the axle, into the other cavity.

Shape: each half is a flattened hemisphere (an ellipsoid cap). The OUTER pole
is flattened into a disc -> the "flat top" of the Imperial look, and also a
stable, support-free base to print on (print crown-down, cavity opening up).
"""

import bpy, bmesh, math, os

# ----------------------------------------------------------------------------
# PARAMETERS  (all millimetres)
# ----------------------------------------------------------------------------
OUTER_DIAMETER = 60.0     # whole yo-yo diameter
HALF_WIDTH     = 15.0     # axial thickness of ONE half (1.5 cm)
FLAT_DIAMETER  = 45.0     # diameter of the flat crown ("flat top" / print base)

WALL           = 1.5#2.0      # dome side-wall thickness
FLOOR          = 2.5      # crown floor thickness
SEAT_Z         = 12.5     # height the lid + boss tops sit at (from crown)
LID_THICK      = 3.0      # lid thickness
STRING_GAP     = 3.0      # total gap between the two assembled lids

N_BOSS         = 3        # number of perimeter screw bosses
BOSS_RADIAL    = 18.0     # boss centre distance from axis
BOSS_OD        = 4.5      # boss outer diameter
PILOT_D        = 2.0      # boss pilot hole (M2 thread-forming)
SCREW_CLEAR_D  = 2.4      # lid clearance hole (M2)
HEAD_CB_D      = 4.2      # lid counterbore for screw head
HEAD_CB_DEPTH  = 2.0

CTR_OD         = 6.0      # (unused now; kept for reference)
LID_CLEAR      = 0.3      # radial clearance lid <-> cavity wall

# --- USB-C slot (shell_usbc only, for Seeed XIAO ESP32C3) ---
# The XIAO ESP32C3 USB-C receptacle is 8.94 mm wide × 3.26 mm tall.
# The slot is cut through the dome wall near the lid (gap) face so a cable
# can plug straight into a XIAO mounted flat inside the shell.
# Adjust USBC_Z_CTR to match your exact board mounting height.
USBC_W       = 9.5    # slot width  (tangential, mm)  — USB-C + 0.56 mm clearance
USBC_H       = 4.0    # slot height (axial, mm)        — USB-C + 0.74 mm clearance
USBC_Z_CTR   = HALF_WIDTH      # slot centre at the rim; board (1 mm) + connector centre (~1.6 mm) above SEAT_Z puts the USB-C port right at this level
USBC_ANG     = math.pi / N_BOSS   # midpoint between two adjacent bosses (60° for N_BOSS=3)

# --- hollow threaded axle (lives on the lids, not the shell) ---
AXLE_BORE      = 6.0      # centre bore for wires (through the whole axle)
THREAD_MAJOR_D = 10.0      # thread major (crest) diameter
THREAD_DEPTH   = 0.9      # thread depth (major->minor on radius)
THREAD_PITCH   = 2.0      # mm per turn (coarse = printable & strong)
THREAD_CLEAR   = 0.35     # radial clearance, female cut larger than male
THREAD_ENGAGE  = 6.0      # threaded engagement length
SMOOTH_LEN     = STRING_GAP   # smooth string-contact section = the gap (3 mm)
FEM_TUBE_OD    = 14.0     # OD of the female socket down-tube (into cavity)
THREAD_SEGS    = 96       # angular resolution of the thread
THREAD_LPP     = 24       # vertical layers per pitch (helix smoothness)

SEGMENTS       = 128      # revolve resolution (lathe smoothness)
OUTPUT_DIR     = os.path.expanduser("~/Documents/Code/yoyo")   # where STLs are written

# ----------------------------------------------------------------------------
# derived
# ----------------------------------------------------------------------------
R        = OUTER_DIAMETER / 2.0
FLAT_R   = FLAT_DIAMETER / 2.0
REC      = STRING_GAP / 2.0                      # each lid recessed below rim
TH_MAJ_R = THREAD_MAJOR_D / 2.0
TH_MIN_R = (THREAD_MAJOR_D - 2.0 * THREAD_DEPTH) / 2.0
# vertical semi-axis of the ellipse so the crown flat lands at FLAT_R:
C = HALF_WIDTH / math.sqrt(1.0 - (FLAT_R / R) ** 2)


def r_outer(z):
    """Outer dome radius at height z (0 = crown, HALF_WIDTH = rim/gap face)."""
    z = min(max(z, 0.0), HALF_WIDTH)
    return R * math.sqrt(max(1.0 - ((HALF_WIDTH - z) / C) ** 2, 0.0))


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
    # make normals consistent / outward
    bm = bmesh.new()
    bm.from_mesh(mesh)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    bm.to_mesh(mesh)
    bm.free()
    return obj


def revolve(name, profile, segs=SEGMENTS):
    """Solid of revolution from a CLOSED (r, z) profile that touches the axis
    (r=0) at its two ends. Produces a watertight lathe solid."""
    eps = 1e-6
    angles = [2.0 * math.pi * k / segs for k in range(segs)]
    verts, ring = [], []   # ring[i] = ('axis', idx) or ('ring', base_idx)
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
        elif ti == 'axis':           # axis -> ring  (bottom fan)
            for k in range(segs):
                k2 = (k + 1) % segs
                faces.append((bi, bj + k, bj + k2))
        else:                        # ring -> axis  (top fan)
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


def add_box(name, sx, sy, sz, x=0.0, y=0.0, z=0.0, rz=0.0):
    """Box of size sx×sy×sz centred at (x,y,z), optionally rotated rz around Z."""
    bpy.ops.mesh.primitive_cube_add(size=1.0, location=(x, y, z))
    obj = bpy.context.active_object
    obj.name = name
    obj.scale = (sx, sy, sz)
    obj.rotation_euler[2] = rz
    bpy.ops.object.transform_apply(scale=True, rotation=True)
    return obj


def thread_r(phase, rmin, rmaj):
    """Trapezoidal thread profile (rise/crest/fall/root each 25% of pitch).
    Coarse & flat-flanked -> prints cleanly on FDM and is mechanically strong."""
    if phase < 0.25:
        return rmin + (rmaj - rmin) * (phase / 0.25)
    if phase < 0.5:
        return rmaj
    if phase < 0.75:
        return rmaj - (rmaj - rmin) * ((phase - 0.5) / 0.25)
    return rmin


def threaded_solid(name, rmin, rmaj, z0, z1, pitch,
                   segs=THREAD_SEGS, lpp=THREAD_LPP):
    """A solid threaded rod: radius height-field r(theta, z) swept as a single
    helix. Periodic in theta (seam closes), capped top & bottom -> watertight.
    Used directly as the male thread, or as a 'tap' to DIFFERENCE a female."""
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
    for i in range(segs):                       # bottom cap (normal down)
        faces.append((c_bot, idx(0, i + 1), idx(0, i)))
    for i in range(segs):                       # top cap (normal up)
        faces.append((c_top, idx(nz - 1, i), idx(nz - 1, i + 1)))
    return new_mesh_object(name, verts, faces)


def boolean(obj, cutter, op):
    """Apply a boolean (op = 'DIFFERENCE' or 'UNION') of cutter into obj,
    then delete cutter."""
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
    try:        # Blender 4.1+
        bpy.ops.wm.stl_export(filepath=filepath, export_selected_objects=True,
                              global_scale=1.0, apply_modifiers=True)
    except Exception:   # older Blender
        bpy.ops.export_mesh.stl(filepath=filepath, use_selection=True,
                                global_scale=1.0)


def boss_positions():
    return [(BOSS_RADIAL * math.cos(2 * math.pi * i / N_BOSS),
             BOSS_RADIAL * math.sin(2 * math.pi * i / N_BOSS))
            for i in range(N_BOSS)]


# ----------------------------------------------------------------------------
# build the SHELL
# ----------------------------------------------------------------------------
def build_shell():
    steps = 48
    # outer solid dome: axis-bottom -> flat crown edge -> dome -> rim -> axis-top
    prof = [(0.0, 0.0), (FLAT_R, 0.0)]
    for s in range(1, steps + 1):
        z = HALF_WIDTH * s / steps
        prof.append((r_outer(z), z))
    prof.append((0.0, HALF_WIDTH))
    shell = revolve("shell", prof)

    # cavity: dome offset inward by WALL, floor at FLOOR, open at the top
    cav = [(0.0, FLOOR)]
    for s in range(0, steps + 1):
        z = FLOOR + (HALF_WIDTH - FLOOR) * s / steps
        cav.append((max(r_outer(z) - WALL, 0.2), z))
    cav.append((0.0, HALF_WIDTH))
    cavity = revolve("cavity", cav)
    boolean(shell, cavity, 'DIFFERENCE')

    # screw bosses + centre boss (union)
    for i, (x, y) in enumerate(boss_positions()):
        b = add_cylinder("boss%d" % i, BOSS_OD, FLOOR - 0.01, SEAT_Z, x, y)
        boolean(shell, b, 'UNION')
    # ctr = add_cylinder("ctrboss", CTR_OD, FLOOR - 0.01, SEAT_Z)
    # boolean(shell, ctr, 'UNION')

    # pilot holes + centre axle hole (difference)
    for i, (x, y) in enumerate(boss_positions()):
        h = add_cylinder("pilot%d" % i, PILOT_D, FLOOR + 0.5, SEAT_Z + 0.1, x, y)
        boolean(shell, h, 'DIFFERENCE')
    # ch = add_cylinder("ctrhole", CTR_HOLE_D, -0.1, SEAT_Z + 0.1)
    # boolean(shell, ch, 'DIFFERENCE')
    return shell


def build_shell_usbc():
    """Shell identical to build_shell() but with a USB-C slot cut through the
    dome wall for the Seeed XIAO ESP32C3. The slot is placed near the lid face
    so a USB-C cable can reach the XIAO mounted flat inside the cavity."""
    shell = build_shell()
    shell.name = "shell_usbc"

    r_mid = r_outer(USBC_Z_CTR)
    # Cutter depth is radial: WALL + generous margin so it clears both surfaces.
    depth = WALL + 6.0
    cx = r_mid * math.cos(USBC_ANG)
    cy = r_mid * math.sin(USBC_ANG)
    cutter = add_box("usbc_cut", depth, USBC_W, USBC_H, cx, cy, USBC_Z_CTR, rz=USBC_ANG)
    boolean(shell, cutter, 'DIFFERENCE')
    return shell


# ----------------------------------------------------------------------------
# build the LIDS  (each carries half of the hollow threaded axle)
# ----------------------------------------------------------------------------
def _lid_base():
    """The flat disc + perimeter screw clearance holes/counterbores, shared by
    both lids. Returns (lid_obj, lid_r). Lid disc spans SEAT_Z .. top_z."""
    lid_r = r_outer(SEAT_Z) - WALL - LID_CLEAR
    top_z = SEAT_Z + LID_THICK
    lid = add_cylinder("lid", 2 * lid_r, SEAT_Z, top_z, segs=SEGMENTS)
    for i, (x, y) in enumerate(boss_positions()):
        c = add_cylinder("clr%d" % i, SCREW_CLEAR_D, SEAT_Z - 0.1, top_z + 0.1, x, y)
        boolean(lid, c, 'DIFFERENCE')
        cb = add_cylinder("cb%d" % i, HEAD_CB_D, top_z - HEAD_CB_DEPTH, top_z + 0.1, x, y)
        boolean(lid, cb, 'DIFFERENCE')
    return lid, lid_r, top_z


def build_lid_male():
    """Lid with a HOLLOW male threaded axle stub sticking out of the gap face.
       gap face (top_z) -> [smooth string section] -> [external thread] -> tip
    """
    lid, lid_r, top_z = _lid_base()
    smooth_z1 = top_z + SMOOTH_LEN                  # end of smooth section
    tip_z     = smooth_z1 + THREAD_ENGAGE           # tip of the threaded part

    # smooth section (the axle the string rides on, OD = thread major)
    smooth = add_cylinder("axsmooth", THREAD_MAJOR_D, top_z, smooth_z1, segs=THREAD_SEGS)
    boolean(lid, smooth, 'UNION')
    # external thread on the tip
    th = threaded_solid("axthread", TH_MIN_R, TH_MAJ_R, smooth_z1, tip_z, THREAD_PITCH)
    boolean(lid, th, 'UNION')
    # hollow centre bore for wires, all the way through lid + axle
    bore = add_cylinder("axbore", AXLE_BORE, SEAT_Z - 0.2, tip_z + 0.2, segs=THREAD_SEGS)
    boolean(lid, bore, 'DIFFERENCE')
    return lid, lid_r


def build_lid_female():
    """Lid with a HOLLOW female threaded socket. The socket is cut into the lid
    and a down-tube that reaches into the cavity (lid alone is too thin for the
    full engagement length). The male stub threads in from the gap face."""
    lid, lid_r, top_z = _lid_base()
    tube_bottom = top_z - THREAD_ENGAGE - 1.5       # down-tube reaches into cavity

    # solid down-tube under the lid to host the threads
    tube = add_cylinder("femtube", FEM_TUBE_OD, tube_bottom, SEAT_Z + 0.01, segs=THREAD_SEGS)
    boolean(lid, tube, 'UNION')
    # cut the internal thread with an oversized 'tap' (male + clearance)
    tap = threaded_solid("femtap", TH_MIN_R + THREAD_CLEAR, TH_MAJ_R + THREAD_CLEAR,
                         top_z - THREAD_ENGAGE, top_z + 0.2, THREAD_PITCH)
    boolean(lid, tap, 'DIFFERENCE')
    # through bore for wires below the threaded region
    bore = add_cylinder("fembore", AXLE_BORE, tube_bottom - 0.2, top_z + 0.2, segs=THREAD_SEGS)
    boolean(lid, bore, 'DIFFERENCE')
    return lid, lid_r


# ----------------------------------------------------------------------------
# main
# ----------------------------------------------------------------------------
def main():
    reset_scene()
    shell = build_shell()
    shell.location = (0.0, 0.0, 0.0)

    shell_u = build_shell_usbc()
    shell_u.location = (70.0, 0.0, 0.0)      # offset in X for preview

    lid_m, lid_r = build_lid_male()
    lid_m.location = (0.0, 0.0, 20.0)
    lid_f, _ = build_lid_female()
    lid_f.location = (0.0, 0.0, 36.0)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    export_stl(shell, os.path.join(OUTPUT_DIR, "shell.stl"))

    shell_u.location = (0.0, 0.0, 0.0)
    export_stl(shell_u, os.path.join(OUTPUT_DIR, "shell_usbc.stl"))
    shell_u.location = (70.0, 0.0, 0.0)

    lid_m.location = (0.0, 0.0, 0.0)
    export_stl(lid_m, os.path.join(OUTPUT_DIR, "lid_male.stl"))
    lid_m.location = (0.0, 0.0, 20.0)

    lid_f.location = (0.0, 0.0, 0.0)
    export_stl(lid_f, os.path.join(OUTPUT_DIR, "lid_female.stl"))
    lid_f.location = (0.0, 0.0, 36.0)

    print("=" * 60)
    print("Wrote shell.stl, shell_usbc.stl, lid_male.stl, lid_female.stl to:", OUTPUT_DIR)
    print("  PRINT: 1x shell, 1x shell_usbc, 1x lid_male, 1x lid_female")
    print("  shell_usbc has a USB-C slot (%.1f x %.1f mm) at r=%.1f mm, z=%.1f mm"
          % (USBC_W, USBC_H, r_outer(USBC_Z_CTR), USBC_Z_CTR))
    print("  -> mount XIAO ESP32C3 flat inside shell_usbc with USB-C end")
    print("     facing the slot (adjust USBC_Z_CTR to match your mount height)")
    print("  yo-yo diameter : %.1f mm" % OUTER_DIAMETER)
    print("  half width     : %.1f mm  (x2 = %.1f mm total)"
          % (HALF_WIDTH, 2 * HALF_WIDTH))
    print("  flat crown dia : %.1f mm" % FLAT_DIAMETER)
    print("  string gap     : %.1f mm" % STRING_GAP)
    print("  lid radius     : %.2f mm" % lid_r)
    print("  bosses         : %dx M2, pilot %.1f mm at r=%.1f mm"
          % (N_BOSS, PILOT_D, BOSS_RADIAL))
    print("  AXLE (hollow)  : major %.1f mm, pitch %.1f mm, engage %.1f mm"
          % (THREAD_MAJOR_D, THREAD_PITCH, THREAD_ENGAGE))
    print("                   bore %.1f mm, flank clearance %.2f mm"
          % (AXLE_BORE, THREAD_CLEAR))
    print("  PRINT crown-down (flat face on bed). Lids: disc on bed,")
    print("  male axle pointing up; female socket mouth down.")
    print("=" * 60)


if __name__ == "__main__":
    main()