"""
Basic Imperial-style yo-yo, parametric, for 3D printing. GLUE variant.

Two halves — lid and shell are ONE piece each. No electronics cavity.
The two halves join via a plain (non-threaded) male peg / female socket
that gets GLUED, not screwed. The peg comes in several cross-section
shapes (round, hexagon, triangle, octagon, dodecagon) — anything but
round locks rotation while the glue cures and gives more surface area to
bond, which is the whole point of ditching threads: no thread means no
self-tightening, so the joint needs to resist twisting on its own. Hex
was picked over a square specifically because a square's 90-degree
corners are a sharp stress-concentration point and print a visible seam
artifact at each corner; a hexagon's 120-degree corners are both more
crack-resistant and the standard, print-friendly "wrench flat" shape
(same reasoning as a hex bolt head).

Only the standalone axle_double_peg is hollow — a WIRE_BORE_D through-hole
down its centre axis, for routing wires through as a future test. The dome
halves (half_male / half_female) are solid; they are not bored through.

Every stem shape is capped at the hexagon's own circumscribed diameter —
none of them are allowed to grow bigger than that to keep an "ideal" wall
around the bore, the way the old thread-major-diameter version did. The
triangle is the one shape whose ideal wall would need more room than the
cap allows; it still gets the full WIRE_BORE_D bore, just with a thinner
enclosing wall than the other shapes (see stem_shape_diameter()).

Run in Blender:  Scripting tab -> open this file -> Run Script
or headless:     blender --background --python basic_glue.py

Exports STLs:
    glue_half_male.stl        - dome half with solid male glue peg
    glue_half_female.stl      - dome half with solid female glue socket
    glue_axle_double_peg*.stl - standalone double-ended peg axle, hollow
                                 down its centre, one file per stem
                                 shape, for joining two half_female
                                 shells into a yo-yo with no printed
                                 male half.

Assembly (male/female pair): glue the male peg into the female socket.
The smooth axle section spans the string gap between the two inner faces.
String wraps the smooth section.

Assembly (double-peg pair): glue axle_double_peg into two half_female
shells, one end each. Same smooth-section string wrap as above. Pick
whichever stem shape you like the feel/grip of — they're interchangeable,
all sized to fit the same round female socket.

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
CROWN_FILLET   = 7.0      # radius of the round-over at the flat crown edge (slightly
                           # more than RIM_FILLET so the crown edge feels less sharp)

# --- hollow glue peg/socket (no threads) ---
PEG_STEM_D     = 10.0     # smooth stem / round peg diameter (spans the string gap
                           # AND is what fits into the female socket)
PEG_ENGAGE     = 4.0      # glue engagement length — shallower than a threaded
                           # joint needs, since the whole peg bonds to the socket
                           # instead of relying on thread flank contact
PEG_CLEARANCE  = 0.15     # radial clearance added to the female socket, per side,
                           # so there's room for glue and the peg isn't a press-fit
SMOOTH_LEN     = STRING_GAP   # smooth string-contact section = gap width
FEM_TUBE_OD    = 16.0     # OD of the female socket's reinforcing collar

WIRE_BORE_D    = 4.0      # centre through-hole diameter, axle_double_peg ONLY —
                           # fits ~4 thin Arduino hookup wires bundled snugly.
                           # Kept small (down from an earlier 6mm) so the
                           # triangle stem can still enclose it under the
                           # hex size cap — see stem_shape_diameter().

PEG_SEGS       = 96
SEGMENTS       = 128

OUTPUT_DIR = os.path.expanduser("~/Documents/Code/yoyo")

# ----------------------------------------------------------------------------
# derived
# ----------------------------------------------------------------------------
R        = OUTER_DIAMETER / 2.0
FLAT_R   = FLAT_DIAMETER / 2.0
C = HALF_WIDTH / math.sqrt(1.0 - (FLAT_R / R) ** 2)


def r_outer(z):
    """Outer dome radius at height z (0 = crown, HALF_WIDTH = inner face)."""
    z = min(max(z, 0.0), HALF_WIDTH)
    return R * math.sqrt(max(1.0 - ((HALF_WIDTH - z) / C) ** 2, 0.0))


def _dome_curve(phi):
    """Point on the true elliptical dome curve at parameter phi (0 = pole)."""
    return R * math.sin(phi), HALF_WIDTH - C * math.cos(phi)


def _dome_normal(phi):
    """Outward unit normal of the dome curve at parameter phi."""
    gr = math.sin(phi) / R
    gz = -math.cos(phi) / C
    n = math.hypot(gr, gz)
    return gr / n, gz / n


def crown_fillet_arc(fillet_r, n_arc=12):
    """Round-over profile points for the flat-crown / dome corner at (FLAT_R, 0).
    Solves numerically (bisection on the curve parameter) for the circle of
    radius fillet_r tangent to BOTH the flat crown line and the true dome
    curve — not just the curve's tangent line at the corner. Using the linear
    tangent approximation instead (as a smaller-fillet shortcut would allow)
    leaves the fillet's dome-side endpoint off the real surface by a visible
    amount once fillet_r is large relative to the dome's curvature, producing
    a step/lip right where the fillet meets the dome. Returns (p1, [interior
    arc pts], p2): p1 sits on the flat crown, p2 sits exactly on the dome
    curve, both (r, z) tuples."""
    phi0 = math.acos(HALF_WIDTH / C)   # corner, at (FLAT_R, 0)

    def f(phi):
        _, z = _dome_curve(phi)
        _, nz = _dome_normal(phi)
        return z - fillet_r * nz - fillet_r

    lo, hi = phi0 + 1e-9, math.pi / 2.0 - 1e-9
    f_hi = f(hi)
    for _ in range(60):
        mid = (lo + hi) / 2.0
        if (f(mid) > 0.0) == (f_hi > 0.0):
            hi = mid
        else:
            lo = mid
    phi_star = (lo + hi) / 2.0

    p2 = _dome_curve(phi_star)
    nr, nz = _dome_normal(phi_star)
    center = (p2[0] - fillet_r * nr, p2[1] - fillet_r * nz)
    p1 = (center[0], 0.0)

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


def hollow_bore(obj, z0, z1, segs=64):
    """Cut a WIRE_BORE_D through-hole along the centre axis from z0 to z1.
    Callers pass a range that overruns both ends of the object by ~0.5mm
    so the cut is a clean through-hole rather than stopping flush at a
    face (which can leave a paper-thin membrane after a coincident cut)."""
    cutter = add_cylinder("wirebore", WIRE_BORE_D, z0, z1, segs=segs)
    boolean(obj, cutter, 'DIFFERENCE')


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
# half_male  — dome + male glue peg (smooth section + insertion peg), solid
# ----------------------------------------------------------------------------
def build_half_male():
    half = build_dome("half_male")
    face_z = HALF_WIDTH
    tip_z  = face_z + SMOOTH_LEN + PEG_ENGAGE   # smooth string section + glue peg

    stem = add_cylinder("axstem", PEG_STEM_D, face_z, tip_z, segs=PEG_SEGS)
    boolean(half, stem, 'UNION')

    return half


# ----------------------------------------------------------------------------
# half_female — dome + female glue socket (blind hole + collar), solid
# ----------------------------------------------------------------------------
def build_half_female():
    half = build_dome("half_female")
    face_z      = HALF_WIDTH
    tube_bottom = face_z - PEG_ENGAGE - 1.5   # collar extends a bit past the
                                               # socket depth for a solid base

    tube = add_cylinder("femtube", FEM_TUBE_OD, tube_bottom, face_z + 0.01,
                        segs=PEG_SEGS)
    boolean(half, tube, 'UNION')

    socket_d = PEG_STEM_D + 2.0 * PEG_CLEARANCE
    socket = add_cylinder("femsocket", socket_d, face_z - PEG_ENGAGE, face_z + 0.2,
                          segs=PEG_SEGS)
    boolean(half, socket, 'DIFFERENCE')

    return half


# ----------------------------------------------------------------------------
# axle_double_peg — standalone axle, glue peg on BOTH ends, for joining
# two half_female shells (no printed male dome half needed)
# ----------------------------------------------------------------------------
# Stem cross-section variants: (suffix, sides). The smooth centre section
# is built from the same n-gon cylinder primitive as a round stem, just
# with fewer vertices — 3 = triangle, 6 = hexagon, 8 = octagon, 12 =
# dodecagon, PEG_SEGS = round. Anything but round locks rotation against
# the socket while glue cures and gives more bonding surface than a bare
# round dowel — that's the whole reason to offer a choice now that there's
# no thread to hold the joint together on its own. Hexagon replaces the
# more obvious "square" choice: same anti-rotation benefit, but 120-degree
# corners instead of 90 (less of a crack-starting stress riser) and it's
# the standard shape for a printable, wrench-friendly grip.
#
# The insertion peg on each end stays ROUND regardless of the centre
# shape, because it has to fit the one round female socket every half_female
# is molded with. Only the free-standing smooth section (gripped by hand,
# never inserted into anything) takes on the shape.
AXLE_STEM_SHAPES = [
    ("round",     PEG_SEGS),
    ("hexagon",   6),
    ("triangle",  3),
    ("octagon",   8),
    ("dodecagon", 12),
]


def stem_shape_diameter(sides):
    """Circumscribed stem diameter for an n-sided prism.

    Ideally each shape's flat-to-flat distance (inradius) would be at
    least the round peg's radius, so it sits proud of the peg ends like a
    bolt head. But a low vertex count needs a much bigger circumradius to
    hit that same inradius (a triangle's circumradius is 2x its inradius),
    and letting every shape chase its own "ideal" size is what made the
    old design's triangle balloon outward. Instead, every shape is capped
    at the hexagon's own diameter: nothing is allowed to be bigger than
    the hex. Shapes with more sides than the hex (octagon, dodecagon) hit
    their ideal size well under that cap anyway. The triangle (fewer sides
    than the hex) is the one shape the cap actually binds: it still gets
    the full WIRE_BORE_D bore through it, just with a thinner enclosing
    wall than the other shapes end up with. Round stays at the plain peg
    diameter, unaffected by any of this."""
    if sides >= PEG_SEGS:
        return PEG_STEM_D
    ideal = PEG_STEM_D / math.cos(math.pi / sides)
    cap = PEG_STEM_D / math.cos(math.pi / 6)   # hexagon's own diameter
    return min(ideal, cap)


def build_axle_double_peg(stem_segs=PEG_SEGS, stem_dia=PEG_STEM_D,
                          name="axle_double_peg"):
    """Symmetric double-ended glue peg. Smooth centre section spans the
    string gap between two assembled half_female inner faces; each end
    then has a round PEG_ENGAGE-long peg that glues into a half_female
    socket.

    stem_segs sets the smooth section's cross-section: a low vertex count
    (3, 6, 8, 12, ...) turns the round stem into a flat-sided prism instead;
    PEG_SEGS gives a plain round cylinder. stem_dia is that prism's
    circumscribed diameter — see stem_shape_diameter() for shaped stems."""
    smooth_z0 = -SMOOTH_LEN / 2.0
    smooth_z1 = SMOOTH_LEN / 2.0

    axle = add_cylinder(name, stem_dia, smooth_z0, smooth_z1, segs=stem_segs)

    peg_pos = add_cylinder("peg_pos", PEG_STEM_D,
                           smooth_z1, smooth_z1 + PEG_ENGAGE, segs=PEG_SEGS)
    boolean(axle, peg_pos, 'UNION')

    peg_neg = add_cylinder("peg_neg", PEG_STEM_D,
                           smooth_z0 - PEG_ENGAGE, smooth_z0, segs=PEG_SEGS)
    boolean(axle, peg_neg, 'UNION')

    hollow_bore(axle, smooth_z0 - PEG_ENGAGE - 0.5, smooth_z1 + PEG_ENGAGE + 0.5)
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

    axles = []
    for i, (suffix, sides) in enumerate(AXLE_STEM_SHAPES):
        obj_name = "axle_double_peg_" + suffix
        stem_dia = stem_shape_diameter(sides)
        axle = build_axle_double_peg(stem_segs=sides, stem_dia=stem_dia,
                                     name=obj_name)
        axle.location = (140.0 + i * 30.0, 0.0, 0.0)
        fname = "glue_axle_double_peg.stl" if suffix == "round" \
            else "glue_axle_double_peg_%s.stl" % suffix
        axles.append((axle, fname))

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    male.location = (0.0, 0.0, 0.0)
    export_stl(male, os.path.join(OUTPUT_DIR, "glue_half_male.stl"))
    male.location = (0.0, 0.0, 0.0)

    female.location = (0.0, 0.0, 0.0)
    export_stl(female, os.path.join(OUTPUT_DIR, "glue_half_female.stl"))
    female.location = (70.0, 0.0, 0.0)

    for axle, fname in axles:
        orig_loc = axle.location.copy()
        axle.location = (0.0, 0.0, 0.0)
        export_stl(axle, os.path.join(OUTPUT_DIR, fname))
        axle.location = orig_loc

    axle_files = ", ".join(fname for _, fname in axles)
    print("=" * 60)
    print("Wrote glue_half_male.stl, glue_half_female.stl,", axle_files, "to:", OUTPUT_DIR)
    print("  PRINT: 1x glue_half_male, 1x glue_half_female  (both crown-down)")
    print("  OR:    2x glue_half_female + 1x glue_axle_double_peg (any shape)")
    print("  yo-yo diameter : %.1f mm" % OUTER_DIAMETER)
    print("  half width     : %.1f mm  (total %.1f mm)" % (HALF_WIDTH, 2 * HALF_WIDTH))
    print("  flat crown dia : %.1f mm" % FLAT_DIAMETER)
    print("  string gap     : %.1f mm" % STRING_GAP)
    print("  glue peg       : stem %.1f mm, engage %.1f mm, clearance %.2f mm"
          % (PEG_STEM_D, PEG_ENGAGE, PEG_CLEARANCE))
    print("  wire bore      : %.1f mm dia, axle_double_peg only (halves are solid)"
          % WIRE_BORE_D)
    print("=" * 60)


if __name__ == "__main__":
    main()
