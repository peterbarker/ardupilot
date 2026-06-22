#!/usr/bin/env python3
"""
Generate the schematic AC3D models for the heliquad (variable-pitch
quad helicopter, Copter FRAME_CLASS 13) and helitiltquadplane (tilting
variable-pitch quadplane) FlightGear visualisations:

    heliquad/Models/heliquad.ac
    helitiltquadplane/Models/helitiltquadplane.ac

Geometry is defined in the FlightGear model frame used by the
animations in each aircraft's Models/*.xml: +x aft, +y starboard,
+z up, origin at the vehicle centre.  AC3D files are y-up, and the
loader rotates them into the model frame, so vertices are converted
at write time.  Run from this directory:

    ./generate_heliquad_models.py

AP_FLAKE8_CLEAN
"""

import os

# nacelle pivot positions, indexed by engine number (== motor number - 1).
# Both vehicles use the AP_MotorsHeli_Quad X layout ordering:
# motor1 front-right, motor2 rear-left, motor3 front-left, motor4 rear-right
PLANE_NACELLE_PIVOTS = [
    (-0.50, 0.45, 0.10),
    (0.45, -0.45, 0.10),
    (-0.50, -0.45, 0.10),
    (0.45, 0.45, 0.10),
]
COPTER_NACELLE_PIVOTS = [
    (-0.30, 0.30, 0.06),
    (0.30, -0.30, 0.06),
    (-0.30, -0.30, 0.06),
    (0.30, 0.30, 0.06),
]

BLADE_LEN = 0.25
BLADE_CHORD = 0.045
DISC_RADIUS = 0.28
ROTOR_Z = 0.085   # rotor plane height above the nacelle pivot

MATERIALS = [
    # name, rgb, trans
    ("fuselage", (0.25, 0.28, 0.32), 0),
    ("wing", (0.80, 0.80, 0.82), 0),
    ("nacelle", (0.85, 0.45, 0.10), 0),
    ("blade", (0.10, 0.10, 0.10), 0),
    ("disc", (0.4, 0.4, 0.45), 0.75),
    ("port", (0.8, 0.1, 0.1), 0),
    ("starboard", (0.1, 0.6, 0.1), 0),
]

MAT_FUSELAGE, MAT_WING, MAT_NACELLE, MAT_BLADE, MAT_DISC, MAT_PORT, MAT_STARBOARD = range(7)


def material_line(name, rgb, trans, spec=0.3, shi=16, emis=(0, 0, 0), amb=0.5):
    r, g, b = rgb
    return ('MATERIAL "%s" rgb %.2f %.2f %.2f  amb %.2f %.2f %.2f  '
            'emis %.2f %.2f %.2f  spec %.2f %.2f %.2f  shi %u  trans %.2f' %
            (name, r, g, b, r * amb, g * amb, b * amb,
             emis[0], emis[1], emis[2], spec, spec, spec, shi, trans))


def poly_object(name, verts, surfs, mat, crease=None):
    """one AC3D poly object; surfs is a list of vertex-index tuples"""
    lines = [
        "OBJECT poly",
        'name "%s"' % name,
    ]
    if crease is not None:
        lines.append("crease %.1f" % crease)
    lines.append("numvert %u" % len(verts))
    for x, y, z in verts:
        # model frame to AC3D y-up frame
        lines.append("%.4f %.4f %.4f" % (x, z, -y))
    lines.append("numsurf %u" % len(surfs))
    for s in surfs:
        # 0x30: shaded polygon, two-sided, so winding order need not be exact
        lines.append("SURF 0x30")
        lines.append("mat %u" % mat)
        lines.append("refs %u" % len(s))
        for idx in s:
            lines.append("%u 0 0" % idx)
    lines.append("kids 0")
    return "\n".join(lines)


BOX_SURFS = [
    (0, 1, 3, 2),
    (4, 6, 7, 5),
    (0, 4, 5, 1),
    (2, 3, 7, 6),
    (0, 2, 6, 4),
    (1, 5, 7, 3),
]


def box(name, xr, yr, zr, mat):
    x0, x1 = xr
    y0, y1 = yr
    z0, z1 = zr
    # vertex index = 4*xi + 2*yi + zi
    verts = [(x, y, z) for x in (x0, x1) for y in (y0, y1) for z in (z0, z1)]
    return poly_object(name, verts, BOX_SURFS, mat)


def bar(name, p0, p1, width, zr, mat):
    """a box from XY point p0 to p1, for diagonal arms"""
    from math import hypot
    (x0, y0), (x1, y1) = p0, p1
    z0, z1 = zr
    length = hypot(x1 - x0, y1 - y0)
    # unit perpendicular in the XY plane
    px = -(y1 - y0) / length * width / 2
    py = (x1 - x0) / length * width / 2
    verts = [(x, y, z)
             for x, y in ((x0 - px, y0 - py), (x0 + px, y0 + py))
             for z in (z0, z1)]
    verts += [(x, y, z)
              for x, y in ((x1 - px, y1 - py), (x1 + px, y1 + py))
              for z in (z0, z1)]
    return poly_object(name, verts, BOX_SURFS, mat)


def disc(name, centre, radius, mat, segments=16):
    from math import cos
    from math import pi
    from math import sin
    cx, cy, cz = centre
    verts = []
    for i in range(segments):
        a = 2 * pi * i / segments
        verts.append((cx + radius * cos(a), cy + radius * sin(a), cz))
    return poly_object(name, verts, [tuple(range(segments))], mat)


def group(name, members):
    out = ["OBJECT group", 'name "%s"' % name, "kids %u" % len(members)]
    out.extend(members)
    return "\n".join(out)


def nacelle(idx, pivots):
    px, py, pz = pivots[idx]
    hub_z = pz + ROTOR_Z
    pod = box("pod%u" % idx,
              (px - 0.035, px + 0.035), (py - 0.035, py + 0.035),
              (pz - 0.06, pz + 0.07), MAT_NACELLE)
    hub = box("hub%u" % idx,
              (px - 0.02, px + 0.02), (py - 0.02, py + 0.02),
              (pz + 0.07, hub_z), MAT_BLADE)
    blade_a = box("blade%ua" % idx,
                  (px - BLADE_CHORD / 2, px + BLADE_CHORD / 2),
                  (py + 0.02, py + 0.02 + BLADE_LEN),
                  (hub_z - 0.004, hub_z + 0.004), MAT_BLADE)
    blade_b = box("blade%ub" % idx,
                  (px - BLADE_CHORD / 2, px + BLADE_CHORD / 2),
                  (py - 0.02 - BLADE_LEN, py - 0.02),
                  (hub_z - 0.004, hub_z + 0.004), MAT_BLADE)
    rotor_disc = disc("disc%u" % idx, (px, py, hub_z), DISC_RADIUS, MAT_DISC)
    rotor = group("rotor%u" % idx, [hub, blade_a, blade_b, rotor_disc])
    return group("nacelle%u" % idx, [pod, rotor])


def legs(pivots):
    out = []
    for i, (px, py, pz) in enumerate(pivots):
        out.append(box("leg%u" % i, (px - 0.01, px + 0.01),
                       (py - 0.01, py + 0.01), (-0.14, pz - 0.05), MAT_FUSELAGE))
    return out


def plane_airframe():
    return [
        box("fuselage", (-0.75, 0.75), (-0.06, 0.06), (-0.07, 0.07), MAT_FUSELAGE),
        box("wing", (-0.15, 0.13), (-1.0, 1.0), (0.06, 0.08), MAT_WING),
        # port/starboard wingtip markers, aviation convention red/green
        box("tip_l", (-0.15, 0.13), (-1.04, -1.0), (0.06, 0.08), MAT_PORT),
        box("tip_r", (-0.15, 0.13), (1.0, 1.04), (0.06, 0.08), MAT_STARBOARD),
        box("aileron_l", (0.13, 0.21), (-0.95, -0.55), (0.065, 0.075), MAT_WING),
        box("aileron_r", (0.13, 0.21), (0.55, 0.95), (0.065, 0.075), MAT_WING),
        box("hstab", (0.60, 0.74), (-0.35, 0.35), (0.015, 0.025), MAT_WING),
        box("elevator", (0.74, 0.83), (-0.35, 0.35), (0.015, 0.025), MAT_WING),
        box("fin", (0.55, 0.75), (-0.005, 0.005), (0.05, 0.30), MAT_WING),
        box("rudder", (0.75, 0.84), (-0.005, 0.005), (0.05, 0.28), MAT_WING),
        # cross-beams carrying the nacelles: one ahead of the wing and
        # one behind it, each spanwise (parallel to the wing). A
        # longitudinal side-boom would sit in the swept disc of the rear
        # rotors once they tilt forward; these lateral beams line up with
        # the nacelle tilt axis instead
        box("crossbar_f", (-0.53, -0.47), (-0.45, 0.45), (0.07, 0.11), MAT_FUSELAGE),
        box("crossbar_r", (0.42, 0.48), (-0.45, 0.45), (0.07, 0.11), MAT_FUSELAGE),
    ] + legs(PLANE_NACELLE_PIVOTS)


def copter_airframe():
    objects = [
        box("body", (-0.12, 0.12), (-0.12, 0.12), (-0.05, 0.07), MAT_FUSELAGE),
        # nose marker so vehicle heading is obvious
        box("nose", (-0.18, -0.12), (-0.03, 0.03), (-0.02, 0.04), MAT_PORT),
    ]
    for i, (px, py, pz) in enumerate(COPTER_NACELLE_PIVOTS):
        objects.append(bar("arm%u" % i, (0, 0), (px, py), 0.05,
                           (pz - 0.04, pz), MAT_FUSELAGE))
    return objects + legs(COPTER_NACELLE_PIVOTS)


# ---------------------------------------------------------------------
# the -nice model: same animated object names driven by the same
# animation XML, but smooth-shaded lofted geometry and a glossier
# palette

NICE_MATERIALS = [
    # name, rgb, trans, spec, shi, emis, amb
    ("pearl",    (0.93, 0.94, 0.96), 0,    0.9, 80, (0, 0, 0), 0.85),
    ("accent",   (0.95, 0.45, 0.08), 0,    0.6, 48, (0, 0, 0), 0.85),
    ("canopy",   (0.10, 0.16, 0.22), 0.35, 1.0, 128, (0, 0, 0), 0.5),
    ("carbon",   (0.10, 0.10, 0.11), 0,    0.5, 40, (0, 0, 0), 0.7),
    ("blurdisc", (0.35, 0.35, 0.40), 0.80, 0.0, 1, (0, 0, 0), 0.5),
    ("navred",   (0.80, 0.05, 0.05), 0,    0.2, 8, (0.9, 0.05, 0.05), 0.5),
    ("navgreen", (0.05, 0.65, 0.10), 0,    0.2, 8, (0.05, 0.8, 0.1), 0.5),
    ("graphite", (0.30, 0.31, 0.35), 0,    0.7, 60, (0, 0, 0), 0.8),
]

(NMAT_PEARL, NMAT_ACCENT, NMAT_CANOPY, NMAT_CARBON,
 NMAT_DISC, NMAT_NAVRED, NMAT_NAVGREEN, NMAT_GRAPHITE) = range(8)

SMOOTH = 61.0   # crease angle for smooth-shaded curved surfaces


def loft(name, sections, mat, closed_ends=True, crease=SMOOTH):
    """skin a set of rings (each a list of 3d points, equal lengths)"""
    n = len(sections[0])
    verts = [pt for ring in sections for pt in ring]
    surfs = []
    for r in range(len(sections) - 1):
        for i in range(n):
            j = (i + 1) % n
            surfs.append((r * n + i, r * n + j,
                          (r + 1) * n + j, (r + 1) * n + i))
    if closed_ends:
        surfs.append(tuple(range(n - 1, -1, -1)))
        last = (len(sections) - 1) * n
        surfs.append(tuple(range(last, last + n)))
    return poly_object(name, verts, surfs, mat, crease)


def ellipse_ring(x, ry, rz, zc, segments=14, y0=0.0):
    from math import cos, pi, sin
    return [(x, y0 + ry * cos(2 * pi * i / segments),
             zc + rz * sin(2 * pi * i / segments))
            for i in range(segments)]


def lathe(name, stations, mat, segments=14, y0=0.0):
    """body of revolution along +x; stations = (x, ry, rz, z_centre)"""
    return loft(name, [ellipse_ring(x, ry, rz, zc, segments, y0)
                       for x, ry, rz, zc in stations], mat)


def tube_z(name, x, y, z0, z1, r, mat, segments=8):
    """vertical tube centred on (x, y)"""
    return loft(name,
                [[(x + r * c, y + r * s, z) for c, s in _circle(segments)]
                 for z in (z0, z1)], mat)


def _circle(segments):
    from math import cos, pi, sin
    return [(cos(2 * pi * i / segments), sin(2 * pi * i / segments))
            for i in range(segments)]


def tube_between(name, p0, p1, r, mat, segments=10):
    """round tube between two 3d points p0 -> p1"""
    from math import cos, pi, sin
    ax = [p1[j] - p0[j] for j in range(3)]
    length = sum(c * c for c in ax) ** 0.5
    u = [c / length for c in ax]
    # any reference not parallel to the axis, to build a cross-section frame
    ref = [0.0, 0.0, 1.0] if abs(u[2]) < 0.9 else [1.0, 0.0, 0.0]
    v = [u[1] * ref[2] - u[2] * ref[1],
         u[2] * ref[0] - u[0] * ref[2],
         u[0] * ref[1] - u[1] * ref[0]]
    vlen = sum(c * c for c in v) ** 0.5
    v = [c / vlen for c in v]
    w = [u[1] * v[2] - u[2] * v[1],
         u[2] * v[0] - u[0] * v[2],
         u[0] * v[1] - u[1] * v[0]]
    rings = []
    for p in (p0, p1):
        ring = []
        for i in range(segments):
            a = 2 * pi * i / segments
            ring.append(tuple(p[j] + r * (cos(a) * v[j] + sin(a) * w[j])
                              for j in range(3)))
        rings.append(ring)
    return loft(name, rings, mat)


AIRFOIL = [
    # unit-chord airfoil outline, (x, z), trailing edge at x=1
    (0.00, 0.000), (0.03, 0.022), (0.15, 0.036), (0.35, 0.038),
    (0.60, 0.026), (1.00, 0.001),
    (0.60, -0.010), (0.35, -0.014), (0.15, -0.012), (0.03, -0.010),
]


def wing_section(xle, y, chord, zc):
    return [(xle + px * chord, y, zc + pz * chord) for px, pz in AIRFOIL]


def wing_loft(name, sections, mat):
    return loft(name, sections, mat)


def nice_plane_airframe():
    objects = []
    # fuselage: slender pod with pointed nose and tapered tail boom
    objects.append(lathe("nose", [
        (-0.80, 0.001, 0.001, 0.015),
        (-0.75, 0.030, 0.034, 0.012),
        (-0.66, 0.052, 0.060, 0.010),
    ], NMAT_ACCENT, segments=16))
    objects.append(lathe("fuselage", [
        (-0.66, 0.052, 0.060, 0.010),
        (-0.50, 0.062, 0.072, 0.012),
        (-0.25, 0.066, 0.078, 0.015),
        (0.10, 0.060, 0.072, 0.020),
        (0.45, 0.040, 0.048, 0.028),
        (0.70, 0.024, 0.028, 0.035),
        (0.84, 0.010, 0.012, 0.040),
    ], NMAT_PEARL, segments=16))
    # tinted canopy blister over the front cabin
    objects.append(lathe("canopy", [
        (-0.60, 0.001, 0.001, 0.070),
        (-0.48, 0.046, 0.050, 0.070),
        (-0.28, 0.054, 0.062, 0.070),
        (-0.10, 0.044, 0.040, 0.070),
        (-0.02, 0.001, 0.001, 0.070),
    ], NMAT_CANOPY, segments=14))
    # wing: tapered, slight dihedral, rounded by the airfoil loft
    for side, sign in (("_r", 1), ("_l", -1)):
        objects.append(wing_loft("wing%s" % side, [
            wing_section(-0.16, sign * 0.05, 0.34, 0.055),
            wing_section(-0.15, sign * 0.45, 0.32, 0.062),
            wing_section(-0.13, sign * 0.80, 0.27, 0.072),
            wing_section(-0.11, sign * 1.00, 0.20, 0.082),
            wing_section(-0.10, sign * 1.05, 0.10, 0.086),
        ], NMAT_PEARL))
    # nav lights on the tips, aviation red/green
    objects.append(lathe("navlight_l", [
        (-0.16, 0.001, 0.001, 0.086), (-0.13, 0.016, 0.016, 0.086),
        (-0.10, 0.001, 0.001, 0.086)], NMAT_NAVRED, segments=8, y0=-1.05))
    objects.append(lathe("navlight_r", [
        (-0.16, 0.001, 0.001, 0.086), (-0.13, 0.016, 0.016, 0.086),
        (-0.10, 0.001, 0.001, 0.086)], NMAT_NAVGREEN, segments=8, y0=1.05))
    # ailerons: thin airfoil-tail wedges hinged on the wing trailing edge
    for side, sign in (("_l", -1), ("_r", 1)):
        y0, y1 = sorted((sign * 0.55, sign * 0.95))
        objects.append(loft("aileron%s" % side, [
            [(0.14, y, 0.070), (0.155, y, 0.073), (0.23, y, 0.068),
             (0.155, y, 0.063)] for y in (y0, y1)], NMAT_ACCENT,
            crease=0.5))
    # tailplane and elevator
    objects.append(wing_loft("hstab", [
        wing_section(0.58, -0.36, 0.17, 0.02),
        wing_section(0.57, 0.0, 0.19, 0.02),
        wing_section(0.58, 0.36, 0.17, 0.02),
    ], NMAT_PEARL))
    objects.append(loft("elevator", [
        [(0.745, y, 0.022), (0.76, y, 0.026), (0.85, y, 0.020),
         (0.76, y, 0.016)] for y in (-0.34, 0.34)], NMAT_ACCENT,
        crease=0.5))
    # swept fin and rudder
    objects.append(loft("fin", [
        [(0.55, -0.012, 0.05), (0.75, -0.006, 0.05),
         (0.75, 0.006, 0.05), (0.55, 0.012, 0.05)],
        [(0.68, -0.007, 0.30), (0.76, -0.004, 0.30),
         (0.76, 0.004, 0.30), (0.68, 0.007, 0.30)],
    ], NMAT_PEARL, crease=0.5))
    objects.append(loft("rudder", [
        [(0.755, -0.005, 0.05), (0.85, -0.001, 0.05),
         (0.85, 0.001, 0.05), (0.755, 0.005, 0.05)],
        [(0.765, -0.004, 0.28), (0.84, -0.001, 0.28),
         (0.84, 0.001, 0.28), (0.765, 0.004, 0.28)],
    ], NMAT_ACCENT, crease=0.5))
    # tail bumper light
    objects.append(lathe("navlight_t", [
        (0.82, 0.001, 0.001, 0.30), (0.84, 0.010, 0.010, 0.30),
        (0.86, 0.001, 0.001, 0.30)], NMAT_PEARL, segments=8))
    # cross-beams carrying the nacelles: spanwise tubes ahead of and
    # behind the wing (parallel to it). A longitudinal side-boom would
    # lie in the rear rotors' forward-tilt disc; these lateral beams line
    # up with the nacelle tilt axis instead
    for tag, bx in (("f", -0.50), ("r", 0.45)):
        objects.append(tube_between("crossbar_%s" % tag,
                                    (bx, -0.45, 0.09), (bx, 0.45, 0.09),
                                    0.024, NMAT_GRAPHITE))
    # landing skids: two rails under the booms on slender struts
    for side, sign in (("_l", -1), ("_r", 1)):
        objects.append(loft("skid%s" % side, [
            ellipse_ring(x, 0.012, 0.012, -0.16, 8, sign * 0.45)
            for x in (-0.55, -0.20, 0.30, 0.50)], NMAT_CARBON))
        for xi, x in enumerate((-0.45, 0.40)):
            objects.append(tube_z("strut%s%u" % (side, xi),
                                  x, sign * 0.45, -0.155, 0.045,
                                  0.008, NMAT_CARBON))
    return objects


def nice_nacelle(idx, pivots):
    px, py, pz = pivots[idx]
    hub_z = pz + ROTOR_Z
    # smooth pod with rounded ends
    pod = lathe("pod%u" % idx, [
        (px - 0.075, 0.001, 0.001, pz),
        (px - 0.065, 0.030, 0.030, pz),
        (px - 0.02, 0.042, 0.042, pz),
        (px + 0.04, 0.038, 0.038, pz),
        (px + 0.075, 0.012, 0.012, pz),
    ], NMAT_GRAPHITE, segments=12, y0=py)
    # mast up to the rotor head
    mast = tube_z("mast%u" % idx, px, py, pz + 0.02, hub_z - 0.015,
                  0.012, NMAT_GRAPHITE)
    # spinning parts: spinner cone, hub, tapered blades, blur disc
    spinner = lathe("spinner%u" % idx, [
        (px - 0.0001, 0.020, 0.020, hub_z - 0.015),
        (px, 0.024, 0.024, hub_z),
        (px + 0.0001, 0.012, 0.012, hub_z + 0.022),
    ], NMAT_ACCENT, segments=10, y0=py)
    blades = []
    for suffix, bsign in (("a", 1), ("b", -1)):
        root = py + bsign * 0.02
        tip = py + bsign * (0.02 + BLADE_LEN)
        near_tip = py + bsign * (0.02 + BLADE_LEN * 0.92)
        blades.append(loft("blade%u%s" % (idx, suffix), [
            [(px - 0.016, root, hub_z - 0.004), (px + 0.016, root, hub_z - 0.004),
             (px + 0.016, root, hub_z + 0.004), (px - 0.016, root, hub_z + 0.004)],
            [(px - 0.023, near_tip, hub_z - 0.003), (px + 0.023, near_tip, hub_z - 0.003),
             (px + 0.023, near_tip, hub_z + 0.003), (px - 0.023, near_tip, hub_z + 0.003)],
            [(px - 0.008, tip, hub_z - 0.001), (px + 0.010, tip, hub_z - 0.001),
             (px + 0.010, tip, hub_z + 0.001), (px - 0.008, tip, hub_z + 0.001)],
        ], NMAT_CARBON, crease=0.5))
    rotor_disc = disc("disc%u" % idx, (px, py, hub_z), DISC_RADIUS,
                      NMAT_DISC, segments=32)
    rotor = group("rotor%u" % idx, [spinner] + blades + [rotor_disc])
    return group("nacelle%u" % idx, [pod, mast, rotor])


def write_model(path, airframe, pivots, materials=None, nacelle_fn=None):
    lines = ["AC3Db"]
    for m in (materials if materials is not None else MATERIALS):
        lines.append(material_line(*m) if isinstance(m, tuple) else m)
    if nacelle_fn is None:
        nacelle_fn = nacelle
    members = airframe + [nacelle_fn(i, pivots) for i in range(4)]
    lines.append(group("world", members))
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def write_nice_xml(here):
    """the -nice aircraft shares the schematic model's animations
    verbatim; regenerate its model XML from the source of truth so
    the two can never drift"""
    src = os.path.join(here, "helitiltquadplane", "Models", "helitiltquadplane.xml")
    dst = os.path.join(here, "helitiltquadplane-nice", "Models", "helitiltquadplane-nice.xml")
    with open(src) as f:
        xml = f.read()
    xml = xml.replace("<path>helitiltquadplane.ac</path>",
                      "<path>helitiltquadplane-nice.ac</path>")
    header = ("<!-- GENERATED by generate_heliquad_models.py from\n"
              "     ../../helitiltquadplane/Models/helitiltquadplane.xml -\n"
              "     edit that file and regenerate -->\n")
    xml = xml.replace("<?xml version=\"1.0\"?>\n",
                      "<?xml version=\"1.0\"?>\n" + header, 1)
    with open(dst, "w") as f:
        f.write(xml)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    write_model(os.path.join(here, "heliquad", "Models", "heliquad.ac"),
                copter_airframe(), COPTER_NACELLE_PIVOTS)
    write_model(os.path.join(here, "helitiltquadplane", "Models", "helitiltquadplane.ac"),
                plane_airframe(), PLANE_NACELLE_PIVOTS)
    write_model(os.path.join(here, "helitiltquadplane-nice", "Models",
                             "helitiltquadplane-nice.ac"),
                nice_plane_airframe(), PLANE_NACELLE_PIVOTS,
                materials=NICE_MATERIALS, nacelle_fn=nice_nacelle)
    write_nice_xml(here)


if __name__ == "__main__":
    main()
