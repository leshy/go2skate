"""A wheel foot for the Go2: the foot end's cup (foot.cup: the slot, the M4 screw and insert holes), flowing down into a fork for
a standard inline-skate wheel on its own two 608 bearings and an 8 mm skate axle.

Foot frame (foot.py): X along the slot; Y across, the axle's (unturned); Z up the foot's axis. Unitree's foot axis stands ~14 deg
off vertical in the stock stance and only ever pitches, so the axle stays level. This builds a left foot turned `a`; a right leg's is
the mirror of the left foot turned the other way.

One body: a loft from the cup's bottom outline down to a rounded end round the axle, turned `a` about the foot's axis on the way
(the skating stride's yaw), with the wheel's swept shape plus clearance taken out of it. So the arms' insides follow the tyre's
round: the most material the wheel leaves room for where they meet the cup.

Every size and the toe angle come from config.py.

    go2skate build               # export(): print/, models/; --show puts the FL foot in the OCP viewer with its wheel and hardware
"""
from functools import cache

import numpy as np
from build123d import (Align, Axis, Box, Cone, Cylinder, GeomType, Plane, Polygon, Pos, RectangleRounded, Rot,
                      export_step, extrude, loft, Torus)

from . import config
from . import foot as F
from .config import SKATE as C

# As in a skate frame: the bearings are pressed into the wheel's hub (a spacer between), the wheel drops in between the arms and
# the axle goes arm, bearing, spacer, bearing, arm; nothing holds the bearings but the wheel.
WHEEL_D, WHEEL_W, TYRE_R = C.wheel.diameter, C.wheel.width, C.wheel.tread
AXLE_D, GRIP = C.axle.d + 2 * C.fit, C.axle.bolt - C.axle.nut_washers  # AXLE_D: the hole; GRIP: the most the bolt clamps
RACE_D, BEARING_D, BEARING_W = C.bearing.race_boss_d, C.bearing.outer_d, C.bearing.width
SIDE_CLEAR, CLEAR, ARM_T, END_R, TOP_GAP, FLOOR = (C.fork.side_clear, C.fork.tyre_clear, C.fork.arm_t, C.fork.end_r, C.fork.top_gap,
                                                   C.fork.cup_floor)
POCKET_R = C.fork.side_clear_r
UP = (Align.CENTER, Align.CENTER, Align.MIN)

TOP = -(F.SLOT_DEEP + FLOOR)  # where the cup stops and the loft starts
AXLE_W = TOP - TOP_GAP - WHEEL_D / 2  # the axle's height in the foot frame
RACE_GAP = C.fork.race_gap  # the wheel's side off the arms; each arm's boss stands out that far, onto the bearing's inner race
INNER = WHEEL_W / 2 + RACE_GAP  # the arms' inside faces off the wheel's middle
OUTER = INNER + ARM_T  # their outer faces
SEAT = min(OUTER, GRIP / 2)  # the washers' faces: sunk into the arms when they're wider than the bolt clamps
WASHER_R = C.axle.washer_d / 2 + 0.5
POCKET = max(INNER, WHEEL_W / 2 + SIDE_CLEAR) if C.fork.side_pocket else INNER  # the arms' inside faces round the bosses


def cup_outline(w):
    """The cup's (length, width, corner radius) at height w, from foot.CUP."""
    ws = [s[0] for s in F.CUP]
    return tuple(float(np.interp(w, ws, [s[i] for s in F.CUP])) for i in (1, 2, 3))


LEGS = ("FL", "FR", "RL", "RR")
TOE = dict(zip(LEGS, C.toe_deg))  # each leg's, the robot's way: + turns the wheel's front towards +Y (left)


def built(leg):
    """The angle a left foot is built at for `leg`: its own, or for a right leg the opposite (the mirror turns it back)."""
    return TOE[leg] if leg[1] == "L" else -TOE[leg]


def turned(shape, a):
    return Pos(0, 0, AXLE_W) * Rot(Z=a) * Pos(0, 0, -AXLE_W) * shape


@cache
def cup_top():
    """The stock cup above TOP, and its face at TOP: the loft starts from that very outline, so there's no seam."""
    top = F.cup() & Pos(0, 0, TOP) * Box(200, 200, 100, align=UP)
    face = min(top.faces().filter_by(GeomType.PLANE), key=lambda f: f.center().Z)
    assert abs(face.center().Z - TOP) < 1e-6
    return top, face


def body(a):
    """The loft: the cup's own section at TOP, through a waist, down to the axle, where it meets the round end's lower half
    exactly: a rectangle the round end's width and the arms' span, its corners cut as the round end's rims are chamfered. So
    the arms' flat sides run on into the round end's faces, and its front and back into its round, with no seam (turned a)."""
    l0, b0, r0 = cup_outline(TOP)
    l1, b1, c = 2 * END_R, 2 * OUTER, C.fork.end_chamfer
    mid = TOP + 0.45 * (AXLE_W - TOP)
    joint = [(l1 / 2, b1 / 2 - c), (l1 / 2 - c, b1 / 2), (-l1 / 2 + c, b1 / 2), (-l1 / 2, b1 / 2 - c),
             (-l1 / 2, -b1 / 2 + c), (-l1 / 2 + c, -b1 / 2), (l1 / 2 - c, -b1 / 2), (l1 / 2, -b1 / 2 + c)] if c > 0 else \
        [(l1 / 2, b1 / 2), (-l1 / 2, b1 / 2), (-l1 / 2, -b1 / 2), (l1 / 2, -b1 / 2)]
    sections = [cup_top()[1],  # a Face: loft ignores a bare Wire
                Plane.XY.offset(mid).rotated((0, 0, a / 2)) * RectangleRounded(0.55 * l0 + 0.45 * l1, max(b0, b1), min(r0, 0.5 * l1 - 0.1)),
                Plane.XY.offset(AXLE_W).rotated((0, 0, a)) * Polygon(*joint, align=None)]
    part = loft(sections)
    end = Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(END_R, b1)
    end = end & Pos(0, 0, AXLE_W) * Box(l1 + 2, b1 + 2, END_R + 2, align=(Align.CENTER, Align.CENTER, Align.MAX))  # its lower half
    if c > 0:  # chamfer the half-rims (arcs) only, not the flat top where the loft sits
        end = end.solids()[0].chamfer(c, None, end.edges().filter_by(GeomType.CIRCLE))
    return part + turned(end, a)


def sweep(gap, a, hub=True):
    """The wheel's swept shape grown `gap` all round (its tread a rounded rectangle across, corners TYRE_R). With `hub`, what the
    fork keeps clear of: that, no wider than INNER either side, with a band INNER either side out to where the tread's round
    starts and POCKET out to POCKET_R."""
    R, wide = WHEEL_D / 2 + gap, WHEEL_W + 2 * gap
    rr = min(TYRE_R + gap, wide / 2 - 0.01)
    k = wide / 2 - rr  # where the tread's rounds are centred, across
    tyre = Cylinder(R - rr, wide) + sum((Pos(0, 0, z) * Torus(R - rr, rr) for z in {-k, k}), start=Cylinder(R, max(2 * k, 0.01)))
    if hub:  # `gap` over the tread, but sideways no further than the arms' faces: no groove where the tread's widest
        tyre = (tyre & Cylinder(R + 1, 2 * INNER)) + Cylinder(R - rr, 2 * INNER)
        if POCKET > INNER:  # the side pocket round the bosses, its edge ramping out at 45° to the arm's face, no step
            d = POCKET - INNER
            ramps = [Pos(0, 0, s * INNER) * Rot(X=0 if s > 0 else 180) * Cone(POCKET_R + d, POCKET_R, d, align=UP) for s in (1, -1)]
            tyre = tyre + Cylinder(POCKET_R, 2 * POCKET) + ramps
    shape = tyre
    shape = Rot(Y=90) * shape  # axle along X, for now
    return turned(Pos(0, 0, AXLE_W) * Rot(Z=90) * shape, a)


@cache
def wheel_foot(a):
    """A left foot, its wheel turned `a`."""
    part = cup_top()[0] + body(a)
    part -= sweep(CLEAR, a)
    part += boss(1, a) + boss(-1, a)
    seat_r, seat_t = C.axle.washer_d / 2 + C.axle.seat_margin, 3.0  # a flat seat for the axle's washer on each arm's outside,
    # exactly at OUTER: filled where the arm's side dips in, spot-faced where it bulges out
    part += [turned(Pos(0, s * (OUTER - seat_t / 2), AXLE_W) * Rot(X=90) * Cylinder(seat_r, seat_t), a) for s in (1, -1)]
    part -= [turned(Pos(0, s * (OUTER + seat_t / 2), AXLE_W) * Rot(X=90) * Cylinder(seat_r, seat_t), a) for s in (1, -1)]
    return part - axle_cut(a)


def boss(s, a):
    """The arm on side `s`'s boss: a 45° cone from its face down to its face on the bearing. The bearings are flush with the
    wheel's sides, so the bosses reach them and the axle clamps the stack."""
    h, r = POCKET - WHEEL_W / 2, RACE_D / 2
    return turned(Plane(origin=(0, s * (POCKET + 0.5), AXLE_W), z_dir=(0, -s, 0)) *
                  (Cylinder(r + h, 0.5, align=UP) + Pos(0, 0, 0.5) * Cone(r + h, r, h, align=UP)), a)


def axle_cut(a):
    """The axle's hole, and the washers' pockets down to SEAT when the arms are wider than the bolt clamps."""
    cut = [turned(Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(AXLE_D / 2, 100), a)]
    if SEAT < OUTER:
        cut += [turned(Pos(0, s * (SEAT + 10), AXLE_W) * Rot(X=90) * Cylinder(WASHER_R, 20), a) for s in (1, -1)]
    return cut


def engraving(leg, outer, lines):
    """Cut `lines` [(2D shapes in the text plane, v)] `label_depth` into the fork's arm: the leg's outer one (`outer`), else the
    other; upright, read from outside; on the round end's flat under the axle's seat. Left legs' outer side is +Y, right legs' -Y
    (their foot is the mirror)."""
    s = (1 if leg[1] == "L" else -1) * (1 if outer else -1)
    d = C.label_depth
    turn = Pos(0, 0, AXLE_W) * Rot(Z=TOE[leg]) * Pos(0, 0, -AXLE_W)
    return [turn * extrude(Plane(origin=(0, s * (OUTER - d), v), x_dir=(-s, 0, 0), z_dir=(0, s, 0)) * shape, d + 0.5, dir=(0, s, 0))
            for shape, v in lines]


def text(t, size):
    from build123d import Text
    return Text(t, font_size=size, font_path=str(config.DATA / "DejaVuSans-Bold.ttf"))  # bundled: the same letters on any machine


def label_cutter(leg):
    """The leg's outer arm: the build's name, and under it the leg and an arrow to the robot's front (+X). The other arm: that
    leg's toe. All under the axle's seat, in the round end's flat band (seat to chamfer)."""
    s = 1 if leg[1] == "L" else -1
    h = C.label_size
    top = AXLE_W - C.axle.washer_d / 2 - 0.6  # just under the axle's washer
    caps = 0.73 * h  # the font's capitals
    name_v, leg_v = top - caps / 2, top - 1.5 * caps - 0.8
    fwd = -s  # +X (the robot's front), in the text plane's own x
    arrow = Pos(fwd * 0.75 * h, 0) * Polygon(*[(fwd * x, y) for x, y in
                                              [(-0.55 * h, 0.12 * h), (0.15 * h, 0.12 * h), (0.15 * h, 0.35 * h), (0.6 * h, 0.0),
                                               (0.15 * h, -0.35 * h), (0.15 * h, -0.12 * h), (-0.55 * h, -0.12 * h)]], align=None)
    leg_line = Pos(-fwd * 0.45 * h, 0) * text(leg, h) + arrow
    leg_line = Pos(-leg_line.bounding_box().center().X, 0) * leg_line  # centred under the axle, like the name over it
    outer = engraving(leg, True, [(text(build_name(), h), name_v), (leg_line, leg_v)])
    r = C.fork.end_r - C.fork.end_chamfer  # the round end's flat
    for size in np.arange(2.5 * h, h / 2, -0.25):  # the toe: as large as fits the band from the head to the chamfer, 1 mm in
        t = text(toe_text(leg), size)
        c, w = 0.73 * size, t.bounding_box().size.X
        v = top - c / 2 if c < top - (AXLE_W - r) - 1.0 else None
        if v is not None and w / 2 + 1.0 <= np.sqrt(max(r ** 2 - (AXLE_W - (v - c / 2)) ** 2, 0)):
            break
    other = engraving(leg, False, [(t, v)])
    return outer + other


def build_name():
    """config's name, or one from toe_deg: S0 straight, V30 every toe out 30, X30 each diagonal pair on a line, else T."""
    fl, fr, rl, rr = C.toe_deg
    if C.name:
        return C.name
    if not any(C.toe_deg):
        return "S0"
    if fl == rl == -fr == -rr:
        return f"V{abs(fl):g}" if fl > 0 else f"A{abs(fl):g}"  # toes out; toes in
    if fl == rr == -fr == -rl:
        return f"X{abs(fl):g}"
    return "T"


def toe_text(leg):
    """The leg's toe as its wheel sees it: 30° out (its front turned away from the body's middle), 30° in, or 0°."""
    t, s = TOE[leg], 1 if leg[1] == "L" else -1
    return "0°" if t == 0 else f"{abs(t):g}° {'out' if t * s > 0 else 'in'}"


def legs():
    """{leg: part}: each turned its own way (a right leg's the mirror of a left foot), engraved (label_cutter)."""
    feet = {leg: wheel_foot(built(leg)) if leg[1] == "L" else wheel_foot(built(leg)).mirror(Plane.XZ) for leg in LEGS}
    if not C.label:
        return feet
    return {leg: q - label_cutter(leg) for leg, q in feet.items()}


def halves(leg):
    """{outer, inner}: `leg`'s foot cut in two along the wheel's middle plane (it crosses no arm, only the cup and the waist; the
    M4s and the axle cross it and clamp the halves), each laid on its cut face, its arm flat over the bed."""
    a = built(leg)
    q = wheel_foot(a)
    ps = {k: q & turned(Pos(0, s * 100, 0) * Box(400, 200, 400), a) for k, s in (("outer", 1), ("inner", -1))}
    side = 1
    if leg[1] == "R":
        ps, side = {k: p.mirror(Plane.XZ) for k, p in ps.items()}, -1
    if C.label:
        cut = label_cutter(leg)
        ps = {k: p - cut for k, p in ps.items()}
    for k, n in (("outer", side), ("inner", -side)):  # n: the half's side along the turned y; its cut face looks the other way
        p = Rot(X=90 * n) * Rot(Z=-TOE[leg]) * ps[k]
        b, low = p.bounding_box(), min(v.Z for v in p.vertices())  # the cut face's own corners: the box is loose round the text
        ps[k] = Pos(-b.center().X, -b.center().Y, -low) * p
    return ps


def wheel(a):
    """For looks and checks: the tyre, round across, turned `a`."""
    return sweep(0.0, a, hub=False)


def hardware(a):
    """For looks: a 608 pressed into each side of the wheel's hub, flush with its faces, the spacer between, the axle through."""
    ring = lambda r0, r1, y0, y1: Pos(0, (y0 + y1) / 2, AXLE_W) * Rot(X=90) * (Cylinder(r1, y1 - y0) - Cylinder(r0, y1 - y0))
    h = WHEEL_W / 2
    parts = [ring(AXLE_D / 2 - 0.1, BEARING_D / 2, s * h - (s > 0) * BEARING_W, s * h + (s < 0) * BEARING_W) for s in (1, -1)]
    parts.append(ring(AXLE_D / 2 - 0.1, 5.0, -h + BEARING_W, h - BEARING_W))  # the spacer
    parts.append(Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(C.axle.d / 2, 2 * OUTER + 4))  # the axle
    return [turned(q, a) for q in parts]


def export(show=False):
    """The four feet into print/ (STL) and models/ (STEP), after its checks; the FL foot in the OCP viewer with `show`."""
    p = wheel_foot(built("FL"))
    print(f"wheel foot: valid {p.is_valid}, {len(p.solids())} solid, {p.volume / 1000 * 1.27:.0f} g PETG; axle at w {AXLE_W:.1f}, "
          f"wheel's bottom at w {AXLE_W - WHEEL_D / 2:.1f} ({-(AXLE_W - WHEEL_D / 2) - 52.5:.1f} longer than the stock foot)")
    print(f"arms outside to outside {2 * OUTER:.2f} mm (the bolt clamps up to {GRIP:.2f}{f', washers sunk {OUTER - SEAT:.2f}' if SEAT < OUTER else ''}); bosses on the bearings, standing {RACE_GAP:.2f} out of the arms; "
          f"wheel {POCKET - WHEEL_W / 2:.2f} off the arms round them{' (pocketed)' if POCKET > INNER else ''}, {RACE_GAP:.2f} beyond; "
          f"arm wall {OUTER - POCKET:.2f}")
    assert RACE_GAP > 0.5 and OUTER - POCKET >= 4.0
    for a in sorted({built(leg) for leg in LEGS}):  # each foot that gets built
        q = wheel_foot(a)
        print(f"turned {a:+g}°: wheel into the foot {getattr(wheel(a) & q, 'volume', 0.0):.2f} mm3, tongue into it "
              f"{getattr(F.tongue() & q, 'volume', 0.0):.2f} mm3")
    if C.split:
        return export_split(show)
    F.PRINT.mkdir(parents=True, exist_ok=True)
    F.MODELS.mkdir(parents=True, exist_ok=True)
    for old in [*F.PRINT.glob("wheel_foot*.stl"), *F.MODELS.glob("wheel_foot*.step")]:
        old.unlink()
    feet = legs()
    for leg, q in feet.items():  # one each: they're labelled, and differ as they're turned
        assert q.is_valid and len(q.solids()) == 1, leg
        F.export_stl_stable(q, F.PRINT / f"wheel_foot_{leg}.stl", tolerance=0.02, angular_tolerance=0.1)
        F.export_step_stable(q, F.MODELS / f"wheel_foot_{leg}.step")
    print("print one each:", ", ".join(f"wheel_foot_{leg}.stl" for leg in feet))
    if not show:
        return feet
    from ocp_vscode import set_port, show as push
    set_port(3939)
    p, w = feet["FL"], wheel(built("FL"))  # shown: the front left, labelled
    w.color, p.color = "#34373c", "orange"
    t = F.tongue(); t.color = "#b0b4ba"
    hw = hardware(built("FL"))
    push(p, w, t, *hw, names=["wheel_foot_FL", "wheel", "calf_tongue", "608_left", "608_right", "spacer", "axle"],
         colors=["orange", "#34373c", "#b0b4ba", "#c4c9cf", "#c4c9cf", "#8b9097", "#6b7178"], alphas=[1, 0.85, 1, 1, 1, 1, 1])
    return feet


def export_split(show=False):
    """The feet in halves into print/: wheel_foot_{leg}_{outer,inner}.stl, laid as they print; with `show`, the FL foot's halves
    together round its wheel in the OCP viewer, one colour each."""
    F.PRINT.mkdir(parents=True, exist_ok=True)
    for old in F.PRINT.glob("wheel_foot*.stl"):
        old.unlink()
    feet = {leg: halves(leg) for leg in LEGS}
    for leg, ps in feet.items():
        for k, q in ps.items():
            assert q.is_valid and len(q.solids()) == 1, (leg, k, len(q.solids()))
            F.export_stl_stable(q, F.PRINT / f"wheel_foot_{leg}_{k}.stl", tolerance=0.02, angular_tolerance=0.1)
        print(f"{leg}: " + ", ".join(f"{k} {q.volume / 1000 * 1.27:.0f} g" for k, q in ps.items()))
    if show:
        from ocp_vscode import set_port, show as push
        set_port(3939)
        a = built("FL")
        q = wheel_foot(a) - label_cutter("FL")
        hs = [q & turned(Pos(0, s * 100, 0) * Box(400, 200, 400), a) for s in (1, -1)]
        push(*hs, wheel(a), names=["outer_half", "inner_half", "wheel"], colors=["orange", "#4a90d9", "#34373c"], alphas=[1, 1, 0.85])
    return feet
