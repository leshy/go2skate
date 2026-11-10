"""A wheel foot for the Go2: the foot end's cup (foot.cup: the slot, the M4 screw and insert holes), flowing down into a fork for
a standard inline-skate wheel on its own two 608 bearings and an 8 mm skate axle.

Foot frame (foot.py): X along the slot; Y across, the axle's (at TOE 0); Z up the foot's axis. Unitree's foot axis stands ~14 deg
off vertical in the stock stance and only ever pitches, so the axle stays level. This builds the left foot; the right is its mirror.

One body: a loft from the cup's bottom outline down to a rounded end round the axle, turned TOE about the foot's axis on the way
(the skating stride's yaw), with the wheel's swept shape plus clearance taken out of it. So the arms' insides follow the tyre's
round: the most material the wheel leaves room for where they meet the cup.

Every size and the toe angle come from config.py.

    uv run python wheel_foot.py     # print/, models/; shows it with the wheel, its hardware and the calf's tongue in the OCP viewer
"""
from functools import cache

import numpy as np
from build123d import (Align, Axis, Box, Cone, Cylinder, GeomType, Plane, Polygon, Pos, RectangleRounded, Rot, export_step, export_stl,
                      extrude, loft, Torus)

import foot as F
from config import SKATE as C

# As in a skate frame: the bearings are pressed into the wheel's hub (a spacer between), the wheel drops in between the arms and
# the axle goes arm, bearing, spacer, bearing, arm; nothing holds the bearings but the wheel.
WHEEL_D, WHEEL_W, TYRE_R = C.wheel.diameter, C.wheel.width, C.wheel.tread
TOE = C.toe_out_deg  # the left foot's: +Z turns the rolling direction (X) towards +Y, out
AXLE_D, AXLE_L, STACK_PLAY = C.axle.hole_d, C.axle.length, C.axle.play
RACE_D, BEARING_D, BEARING_W = C.bearing.race_boss_d, C.bearing.outer_d, C.bearing.width
SIDE_CLEAR, CLEAR, ARM_T, END_R, TOP_GAP, FLOOR = (C.fork.side_clear, C.fork.tyre_clear, C.fork.arm_t, C.fork.end_r, C.fork.top_gap,
                                                   C.fork.cup_floor)
POCKET_R = C.fork.side_clear_r
UP = (Align.CENTER, Align.CENTER, Align.MIN)

TOP = -(F.SLOT_DEEP + FLOOR)  # where the cup stops and the loft starts
AXLE_W = TOP - TOP_GAP - WHEEL_D / 2  # the axle's height in the foot frame
OUTER = (AXLE_L + STACK_PLAY) / 2  # the arms' outer faces off the wheel's middle: the stack the axle clamps
INNER = OUTER - ARM_T  # the arms' inside faces
RACE_GAP = INNER - WHEEL_W / 2  # the wheel's side off them; each arm's boss stands out that far, onto the bearing's inner race
POCKET = max(INNER, WHEEL_W / 2 + SIDE_CLEAR) if C.fork.side_pocket else INNER  # the arms' inside faces round the bosses


def cup_outline(w):
    """The cup's (length, width, corner radius) at height w, from foot.CUP."""
    ws = [s[0] for s in F.CUP]
    return tuple(float(np.interp(w, ws, [s[i] for s in F.CUP])) for i in (1, 2, 3))


def turned(shape):
    return Pos(0, 0, AXLE_W) * Rot(Z=TOE) * Pos(0, 0, -AXLE_W) * shape


@cache
def cup_top():
    """The stock cup above TOP, and its face at TOP: the loft starts from that very outline, so there's no seam."""
    top = F.cup() & Pos(0, 0, TOP) * Box(200, 200, 100, align=UP)
    face = min(top.faces().filter_by(GeomType.PLANE), key=lambda f: f.center().Z)
    assert abs(face.center().Z - TOP) < 1e-6
    return top, face


def body():
    """The loft: the cup's own section at TOP, through a waist, down to the axle, where it meets the round end's lower half
    exactly: a rectangle the round end's width and the arms' span, its corners cut as the round end's rims are chamfered. So
    the arms' flat sides run on into the round end's faces, and its front and back into its round, with no seam (turned TOE)."""
    l0, b0, r0 = cup_outline(TOP)
    l1, b1, c = 2 * END_R, 2 * OUTER, C.fork.end_chamfer
    mid = TOP + 0.45 * (AXLE_W - TOP)
    joint = [(l1 / 2, b1 / 2 - c), (l1 / 2 - c, b1 / 2), (-l1 / 2 + c, b1 / 2), (-l1 / 2, b1 / 2 - c),
             (-l1 / 2, -b1 / 2 + c), (-l1 / 2 + c, -b1 / 2), (l1 / 2 - c, -b1 / 2), (l1 / 2, -b1 / 2 + c)] if c > 0 else \
        [(l1 / 2, b1 / 2), (-l1 / 2, b1 / 2), (-l1 / 2, -b1 / 2), (l1 / 2, -b1 / 2)]
    sections = [cup_top()[1],  # a Face: loft ignores a bare Wire
                Plane.XY.offset(mid).rotated((0, 0, TOE / 2)) * RectangleRounded(0.55 * l0 + 0.45 * l1, max(b0, b1), min(r0, 0.5 * l1 - 0.1)),
                Plane.XY.offset(AXLE_W).rotated((0, 0, TOE)) * Polygon(*joint, align=None)]
    part = loft(sections)
    end = Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(END_R, b1)
    end = end & Pos(0, 0, AXLE_W) * Box(l1 + 2, b1 + 2, END_R + 2, align=(Align.CENTER, Align.CENTER, Align.MAX))  # its lower half
    if c > 0:  # chamfer the half-rims (arcs) only, not the flat top where the loft sits
        end = end.solids()[0].chamfer(c, None, end.edges().filter_by(GeomType.CIRCLE))
    return part + turned(end)


def sweep(gap, hub=True):
    """The wheel's swept shape grown `gap` all round (its tread a rounded rectangle across, corners TYRE_R). With `hub`, what the
    fork keeps clear of: that, no wider than INNER either side, with a band INNER either side out to where the tread's round
    starts and POCKET out to POCKET_R."""
    R, wide = WHEEL_D / 2 + gap, WHEEL_W + 2 * gap
    rr = min(TYRE_R + gap, wide / 2 - 0.01)
    a = wide / 2 - rr  # where the tread's rounds are centred, across
    tyre = Cylinder(R - rr, wide) + sum((Pos(0, 0, z) * Torus(R - rr, rr) for z in {-a, a}), start=Cylinder(R, max(2 * a, 0.01)))
    if hub:  # `gap` over the tread, but sideways no further than the arms' faces: no groove where the tread's widest
        tyre = (tyre & Cylinder(R + 1, 2 * INNER)) + Cylinder(R - rr, 2 * INNER)
        if POCKET > INNER:  # the side pocket round the bosses, its edge ramping out at 45° to the arm's face, no step
            d = POCKET - INNER
            ramps = [Pos(0, 0, s * INNER) * Rot(X=0 if s > 0 else 180) * Cone(POCKET_R + d, POCKET_R, d, align=UP) for s in (1, -1)]
            tyre = tyre + Cylinder(POCKET_R, 2 * POCKET) + ramps
    shape = tyre
    shape = Rot(Y=90) * shape  # axle along X, for now
    return turned(Pos(0, 0, AXLE_W) * Rot(Z=90) * shape)


def wheel_foot():
    part = cup_top()[0] + body()
    part -= sweep(CLEAR)
    face = WHEEL_W / 2  # the bearings are flush with the wheel's sides: the bosses reach them, so the axle clamps the stack
    h, r = POCKET - face, RACE_D / 2  # each boss: a 45° cone from the arm's face down to its face on the bearing
    bosses = [Plane(origin=(0, s * (POCKET + 0.5), AXLE_W), z_dir=(0, -s, 0)) *
              (Cylinder(r + h, 0.5, align=UP) + Pos(0, 0, 0.5) * Cone(r + h, r, h, align=UP)) for s in (1, -1)]
    part += turned(bosses[0]) + turned(bosses[1])
    seat_r, seat_t = C.axle.head_d / 2 + C.axle.seat_margin, 3.0  # a flat seat for the axle's head on each arm's outside,
    # exactly at OUTER: filled where the arm's side dips in, spot-faced where it bulges out
    part += [turned(Pos(0, s * (OUTER - seat_t / 2), AXLE_W) * Rot(X=90) * Cylinder(seat_r, seat_t)) for s in (1, -1)]
    part -= [turned(Pos(0, s * (OUTER + seat_t / 2), AXLE_W) * Rot(X=90) * Cylinder(seat_r, seat_t)) for s in (1, -1)]
    return part - turned(Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(AXLE_D / 2, 100))


def sides():
    """{'left': part, 'right': part}: the right foot is the left's mirror (they're the same at no toe)."""
    left = wheel_foot()
    return {"left": left, "right": left.mirror(Plane.XZ)}


LEGS = ("FL", "FR", "RL", "RR")


def label_cutter(leg):
    """What engraves `leg` and an arrow to the robot's front (+X) into its fork's outer side: on the round end's flat, under the
    axle's seat, reading upright from outside. Left legs' outer side is +Y, right legs' -Y (their foot is the mirror)."""
    from build123d import Text
    s = 1 if leg[1] == "L" else -1
    v = AXLE_W - (C.axle.head_d / 2 + C.axle.seat_margin + C.fork.end_r - C.fork.end_chamfer) / 2  # mid-way down the flat
    d = C.label_depth
    plane = Plane(origin=(0, s * (OUTER - d), v), x_dir=(-s, 0, 0), z_dir=(0, s, 0))  # upright, read from outside
    h = C.label_size
    fwd = -s  # +X (the robot's front), in the plane's own x
    text = plane * Pos(-fwd * 0.45 * h, 0) * Text(leg, font_size=h)
    arrow = plane * Pos(fwd * 0.75 * h, 0) * Polygon(*[(fwd * x, y) for x, y in
                                                       [(-0.55 * h, 0.12 * h), (0.15 * h, 0.12 * h), (0.15 * h, 0.35 * h), (0.6 * h, 0.0),
                                                        (0.15 * h, -0.35 * h), (0.15 * h, -0.12 * h), (-0.55 * h, -0.12 * h)]], align=None)
    cut = Pos(0, 0, AXLE_W) * Rot(Z=s * TOE) * Pos(0, 0, -AXLE_W) * extrude(text + arrow, d + 0.5)
    return cut


def legs():
    """{leg: part}: left legs the left foot, right legs its mirror, each engraved with its leg and the front's arrow."""
    left = wheel_foot()
    feet = {"L": left, "R": left.mirror(Plane.XZ)}
    return {leg: feet[leg[1]] - label_cutter(leg) if C.label else feet[leg[1]] for leg in LEGS}


def wheel():
    """For looks and checks: the tyre, round across."""
    return sweep(0.0, hub=False)


def hardware():
    """For looks: a 608 pressed into each side of the wheel's hub, flush with its faces, the spacer between, the axle through."""
    ring = lambda r0, r1, y0, y1: Pos(0, (y0 + y1) / 2, AXLE_W) * Rot(X=90) * (Cylinder(r1, y1 - y0) - Cylinder(r0, y1 - y0))
    h = WHEEL_W / 2
    parts = [ring(AXLE_D / 2 - 0.1, BEARING_D / 2, s * h - (s > 0) * BEARING_W, s * h + (s < 0) * BEARING_W) for s in (1, -1)]
    parts.append(ring(AXLE_D / 2 - 0.1, 5.0, -h + BEARING_W, h - BEARING_W))  # the spacer
    parts.append(Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(C.axle.d / 2, 2 * OUTER + 4))  # the axle
    return [turned(q) for q in parts]


if __name__ == "__main__":
    p = wheel_foot()
    print(f"wheel foot: valid {p.is_valid}, {len(p.solids())} solid, {p.volume / 1000 * 1.27:.0f} g PETG; axle at w {AXLE_W:.1f}, "
          f"wheel's bottom at w {AXLE_W - WHEEL_D / 2:.1f} ({-(AXLE_W - WHEEL_D / 2) - 52.5:.1f} longer than the stock foot)")
    w = wheel()
    print(f"arms outside to outside {2 * OUTER:.2f} mm (axle {AXLE_L:.1f}); bosses on the bearings, standing {RACE_GAP:.2f} out of the arms; "
          f"wheel {POCKET - WHEEL_W / 2:.2f} off the arms round them{' (pocketed)' if POCKET > INNER else ''}, {RACE_GAP:.2f} beyond; "
          f"arm wall {OUTER - POCKET:.2f}")
    assert RACE_GAP > 0.5 and OUTER - POCKET >= 4.0
    print(f"wheel into the foot: {getattr(w & p, 'volume', 0.0):.2f} mm3; tongue into it: {getattr(F.tongue() & p, 'volume', 0.0):.2f} mm3")
    F.PRINT.mkdir(exist_ok=True)
    F.MODELS.mkdir(exist_ok=True)
    for old in [*F.PRINT.glob("wheel_foot*.stl"), *F.MODELS.glob("wheel_foot*.step")]:
        old.unlink()
    feet = legs()
    for leg, q in feet.items():  # one each: they're labelled, and left and right differ with toe-out
        assert q.is_valid and len(q.solids()) == 1, leg
        export_stl(q, str(F.PRINT / f"wheel_foot_{leg}.stl"), tolerance=0.02, angular_tolerance=0.1)
        F.export_step_stable(q, F.MODELS / f"wheel_foot_{leg}.step")
    print("print one each:", ", ".join(f"wheel_foot_{leg}.stl" for leg in feet))
    from ocp_vscode import set_port, show
    set_port(3939)
    p = feet["FL"]  # shown: the front left, labelled
    w.color, p.color = "#34373c", "orange"
    t = F.tongue(); t.color = "#b0b4ba"
    hw = hardware()
    show(p, w, t, *hw, names=["wheel_foot_FL", "wheel", "calf_tongue", "608_left", "608_right", "spacer", "axle"],
         colors=["orange", "#34373c", "#b0b4ba", "#c4c9cf", "#c4c9cf", "#8b9097", "#6b7178"], alphas=[1, 0.85, 1, 1, 1, 1, 1])
