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
import numpy as np
from build123d import (Align, Axis, Box, Cylinder, Plane, Pos, RectangleRounded, Rot, export_step, export_stl, loft, Torus)

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
UP = (Align.CENTER, Align.CENTER, Align.MIN)

TOP = -(F.SLOT_DEEP + FLOOR)  # where the cup stops and the loft starts
AXLE_W = TOP - TOP_GAP - WHEEL_D / 2  # the axle's height in the foot frame
OUTER = (AXLE_L + STACK_PLAY) / 2  # the arms' outer faces off the wheel's middle: the stack the axle clamps
INNER = OUTER - ARM_T  # the bosses' faces, against the inner races
RACE_GAP = INNER - WHEEL_W / 2  # the wheel's side off them
POCKET = WHEEL_W / 2 + SIDE_CLEAR  # the arms' inside faces, round the bosses


def cup_outline(w):
    """The cup's (length, width, corner radius) at height w, from foot.CUP."""
    ws = [s[0] for s in F.CUP]
    return tuple(float(np.interp(w, ws, [s[i] for s in F.CUP])) for i in (1, 2, 3))


def turned(shape):
    return Pos(0, 0, AXLE_W) * Rot(Z=TOE) * Pos(0, 0, -AXLE_W) * shape


def body():
    """The loft: the cup's outline at TOP, through a waist, to a stadium round the axle (turned TOE), and the round end under it."""
    l0, b0, r0 = cup_outline(TOP + 1.0)
    l1, b1 = 2 * END_R, 2 * OUTER
    mid = TOP + 0.45 * (AXLE_W - TOP)
    sections = [Plane.XY.offset(TOP + 1.0) * RectangleRounded(l0, b0, r0),
                Plane.XY.offset(mid).rotated((0, 0, TOE / 2)) * RectangleRounded(0.55 * l0 + 0.45 * l1, max(b0, b1), min(r0, 0.5 * l1 - 0.1)),
                Plane.XY.offset(AXLE_W).rotated((0, 0, TOE)) * RectangleRounded(l1, b1, END_R - 0.1)]
    part = loft(sections)
    end = Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(END_R, b1)  # the round end below the axle
    return part + turned(end)


def sweep(gap, hub=True):
    """The wheel's swept shape grown `gap` all round (its tread a rounded rectangle across, corners TYRE_R), and with `hub` a band
    POCKET either side out to where the tread's round starts: what the fork keeps clear of."""
    R, wide = WHEEL_D / 2 + gap, WHEEL_W + 2 * gap
    rr = min(TYRE_R + gap, wide / 2 - 0.01)
    a = wide / 2 - rr  # where the tread's rounds are centred, across
    shape = Cylinder(R - rr, 2 * POCKET if hub else wide) + sum((Pos(0, 0, z) * Torus(R - rr, rr) for z in {-a, a}), start=Cylinder(R, max(2 * a, 0.01)))
    shape = Rot(Y=90) * shape  # axle along X, for now
    return turned(Pos(0, 0, AXLE_W) * Rot(Z=90) * shape)


def wheel_foot():
    cup = F.cup() & Pos(0, 0, TOP) * Box(200, 200, 100, align=UP)  # the cup above TOP
    part = cup + body()
    part -= sweep(CLEAR)
    bosses = [Plane(origin=(0, s * (POCKET + 0.5), AXLE_W), z_dir=(0, -s, 0)) * Cylinder(RACE_D / 2, POCKET - INNER + 0.5, align=UP) for s in (1, -1)]
    part += turned(bosses[0]) + turned(bosses[1])
    return part - turned(Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(AXLE_D / 2, 100))


def sides():
    """{'left': part, 'right': part}: the right foot is the left's mirror (they're the same at no toe)."""
    left = wheel_foot()
    return {"left": left, "right": left.mirror(Plane.XZ)}


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
    print(f"arms outside to outside {2 * OUTER:.2f} mm (axle {AXLE_L:.1f}); wheel {RACE_GAP:.2f} off the bosses, {SIDE_CLEAR:.1f} off the arms; "
          f"arm wall {OUTER - POCKET:.2f} round the bosses")
    assert RACE_GAP > 0.5 and OUTER - POCKET >= 4.0
    print(f"wheel into the foot: {getattr(w & p, 'volume', 0.0):.2f} mm3; tongue into it: {getattr(F.tongue() & p, 'volume', 0.0):.2f} mm3")
    F.PRINT.mkdir(exist_ok=True)
    F.MODELS.mkdir(exist_ok=True)
    for old in [*F.PRINT.glob("wheel_foot*.stl"), *F.MODELS.glob("wheel_foot*.step")]:
        old.unlink()
    parts = {"": p} if TOE == 0 else {"_left": p, "_right": p.mirror(Plane.XZ)}  # four alike, or two of each
    for suffix, q in parts.items():
        export_stl(q, str(F.PRINT / f"wheel_foot{suffix}.stl"), tolerance=0.02, angular_tolerance=0.1)
        export_step(q, str(F.MODELS / f"wheel_foot{suffix}.step"))
    print("print:", ", ".join(f"wheel_foot{k}.stl x {4 // len(parts)}" for k in parts))
    from ocp_vscode import set_port, show
    set_port(3939)
    w.color, p.color = "dimgray", "orange"
    t = F.tongue(); t.color = "steelblue"
    hw = hardware()
    show(p, w, t, *hw, names=["wheel_foot", "wheel", "calf_tongue", "608_left", "608_right", "spacer", "axle"],
         colors=["orange", "dimgray", "steelblue", "gold", "gold", "silver", "silver"], alphas=[1, 0.35, 1, 1, 1, 1, 1])
