"""A wheel foot for the Go2: the foot end's cup (foot.cup: the slot, the M4 screw and insert holes), flowing down into a fork for
a standard inline-skate wheel on its own two 608 bearings and an 8 mm skate axle.

Foot frame (foot.py): X along the slot; Y across, the axle's (at TOE 0); Z up the foot's axis. Unitree's foot axis stands ~14 deg
off vertical in the stock stance and only ever pitches, so the axle stays level.

One body: a loft from the cup's bottom outline down to a rounded end round the axle, turned TOE about the foot's axis on the way
(the skating stride's yaw), with the wheel's swept shape plus clearance taken out of it. So the arms' insides follow the tyre's
round: the most material the wheel leaves room for where they meet the cup.

Parametric in the wheel: WHEEL_D, WHEEL_W (inline wheels: 72-110 across, ~24 wide), TYRE_R (its profile's round).

    .venv/bin/python wheel_foot.py     # wheel_foot.stl, wheel_foot.step; shows it with the wheel and the calf's tongue in the viewer
"""
import numpy as np
from build123d import (Align, Axis, Box, Cylinder, Plane, Pos, RectangleRounded, Rot, export_step, export_stl, loft, Torus)

import foot as F

WHEEL_D, WHEEL_W, TYRE_R = 80.0, 24.0, 12.0  # the wheel: across, wide, its tread's round across (half the width: fully round)
TOE = 0.0  # deg the wheel turns about the foot's axis
AXLE_D = 8.2  # an 8 mm skate axle (or M8 bolt) through both arms
RACE_D, RACE_GAP = 12.0, 1.0  # a boss on each arm's inside bears on the bearing's inner race only, standing the wheel this far off the arm
# As in a skate frame: the 608s are pressed into the wheel's hub (a spacer between), the wheel drops in between the arms and the
# axle goes arm, bearing, spacer, bearing, arm; nothing holds the bearings but the wheel.
CLEAR = 2.0  # round the tyre
ARM_T, END_R = 6.0, 11.0  # the arms: thick at the axle; their round end about it
TOP_GAP = 4.0  # the tyre's top under the cup's bottom (TOP)
FLOOR = 4.0  # the cup kept this far under its slot
UP = (Align.CENTER, Align.CENTER, Align.MIN)

TOP = -(F.SLOT_DEEP + FLOOR)  # where the cup stops and the loft starts
AXLE_W = TOP - TOP_GAP - WHEEL_D / 2  # the axle's height in the foot frame
INNER = WHEEL_W / 2 + RACE_GAP  # the arms' inner faces off the wheel's middle
OUTER = INNER + ARM_T


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
    INNER either side out to where the tread's round starts: what the fork keeps clear of."""
    R, wide = WHEEL_D / 2 + gap, WHEEL_W + 2 * gap
    rr = min(TYRE_R + gap, wide / 2 - 0.01)
    a = wide / 2 - rr  # where the tread's rounds are centred, across
    shape = Cylinder(R - rr, 2 * INNER if hub else wide) + sum((Pos(0, 0, z) * Torus(R - rr, rr) for z in {-a, a}), start=Cylinder(R, max(2 * a, 0.01)))
    shape = Rot(Y=90) * shape  # axle along X, for now
    return turned(Pos(0, 0, AXLE_W) * Rot(Z=90) * shape)


def wheel_foot():
    cup = F.cup() & Pos(0, 0, TOP) * Box(200, 200, 100, align=UP)  # the cup above TOP
    part = cup + body()
    part -= sweep(CLEAR)
    bosses = [Plane(origin=(0, s * (INNER + 0.5), AXLE_W), z_dir=(0, -s, 0)) * Cylinder(RACE_D / 2, RACE_GAP + 0.5, align=UP) for s in (1, -1)]
    part += turned(bosses[0]) + turned(bosses[1])
    return part - turned(Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(AXLE_D / 2, 100))


def wheel():
    """For looks and checks: the tyre, round across."""
    return sweep(0.0, hub=False)


if __name__ == "__main__":
    p = wheel_foot()
    print(f"wheel foot: valid {p.is_valid}, {len(p.solids())} solid, {p.volume / 1000 * 1.27:.0f} g PETG; axle at w {AXLE_W:.1f}, "
          f"wheel's bottom at w {AXLE_W - WHEEL_D / 2:.1f} ({-(AXLE_W - WHEEL_D / 2) - 52.5:.1f} longer than the stock foot)")
    w = wheel()
    print(f"wheel into the foot: {getattr(w & p, 'volume', 0.0):.2f} mm3; tongue into it: {getattr(F.tongue() & p, 'volume', 0.0):.2f} mm3")
    export_stl(p, str(F.HERE / "wheel_foot.stl"), tolerance=0.02, angular_tolerance=0.1)
    export_step(p, str(F.HERE / "wheel_foot.step"))
    from ocp_vscode import set_port, show
    set_port(3939)
    w.color, p.color = "dimgray", "orange"
    t = F.tongue(); t.color = "steelblue"
    show(p, w, t, names=["wheel_foot", "wheel", "calf_tongue"])
