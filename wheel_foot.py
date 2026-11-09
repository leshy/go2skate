"""A wheel foot for the Go2: the foot end's cup (foot.cup: the slot, the M4 screw and insert holes), and under it a fork for a
standard inline-skate wheel on its own two 608 bearings and an 8 mm skate axle.

Foot frame (foot.py): X along the slot, the wheel's rolling direction; Y across, the axle's; Z up the foot's axis. Unitree's
foot axis stands ~14 deg off vertical in the stock stance and only ever pitches, so the axle stays level.

Parametric in the wheel: WHEEL_D, WHEEL_W (inline wheels: 72-110 across, ~24 wide). TOE turns the fork about the foot's axis,
for a skating stride.

    .venv/bin/python wheel_foot.py     # wheel_foot.stl, wheel_foot.step; shows it with the wheel and the calf's tongue in the viewer
"""
import numpy as np
from build123d import (Align, Axis, Box, Circle, Rectangle, Cylinder, Plane, Pos, Rot, Torus, export_step, export_stl, extrude, fillet,
                       make_hull)

import foot as F

WHEEL_D, WHEEL_W = 80.0, 24.0  # the wheel: across, wide (set these to the wheels you get)
AXLE_D = 8.2  # an 8 mm skate axle (or M8 bolt) through both arms
RACE_D, RACE_GAP = 12.0, 1.0  # a boss on each arm's inside bears on the bearing's inner race only, standing the wheel this far off the arm
ARM_T, ARM_END_R = 6.0, 11.0  # the fork's arms: thick; round about the axle
TOP_GAP = 3.0  # the wheel's top under the cup
FLOOR = 4.0  # the cup kept this far under its slot, the fork's top plate
TOE = 0.0  # deg about the foot's axis
UP = (Align.CENTER, Align.CENTER, Align.MIN)

TOP = -(F.SLOT_DEEP + FLOOR)  # where the cup stops and the fork starts
AXLE_W = TOP - TOP_GAP - WHEEL_D / 2  # the axle's height in the foot frame
INNER = WHEEL_W / 2 + RACE_GAP  # the arms' inner faces off the middle


def fork():
    """The two arms, each the hull of a bar along the cup's bottom and a round end about the axle, and a boss on its inside out to
    the bearing's inner race."""
    length = F.CUP[0][1] - 6.0  # just inside the cup's bottom outline
    prof = make_hull((Pos(0, AXLE_W) * Circle(ARM_END_R)).edges() + (Pos(0, TOP + 1.0) * Rectangle(length, 2.0)).edges())
    out = []
    for s in (1, -1):
        out.append(Plane(origin=(0, s * (INNER + ARM_T / 2), 0), x_dir=(1, 0, 0), z_dir=(0, -1, 0)) * extrude(prof, ARM_T / 2, both=True))
        out.append(Plane(origin=(0, s * (INNER + 0.5), AXLE_W), z_dir=(0, -s, 0)) * Cylinder(RACE_D / 2, RACE_GAP + 0.5, align=UP))
    return out


def wheel_foot():
    cup = F.cup() & Pos(0, 0, TOP) * Box(200, 200, 100, align=UP)  # the cup above TOP
    part = cup
    for a in fork():
        part += a
    part -= Pos(0, 0, AXLE_W) * Rot(X=90) * Cylinder(AXLE_D / 2, 100)
    return Rot(Z=TOE) * part if TOE else part


def wheel():
    """For looks and checks: a urethane tyre round a hub, rounded across."""
    r = WHEEL_W / 2
    tyre = Pos(0, 0, AXLE_W) * Rot(X=90) * (Cylinder(WHEEL_D / 2 - r, WHEEL_W) + Torus(WHEEL_D / 2 - r, r))
    return tyre


if __name__ == "__main__":
    p = wheel_foot()
    print(f"wheel foot: valid {p.is_valid}, {len(p.solids())} solid, {p.volume / 1000 * 1.27:.0f} g PETG; axle at w {AXLE_W:.1f}, "
          f"wheel's bottom at w {AXLE_W - WHEEL_D / 2:.1f} (the stock foot's at -52.5: {-(AXLE_W - WHEEL_D / 2) - 52.5:.1f} taller)")
    w = wheel()
    print(f"wheel into the foot: {getattr(w & p, 'volume', 0.0):.2f} mm3; tongue into it: {getattr(F.tongue() & p, 'volume', 0.0):.2f} mm3")
    export_stl(p, str(F.HERE / "wheel_foot.stl"), tolerance=0.02, angular_tolerance=0.1)
    export_step(p, str(F.HERE / "wheel_foot.step"))
    from ocp_vscode import set_port, show
    set_port(3939)
    w.color, p.color = "dimgray", "orange"
    t = F.tongue(); t.color = "steelblue"
    show(p, w, t, names=["wheel_foot", "wheel", "calf_tongue"])
