"""The Go2's detachable foot end, modelled from Unitree's mesh (vendor/, unitree_mujoco) and Ivan's measurements, to build a
wheel foot from.

Frame (the foot's own): X along the slot (u), Y across it, Z up the foot's axis; the origin on the top of the ears, mid-slot.
Unitree's calf frame (knee at the origin, foot down -z) maps into it by `TO_FOOT`: the foot's axis leans A_DEG forward of the calf's z.

The real part: a grey cup (the ears either side of a slot the calf's tongue drops into, two M4 screws across through both ears
and the tongue), a black rubber sole under it. Seam at SEAM.

    .venv/bin/python foot.py      # writes foot.step, foot.stl and vendor/foot_detachable_foot_frame.stl (the mesh, same frame)
"""
from pathlib import Path

import numpy as np
from build123d import (Align, Axis, Box, Cylinder, Plane, Pos, Rot, RectangleRounded, Solid, Wire, export_step, export_stl,
                       extrude, fillet, loft, Sketch)

HERE = Path(__file__).parent
A_DEG = 31.5  # the foot's axis from the calf's z, towards +x
TO_FOOT_OFFSET = (-109.478, 0.0, 150.840)  # after the rotation, to put the origin on the ears' top, mid-slot

SEAM = -27.0  # grey cup over the rubber
# The calf's tongue (Unitree's mesh): a plate TONGUE_HALF either side of the middle, from TONGUE_BACK along to its front edge, which
# leans out from TONGUE_FRONT at the top to its bottom; TONGUE_DEEP down, its bottom edges rounded TONGUE_R. The calf's base, an
# oval BASE_L x BASE_W centred at BASE_U, sits on the ears' top.
TONGUE_HALF, TONGUE_DEEP, TONGUE_R = 5.3, 12.3, 1.5  # 5 shallower than the mesh says: the test print
TONGUE_BACK, TONGUE_FRONT = -20.3, ((15.0, 0.0), (15.0 + 6.0 * 12.3 / 17.3, -12.3))  # u; (u, w) top and bottom of its front edge
BASE_L, BASE_W, BASE_U = 35.5, 27.0, -2.5
FIT = 0.2  # clearance each side of the tongue, in the slot
SLOT_HALF, SLOT_DEEP, SLOT_R = TONGUE_HALF + FIT, TONGUE_DEEP + 0.7, TONGUE_R + FIT  # the slot: open at both ends
EAR_EDGE_R = 5.0  # the ears' top edges, rounded
FRONT_SLOPE = ((13.0, 0.0), (22.5, -18.0))  # (u, w): the +u end of the ears falls away from the top along this line
HOLE_R, HOLE_PITCH, HOLE_DOWN, HOLE_U = 2.2, 15.5, 5.5, 0.0  # M4 clearance; 15.5 apart along the slot, 5.5 under the top (test print 1), centred at u
HEAD_R, HEAD_DEEP, HEAD_SIDE = 3.5, 9.0, 1  # the screws' heads sunk this wide, this deep from the ear's outside, on the +y ear
# The cup's and the sole's outlines, bottom up: (w, length along u, width across, corner radius)
CUP = [(SEAM, 45.0, 38.0, 12.0), (-20.7, 42.5, 35.4, 12.0), (-10.8, 37.5, 32.0, 10.0), (0.0, 37.0, 28.0, 9.0)]
SOLE = [(-52.5, 22.0, 20.0, 8.0), (-49.0, 36.0, 32.0, 12.0), (-40.3, 43.5, 39.4, 14.0), (-30.5, 45.0, 38.0, 13.0), (SEAM, 45.0, 38.0, 12.0)]


def stack(sections):
    return loft([Plane.XY.offset(w) * RectangleRounded(l, b, r) for w, l, b, r in sections])


def cup():
    part = stack(CUP)
    (u0, w0), (u1, w1) = FRONT_SLOPE
    n = np.array([w0 - w1, 0, u1 - u0]); n /= np.linalg.norm(n)  # out of the sloped face, up and forward
    part -= Plane(origin=(u0, 0, w0), z_dir=tuple(n)) * Box(200, 200, 100, align=(Align.CENTER, Align.CENTER, Align.MIN))
    slot = Pos(0, 0, -SLOT_DEEP) * Box(200, 2 * SLOT_HALF, SLOT_DEEP + 10, align=(Align.CENTER, Align.CENTER, Align.MIN))
    part -= fillet(slot.edges().filter_by(Axis.X).group_by(Axis.Z)[0], SLOT_R)
    top = part.edges().group_by(Axis.Z)[-1]
    try:
        part = fillet(top, EAR_EDGE_R)
    except Exception:
        pass  # ponytail: the ears' rounding is cosmetic; skipped if OCCT can't do it on this outline
    holes = []
    for s in (-1, 1):
        u = HOLE_U + s * HOLE_PITCH / 2
        holes.append(Pos(u, 0, -HOLE_DOWN) * Rot(X=90) * Cylinder(HOLE_R, 60))
        out = max(p[0].Y * HEAD_SIDE for p in part.find_intersection_points(Axis((u, 0, -HOLE_DOWN), (0, HEAD_SIDE, 0))))  # the ear's outside
        holes.append(Plane(origin=(u, HEAD_SIDE * (out - HEAD_DEEP), -HOLE_DOWN), z_dir=(0, HEAD_SIDE, 0)) * Cylinder(HEAD_R, 20, align=(Align.CENTER, Align.CENTER, Align.MIN)))
        print(f"head pocket at u {u:+.1f}: ear's outside y {out:.2f}, {out - HEAD_DEEP - SLOT_HALF:.2f} of the ear left under it")
    return part.cut(*holes)


def tongue():
    """The calf's tongue, for checks: what the slot takes."""
    (u0, w0), (u1, w1) = TONGUE_FRONT
    side = [(TONGUE_BACK, 0.0), (u0, w0), (u1, w1), (TONGUE_BACK, w1)]
    from build123d import Polygon
    plate = extrude(Plane.XZ * Polygon(*side, align=None), TONGUE_HALF, both=True)
    plate = fillet(plate.edges().filter_by(Axis.Y, reverse=True).group_by(Axis.Z)[0], TONGUE_R)
    holes = [Pos(HOLE_U + s * HOLE_PITCH / 2, 0, -HOLE_DOWN) * Rot(X=90) * Cylinder(HOLE_R - 0.2, 60) for s in (-1, 1)]  # tapped M4
    return plate.cut(*holes)


def sole():
    return stack(SOLE)


def foot():
    return cup() + sole()


def mesh_in_foot_frame():
    import trimesh
    a = np.radians(A_DEG)
    R = np.eye(4)
    R[:3, :3] = [[np.cos(a), 0, -np.sin(a)], [0, 1, 0], [np.sin(a), 0, np.cos(a)]]
    m = trimesh.load(HERE / "vendor/foot_detachable_mm.stl")
    m.apply_transform(R)
    m.apply_translation(TO_FOOT_OFFSET)
    return m


if __name__ == "__main__":
    f = foot()
    print(f"foot: valid {f.is_valid}, {len(f.solids())} solid, {f.volume / 1000:.1f} cm3, {f.bounding_box()}")
    export_step(f, str(HERE / "foot.step"))
    export_stl(f, str(HERE / "foot.stl"), tolerance=0.02, angular_tolerance=0.1)
    c = Pos(0, 0, -SEAM) * cup()  # a fit test: the cup alone, on its seam face
    export_stl(c, str(HERE / "cup_fit_test.stl"), tolerance=0.02, angular_tolerance=0.1)
    print(f"cup_fit_test.stl: {c.volume / 1000 * 1.27:.0f} g PETG")
    t = tongue()
    export_stl(t, str(HERE / "tongue.stl"), tolerance=0.02, angular_tolerance=0.1)
    print(f"tongue: valid {t.is_valid}, {t.bounding_box()}; in the cup {getattr(t & cup(), "volume", 0.0):.3f} mm3")
    m = mesh_in_foot_frame()
    m.export(HERE / "vendor/foot_detachable_foot_frame.stl")
    print("mesh in the foot frame:", m.bounds.round(1).tolist())
