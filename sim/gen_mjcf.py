"""The Go2 on skates for MuJoCo, built from the CAD: Menagerie's go2.xml with each foot swapped for our printed wheel foot and
an inline wheel on a free hinge. Every size, the toe angle and the physics come from ../config.py; rerun after changing it (or
foot.py, wheel_foot.py), and the model follows the parts.

Each calf loses the stock foot: its visual mesh, the 22 mm collision sphere, the rubber sole (Menagerie's calf_1) and the grey
cup (the bottom of calf_0), drawn instead with our calf cut off at the cup (vendor/calf_upper_mm.stl, the ScanRuler cut). It gains:
  <leg>_skate   fixed to the calf: the printed wheel foot, visual, its own mass
  <leg>_wheel   on <leg>_wheel_joint, a hinge along the axle: the wheel, collision geom still named <leg> (contact code keyed on
                the foot's geom name keeps working), visual
Left legs (FL, RL) get the left foot, right legs its mirror image. The 'home' keyframe keeps Menagerie's joint angles; check.py
raises its base until the wheels just touch the floor.

Writes everything into generated/ (don't edit it, rerun this): go2skate.xml, scene.xml (floor + the robot), meshes/*.stl
(metres, in the calf or wheel frame), spec.json (the same numbers for whoever trains on it).

    uv run python sim/gen_mjcf.py       # then: uv run python sim/check.py, uv run python sim/view.py
"""
import json
import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from build123d import CenterOf, export_stl

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE.parent))
import foot as F  # noqa: E402
import wheel_foot as W  # noqa: E402
from config import SKATE as C  # noqa: E402

MENAGERIE = HERE / "vendor/go2_menagerie/unitree_go2"
OUT = HERE / "generated"
LEGS = ("FL", "FR", "RL", "RR")
SIDE = {"FL": "left", "RL": "left", "FR": "right", "RR": "right"}
MIRROR = np.diag([1.0, -1.0, 1.0])  # left to right: the calf frames are alike, the feet mirror about the leg's plane


def calf_from_foot():
    """4x4, mm: the foot's frame (foot.py) into Unitree's calf frame (knee at the origin, foot down -z), the inverse of TO_FOOT."""
    a = np.radians(F.A_DEG)
    R = np.array([[np.cos(a), 0, -np.sin(a)], [0, 1, 0], [np.sin(a), 0, np.cos(a)]])
    T = np.eye(4)
    T[:3, :3] = R.T
    T[:3, 3] = -R.T @ np.array(F.TO_FOOT_OFFSET)
    return T


def tessellated(shape, path):
    export_stl(shape, str(path), tolerance=0.05, angular_tolerance=0.2)
    return trimesh.load(path)


def solid_inertia(part, T, mass):
    """Centre of mass (m) and inertia about it (kg·m²), in the calf frame, of a CAD solid (foot frame, mm) moved by T: from the
    B-rep, which is exact (its tessellation needn't be watertight)."""
    com = (T @ [*part.center(CenterOf.MASS), 1])[:3] / 1000.0
    I = np.array(part.matrix_of_inertia) * (mass / part.volume) * 1e-6  # unit density, mm^5 -> kg·m²
    R = T[:3, :3]
    return com, R @ I @ R.T


def mjcf_inertial(com, I, mass):
    return dict(pos=" ".join(f"{v:.6g}" for v in com), mass=f"{mass:.6g}",
                fullinertia=" ".join(f"{v:.6g}" for v in (I[0, 0], I[1, 1], I[2, 2], I[0, 1], I[0, 2], I[1, 2])))


def vec(v):
    return " ".join(f"{x:.6g}" for x in v)


def build():
    out = OUT / "meshes"
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("*.stl"):
        old.unlink()
    T = calf_from_foot()
    centre = (T @ [0, 0, W.AXLE_W, 1])[:3] / 1000.0  # on the foot's axis: the toe turns the wheel about it, so both sides share it
    t = np.radians(W.TOE)
    axle = T[:3, :3] @ [-np.sin(t), np.cos(t), 0.0]  # the left wheel's: the foot's +Y turned TOE about its axis

    part = W.wheel_foot()  # the left foot
    skate_mass = part.volume / 1000 * C.sim.petg_density / 1000  # kg
    com, I = solid_inertia(part, T, skate_mass)
    wheel_com, wheel_I = solid_inertia(W.wheel(), T, C.wheel.mass_kg)
    side = {"left": dict(axle=axle, com=com, I=I, wheel_I=wheel_I),
            "right": dict(axle=MIRROR @ axle, com=MIRROR @ com, I=MIRROR @ I @ MIRROR, wheel_I=MIRROR @ wheel_I @ MIRROR)}
    skate = tessellated(part, out / "skate_left.stl").apply_transform(T).apply_scale(1e-3)
    wheel = tessellated(W.wheel(), out / "wheel_left.stl").apply_transform(T).apply_scale(1e-3).apply_translation(-centre)
    flip = np.eye(4)
    flip[:3, :3] = MIRROR
    for name, m in (("skate", skate), ("wheel", wheel)):
        m.export(out / f"{name}_left.stl")
        m.copy().apply_transform(flip).export(out / f"{name}_right.stl")  # trimesh keeps the faces outward
    calf = trimesh.load(HERE.parent / "vendor/calf_upper_mm.stl").apply_scale(1e-3)  # the calf cut off above the foot, calf frame
    calf.export(out / "calf_upper.stl")
    calf.copy().apply_transform(flip).export(out / "calf_upper_mirror.stl")  # the right legs' (Menagerie mirrors its own the same way)

    tree = ET.parse(MENAGERIE / "go2.xml")
    root = tree.getroot()
    root.set("model", "go2skate")
    root.find("compiler").set("meshdir", ".")
    asset = root.find("asset")
    for m in asset.findall("mesh"):  # Menagerie's own meshes, where they are vendored
        m.set("file", os.path.relpath(MENAGERIE / "assets" / m.get("file"), OUT))
    for name in ("skate_left", "skate_right", "wheel_left", "wheel_right", "calf_upper", "calf_upper_mirror"):
        ET.SubElement(asset, "mesh", name=name, file=f"meshes/{name}.stl")
    ET.SubElement(asset, "material", name="petg", rgba="0.95 0.55 0.15 1")
    ET.SubElement(asset, "material", name="urethane", rgba="0.15 0.15 0.18 1")

    for leg in LEGS:
        s = side[SIDE[leg]]
        calf = root.find(f".//body[@name='{leg}_calf']")
        for g in list(calf.findall("geom")):
            m = g.get("mesh")
            if m in ("foot", "calf_1", "calf_mirror_1") or g.get("name") == leg:  # the foot, its sole
                calf.remove(g)
            elif m in ("calf_0", "calf_mirror_0"):  # the calf down through the cup: ours stops above it
                g.set("mesh", "calf_upper" if m == "calf_0" else "calf_upper_mirror")
        sk = ET.SubElement(calf, "body", name=f"{leg}_skate")
        ET.SubElement(sk, "inertial", **mjcf_inertial(s["com"], s["I"], skate_mass))
        ET.SubElement(sk, "geom", mesh=f"skate_{SIDE[leg]}", material="petg", **{"class": "visual"})
        wb = ET.SubElement(sk, "body", name=f"{leg}_wheel", pos=vec(centre))
        ET.SubElement(wb, "inertial", **mjcf_inertial((0, 0, 0), s["wheel_I"], C.wheel.mass_kg))
        ET.SubElement(wb, "joint", name=f"{leg}_wheel_joint", type="hinge", axis=vec(s["axle"]), limited="false",
                      damping=f"{C.sim.bearing_damping_nms:g}", frictionloss=f"{C.sim.bearing_friction_nm:g}", armature="0")
        ET.SubElement(wb, "geom", mesh=f"wheel_{SIDE[leg]}", material="urethane", **{"class": "visual"})
        ET.SubElement(wb, "geom", name=leg, type="mesh", mesh=f"wheel_{SIDE[leg]}", group="3", condim="6", priority="1",
                      friction=vec(C.sim.contact_friction), solimp="0.9 0.95 0.001")

    key = root.find(".//key[@name='home']")
    q = key.get("qpos").split()
    legs = [q[7 + 3 * i:10 + 3 * i] + ["0"] for i in range(len(LEGS))]  # each wheel's angle follows its own leg's three joints
    key.set("qpos", " ".join(q[:7] + [v for leg in legs for v in leg]))
    ET.indent(tree)
    tree.write(OUT / "go2skate.xml")

    scene = ET.parse(MENAGERIE / "scene.xml")
    scene.getroot().set("model", "go2skate scene")
    scene.getroot().find("include").set("file", "go2skate.xml")
    scene.write(OUT / "scene.xml")

    rounded = lambda a: np.round(np.asarray(a, dtype=float), 8).tolist()
    spec = {
        "source": "generated by sim/gen_mjcf.py from config.py, foot.py and wheel_foot.py: edit those, not this",
        "frame": "Unitree calf frame (= Menagerie <leg>_calf body): origin at the knee, x forward, y left, z up the shank, foot down -z",
        "toe_out_deg": W.TOE,
        "wheel_centre_m": rounded(centre),
        "stock_foot_ball_centre_m": [-0.002, 0.0, -0.213],
        "axle_dir": {leg: rounded(side[SIDE[leg]]["axle"]) for leg in LEGS},
        "wheel_radius_m": W.WHEEL_D / 2000,
        "wheel_width_m": W.WHEEL_W / 1000,
        "tread_radius_m": W.TYRE_R / 1000,
        "tread": "its profile across is rounded at tread_radius (width/2: fully round, a torus-like tyre); upright, a sphere of "
                 "wheel_radius at wheel_centre touches the floor at the same point",
        "spinning": {"what": "the wheel with its bearings' outer races (inner races, spacer and axle are clamped to the bracket)",
                     "mass_kg": C.wheel.mass_kg,
                     "inertia_kgm2_about_centre": {leg: rounded(side[SIDE[leg]]["wheel_I"]) for leg in LEGS}},
        "bracket": {"what": "the printed wheel foot, fixed to the calf (add to the calf's body)", "mass_kg": round(skate_mass, 4),
                    "com_m": {leg: rounded(side[SIDE[leg]]["com"]) for leg in LEGS},
                    "inertia_kgm2_about_com": {leg: rounded(side[SIDE[leg]]["I"]) for leg in LEGS}},
        "meshes_visual": {"bracket": "meshes/skate_{left,right}.stl (calf frame, metres)",
                          "wheel": "meshes/wheel_{left,right}.stl (calf orientation, centred on the wheel)",
                          "left_legs": ["FL", "RL"], "right_legs": ["FR", "RR"]},
        "bearing_friction_Nm": C.sim.bearing_friction_nm,
        "bearing_damping_Nms": C.sim.bearing_damping_nms,
        "contact_friction_slide_torsion_rolling": list(C.sim.contact_friction),
        "notes": ["the stock foot's mass stays in Menagerie's calf inertial; the bracket's is added on top",
                  "bracket mass is solid PETG (an upper bound: infill makes it lighter)",
                  "wheel mass and friction are estimates (config.py); masses, centre and inertias come from the CAD"],
    }
    (OUT / "spec.json").write_text(json.dumps(spec, indent=2) + "\n")
    print(f"wheel centre in the calf frame {np.round(centre * 1000, 1)} mm (stock foot ball (-2, 0, -213)); toe out {W.TOE:g}°, "
          f"left axle {np.round(axle, 3)}; skate {skate_mass * 1000:.0f} g, wheel {C.wheel.mass_kg * 1000:.0f} g")


if __name__ == "__main__":
    build()
