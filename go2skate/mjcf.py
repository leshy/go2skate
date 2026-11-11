"""The Go2 on skates for MuJoCo, built from the CAD: Menagerie's go2.xml with each foot swapped for our printed wheel foot and
an inline wheel on a free hinge. Every size, the toe angle and the physics come from the config; rebuild after changing it,
and the model follows the parts.

Each calf loses the stock foot: its visual mesh, the 22 mm collision sphere, the rubber sole (Menagerie's calf_1) and the grey
cup (the bottom of calf_0), drawn instead with our calf cut off at the cup (data/calf_upper_mm.stl, the ScanRuler cut). It gains:
  <leg>_skate   fixed to the calf: the printed wheel foot, visual, its own mass
  <leg>_wheel   on <leg>_wheel_joint, a hinge along the axle: the wheel, collision geom still named <leg> (contact code keyed on
                the foot's geom name keeps working), visual
Each leg gets its own foot, turned its own way (config toe_deg). The 'home' keyframe keeps Menagerie's joint angles; check.py
raises its base until the wheels just touch the floor.

Writes everything into <out>/sim/generated/ (don't edit it, rebuild): go2skate.xml, scene.xml (floor + the robot),
meshes/*.stl (metres, in the calf or wheel frame), spec.json (the same numbers for whoever trains on it).

    go2skate build      # builds the part and this; then go2skate check, go2skate view
"""
import json
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
from build123d import CenterOf, export_stl

from . import config
from . import foot as F
from . import wheel_foot as W
from .config import SKATE as C

MENAGERIE = config.DATA / "go2_menagerie/unitree_go2"
OUT = config.OUT / "sim/generated"
LEGS = ("FL", "FR", "RL", "RR")
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


def rot_z(deg):
    t = np.radians(deg)
    return np.array([[np.cos(t), -np.sin(t), 0], [np.sin(t), np.cos(t), 0], [0, 0, 1]])


def tread_hull(R):
    """The wheel's collision shape: the convex hull of its tread's round (a tube of TYRE_R round a circle of radius R_wheel -
    TYRE_R), hull_around x hull_across points over the round's outer half, metres, centred on the wheel; R turns the wheel's own
    frame (rolling in X-Z, axle along Y) into the calf's."""
    r_wheel, rr = W.WHEEL_D / 2000, W.TYRE_R / 1000
    a = W.WHEEL_W / 2000 - rr  # where the round's centre sits across, either side (0: fully round)
    pts = []
    for th in np.linspace(0, 2 * np.pi, C.sim.hull_around, endpoint=False):
        for ph in np.linspace(-np.pi / 2, np.pi / 2, C.sim.hull_across):  # outward over the tread, side to side
            rad = r_wheel - rr + rr * np.cos(ph)
            y = (a if ph >= 0 else -a) + rr * np.sin(ph)
            pts.append((rad * np.cos(th), y, rad * np.sin(th)))
    hull = trimesh.Trimesh(np.array(pts) @ R.T).convex_hull
    return hull


def build():
    out = OUT / "meshes"
    out.mkdir(parents=True, exist_ok=True)
    for old in out.glob("*.stl"):
        old.unlink()
    T = calf_from_foot()
    centre = (T @ [0, 0, W.AXLE_W, 1])[:3] / 1000.0  # on the foot's axis: the toe turns the wheel about it, so every leg shares it
    flip = np.eye(4)
    flip[:3, :3] = MIRROR
    side = {}
    for leg in LEGS:  # a left foot turned W.built(leg); a right leg's is its mirror
        a = W.built(leg)
        t = np.radians(a)
        part = W.wheel_foot(a)
        mass = part.volume / 1000 * C.sim.petg_density / 1000  # kg
        com, I = solid_inertia(part, T, mass)
        _, wheel_I = solid_inertia(W.wheel(a), T, C.wheel.mass_kg)
        s = dict(axle=T[:3, :3] @ [-np.sin(t), np.cos(t), 0.0], com=com, I=I, wheel_I=wheel_I, mass=mass)  # axle: +Y turned a
        meshes = {"skate": tessellated(part, out / f"skate_{leg}.stl").apply_transform(T).apply_scale(1e-3),
                  "wheel": tessellated(W.wheel(a), out / f"wheel_{leg}.stl").apply_transform(T).apply_scale(1e-3).apply_translation(-centre),
                  "tread": tread_hull(T[:3, :3] @ rot_z(a))}
        if leg[1] == "R":
            s = dict(axle=MIRROR @ s["axle"], com=MIRROR @ com, I=MIRROR @ I @ MIRROR, wheel_I=MIRROR @ wheel_I @ MIRROR, mass=mass)
            meshes = {k: m.apply_transform(flip) for k, m in meshes.items()}  # trimesh keeps the faces outward
        for k, m in meshes.items():
            m.export(out / f"{k}_{leg}.stl")
        side[leg] = s
    calf = trimesh.load(config.DATA / "calf_upper_mm.stl").apply_scale(1e-3)  # the calf cut off above the foot, calf frame
    calf.export(out / "calf_upper.stl")
    calf.copy().apply_transform(flip).export(out / "calf_upper_mirror.stl")  # the right legs' (Menagerie mirrors its own the same way)

    tree = ET.parse(MENAGERIE / "go2.xml")
    root = tree.getroot()
    root.set("model", "go2skate")
    root.find("compiler").set("meshdir", ".")
    asset = root.find("asset")
    for m in asset.findall("mesh"):  # Menagerie's own meshes, where they are vendored
        m.set("file", os.path.relpath(MENAGERIE / "assets" / m.get("file"), OUT))
    for name in [f"{k}_{leg}" for leg in LEGS for k in ("skate", "wheel", "tread")] + ["calf_upper", "calf_upper_mirror"]:
        ET.SubElement(asset, "mesh", name=name, file=f"meshes/{name}.stl")
    ET.SubElement(asset, "material", name="petg", rgba="0.95 0.55 0.15 1")
    ET.SubElement(asset, "material", name="urethane", rgba="0.15 0.15 0.18 1")

    for leg in LEGS:
        s = side[leg]
        calf = root.find(f".//body[@name='{leg}_calf']")
        for g in list(calf.findall("geom")):
            m = g.get("mesh")
            if m in ("foot", "calf_1", "calf_mirror_1") or g.get("name") == leg:  # the foot, its sole
                calf.remove(g)
            elif m in ("calf_0", "calf_mirror_0"):  # the calf down through the cup: ours stops above it
                g.set("mesh", "calf_upper" if m == "calf_0" else "calf_upper_mirror")
        sk = ET.SubElement(calf, "body", name=f"{leg}_skate")
        ET.SubElement(sk, "inertial", **mjcf_inertial(s["com"], s["I"], s["mass"]))
        ET.SubElement(sk, "geom", mesh=f"skate_{leg}", material="petg", **{"class": "visual"})
        wb = ET.SubElement(sk, "body", name=f"{leg}_wheel", pos=vec(centre))
        ET.SubElement(wb, "inertial", **mjcf_inertial((0, 0, 0), s["wheel_I"], C.wheel.mass_kg))
        ET.SubElement(wb, "joint", name=f"{leg}_wheel_joint", type="hinge", axis=vec(s["axle"]), limited="false",
                      damping=f"{C.sim.bearing_damping_nms:g}", frictionloss=f"{C.sim.bearing_friction_nm:g}", armature="0")
        ET.SubElement(wb, "geom", mesh=f"wheel_{leg}", material="urethane", **{"class": "visual"})
        shape = {"sphere": dict(type="sphere", size=f"{W.WHEEL_D / 2000:g}"), "hull": dict(type="mesh", mesh=f"tread_{leg}"),
                 "mesh": dict(type="mesh", mesh=f"wheel_{leg}")}[C.sim.wheel_collision]
        ET.SubElement(wb, "geom", name=leg, group="3", condim="6", priority="1", friction=vec(C.sim.contact_friction),
                      solimp="0.9 0.95 0.001", **shape)

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
        "source": "generated by go2skate (go2skate build) from its config: rebuild, don't edit",
        "frame": "Unitree calf frame (= Menagerie <leg>_calf body): origin at the knee, x forward, y left, z up the shank, foot down -z",
        "toe_deg": dict(W.TOE),
        "toe_deg_means": "each wheel turned about its foot's axis, + its front towards the robot's left (+y)",
        "wheel_centre_m": rounded(centre),
        "stock_foot_ball_centre_m": [-0.002, 0.0, -0.213],
        "axle_dir": {leg: rounded(side[leg]["axle"]) for leg in LEGS},
        "wheel_radius_m": W.WHEEL_D / 2000,
        "wheel_width_m": W.WHEEL_W / 1000,
        "tread_radius_m": W.TYRE_R / 1000,
        "tread": "its profile across is rounded at tread_radius (width/2: fully round, a torus-like tyre); upright, a sphere of "
                 "wheel_radius at wheel_centre touches the floor at the same point",
        "spinning": {"what": "the wheel with its bearings' outer races (inner races, spacer and axle are clamped to the bracket)",
                     "mass_kg": C.wheel.mass_kg,
                     "inertia_kgm2_about_centre": {leg: rounded(side[leg]["wheel_I"]) for leg in LEGS}},
        "bracket": {"what": "the printed wheel foot, fixed to the calf (add to the calf's body)", "mass_kg": {leg: round(side[leg]["mass"], 4) for leg in LEGS},
                    "com_m": {leg: rounded(side[leg]["com"]) for leg in LEGS},
                    "inertia_kgm2_about_com": {leg: rounded(side[leg]["I"]) for leg in LEGS}},
        "meshes_visual": {"bracket": "meshes/skate_<leg>.stl (calf frame, metres)",
                          "wheel": "meshes/wheel_<leg>.stl (calf orientation, centred on the wheel)"},
        "collision": C.sim.wheel_collision,
        "bearing_friction_Nm": C.sim.bearing_friction_nm,
        "bearing_damping_Nms": C.sim.bearing_damping_nms,
        "contact_friction_slide_torsion_rolling": list(C.sim.contact_friction),
        "notes": ["the stock foot's mass stays in Menagerie's calf inertial; the bracket's is added on top",
                  "bracket mass is solid PETG (an upper bound: infill makes it lighter)",
                  "wheel mass and friction are estimates (config.py); masses, centre and inertias come from the CAD"],
    }
    (OUT / "spec.json").write_text(json.dumps(spec, indent=2) + "\n")
    print(f"wheel centre in the calf frame {np.round(centre * 1000, 1)} mm (stock foot ball (-2, 0, -213)); toe "
          + ", ".join(f"{leg} {W.TOE[leg]:+g}°" for leg in LEGS)
          + f"; skate {side['FL']['mass'] * 1000:.0f} g, wheel {C.wheel.mass_kg * 1000:.0f} g")


