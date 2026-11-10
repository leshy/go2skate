# go2skate

Inline-skate wheel feet for the Unitree Go2: the printed part, and the simulation model generated from it.

![The Go2 on its wheel feet in MuJoCo, standing in the home pose](assets/mujoco.png)

The CAD is the source of truth. Everything under `print/`, `models/` and `sim/` is generated from it, so edit the CAD and rerun.

|                                     |                                                                                                                              |
|-------------------------------------|------------------------------------------------------------------------------------------------------------------------------|
| `foot.py`                           | Go2's detachable foot end (the cup the calf's tongue drops into), from Unitree's mesh and measurements                       |
| `wheel_foot.py`                     | the wheel foot: the foot's cup over a fork for an 80 x 24.5 mm inline wheel on two 608s and a 38 mm two-piece skate axle     |
| `print/`                            | `wheel_foot.stl` (print four), `cup_fit_test.stl` (the foot's cup over the calf's tongue, to check the fit)                  |
| `models/`                           | CAD exports: `wheel_foot.step`, the stock foot (`foot.step`/`.stl`), the calf's `tongue.stl`                                 |
| `sim/gen_mjcf.py`                   | builds the MuJoCo model from the CAD: Menagerie's Go2 with each foot swapped for the wheel foot and a free wheel             |
| `sim/spec.json`                     | **the contract** for whoever trains on it: wheel centre and axle in the calf frame, sizes, tread, masses, inertias, friction |
| `sim/go2skate.xml`, `sim/scene.xml` | the generated MJCF, a reference build (load `scene.xml`)                                                                     |
| `sim/meshes/`                       | visual meshes, metres: the bracket in the calf frame, the wheel in its own                                                   |
| `sim/view.py`                       | MuJoCo's viewer, the robot holding `home` under PD on its wheels                                                             |
| `sim/check.py`                      | loads it, sets the `home` keyframe's base height so the wheels touch, holds `home` under PD for a second                     |
| `sim/vendor/go2_menagerie/`         | MuJoCo Menagerie's `unitree_go2`, byte-identical (see its `PROVENANCE.md`, BSD-3)                                            |

```
uv run python foot.py            # models/foot.*, models/tongue.stl, print/cup_fit_test.stl
uv run python wheel_foot.py      # print/wheel_foot.stl, models/wheel_foot.step, the OCP viewer
uv run python sim/gen_mjcf.py    # go2skate.xml, scene.xml, meshes/, spec.json
uv run python sim/check.py       # sanity: loads, touches down, stands
uv run python sim/view.py        # MuJoCo's viewer, holding 'home' on its wheels
```

`uv run` installs everything from `pyproject.toml` on the first run (build123d with its OCP kernel, trimesh, MuJoCo).

## For the sim side

Frame: Unitree's calf frame, which is Menagerie's `<leg>_calf` body: origin at the knee, x forward, y left, z up the shank, foot down -z.
All four legs are the same, with no toe-out. Names follow the usual pattern: body `<leg>_wheel` under the calf, the unactuated hinge
`<leg>_wheel_joint`, and the wheel's collision geom keeps the foot's name (`FL`, `FR`, `RL`, `RR`).

Read `spec.json` and build your own collision primitive from it; the generated MJCF uses the wheel's convex mesh. The wheel mass and
the friction values are estimates. The masses, centre and inertias come from the CAD.

## What to buy

For one robot (four wheel feet):

| Part               | Spec                                                                                                                                                       | Qty |
|--------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------|-----|
| Inline-skate wheel | **80 mm** diameter, about **24 mm** wide (designed for 24.5), a round tread (the usual inline profile), urethane, 78A to 85A (softer grips better indoors) | 4   |
| Bearing            | **608** (8 x 22 x 7 mm), shielded: the standard skate bearing, two per wheel                                                                               | 8   |
| Bearing spacer     | 8 mm bore, sits inside the wheel's hub between the two bearings; usually comes with the wheels or the bearings                                             | 4   |
| Axle               | 8 mm **two-piece inline-skate axle**, **38 mm** under the heads                                                                                            | 4   |
| Screw              | M4 x 25 socket cap: two per foot, across through both ears and the calf's tongue                                                                           | 8   |
| Heat-set insert    | M4, 4 mm long (5.5 mm hole), pressed into the far ear                                                                                                      | 8   |

The wheel's size is set in `wheel_foot.py` (`WHEEL_D`, `WHEEL_W`, `TYRE_R`), and the axle's length in `AXLE_L`. If yours differ, measure and
change them there, then rerun `wheel_foot.py` and `sim/gen_mjcf.py`. The arms always come out 0.3 mm wider than the axle, so it clamps.
A wheel much bigger than 80 mm makes the legs effectively longer (the wheel's bottom is already 48.5 mm under the stock foot's).

Assembly: press the bearings into the wheel, one flush with each side, with the spacer between. Drop the wheel into the fork and
push the axle through: arm, bearing, spacer, bearing, arm. The bosses on the arms bear on the bearings' inner races only. Tighten
the axle snug, not hard: the wheel should still spin freely.
