# go2skate

Inline-skate wheel feet for the Unitree Go2: the printed part, and the simulation model generated from it.

<p>
  <img src="assets/model.png" width="49%" alt="The front-left wheel foot in the CAD viewer, with its wheel, bearings and axle">
  <img src="assets/mujoco.png" width="49%" alt="The Go2 on its wheel feet in MuJoCo, standing in the home pose">
</p>

## Generate models

1. Edit `go2skate/config.py` to configure wheels, mass, bearings, axle etc

2. Run the viewer
`uv run python -m ocp_viewer --port 3939` browse to http://localhost:3939/viewer

3. Generate print/ (STL), models/ (STEP), sim/generated/ (MuJoCo)
`uv run go2skate build --show`

You can check the stock foot and a fit-test cup (print/cup_fit_test.stl)
`uv run go2skate build --stock` 

4. view in mujoco 
`uv run go2skate view`

## CLI

Change any setting for one build with `--set`, by its dotted name, and write somewhere else with `--out`:

```
uv run go2skate build --set toe_out_deg=20 --set wheel.diameter=72 --set fork.end_r=18 --out builds/72mm
uv run go2skate check --out builds/72mm
```

The four feet come out labelled FL, FR, RL, RR (print one each): left and right are mirror images once the wheels toe out.

## Sim files

The simulation, in `sim/generated/`:

|             |                                                                                                                                                |
|-------------|------------------------------------------------------------------------------------------------------------------------------------------------|
| `scene.xml` | the robot on its wheels, on a floor (`go2skate.xml` without the floor): Menagerie's Go2 with its feet swapped                                  |
| `spec.json` | the same geometry and physics as numbers: each leg's wheel centre and axle in the calf frame, wheel size and tread, masses, inertias, friction |
| `meshes/`   | the visual meshes the MJCF loads                                                                                                               |

Each wheel collides as a sphere of its radius at its centre (`sim.wheel_collision`), exact upright for the round tread, a few
mm off as it leans hard, and the fastest. `--set sim.wheel_collision=hull` uses a convex hull of the tread (within 0.3 mm at any
lean), `mesh` the CAD wheel itself; both cost more contact time. `spec.json` has the exact tread to build your own.

## Using it from another project

Add it as a dependency:

```
uv add git+https://github.com/leshy/go2skate
```

and build the skate you want into your own tree, from Python:

```python
import go2skate
from go2skate import Skate, Wheel

out = go2skate.build(Skate(toe_out_deg=20, wheel=Wheel(diameter=72)), out="assets/go2skate", cad=False)
# out / "sim/generated/scene.xml", out / "sim/generated/spec.json", ...
```

or from the command line: `go2skate build --set toe_out_deg=20 --out assets/go2skate --no-cad`. `cad=False` (`--no-cad`) skips
the printable parts

## BOM

For one robot (four wheel feet), with the default config:

| Part               | Spec                                                                             | Qty |
|--------------------|----------------------------------------------------------------------------------|-----|
| Inline-skate wheel | **80 mm** diameter, about **24 mm** wide (designed for 24.5)                     | 4   |
| Bearing            | **608** (8 x 22 x 7 mm), shielded: the standard skate bearing, two per wheel     | 8   |
| Bearing spacer     | 8 mm bore, sits inside the wheel's hub between the two bearings                  | 4   |
| Axle               | 8 mm **two-piece inline-skate axle**, **38 mm** under the heads, heads up to 15 mm| 4   |
| Screw              | M4 x 25 socket cap: two per foot, across through both ears and the calf's tongue | 8   |
| Heat-set insert    | M4, 4 mm long (5.5 mm hole), pressed into the far ear                            | 8   |

Different wheel or axle? Measure it, set it in the config (or `--set`) and rebuild. A wheel much bigger than 80 mm makes the legs effectively
longer: the wheel's bottom is already 48.5 mm under the stock foot's.

