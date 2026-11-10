# go2skate

Inline-skate wheel feet for the Unitree Go2: the printed part, and the simulation model generated from it.

<p>
  <img src="assets/model.png" width="49%" alt="The front-left wheel foot in the CAD viewer, with its wheel, bearings and axle">
  <img src="assets/mujoco.png" width="49%" alt="The Go2 on its wheel feet in MuJoCo, standing in the home pose">
</p>

## Generate models

### Configure

edit `config.py` for wheel config (diameter, width, tread, mass), the bearings, the axle (diameter, length), and the physics the simulation uses. Edit a default and rerun the scripts below.

### Generate models for printing

to view parts you can optionally run `uv run python -m ocp_viewer --port 3939      # then browse to http://localhost:3939/viewer`

`uv run python wheel_foot.py` generates the four feet, labelled FL, FR, RL, RR: stl in `print/` (print one each), step in `models/`.
`uv run python foot.py` generates the stock foot (`models/`) and a fit-test cup to print first (`print/cup_fit_test.stl`).

### Generate simulation for simulating

`uv run python sim/gen_mjcf.py` generates simulation assets

```
sim/generated/go2skate.xml
sim/generated/scene.xml
sim/generated/spec.json
sim/generated/meshes/
```

`uv run python sim/view.py` runs mujoco

## Using it from another project

The generated simulation is plain files, so a training repo doesn't need this project's Python (or its CAD kernel) to use
it. Add the repo as a git submodule pinned to a commit and load straight from `sim/generated/`:

```
git submodule add https://github.com/leshy/go2skate third_party/go2skate
```

- `third_party/go2skate/sim/generated/scene.xml`: the robot on its wheels, on a floor (`go2skate.xml` without the floor)
- `third_party/go2skate/sim/generated/spec.json`: the same geometry and physics as numbers, to build your own model from:
  each leg's wheel centre and axle in the calf frame, wheel size and tread, masses, inertias, friction
- `third_party/go2skate/sim/generated/meshes/`: the visual meshes the MJCF loads

Bump the submodule to pick up a new version. For other settings (toe angle, wheel size, ...), clone it, edit `config.py`, and
run `uv run python wheel_foot.py` and `uv run python sim/gen_mjcf.py`: the masses and inertias come from the CAD solid, so
regenerating needs the CAD dependencies, which `uv` installs.

## BOM

For one robot (four wheel feet), with the defaults in `config.py`:

| Part               | Spec                                                                             | Qty |
|--------------------|----------------------------------------------------------------------------------|-----|
| Inline-skate wheel | **80 mm** diameter, about **24 mm** wide (designed for 24.5)                     | 4   |
| Bearing            | **608** (8 x 22 x 7 mm), shielded: the standard skate bearing, two per wheel     | 8   |
| Bearing spacer     | 8 mm bore, sits inside the wheel's hub between the two bearings                  | 4   |
| Axle               | 8 mm **two-piece inline-skate axle**, **38 mm** under the heads, heads up to 15 mm| 4   |
| Screw              | M4 x 25 socket cap: two per foot, across through both ears and the calf's tongue | 8   |
| Heat-set insert    | M4, 4 mm long (5.5 mm hole), pressed into the far ear                            | 8   |

Different wheel or axle? Measure it, set it in `config.py` and rerun. A wheel much bigger than 80 mm makes the legs effectively
longer: the wheel's bottom is already 48.5 mm under the stock foot's.

