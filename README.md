# go2skate

Inline-skate wheel feet for the Unitree Go2: the printed part, and the simulation model generated from it.

![The Go2 on its wheel feet in MuJoCo, standing in the home pose](assets/mujoco.png)

The Go2's feet come off: each is a grey cup held on the end of the calf by two M4 screws, with a rubber sole under it. A wheel foot
replaces one. It keeps the same cup at the top, so it bolts on the same way, and grows a fork under it that holds an inline-skate
wheel on its axle.

The CAD is the source of truth. Everything under `print/`, `models/` and `sim/` is generated from it.

## The pieces

- **`config.py`**: every number you might change. The wheel (diameter, width, tread, mass), the bearings, the axle (diameter,
  length), the toe-out angle, how the fork fits round the wheel (arm thickness, clearances), and the physics the simulation uses.
  Edit a default and rerun the scripts below.
- **`foot.py`**: a model of Unitree's **stock** foot, rebuilt from their mesh and our measurements. Its cup is the interface to the
  robot: the slot the calf's tongue drops into, and the two M4 holes across it. `wheel_foot.py` takes that cup from here. Run on its
  own, it exports the stock foot for reference and a short fit-test cup to print first, to check the slot fits your calf.
- **`wheel_foot.py`**: **the part you print**. It takes the stock foot's cup and lofts it down into a fork for the wheel, cut round
  the wheel's swept shape with clearance, with a boss on each arm against the bearings' inner races. It checks the arms come out the
  axle's length apart and that nothing touches the wheel or the tongue, then exports and shows it in the CAD viewer.
- **`sim/`**: the MuJoCo model built from the CAD (see [For the sim side](#for-the-sim-side)).

| Output                              |                                                                                                     |
|-------------------------------------|-----------------------------------------------------------------------------------------------------|
| `print/wheel_foot.stl`              | print four (or `wheel_foot_left.stl` and `_right.stl`, two of each, with toe-out)                   |
| `print/cup_fit_test.stl`            | the cup alone, to test the fit on the calf before printing the whole part                           |
| `models/`                           | STEP and STL exports: the wheel foot, the stock foot, the calf's tongue                             |
| `sim/go2skate.xml`, `sim/scene.xml` | the generated MJCF: Menagerie's Go2 with wheels for feet (load `scene.xml`)                         |
| `sim/spec.json`                     | the same numbers for whoever trains on it, without the MJCF                                         |
| `sim/meshes/`                       | visual meshes in metres: the bracket in the calf frame, the wheel centred on itself, left and right |

## Running it

[uv](https://docs.astral.sh/uv/) installs everything from `pyproject.toml` on the first run: build123d with its OCP kernel,
trimesh, ocp_vscode and MuJoCo.

```
uv run python foot.py            # the stock foot and the fit-test cup
uv run python wheel_foot.py      # the wheel foot: print/, models/, and the CAD viewer
uv run python sim/gen_mjcf.py    # the MuJoCo model, meshes and spec.json, from the CAD
uv run python sim/check.py       # loads it, sets the standing height so the wheels touch, checks it stands
uv run python sim/view.py        # MuJoCo's viewer: the robot standing on its wheels
```

After changing `config.py`, rerun `wheel_foot.py`, then `sim/gen_mjcf.py` and `sim/check.py`.

### Viewing the part (CAD)

`wheel_foot.py` shows the part in the [OCP CAD viewer](https://github.com/bernhard-42/vscode-ocp-cad-viewer) on port 3939, with
the wheel (see-through), its bearings, spacer and axle, and the calf's tongue in its slot. Start the viewer once, in its own terminal,
and open it in a browser:

```
uv run python -m ocp_vscode --port 3939      # then browse to http://localhost:3939/viewer
```

(or use the OCP CAD Viewer extension in VS Code). Then run `wheel_foot.py`, and the part appears.

### Viewing the robot (MuJoCo)

```
uv run python sim/view.py
```

opens MuJoCo's viewer with the robot holding its standing pose under a PD loop (kp 40, kd 1), like the low-level loop a policy would
drive. The wheels spin freely, so it rolls when pushed: double-click a body, then Ctrl and right-drag. In the viewer's left panel,
under *Rendering* and *Group enable*, group 3 shows the collision geoms.

`uv run python -m mujoco.viewer --mjcf sim/scene.xml` opens the bare model instead. Its motors are torque motors with nothing driving
them, so the robot just slumps.

## Configuring

All in `config.py`, as frozen dataclasses with defaults:

| | |
|---|---|
| `Wheel` | `diameter` 80, `width` 24.5, `tread_r` (None: fully round, half the width), `mass_kg` 0.090 |
| `Bearing` | `outer_d` 22, `width` 7 (a 608), `race_boss_d` 10.5 (the boss on the inner race, inside the shield) |
| `Axle` | `d` 8, `hole_d` 8.2, `length` 38 (under its heads), `play` 0.3 (the arms are `length + play` apart) |
| `Fork` | `arm_t` 6, `end_r` 11, `tyre_clear` 2, `side_clear` 2 (arms off the wheel's sides), `top_gap` 4, `cup_floor` 4 |
| `Sim` | bearing friction and damping, contact friction (slide, torsion, rolling), PETG density for the bracket's mass |
| `toe_out_deg` | 0: each wheel turned this far about the foot's own axis, its front away from the body's centre line |

With toe-out, the left and right feet are mirror images, so `wheel_foot.py` exports two of each. The toe turns about the foot's own
axis, which stands about 14° off vertical when the robot stands. That isn't the same as turning about the vertical: at 30°, the left
axle comes out (−0.426, 0.866, 0.261) in the calf frame. `spec.json` gives every leg's axle, whatever the angle.

## For the sim side

Frame: Unitree's calf frame, which is Menagerie's `<leg>_calf` body: origin at the knee, x forward, y left, z up the shank, foot down
−z. Names follow the usual pattern: body `<leg>_wheel` under the calf, the unactuated hinge `<leg>_wheel_joint`, and the wheel's
collision geom keeps the foot's name (`FL`, `FR`, `RL`, `RR`). The calf is drawn cut off above the stock foot.

`spec.json` is the contract: the wheel's centre and each leg's axle, its radius, width and tread, the spinning part's mass and inertia,
the bracket's mass, centre of mass and inertia, and the friction. Build your own collision primitive from it if you like: the
generated MJCF uses the wheel's convex mesh. The wheel mass and the friction are estimates. The centre, masses and inertias come from
the CAD.

## What to buy

For one robot (four wheel feet), with the defaults in `config.py`:

| Part | Spec | Qty |
|---|---|---|
| Inline-skate wheel | **80 mm** diameter, about **24 mm** wide (designed for 24.5), a round tread (the usual inline profile), urethane, 78A to 85A (softer grips better indoors) | 4 |
| Bearing | **608** (8 x 22 x 7 mm), shielded: the standard skate bearing, two per wheel | 8 |
| Bearing spacer | 8 mm bore, sits inside the wheel's hub between the two bearings; usually comes with the wheels or the bearings | 4 |
| Axle | 8 mm **two-piece inline-skate axle**, **38 mm** under the heads | 4 |
| Screw | M4 x 25 socket cap: two per foot, across through both ears and the calf's tongue | 8 |
| Heat-set insert | M4, 4 mm long (5.5 mm hole), pressed into the far ear | 8 |

Different wheel or axle? Measure it, set it in `config.py` and rerun. A wheel much bigger than 80 mm makes the legs effectively
longer: the wheel's bottom is already 48.5 mm under the stock foot's.

Assembly: press the bearings into the wheel, one flush with each side, with the spacer between. Drop the wheel into the fork and
push the axle through: arm, bearing, spacer, bearing, arm. The bosses on the arms bear on the bearings' inner races only. Tighten
the axle snug, not hard: the wheel should still spin freely.
