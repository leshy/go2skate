"""Everything about the skate you might change: the wheel, its bearings and axle, the toe angle, how the fork fits round them,
and the physics the simulation gives them. Change a default here and rerun `go2skate build`, or pass your own Skate(...) to
go2skate.build(), or override fields on the command line: `go2skate build --set toe_out_deg=20 --set wheel.diameter=72`.

Lengths in mm unless a name says otherwise.
"""
from dataclasses import dataclass, field
from pathlib import Path


@dataclass(frozen=True)
class Wheel:
    """An inline-skate wheel: 72 to 110 across, about 24 wide."""
    diameter: float = 80.0
    width: float = 24.5
    tread_r: float | None = None  # its tread's round across; None: fully round (half the width), as most inline wheels are
    mass_kg: float = 0.090  # with its bearings: an estimate until weighed

    @property
    def tread(self):
        return self.width / 2 if self.tread_r is None else self.tread_r


@dataclass(frozen=True)
class Bearing:
    """Two per wheel, pressed into its hub flush with its faces, a spacer between: a 608 unless the wheel takes something else."""
    outer_d: float = 22.0
    width: float = 7.0
    race_boss_d: float = 10.5  # the face of the boss on each arm that bears on the inner race only, inside the shield (~12 on a
    # 608); the boss widens at 45° back to the arm, so it prints without overhang


@dataclass(frozen=True)
class Axle:
    """A two-piece inline-skate axle (or a bolt) through both arms."""
    d: float = 8.0  # its hole in the arms is this plus Skate.fit each side
    length: float = 38.0  # under its heads, measured: the arms are made this plus `play` wide outside to outside
    play: float = 0.3  # so it clamps the stack before its halves bottom out
    head_d: float = 15.0  # its heads: each arm's outside is flat round the hole this wide, plus `seat_margin` all round
    seat_margin: float = 1.0


@dataclass(frozen=True)
class Screws:
    """The two M4s across the foot's ears and the calf's tongue: holes and head pockets are these plus Skate.fit each side."""
    d: float = 4.0
    head_d: float = 7.0  # socket cap


@dataclass(frozen=True)
class Fork:
    """How the fork fits round the wheel."""
    arm_t: float = 6.0  # each arm's thickness at the axle, outside to inside face; its boss stands on out to the bearing
    end_r: float = 20.0  # the arms' round end about the axle (radius: 40 across)
    end_chamfer: float = 1.5  # round the outside of that round end (the axle's head still seats on the flat inside it)
    tyre_clear: float = 2.0  # round the tread
    side_pocket: bool = False  # pocket each arm's inside face back round its boss, room to flex off the wheel's flat sides
    side_clear: float = 2.0  # with the pocket: how far the arm's face stands off the wheel's side there
    side_clear_r: float = 17.0  # with the pocket: how far from the axle it reaches (the wheel's hub), ramping out at 45° to the
    # arm's face beyond, so it doesn't eat into the arms' roots at the cup
    top_gap: float = 4.0  # the tread's top under the cup
    cup_floor: float = 4.0  # the cup kept this far under its slot


@dataclass(frozen=True)
class Sim:
    """What the MuJoCo model gives the wheels: estimates."""
    bearing_friction_nm: float = 0.001  # two shielded 608s, free-spinning
    bearing_damping_nms: float = 1e-4
    contact_friction: tuple = (0.9, 0.005, 0.0008)  # slide, torsion, rolling (m: rolling resistance ~0.02 times the radius)
    petg_density: float = 1.27  # g/cm³, solid: the print's infill makes it lighter
    wheel_collision: str = "sphere"  # "sphere": radius = the wheel's, at its centre, exact upright for the round tread and fast;
    # "mesh": the wheel's convex hull, exact as it leans too, but MuJoCo's mesh contacts slow the physics several times


@dataclass(frozen=True)
class Skate:
    wheel: Wheel = field(default_factory=Wheel)
    bearing: Bearing = field(default_factory=Bearing)
    axle: Axle = field(default_factory=Axle)
    screws: Screws = field(default_factory=Screws)
    fork: Fork = field(default_factory=Fork)
    sim: Sim = field(default_factory=Sim)
    fit: float = 0.025  # clearance a side for printed fits (axle and screw holes, head pockets): tuned on the H2S with top_mount's
    # fit_test.py. Not the heat-set inserts' holes (their maker's size) or the calf tongue's slot (foot.FIT, proven by a test print)
    toe_out_deg: float = 30.0  # each wheel turned about the foot's own axis (~14° off vertical standing), its front away from the
    # body's centre line, like a skater's V; non-zero makes left and right feet mirror images
    label: bool = True  # engrave each foot's leg (FL, FR, RL, RR) and an arrow to the robot's front on its fork's outer side
    label_size: float = 6.0  # the text's height
    label_depth: float = 0.6


SKATE = Skate()
DATA = Path(__file__).parent / "data"  # the meshes the build needs: Unitree's foot and calf, Menagerie's Go2
OUT = Path.cwd()  # where builds write print/, models/ and sim/generated/: go2skate.build(out=...) sets it
