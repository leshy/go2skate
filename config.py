"""Everything about the skate you might change: the wheel, its bearings and axle, the toe angle, how the fork fits round them,
and the physics the simulation gives them. Change a default here, then rerun:

    uv run python wheel_foot.py      # the part: print/, models/
    uv run python sim/gen_mjcf.py    # the MuJoCo model and spec.json

Lengths in mm unless a name says otherwise.
"""
from dataclasses import dataclass, field


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
    race_boss_d: float = 10.5  # the boss on each arm that bears on the inner race only, inside the shield (~12 on a 608)


@dataclass(frozen=True)
class Axle:
    """A two-piece inline-skate axle (or a bolt) through both arms."""
    d: float = 8.0
    hole_d: float = 8.2  # its hole in the arms
    length: float = 38.0  # under its heads, measured: the arms are made this plus `play` wide outside to outside
    play: float = 0.3  # so it clamps the stack before its halves bottom out


@dataclass(frozen=True)
class Fork:
    """How the fork fits round the wheel."""
    arm_t: float = 6.0  # each arm's thickness at the axle, its outside to its boss's face
    end_r: float = 11.0  # the arms' round end about the axle
    tyre_clear: float = 2.0  # round the tread
    side_clear: float = 2.0  # each arm's inside face, round its boss, off the wheel's side: room to flex
    top_gap: float = 4.0  # the tread's top under the cup
    cup_floor: float = 4.0  # the cup kept this far under its slot


@dataclass(frozen=True)
class Sim:
    """What the MuJoCo model gives the wheels: estimates."""
    bearing_friction_nm: float = 0.001  # two shielded 608s, free-spinning
    bearing_damping_nms: float = 1e-4
    contact_friction: tuple = (0.9, 0.005, 0.0008)  # slide, torsion, rolling (m: rolling resistance ~0.02 times the radius)
    petg_density: float = 1.27  # g/cm³, solid: the print's infill makes it lighter


@dataclass(frozen=True)
class Skate:
    wheel: Wheel = field(default_factory=Wheel)
    bearing: Bearing = field(default_factory=Bearing)
    axle: Axle = field(default_factory=Axle)
    fork: Fork = field(default_factory=Fork)
    sim: Sim = field(default_factory=Sim)
    toe_out_deg: float = 0.0  # each wheel turned about the foot's own axis (~14° off vertical standing), its front away from the
    # body's centre line; non-zero makes left and right feet mirror images: two of each


SKATE = Skate()
