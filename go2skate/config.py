"""Everything about the skate you might change: the wheel, its bearings and axle, the toe angle, how the fork fits round them,
and the physics the simulation gives them. Change a default here and rerun `go2skate build`, or pass your own Skate(...) to
go2skate.build(), or override fields on the command line: `go2skate build --set toe_deg=20,-20,20,-20 --set wheel.diameter=72`.

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
    d: float = 6.0  # M6, through 8->6 mm inserts in the 608s (8: a standard skate axle); the arms' hole is this plus Skate.fit a side
    bolt: float = 50.0  # M6 x 50: its length under the head
    nut_washers: float = 8.75  # what the two washers and the nut take of it: it clamps at most bolt - nut_washers (41.25) of arm
    washer_d: float = 18.0  # each arm's outside is flat round the hole this wide, plus `seat_margin` all round; arms wider outside
    # to outside than the bolt clamps sink the washers into them, a pocket this wide
    seat_margin: float = 1.0


@dataclass(frozen=True)
class Screws:
    """The two M4s across the foot's ears and the calf's tongue: holes, head and nut pockets are these plus `fit` each side."""
    d: float = 4.0
    head_d: float = 7.0  # socket cap
    nut: bool = True  # a hex nut sunk in the far ear; False: a 4 mm heat-set insert there instead
    nut_af: float = 7.0  # M4 nut (ISO 4032): across flats, thick
    nut_h: float = 3.2
    fit: float = 0.15  # clearance a side, looser than Skate.fit: holes 4.3, head pocket 7.3, nut 7.3 across flats


@dataclass(frozen=True)
class Fork:
    """How the fork fits round the wheel."""
    arm_t: float = 6.0  # each arm's thickness at the axle, outside to inside face; its boss stands on out to the bearing
    race_gap: float = 0.9  # each arm's inside face off the wheel's side: its boss stands out this far onto the bearing's inner race
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
    wheel_collision: str = "sphere"  # "sphere": the wheel's radius at its centre, exact upright, up to ~4 mm off leaning 30°, the
    # fastest; "hull": a convex hull of the tread's round (hull_around x hull_across points), within 0.3 mm at any lean, but
    # its many near-floor points made contact ~3x slower here; "mesh": the CAD wheel's tessellation, exact, ~3x slower in impacts
    hull_around: int = 36  # 0.15 mm chord sag round an 80 mm wheel
    hull_across: int = 12  # 0.13 mm across the 12.25 tread round


@dataclass(frozen=True)
class Skate:
    wheel: Wheel = field(default_factory=Wheel)
    bearing: Bearing = field(default_factory=Bearing)
    axle: Axle = field(default_factory=Axle)
    screws: Screws = field(default_factory=Screws)
    fork: Fork = field(default_factory=Fork)
    sim: Sim = field(default_factory=Sim)
    fit: float = 0.15  # clearance a side in the axle holes: 6.3 for the M6 (the arms print with the hole on its side, so it closes
    # up). The M4s have Screws.fit, the calf tongue's slot foot.FIT
    toe_deg: tuple = (30.0, -30.0, 30.0, -30.0)  # FL, FR, RL, RR: each wheel turned about its foot's own axis (~14° off vertical
    # standing), + turns its front to the robot's left. V, every toe out like a skater's: (30, -30, 30, -30). X, each diagonal pair
    # on one line through the body's middle: (30, -30, -30, 30). All 0: straight
    label: bool = True  # engrave, under the axle: on the fork's outer arm the build's name over the leg (FL, FR, RL, RR) and an
    # arrow to the robot's front; on the other arm that foot's toe ("30° out")
    name: str = ""  # the build's name, engraved: "" names it from toe_deg (V30, X30, S0 straight, else T)
    label_size: float = 5.0  # the text's size (its capitals ~0.73 of it): two lines fit the flat under the axle's seat
    label_depth: float = 1.0
    split: bool = False  # print each foot in two halves, cut along the wheel's middle plane (no arm crosses it): each lies on its cut
    # face, its arm flat over the bed (support under the arm), layers along the arm. The M4s and the axle clamp the halves together


SKATE = Skate()
DATA = Path(__file__).parent / "data"  # the meshes the build needs: Unitree's foot and calf, Menagerie's Go2
OUT = Path.cwd()  # where builds write print/, models/ and sim/generated/: go2skate.build(out=...) sets it
