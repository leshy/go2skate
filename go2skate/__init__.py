"""Inline-skate wheel feet for the Unitree Go2: the printed parts and the MuJoCo model, built from one config.

    import go2skate
    from go2skate import Skate, Wheel

    go2skate.build()                                              # the defaults, into the current directory
    go2skate.build(Skate(toe_out_deg=20, wheel=Wheel(diameter=72)), out="assets/skate")

writes <out>/print/ (STL to print), <out>/models/ (STEP) and <out>/sim/generated/ (go2skate.xml, scene.xml, meshes/,
spec.json). The command line does the same: `go2skate build --set toe_out_deg=20 --out assets/skate`.
"""
import importlib
from pathlib import Path

from . import config
from .config import SKATE, Axle, Bearing, Fork, Screws, Sim, Skate, Wheel

__all__ = ["build", "Skate", "Wheel", "Bearing", "Axle", "Screws", "Fork", "Sim", "SKATE"]


def build(skate=None, out=None, cad=True, sim=True, stock=False, show=False):
    """Build everything from `skate` (default: config.SKATE) into `out` (default: the current directory).

    cad: the four wheel feet, print/wheel_foot_{FL,FR,RL,RR}.stl and models/*.step
    sim: the MuJoCo model, sim/generated/
    stock: also the stock foot and the fit-test cup (models/foot.*, print/cup_fit_test.stl)
    show: put the FL foot in the OCP viewer on port 3939
    Returns the path written to.
    """
    # ponytail: the modules read config at import (their sizes are module constants), so a build swaps the config and
    # reloads them; pass the config through explicitly if two builds ever need to run side by side
    config.SKATE = skate or config.SKATE
    config.OUT = Path(out or Path.cwd()).resolve()
    from . import foot, mjcf, wheel_foot
    for m in (foot, wheel_foot, mjcf):
        importlib.reload(m)
    if stock:
        foot.export(show=False)
    if cad:
        wheel_foot.export(show=show)
    if sim:
        mjcf.build()
    return config.OUT
