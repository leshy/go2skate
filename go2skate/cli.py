"""go2skate build | check | view

    go2skate build                                    # the defaults, into the current directory
    go2skate build --set toe_deg=20,-20,20,-20 --set wheel.diameter=72 --out assets/skate
    go2skate build --no-sim --show                    # just the parts, the FL foot in the OCP viewer
    go2skate check                                    # touch down, set the standing height, stand for a second
    go2skate view                                     # MuJoCo's viewer, the robot standing on its wheels

--set takes any field of config.Skate by its dotted name (wheel.width, axle.length, fork.end_r, toe_deg, fit, ...).
"""
import argparse
import dataclasses
from pathlib import Path

from . import config


def override(skate, assignment):
    """`skate` with one 'dotted.field=value' changed, the value parsed as the field's current type."""
    key, _, text = assignment.partition("=")
    *path, name = key.strip().split(".")
    chain = [skate]
    for p in path:
        chain.append(getattr(chain[-1], p))
    old = getattr(chain[-1], name)  # AttributeError names a field that doesn't exist
    if isinstance(old, bool):
        value = text.strip().lower() in ("1", "true", "yes", "on")
    elif isinstance(old, tuple):
        value = tuple(float(v) for v in text.split(","))
    elif old is None:
        value = None if text.strip().lower() == "none" else float(text)
    else:
        value = type(old)(text)
    new = dataclasses.replace(chain[-1], **{name: value})
    for parent, p in zip(reversed(chain[:-1]), reversed(path)):
        new = dataclasses.replace(parent, **{p: new})
    return new


def main(argv=None):
    ap = argparse.ArgumentParser(prog="go2skate", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="the parts and the MuJoCo model")
    b.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE", help="override a config field (repeatable)")
    b.add_argument("--out", type=Path, default=Path.cwd(), help="write print/, models/ and sim/generated/ here (default: .)")
    b.add_argument("--no-cad", action="store_true", help="skip the printed parts")
    b.add_argument("--no-sim", action="store_true", help="skip the MuJoCo model")
    b.add_argument("--stock", action="store_true", help="also the stock foot and the fit-test cup")
    b.add_argument("--show", action="store_true", help="put the FL foot in the OCP viewer (port 3939)")
    for name, help_ in (("check", "touch down, set the standing height, stand for a second"), ("view", "MuJoCo's viewer")):
        p = sub.add_parser(name, help=help_)
        p.add_argument("--out", type=Path, default=Path.cwd(), help="where `build` wrote sim/generated/ (default: .)")
    a = ap.parse_args(argv)

    if a.cmd == "build":
        from . import build
        skate = config.SKATE
        for s in a.set:
            skate = override(skate, s)
        print("wrote", build(skate, out=a.out, cad=not a.no_cad, sim=not a.no_sim, stock=a.stock, show=a.show))
        return
    config.OUT = a.out.resolve()
    if a.cmd == "check":
        from .check import run
    else:
        from .view import run
    run()


if __name__ == "__main__":
    main()
