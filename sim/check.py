"""Load the generated model, put it in 'home' with the base raised until the wheels just touch the floor, write that height
back into go2skate.xml's keyframe, and hold it there for a second under PD to see it stands on its wheels.

    uv run python sim/check.py
"""
import re
from pathlib import Path

import mujoco
import numpy as np

HERE = Path(__file__).parent
LEGS = ("FL", "FR", "RL", "RR")


def lowest(model, data):
    """The lowest point of each wheel's collision mesh, world z."""
    out = {}
    for leg in LEGS:
        g = model.geom(leg).id
        m = model.geom_dataid[g]
        v = model.mesh_vert[model.mesh_vertadr[m]:model.mesh_vertadr[m] + model.mesh_vertnum[m]]
        world = data.geom_xpos[g] + v @ data.geom_xmat[g].reshape(3, 3).T
        out[leg] = world[:, 2].min()
    return out


model = mujoco.MjModel.from_xml_path(str(HERE / "generated/scene.xml"))
data = mujoco.MjData(model)
k = model.key("home").id
mujoco.mj_resetDataKeyframe(model, data, k)
mujoco.mj_forward(model, data)
low = lowest(model, data)
z = data.qpos[2] - min(low.values())
print("wheels' lowest points at the keyframe's base height:", {k_: round(v, 4) for k_, v in low.items()})
print(f"base height for the wheels to touch: {z:.4f} m (stock feet: 0.27)")

xml = HERE / "generated/go2skate.xml"
text = xml.read_text()
text = re.sub(r'(<key name="home" qpos=")(\S+) (\S+) (\S+)', lambda m: f"{m.group(1)}{m.group(2)} {m.group(3)} {z:.4f}", text, count=1)
xml.write_text(text)

model = mujoco.MjModel.from_xml_path(str(HERE / "generated/scene.xml"))
data = mujoco.MjData(model)
mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
KP, KD = 40.0, 1.0  # Menagerie's actuators are torque motors: hold 'home' with PD, as Go2 policies' low-level loop does
home = model.key("home").qpos.copy()
qadr = [model.jnt_qposadr[model.actuator_trnid[i, 0]] for i in range(model.nu)]
vadr = [model.jnt_dofadr[model.actuator_trnid[i, 0]] for i in range(model.nu)]
for _ in range(int(1.0 / model.opt.timestep)):
    data.ctrl[:] = KP * (home[qadr] - data.qpos[qadr]) - KD * data.qvel[vadr]
    mujoco.mj_step(model, data)
print(f"after 1 s holding 'home': base at z {data.qpos[2]:.4f} m, drifted {np.linalg.norm(data.qpos[:2]):.4f} m; "
      f"{model.nbody} bodies, {model.njnt} joints, mass {sum(model.body_mass):.2f} kg")
