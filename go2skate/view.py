"""The skating Go2 in MuJoCo's viewer, holding 'home' with PD so it stands on its wheels. Double-click a body, then Ctrl +
right-drag to push it around.

    go2skate view
"""
import time
from pathlib import Path

import mujoco
import mujoco.viewer

from . import config

KP, KD = 40.0, 1.0  # Menagerie's actuators are torque motors: this is the low-level loop a policy would sit on


def run():
    """MuJoCo's viewer, the robot holding 'home' under PD."""
    model = mujoco.MjModel.from_xml_path(str(config.OUT / "sim/generated/scene.xml"))
    data = mujoco.MjData(model)
    key = model.key("home").id
    mujoco.mj_resetDataKeyframe(model, data, key)
    home = model.key("home").qpos.copy()
    qadr = [model.jnt_qposadr[model.actuator_trnid[i, 0]] for i in range(model.nu)]
    vadr = [model.jnt_dofadr[model.actuator_trnid[i, 0]] for i in range(model.nu)]

    with mujoco.viewer.launch_passive(model, data) as v:
        while v.is_running():
            t = time.time()
            data.ctrl[:] = KP * (home[qadr] - data.qpos[qadr]) - KD * data.qvel[vadr]
            mujoco.mj_step(model, data)
            v.sync()
            time.sleep(max(0.0, model.opt.timestep - (time.time() - t)))
