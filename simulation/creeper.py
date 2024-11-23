import os

import mujoco
import mujoco.viewer


def move():
    model = mujoco.MjModel.from_xml_path(
        f"{os.path.dirname(__file__)}/../data/Creeper/untitled.xml"
    )
    data = mujoco.MjData(model)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(model, data)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()


if __name__ == "__main__":
    move()
