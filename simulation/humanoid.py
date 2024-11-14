import os
import time

import mujoco
import mujoco.viewer

m = mujoco.MjModel.from_xml_path(f"{os.path.dirname(__file__)}/models/humanoid.xml")
# m = mujoco.MjModel.from_xml_path(f"{os.path.dirname(__file__)}/envs/assets/kxr_l2_humanoid.xml")
# m = mujoco.MjModel.from_xml_path(f"{os.path.dirname(__file__)}/../data/body_c_forupdf.urdf")
d = mujoco.MjData(m)

with mujoco.viewer.launch_passive(m, d) as viewer:
    while viewer.is_running():
        step_start = time.time()

        # mj_step can be replaced with code that also evaluates
        # a policy and applies a control signal before stepping the physics.
        mujoco.mj_step(m, d)

        # Example modification of a viewer option: toggle contact points every two seconds.
        with viewer.lock():
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(d.time % 2)

        # Pick up changes to the physics state, apply perturbations, update options from GUI.
        viewer.sync()

        # Rudimentary time keeping, will drift relative to wall clock.
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
