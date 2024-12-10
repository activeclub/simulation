import os
import time

import mujoco
import mujoco.viewer
from huggingface_hub import hf_hub_download

m = mujoco.MjModel.from_xml_path(f"{os.path.dirname(__file__)}/models/humanoid.xml")
# m = mujoco.MjModel.from_xml_path(
#     f"{os.path.dirname(__file__)}/envs/assets/kxr_l2_humanoid.xml"
# )
# m = mujoco.MjModel.from_xml_path(
#     f"{os.path.dirname(__file__)}/../data/body_d_forurdf.urdf"
# )
d = mujoco.MjData(m)


def load_from_hub(org: str, repo: str, filename: str):
    """Load a model from the Hugging Face Hub.
    Example:
        load_from_hub("sb3", "sac-Humanoid-v3", "config.yml")
    """
    return hf_hub_download(
        repo_id=f"{org}/{repo}",
        filename=filename,
        library_name="huggingface-sb3",
        library_version="2.1",
    )


def run():
    pass


def view():
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # mj_step can be replaced with code that also evaluates a policy
            # and applies a control signal before stepping the physics.
            mujoco.mj_step(m, d)

            # Pick up changes to the physics state,
            # apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    view()
