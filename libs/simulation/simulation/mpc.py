import os
import pathlib

import mujoco

from mujoco_mpc import agent as agent_lib

mujoco_mpc_base_path = f"{os.path.dirname(__file__)}/../mujoco_mpc"

model_path = f"{mujoco_mpc_base_path}/build/mjpc/tasks/humanoid/walk/task.xml"
task_id = "Humanoid Walk"

# model_path = f"{mujoco_mpc_base_path}/build/mjpc/tasks/cartpole/task.xml"
# task_id = "Cartpole"

model = mujoco.MjModel.from_xml_path(str(model_path))

with agent_lib.Agent(
    server_binary_path=pathlib.Path(agent_lib.__file__).parent
    / "mjpc"
    / "ui_agent_server",
    task_id=task_id,
    model=model,
) as agent:
    while True:
        pass
