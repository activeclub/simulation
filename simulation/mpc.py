import os
import pathlib

import mujoco

from mujoco_mpc import agent as agent_lib

model_path = (
    f"{os.path.dirname(__file__)}/../mujoco_mpc/build/mjpc/tasks/humanoid/walk/task.xml"
)
model = mujoco.MjModel.from_xml_path(str(model_path))

with agent_lib.Agent(
    server_binary_path=pathlib.Path(agent_lib.__file__).parent
    / "mjpc"
    / "ui_agent_server",
    task_id="Humanoid Walk",
    model=model,
) as agent:
    while True:
        pass
