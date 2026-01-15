import mujoco
from mujoco import viewer  # viewer is part of the mujoco package
# import gym

# Load your model (ensure the path to your MJCF/XML model is correct)
XML_Model = model = mujoco.MjModel.from_xml_path("models/franka_fr3/scene.xml")
# model = mujoco.MjModel.from_xml_path("models/boston_dynamics_spot/spot.xml")

data = mujoco.MjData(model)

# Launch an interactive viewer instead of a passive one
vis = viewer.launch(model, data)

while vis.is_running():
    mujoco.mj_step(model, data)
    vis.render()

vis.close()
