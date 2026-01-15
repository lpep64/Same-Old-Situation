import sys
import os
sys.path.append(os.path.dirname(__file__))

from mujoco_parser import MuJoCoParserClass
import numpy as np

xml_path = os.path.join(os.path.dirname(__file__), 'niryo_conveyor.xml')
env = MuJoCoParserClass(name='Test', rel_xml_path=xml_path, VERBOSE=False)

# Check initial configuration
env.reset()
env.forward()
print(f'Robot base position: {env.data.body("robot_base").xpos}')
print(f'End effector position (default): {env.data.body("hand_link").xpos}')
print(f'Joint positions (default): {env.data.qpos[env.rev_joint_qpos_idxs]}')

# Try the home keyframe
env.reset()
env.data.qpos[env.rev_joint_qpos_idxs] = np.array([0.0, -0.5, 1.0, 0.0, 0.0, 0.0])
env.forward()
print(f'\nEnd effector position (home keyframe): {env.data.body("hand_link").xpos}')
print(f'Joint positions (home): {env.data.qpos[env.rev_joint_qpos_idxs]}')

# Check conveyor block position
print(f'\nConveyor block position: {env.data.body("conveyor_block_blue").xpos}')
