#!/usr/bin/env python3
"""Niryo One Robotic Arm Demo"""
import mujoco
import numpy as np

xml = """<mujoco model="niryo"><option timestep="0.001"/><worldbody>
<light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
<geom name="ground" type="plane" size="1 1 1" rgba="0.9 0.9 0.9 0.1"/>
<body name="base" pos="0 0 0" mass="1"><geom type="cylinder" size="0.08 0.05" rgba="0.3 0.3 0.3 1"/>
<body name="arm1" pos="0 0 0.15" mass="0.4"><joint name="j1" type="hinge" axis="0 0 1"/><geom type="capsule" size="0.04 0.15" rgba="0.5 0.5 0.5 1"/>
<body name="arm2" pos="0 0 0.15" mass="0.3"><joint name="j2" type="hinge" axis="0 1 0"/><geom type="capsule" size="0.035 0.15" rgba="0.6 0.6 0.6 1"/>
<body name="ee" pos="0 0 0.15" mass="0.2"><geom type="sphere" size="0.03" rgba="0.2 0.8 0.2 1"/></body></body></body></body></worldbody>
<actuator><motor name="m1" joint="j1" ctrlrange="-1 1"/><motor name="m2" joint="j2" ctrlrange="-1 1"/></actuator></mujoco>"""

def main():
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    print("Niryo One Robotic Arm Demo\n" + "-"*50)
    for step in range(200):
        t = step * 0.01
        data.ctrl[0] = 0.5 * np.sin(t)
        data.ctrl[1] = 0.3 * np.sin(t + np.pi/2)
        mujoco.mj_step(model, data)
        if step % 50 == 0:
            print(f"Step {step}: angles=[{data.qpos[0]:.3f}, {data.qpos[1]:.3f}]")
    print("Demo complete.")

if __name__ == "__main__":
    main()

