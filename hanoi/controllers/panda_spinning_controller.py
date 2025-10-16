#!/usr/bin/env python3
import mujoco
import mujoco.viewer
import numpy as np
import time
import os

class PandaSpinningController:
    def __init__(self, model_path="../../models/robots/franka_emika_panda/panda.xml"):
        """Initialize the Panda robot controller."""
        # Check if model file exists
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Robot model not found at: {model_path}")
        
        try:
            # Load the MuJoCo model
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
        except Exception as e:
            raise RuntimeError(f"Failed to load robot model: {e}")
        
        # Get joint and actuator indices
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        
        try:
            self.joint_indices = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) 
                                 for name in self.joint_names}
            
            # Gripper actuator index (actuator8 controls the gripper)
            self.gripper_actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, "actuator8")
            
            # Actuator indices for arm joints
            self.actuator_indices = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"actuator{i+1}") 
                                    for i, name in enumerate(self.joint_names)}
        except Exception as e:
            raise RuntimeError(f"Failed to find robot joints/actuators: {e}")
        
        # Control parameters
        self.time = 0.0
        self.spinning_joints = ['joint1', 'joint7']  # Joints that will spin
        self.spin_speed = 2.0  # radians per second
        self.gripper_frequency = 0.5  # Hz (cycles per second)
        
        # Home position
        self.home_qpos = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
        
        # Reset to home position
        self.reset_to_home()
        
        print("Panda Spinning Controller initialized!")
        print("Spinning joints:", self.spinning_joints)
    
    def reset_to_home(self):
        """Reset the robot to home position."""
        for i, joint_name in enumerate(self.joint_names):
            joint_id = self.joint_indices[joint_name]
            self.data.qpos[joint_id] = self.home_qpos[i]
        
        # Set gripper to closed position
        self.data.ctrl[self.gripper_actuator_id] = 255  # Closed
        
        # Set arm joint controls to home position
        for i, joint_name in enumerate(self.joint_names):
            actuator_id = self.actuator_indices[joint_name]
            self.data.ctrl[actuator_id] = self.home_qpos[i]
        
        mujoco.mj_forward(self.model, self.data)

    def update_control(self, dt):
        """Update robot control commands."""
        self.time += dt
        
        # Control spinning joints
        for joint_name in self.spinning_joints:
            actuator_id = self.actuator_indices[joint_name]
            
            if joint_name == 'joint1':
                # Spin joint1 continuously
                self.data.ctrl[actuator_id] = self.spin_speed * self.time
                
            elif joint_name == 'joint7':
                # Spin joint7 in opposite direction
                self.data.ctrl[actuator_id] = -self.spin_speed * self.time
        
        # Control gripper - oscillate between open and closed
        gripper_phase = 2 * np.pi * self.gripper_frequency * self.time
        gripper_control = 127.5 + 127.5 * np.sin(gripper_phase)  # Oscillate between 0 and 255
        self.data.ctrl[self.gripper_actuator_id] = gripper_control
        
        # Keep other joints at their home positions (for stability)
        stable_joints = ['joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for i, joint_name in enumerate(stable_joints):
            actuator_id = self.actuator_indices[joint_name]
            home_index = self.joint_names.index(joint_name)
            self.data.ctrl[actuator_id] = self.home_qpos[home_index]


    def run_simulation(self):
        """Run the interactive simulation."""
        last_status_time = 0
        
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # Set camera position for better view
            viewer.cam.distance = 2.5
            viewer.cam.elevation = -30
            viewer.cam.azimuth = 45
            viewer.cam.lookat[:] = [0.0, 0.0, 0.5]
            
            print("MuJoCo Viewer launched!")
            print("The robot will start spinning joint1 and joint7, and opening/closing grippers")
            print("Close the viewer window to exit the simulation")
            
            while viewer.is_running():
                step_start = time.time()
                
                # Update control and step simulation
                self.update_control(self.model.opt.timestep)
                mujoco.mj_step(self.model, self.data)
                
                # Sync with real-time
                viewer.sync()
                
                # Sleep to maintain real-time execution
                time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)


def main():
    """Main function to run the simulation."""
    try:
        print("Attempting to load Panda robot...")
        controller = PandaSpinningController()
        print("Robot loaded successfully! Starting simulation...")
        controller.run_simulation()
    except FileNotFoundError as e:
        print(f"\n❌ Robot model not found: {e}")
        print("Please ensure the Panda robot model exists at the specified path.")
        return 1
    except RuntimeError as e:
        print(f"\n❌ Runtime error: {e}")
        print("The robot model may be incompatible or corrupted.")
        return 1
    except KeyboardInterrupt:
        print("\n✅ Simulation interrupted by user")
        return 0
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    main()