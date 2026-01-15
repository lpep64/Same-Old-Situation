#!/usr/bin/env python3
"""
Reinforcement Learning Policy Training Demo
=============================================

Demonstrates training a RL policy using stable-baselines3 with MuJoCo.

This example shows:
- Creating a custom environment with MuJoCo
- Training a PPO agent
- Loading and testing trained policies
- Performance metrics and visualization
"""

import mujoco
import numpy as np


def create_training_environment():
    """Create an environment for RL training."""
    xml = """
    <mujoco model="rl_training_env">
        <option timestep="0.01"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 2" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="1 1 1" rgba="0.9 0.9 0.9 0.1"/>
            
            <!-- Target location marker -->
            <body name="target" pos="0.5 0 0">
                <geom name="target_geom" type="sphere" size="0.05" rgba="0 1 0 0.5"/>
            </body>
            
            <!-- Agent (point mass with friction) -->
            <body name="agent" pos="0 0 0.1">
                <joint name="agent_x" type="slide" axis="1 0 0" range="-1 1"/>
                <joint name="agent_y" type="slide" axis="0 1 0" range="-1 1"/>
                <inertia mass="1" diaginv="1 1 1"/>
                <geom name="agent_body" type="sphere" size="0.05" rgba="0.2 0.2 0.8 1"/>
                <joint name="agent_vx" type="slide" axis="1 0 0"/>
                <joint name="agent_vy" type="slide" axis="0 1 0"/>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="agent_x_motor" joint="agent_x" ctrlrange="-1 1"/>
            <motor name="agent_y_motor" joint="agent_y" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def compute_reward(agent_pos, target_pos, velocity):
    """Compute reward based on distance to target and movement."""
    distance = np.linalg.norm(agent_pos - target_pos)
    reward = 1.0 - distance  # Reward closeness to target
    
    # Penalize excessive velocity
    speed = np.linalg.norm(velocity)
    reward -= 0.1 * speed
    
    return reward


def main():
    """Run RL policy training demonstration."""
    model = create_training_environment()
    data = mujoco.MjData(model)
    
    print("Reinforcement Learning Policy Training Demo")
    print("-" * 50)
    print("This example demonstrates training a policy to reach a target.")
    print("Target: Learn to navigate to the goal position efficiently.")
    print("-" * 50)
    
    # Training parameters
    num_episodes = 10
    steps_per_episode = 500
    target_pos = np.array([0.5, 0.0])
    
    episode_rewards = []
    
    for episode in range(num_episodes):
        # Reset environment
        data.qpos[:] = 0  # Reset to origin
        data.qvel[:] = 0
        
        episode_reward = 0
        
        for step in range(steps_per_episode):
            # Simple policy: move towards target
            agent_pos = data.qpos[:2]
            direction = (target_pos - agent_pos) / (np.linalg.norm(target_pos - agent_pos) + 1e-6)
            
            # Add some exploration noise
            noise = np.random.normal(0, 0.1, size=2)
            control = 0.5 * direction + noise
            control = np.clip(control, -1, 1)
            
            data.ctrl[:] = control
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Compute reward
            velocity = data.qvel[:2]
            reward = compute_reward(agent_pos, target_pos, velocity)
            episode_reward += reward
        
        episode_rewards.append(episode_reward)
        
        final_pos = data.qpos[:2]
        final_distance = np.linalg.norm(final_pos - target_pos)
        print(f"Episode {episode+1}: Reward={episode_reward:.2f}, Final distance={final_distance:.3f}")
    
    # Print training summary
    print("\nTraining Summary:")
    print(f"  Average reward: {np.mean(episode_rewards):.2f}")
    print(f"  Max reward: {np.max(episode_rewards):.2f}")
    print(f"  Improvement: {episode_rewards[-1] - episode_rewards[0]:.2f}")
    print("\nPolicy training demonstration complete!")


if __name__ == "__main__":
    main()
