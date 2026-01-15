#!/usr/bin/env python3
"""
Quick Reinforcement Learning Training Example

This script shows how to quickly train an RL agent on a MuJoCo environment
and visualize the results.
"""

import gymnasium as gym
import numpy as np
import time
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback

def quick_rl_demo():
    """Quick RL training demonstration."""
    print("🚀 Quick RL Training Demo")
    print("=" * 50)
    
    # Choose environment
    env_name = "InvertedPendulum-v4"  # Simple, trains quickly
    print(f"Environment: {env_name}")
    
    # Create vectorized environment for faster training
    vec_env = make_vec_env(env_name, n_envs=8)
    
    # Create evaluation environment
    eval_env = gym.make(env_name)
    
    # Setup evaluation callback
    eval_callback = EvalCallback(
        eval_env, 
        best_model_save_path='./demos/ml_examples/',
        log_path='./demos/ml_examples/',
        eval_freq=2000,
        deterministic=True,
        render=False
    )
    
    # Create PPO agent
    print("Creating PPO agent...")
    model = PPO(
        "MlpPolicy", 
        vec_env,
        learning_rate=3e-4,
        n_steps=1024,
        batch_size=64,
        n_epochs=4,
        verbose=1
    )
    
    # Train
    print("Training agent (this may take a few minutes)...")
    start_time = time.time()
    model.learn(total_timesteps=50000, callback=eval_callback)
    training_time = time.time() - start_time
    
    print(f"Training completed in {training_time:.1f} seconds")
    
    # Test trained agent
    print("\\nTesting trained agent...")
    obs, info = eval_env.reset()
    total_reward = 0
    steps = 0
    
    for _ in range(1000):
        action, _states = model.predict(obs, deterministic=True)
        obs, reward, terminated, truncated, info = eval_env.step(action)
        total_reward += reward
        steps += 1
        
        if terminated or truncated:
            print(f"Episode finished: {steps} steps, reward: {total_reward:.2f}")
            break
    
    # Save the final model
    model.save("demos/ml_examples/quick_rl_model")
    print("Model saved to demos/ml_examples/quick_rl_model")
    
    # Cleanup
    vec_env.close()
    eval_env.close()
    
    return total_reward > 900  # Success threshold

if __name__ == "__main__":
    success = quick_rl_demo()
    print(f"\\n{'✅ SUCCESS' if success else '⚠️ NEEDS MORE TRAINING'}")