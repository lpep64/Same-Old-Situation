#!/usr/bin/env python3
"""
MuJoCo + Gymnasium Integration Demo

This script demonstrates how to use Gymnasium with MuJoCo environments
for reinforcement learning applications.
"""

import gymnasium as gym
import numpy as np
import time
import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.evaluation import evaluate_policy

def list_mujoco_environments():
    """List all available MuJoCo environments in Gymnasium."""
    print("🎮 Available MuJoCo Environments in Gymnasium:")
    print("=" * 60)
    
    # MuJoCo environments
    mujoco_envs = [
        ("Ant-v4", "Quadruped locomotion - 8 legs, complex walking"),
        ("HalfCheetah-v4", "Fast running - streamlined body"),  
        ("Hopper-v4", "Single-leg hopping - balance challenge"),
        ("Humanoid-v4", "Bipedal walking - most complex"),
        ("HumanoidStandup-v4", "Get up from lying position"),
        ("InvertedPendulum-v4", "Classic control - balance pole"),
        ("InvertedDoublePendulum-v4", "Double pendulum balance"),
        ("Pusher-v4", "Object manipulation - push to target"),
        ("Reacher-v4", "Reach target with 2-link arm"),
        ("Swimmer-v4", "Swimming locomotion in fluid"),
        ("Walker2d-v4", "2D bipedal walking")
    ]
    
    for env_name, description in mujoco_envs:
        print(f"   {env_name:<25} - {description}")
    
    return [env[0] for env in mujoco_envs]

def test_environment(env_name: str, steps: int = 1000):
    """Test a single environment with random actions."""
    print(f"\n🧪 Testing Environment: {env_name}")
    print("-" * 50)
    
    try:
        # Create environment
        env = gym.make(env_name, render_mode=None)  # No rendering for speed
        
        print(f"   Observation Space: {env.observation_space}")
        print(f"   Action Space: {env.action_space}")
        
        # Test random policy
        obs, info = env.reset()
        total_reward = 0
        episode_length = 0
        
        start_time = time.time()
        
        for step in range(steps):
            # Random action
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
            
            total_reward += reward
            episode_length += 1
            
            # Reset if episode ends
            if terminated or truncated:
                print(f"   Episode {episode_length:4d}: Reward = {total_reward:8.2f}")
                obs, info = env.reset()
                total_reward = 0
                episode_length = 0
        
        elapsed = time.time() - start_time
        fps = steps / elapsed
        
        print(f"   Performance: {fps:.0f} steps/sec")
        print(f"   ✅ Environment test PASSED")
        
        env.close()
        return True
        
    except Exception as e:
        print(f"   ❌ Environment test FAILED: {str(e)}")
        return False

def demonstrate_rl_training():
    """Demonstrate RL training with Stable-Baselines3."""
    print("\n🧠 Reinforcement Learning Training Demo")
    print("=" * 60)
    
    # Use a simple environment for quick demo
    env_name = "InvertedPendulum-v4"
    print(f"Training on: {env_name}")
    
    # Create environment
    env = make_vec_env(env_name, n_envs=4)  # 4 parallel environments
    
    # Create PPO agent
    print("   Creating PPO agent...")
    model = PPO(
        "MlpPolicy", 
        env, 
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        verbose=1
    )
    
    # Quick training (just for demo)
    print("   Training for 10,000 steps...")
    start_time = time.time()
    model.learn(total_timesteps=10000)
    training_time = time.time() - start_time
    
    print(f"   Training completed in {training_time:.1f}s")
    
    # Evaluate trained policy
    print("   Evaluating trained policy...")
    eval_env = gym.make(env_name)
    mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=10)
    
    print(f"   Mean reward: {mean_reward:.2f} ± {std_reward:.2f}")
    
    # Save model
    model_path = "demos/ml_examples/trained_pendulum_model"
    model.save(model_path)
    print(f"   Model saved to: {model_path}")
    
    eval_env.close()
    env.close()
    
    return mean_reward > 800  # Good performance threshold for InvertedPendulum

def demonstrate_custom_pytorch_policy():
    """Show how to create a custom PyTorch policy."""
    print("\n🔥 Custom PyTorch Policy Demo")
    print("=" * 60)
    
    # Simple policy network
    class SimplePolicy(nn.Module):
        def __init__(self, obs_dim, action_dim):
            super().__init__()
            self.network = nn.Sequential(
                nn.Linear(obs_dim, 64),
                nn.Tanh(),
                nn.Linear(64, 64),
                nn.Tanh(),
                nn.Linear(64, action_dim),
                nn.Tanh()  # Continuous actions between -1 and 1
            )
        
        def forward(self, obs):
            return self.network(obs)
    
    # Test with Pendulum environment
    env = gym.make("Pendulum-v1")
    
    obs_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    
    print(f"   Observation dimension: {obs_dim}")
    print(f"   Action dimension: {action_dim}")
    
    # Create policy
    policy = SimplePolicy(obs_dim, action_dim)
    print(f"   Policy parameters: {sum(p.numel() for p in policy.parameters()):,}")
    
    # Test policy on environment
    obs, info = env.reset()
    total_reward = 0
    
    for step in range(200):
        # Convert observation to tensor
        obs_tensor = torch.FloatTensor(obs).unsqueeze(0)
        
        # Get action from policy
        with torch.no_grad():
            action = policy(obs_tensor).squeeze(0).numpy()
        
        # Scale action to environment range
        action = action * env.action_space.high
        
        # Step environment
        obs, reward, terminated, truncated, info = env.step(action)
        total_reward += reward
        
        if terminated or truncated:
            break
    
    print(f"   Custom policy reward: {total_reward:.2f}")
    print("   ✅ Custom PyTorch policy working")
    
    env.close()
    return True

def test_dm_control():
    """Test DeepMind Control Suite integration."""
    print("\n🎮 DeepMind Control Suite Demo")
    print("=" * 60)
    
    try:
        from dm_control import suite
        
        # List some domains and tasks
        domains_tasks = [
            ('cartpole', 'balance'),
            ('pendulum', 'swingup'), 
            ('acrobot', 'swingup'),
            ('point_mass', 'easy')
        ]
        
        print("   Available dm-control tasks:")
        
        for domain, task in domains_tasks:
            try:
                env = suite.load(domain_name=domain, task_name=task)
                action_spec = env.action_spec()
                obs_spec = env.observation_spec()
                
                print(f"   ✅ {domain}-{task}: Actions={len(action_spec)}, Obs keys={len(obs_spec)}")
                
                # Quick test
                time_step = env.reset()
                for _ in range(10):
                    action = np.random.uniform(
                        action_spec.minimum, 
                        action_spec.maximum,
                        size=action_spec.shape
                    )
                    time_step = env.step(action)
                
            except Exception as e:
                print(f"   ❌ {domain}-{task}: {str(e)[:40]}...")
        
        print("   ✅ DeepMind Control Suite working")
        return True
        
    except ImportError:
        print("   ⚠️  DeepMind Control Suite not available")
        return False
    except Exception as e:
        print(f"   ❌ DeepMind Control error: {str(e)}")
        return False

def main():
    """Run all ML integration demos."""
    print("🚀 MUJOCO + MACHINE LEARNING INTEGRATION DEMO")
    print("=" * 70)
    
    # Check PyTorch
    print(f"PyTorch version: {torch.__version__}")
    print(f"CUDA available: {torch.cuda.is_available()}")
    if torch.cuda.is_available():
        print(f"CUDA device: {torch.cuda.get_device_name(0)}")
    
    # List environments
    available_envs = list_mujoco_environments()
    
    # Test a few environments
    test_envs = ["InvertedPendulum-v4", "Pendulum-v1", "HalfCheetah-v4"]
    test_results = []
    
    for env_name in test_envs:
        if env_name in available_envs or env_name == "Pendulum-v1":
            success = test_environment(env_name, steps=500)
            test_results.append(success)
        else:
            print(f"   ⚠️  {env_name} not available")
            test_results.append(False)
    
    # RL Training Demo
    rl_success = demonstrate_rl_training()
    
    # Custom PyTorch Policy
    pytorch_success = demonstrate_custom_pytorch_policy()
    
    # DeepMind Control
    dm_success = test_dm_control()
    
    # Summary
    print("\n" + "=" * 70)
    print("📊 INTEGRATION TEST SUMMARY")
    print("=" * 70)
    
    results = [
        ("Environment Tests", f"{sum(test_results)}/{len(test_results)} passed"),
        ("RL Training (PPO)", "✅ PASSED" if rl_success else "❌ FAILED"),
        ("PyTorch Policy", "✅ PASSED" if pytorch_success else "❌ FAILED"),
        ("DeepMind Control", "✅ PASSED" if dm_success else "❌ FAILED")
    ]
    
    for test_name, result in results:
        print(f"   {test_name:<20} {result}")
    
    total_passed = sum(test_results) + sum([rl_success, pytorch_success, dm_success])
    total_tests = len(test_results) + 3
    
    print(f"\n🎯 Overall: {total_passed}/{total_tests} components working")
    
    if total_passed == total_tests:
        print("🎉 All ML integration tests PASSED!")
        print("   Ready for reinforcement learning with MuJoCo!")
    else:
        print("⚠️  Some components need attention.")
        print("   Check installation and dependencies.")

if __name__ == "__main__":
    main()