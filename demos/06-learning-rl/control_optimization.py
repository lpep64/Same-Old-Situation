#!/usr/bin/env python3
"""
Optimal Control Optimization Demo
==================================

Demonstrates optimal control problem solving using MuJoCo and optimization.

This example shows:
- Setting up optimal control problems
- Using trajectory optimization
- Cost function definition
- Control constraint satisfaction
"""

import mujoco
import numpy as np
from scipy.optimize import minimize


def create_control_env():
    """Create an environment for optimal control."""
    xml = """
    <mujoco model="optimal_control_env">
        <option timestep="0.01"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 2" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="1 1 1" rgba="0.9 0.9 0.9 0.1"/>
            
            <!-- Cart with attached pole -->
            <body name="cart" pos="0 0 0.1">
                <joint name="cart_x" type="slide" axis="1 0 0" range="-1 1"/>
                <inertia mass="1" diaginv="1 1 1"/>
                <geom name="cart" type="box" size="0.1 0.1 0.05" rgba="0.7 0.7 0.7 1"/>
                
                <!-- Pole attached to cart -->
                <body name="pole" pos="0 0 0.05">
                    <joint name="pole_angle" type="hinge" axis="0 1 0"/>
                    <inertia mass="0.5" diaginv="0.1 0.1 0.1"/>
                    <geom name="pole" type="capsule" size="0.03" fromto="0 0 0 0 0 0.5" rgba="0.3 0.3 0.8 1"/>
                    
                    <body name="pole_tip" pos="0 0 0.5">
                        <geom name="pole_tip" type="sphere" size="0.05" rgba="0.2 0.2 0.6 1"/>
                    </body>
                </body>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="cart_motor" joint="cart_x" ctrlrange="-10 10"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def simulate_trajectory(model, controls, num_steps=100):
    """Simulate a trajectory given control inputs."""
    data = mujoco.MjData(model)
    data.qpos[:] = 0  # Start at equilibrium
    data.qpos[1] = np.pi  # Pole starts up
    
    trajectory = [data.qpos.copy()]
    
    for i in range(min(len(controls), num_steps)):
        data.ctrl[0] = controls[i]
        mujoco.mj_step(model, data)
        trajectory.append(data.qpos.copy())
    
    return np.array(trajectory)


def cost_function(controls, model, target_state):
    """Compute cost of a control trajectory."""
    trajectory = simulate_trajectory(model, controls)
    
    # Cost: squared distance from target state
    final_state = trajectory[-1] if len(trajectory) > 0 else np.zeros(2)
    state_cost = np.sum((final_state - target_state)**2)
    
    # Control cost: minimize control effort
    control_cost = 0.01 * np.sum(controls**2)
    
    # Constraint cost: penalize cart leaving bounds
    cart_positions = trajectory[:, 0]
    bounds_cost = 0
    for pos in cart_positions:
        if np.abs(pos) > 0.9:
            bounds_cost += 10 * (np.abs(pos) - 0.9)**2
    
    total_cost = state_cost + control_cost + bounds_cost
    return total_cost


def main():
    """Run optimal control demonstration."""
    model = create_control_env()
    
    print("Optimal Control Optimization Demo")
    print("-" * 50)
    print("This example demonstrates solving optimal control problems.")
    print("Target: Swing and balance a pole with minimal control effort.")
    print("-" * 50)
    
    # Problem setup
    num_steps = 50
    target_state = np.array([0.0, np.pi])  # Pole upright at origin
    
    # Initial guess for control
    x0 = np.zeros(num_steps)
    
    print("\nOptimizing control trajectory...")
    print("This may take a moment...")
    
    # Optimize
    result = minimize(
        lambda u: cost_function(u, model, target_state),
        x0,
        method='BFGS',
        options={'disp': False}
    )
    
    optimal_controls = result.x
    optimal_cost = result.fun
    
    print(f"Optimization complete!")
    print(f"  Final cost: {optimal_cost:.4f}")
    print(f"  Iterations: {result.nit}")
    
    # Simulate optimal trajectory
    print("\nSimulating optimal trajectory...")
    data = mujoco.MjData(model)
    data.qpos[:] = [0, np.pi]  # Start with pole upright
    
    for step in range(200):
        if step < len(optimal_controls):
            data.ctrl[0] = optimal_controls[step]
        
        mujoco.mj_step(model, data)
        
        if step % 50 == 0:
            print(f"  Step {step}: Cart position={data.qpos[0]:.3f}, Pole angle={data.qpos[1]:.3f}")
    
    print("\nOptimal control demonstration complete!")


if __name__ == "__main__":
    main()
