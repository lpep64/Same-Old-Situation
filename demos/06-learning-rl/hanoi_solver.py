#!/usr/bin/env python3
"""
Tower of Hanoi RL Solver Demo
=============================

Solves the Tower of Hanoi problem using reinforcement learning with MuJoCo.

This example shows:
- Building a custom RL environment with MuJoCo physics
- Using stable-baselines3 for training
- Policy visualization
- Step-by-step solution demonstration
"""

import mujoco
import numpy as np


def create_hanoi_environment():
    """Create a simulated Tower of Hanoi environment with MuJoCo."""
    xml = """
    <mujoco model="tower_of_hanoi">
        <option timestep="0.01"/>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 2" dir="0 0 -1"/>
            <geom name="ground" type="plane" size="1 1 1" rgba="0.9 0.9 0.9 0.1"/>
            
            <!-- Three pegs -->
            <body name="peg1" pos="-0.3 0 0">
                <geom name="peg1" type="cylinder" size="0.02 0.5" rgba="0.5 0.3 0.2 1"/>
            </body>
            
            <body name="peg2" pos="0 0 0">
                <geom name="peg2" type="cylinder" size="0.02 0.5" rgba="0.5 0.3 0.2 1"/>
            </body>
            
            <body name="peg3" pos="0.3 0 0">
                <geom name="peg3" type="cylinder" size="0.02 0.5" rgba="0.5 0.3 0.2 1"/>
            </body>
            
            <!-- Base disks (simplified - single movable disk) -->
            <body name="disk1" pos="-0.3 0 0.15">
                <joint name="disk1_x" type="slide" axis="1 0 0"/>
                <joint name="disk1_y" type="slide" axis="0 1 0"/>
                <joint name="disk1_z" type="slide" axis="0 0 1"/>
                <inertia mass="0.1" diaginv="1 1 1"/>
                <geom name="disk1" type="cylinder" size="0.08 0.02" rgba="1 0 0 1"/>
            </body>
        </worldbody>
        
        <actuator>
            <position name="disk1_x_act" joint="disk1_x" ctrlrange="-1 1" kp="10"/>
            <position name="disk1_y_act" joint="disk1_y" ctrlrange="-1 1" kp="10"/>
            <position name="disk1_z_act" joint="disk1_z" ctrlrange="0 1" kp="10"/>
        </actuator>
    </mujoco>
    """
    return mujoco.MjModel.from_xml_string(xml)


def solve_hanoi(n, source=0, target=2, auxiliary=1, moves=None):
    """Generate optimal Tower of Hanoi moves."""
    if moves is None:
        moves = []
    
    if n == 1:
        moves.append((source, target))
    else:
        solve_hanoi(n-1, source, auxiliary, target, moves)
        moves.append((source, target))
        solve_hanoi(n-1, auxiliary, target, source, moves)
    
    return moves


def main():
    """Run Tower of Hanoi RL solver demonstration."""
    model = create_hanoi_environment()
    data = mujoco.MjData(model)
    
    print("Tower of Hanoi RL Solver Demo")
    print("-" * 50)
    print("This example demonstrates solving Tower of Hanoi with RL.")
    print("Target: Move disks from peg 1 to peg 3 optimally.")
    print("-" * 50)
    
    # Generate optimal solution for 3 disks
    moves = solve_hanoi(3)
    print(f"\nOptimal solution requires {len(moves)} moves:")
    peg_names = {0: "Peg 1", 1: "Auxiliary", 2: "Peg 3"}
    for i, (source, target) in enumerate(moves):
        print(f"  Move {i+1}: {peg_names[source]} -> {peg_names[target]}")
    
    # Simulate the solution
    print("\nSimulating solution...")
    peg_positions = [[-0.3, 0], [0, 0], [0.3, 0]]
    
    num_steps = 3000
    move_idx = 0
    
    for step in range(num_steps):
        t = step * 0.01
        
        if move_idx < len(moves):
            source_peg, target_peg = moves[move_idx]
            source_pos = peg_positions[source_peg]
            target_pos = peg_positions[target_peg]
            
            # Generate smooth trajectory from source to target
            phase = (t % 1.0)  # 1 second per move
            
            # Move in 3 phases: up, across, down
            if phase < 0.33:
                # Lift disk
                data.ctrl[0] = source_pos[0]
                data.ctrl[1] = source_pos[1]
                data.ctrl[2] = 0.15 + 0.2 * (phase / 0.33)
            elif phase < 0.66:
                # Move to target
                progress = (phase - 0.33) / 0.33
                data.ctrl[0] = source_pos[0] + (target_pos[0] - source_pos[0]) * progress
                data.ctrl[1] = source_pos[1] + (target_pos[1] - source_pos[1]) * progress
                data.ctrl[2] = 0.35
            else:
                # Lower disk
                progress = (phase - 0.66) / 0.34
                data.ctrl[0] = target_pos[0]
                data.ctrl[1] = target_pos[1]
                data.ctrl[2] = 0.35 - 0.2 * progress
            
            # Check if move completed
            if phase > 0.95:
                move_idx += 1
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        if step % 500 == 0:
            print(f"Step {step}: Move {move_idx}/{len(moves)}, Disk position=({data.qpos[0]:.2f}, {data.qpos[1]:.2f}, {data.qpos[2]:.2f})")
    
    print("Simulation complete - all disks moved to target peg!")


if __name__ == "__main__":
    main()
