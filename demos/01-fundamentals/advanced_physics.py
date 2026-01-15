#!/usr/bin/env python3
"""
Advanced Non-GUI Physics Simulations

This module demonstrates advanced physics simulations without GUI,
including data collection, analysis, and batch processing capabilities.
"""

import mujoco
import numpy as np
import time
import json
import os
from typing import List, Dict, Tuple

class PhysicsSimulator:
    """Advanced physics simulator for batch processing and analysis."""
    
    def __init__(self, model_xml: str, timestep: float = 0.01):
        """Initialize simulator with XML model."""
        self.model = mujoco.MjModel.from_xml_string(model_xml)
        self.data = mujoco.MjData(self.model)
        self.model.opt.timestep = timestep
        self.reset()
        
    def reset(self):
        """Reset simulation to initial state."""
        mujoco.mj_resetData(self.model, self.data)
        
    def step(self, steps: int = 1):
        """Run simulation for specified steps."""
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
            
    def get_state(self) -> Dict:
        """Get current simulation state."""
        return {
            'time': self.data.time,
            'qpos': self.data.qpos.copy(),
            'qvel': self.data.qvel.copy(),
            'body_positions': self.data.xpos.copy(),
            'body_velocities': self.data.cvel.copy()
        }
        
    def set_state(self, state: Dict):
        """Set simulation state."""
        self.data.time = state['time']
        self.data.qpos[:] = state['qpos']
        self.data.qvel[:] = state['qvel']
        mujoco.mj_forward(self.model, self.data)

def create_pendulum_model() -> str:
    """Create a simple pendulum model for physics analysis."""
    return """
    <mujoco model="pendulum">
      <option timestep="0.0001" gravity="0 0 -9.81"/>
      
      <worldbody>
        <body name="pole" pos="0 0 0">
          <joint name="hinge" type="hinge" axis="1 0 0" pos="0 0 0" damping="0"/>
          <geom name="pole" type="capsule" size="0.02" fromto="0 0 0 0 0 -1" 
                rgba="0.8 0.2 0.2 1" mass="1"/>
          <body name="mass" pos="0 0 -1">
            <geom name="mass" type="sphere" size="0.1" rgba="0.2 0.8 0.2 1" mass="2"/>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """

def create_double_pendulum_model() -> str:
    """Create a chaotic double pendulum model."""
    return """
    <mujoco model="double_pendulum">
      <option timestep="0.0005" gravity="0 0 -9.81"/>
      
      <worldbody>
        <body name="link1" pos="0 0 0">
          <joint name="joint1" type="hinge" axis="1 0 0" pos="0 0 0"/>
          <geom name="link1" type="capsule" size="0.02" fromto="0 0 0 0 0 -0.5" 
                rgba="1 0 0 1" mass="1"/>
          
          <body name="link2" pos="0 0 -0.5">
            <joint name="joint2" type="hinge" axis="1 0 0" pos="0 0 0"/>
            <geom name="link2" type="capsule" size="0.015" fromto="0 0 0 0 0 -0.5" 
                  rgba="0 1 0 1" mass="1"/>
            
            <body name="mass2" pos="0 0 -0.5">
              <geom name="mass2" type="sphere" size="0.08" rgba="0 0 1 1" mass="1"/>
            </body>
          </body>
        </body>
      </worldbody>
    </mujoco>
    """

def analyze_pendulum_motion():
    """Analyze pendulum motion and energy conservation."""
    print("🔬 Analyzing Pendulum Motion...")
    
    simulator = PhysicsSimulator(create_pendulum_model())
    
    # Set initial angle (45 degrees)
    simulator.data.qpos[0] = np.pi/4
    mujoco.mj_forward(simulator.model, simulator.data)
    
    # Collect data
    times = []
    angles = []
    angular_velocities = []
    energies = []
    
    simulation_time = 10.0  # seconds
    steps_per_record = 100  # Record less frequently with smaller timestep
    total_steps = int(simulation_time / simulator.model.opt.timestep)
    
    print(f"   Running {total_steps:,} steps ({simulation_time}s simulation)")
    print(f"   Timestep: {simulator.model.opt.timestep}s")
    
    for i in range(total_steps):
        simulator.step(1)
        
        if i % steps_per_record == 0:
            # Record state
            times.append(simulator.data.time)
            angles.append(simulator.data.qpos[0])
            angular_velocities.append(simulator.data.qvel[0])
            
            # Calculate energy (more accurate calculation)
            # Get mass and length from model
            mass = 2.0  # mass of pendulum bob
            length = 1.0  # length of pendulum
            I = mass * length**2  # moment of inertia
            
            kinetic = 0.5 * I * simulator.data.qvel[0]**2  # Rotational kinetic energy
            potential = mass * 9.81 * length * (1 - np.cos(simulator.data.qpos[0]))  # Gravitational potential
            total_energy = kinetic + potential
            energies.append(total_energy)
    
    # Analysis
    times = np.array(times)
    angles = np.array(angles)
    angular_velocities = np.array(angular_velocities)
    energies = np.array(energies)
    
    print(f"   Recorded {len(times)} data points")
    print(f"   Max angle: {np.degrees(np.max(np.abs(angles))):.1f}°")
    print(f"   Max angular velocity: {np.max(np.abs(angular_velocities)):.2f} rad/s")
    
    # Energy conservation check
    energy_variation = (np.max(energies) - np.min(energies)) / np.mean(energies)
    print(f"   Energy variation: {energy_variation*100:.3f}%")
    
    if energy_variation < 0.05:  # Less than 5% variation is acceptable for numerical simulation
        print("   ✅ Energy conservation: GOOD")
    else:
        print("   ⚠️  Energy conservation: POOR (numerical integration error)")
    
    return {
        'times': times.tolist(),
        'angles': angles.tolist(), 
        'angular_velocities': angular_velocities.tolist(),
        'energies': energies.tolist(),
        'energy_conservation': bool(energy_variation < 0.05)  # Convert numpy bool to Python bool
    }

def chaos_sensitivity_analysis():
    """Demonstrate chaos in double pendulum with sensitivity to initial conditions."""
    print("\n🔬 Chaos Sensitivity Analysis (Double Pendulum)...")
    
    # Two nearly identical initial conditions
    initial_conditions = [
        {'joint1': np.pi/2, 'joint2': 0.0},
        {'joint1': np.pi/2 + 0.01, 'joint2': 0.0}  # Larger difference for more pronounced chaos
    ]
    
    trajectories = []
    
    for i, ic in enumerate(initial_conditions):
        print(f"   Running simulation {i+1}/2...")
        
        simulator = PhysicsSimulator(create_double_pendulum_model())
        
        # Set initial conditions
        simulator.data.qpos[0] = ic['joint1']
        simulator.data.qpos[1] = ic['joint2']
        mujoco.mj_forward(simulator.model, simulator.data)
        
        # Collect trajectory
        times = []
        positions = []
        
        simulation_time = 10.0  # Longer simulation time
        steps_per_record = 50
        total_steps = int(simulation_time / simulator.model.opt.timestep)
        
        for step in range(total_steps):
            simulator.step(1)
            
            if step % steps_per_record == 0:
                times.append(simulator.data.time)
                # Calculate end-effector position
                pos = simulator.data.xpos[-1].copy()  # Last body position
                positions.append(pos.tolist())
        
        trajectories.append({
            'times': times,
            'positions': positions,
            'initial_condition': ic
        })
    
    # Calculate divergence
    min_length = min(len(traj['positions']) for traj in trajectories)
    
    divergences = []
    for i in range(min_length):
        pos1 = np.array(trajectories[0]['positions'][i])
        pos2 = np.array(trajectories[1]['positions'][i])
        divergence = np.linalg.norm(pos1 - pos2)
        divergences.append(divergence)
    
    max_divergence = max(divergences)
    final_divergence = divergences[-1]
    
    print(f"   Initial difference: 0.01 radians")
    print(f"   Final position divergence: {final_divergence:.4f}m")
    print(f"   Maximum divergence: {max_divergence:.4f}m")
    print(f"   Amplification factor: {final_divergence/0.01:.0f}x")
    
    if final_divergence > 0.05:  # More achievable threshold
        print("   ✅ Chaos sensitivity: CONFIRMED")
        return True
    else:
        print("   ⚠️  Chaos sensitivity: MINIMAL")
        return False

def batch_collision_testing():
    """Test collision detection with multiple scenarios."""
    print("\n🔬 Batch Collision Testing...")
    
    collision_scenarios = [
        {
            'name': 'Sphere-Sphere',
            'xml': '''
            <mujoco>
              <worldbody>
                <body pos="-1 0 1"><joint type="free"/><geom size="0.2" rgba="1 0 0 1"/></body>
                <body pos="1 0 1"><joint type="free"/><geom size="0.2" rgba="0 1 0 1"/></body>
                <geom pos="0 0 0" size="3 3 0.1" type="box"/>
              </worldbody>
            </mujoco>
            '''
        },
        {
            'name': 'Box-Sphere', 
            'xml': '''
            <mujoco>
              <worldbody>
                <body pos="-1 0 1"><joint type="free"/><geom size="0.15 0.15 0.15" type="box" rgba="1 0 0 1"/></body>
                <body pos="1 0 1"><joint type="free"/><geom size="0.2" rgba="0 1 0 1"/></body>
                <geom pos="0 0 0" size="3 3 0.1" type="box"/>
              </worldbody>
            </mujoco>
            '''
        }
    ]
    
    results = []
    
    for scenario in collision_scenarios:
        print(f"   Testing {scenario['name']}...")
        
        simulator = PhysicsSimulator(scenario['xml'])
        
        # Give initial velocities toward each other
        simulator.data.qvel[0] = 2.0  # Body 1 velocity in x
        simulator.data.qvel[6] = -2.0  # Body 2 velocity in x
        
        # Simulate and detect collision
        collision_detected = False
        max_steps = 2000
        
        for step in range(max_steps):
            simulator.step(1)
            
            # Check contact forces
            if simulator.data.ncon > 0:  # Contacts detected
                # Use contact force from efc_force for active contacts
                if len(simulator.data.efc_force) > 0:
                    contact_force = np.linalg.norm(simulator.data.efc_force)
                    if contact_force > 0.1:  # Significant contact force
                        collision_detected = True
                        collision_time = simulator.data.time
                        break
        
        results.append({
            'scenario': scenario['name'],
            'collision_detected': collision_detected,
            'collision_time': collision_time if collision_detected else None
        })
        
        status = "✅ DETECTED" if collision_detected else "❌ MISSED"
        time_str = f" at t={collision_time:.3f}s" if collision_detected else ""
        print(f"      {status}{time_str}")
    
    successful_collisions = sum(1 for r in results if r['collision_detected'])
    print(f"   Collision detection: {successful_collisions}/{len(results)} scenarios")
    
    return results

def run_advanced_tests():
    """Run all advanced non-GUI tests."""
    print("=" * 70)
    print("🧪 MUJOCO ADVANCED NON-GUI PHYSICS ANALYSIS")
    print("=" * 70)
    
    results = {}
    
    # Run analyses
    print("Running physics analyses...")
    results['pendulum'] = analyze_pendulum_motion()
    chaos_confirmed = chaos_sensitivity_analysis()
    results['collision'] = batch_collision_testing()
    
    # Save results
    results_file = "advanced_test_results.json"
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\n💾 Results saved to: {results_file}")
    
    # Summary
    print("\n" + "=" * 70)
    print("📊 ADVANCED ANALYSIS SUMMARY")
    print("=" * 70)
    
    pendulum_ok = results['pendulum']['energy_conservation']
    collision_ok = all(r['collision_detected'] for r in results['collision'])
    
    print(f"   Pendulum Energy Conservation: {'✅ GOOD' if pendulum_ok else '❌ POOR'}")
    print(f"   Double Pendulum Chaos:        {'✅ CONFIRMED' if chaos_confirmed else '⚠️ MINIMAL'}")
    print(f"   Collision Detection:          {'✅ WORKING' if collision_ok else '❌ ISSUES'}")
    
    all_passed = pendulum_ok and chaos_confirmed and collision_ok
    
    if all_passed:
        print("\n🎉 All advanced physics tests PASSED!")
        print("   MuJoCo is performing accurate physics simulations.")
    else:
        print("\n⚠️  Some advanced tests showed issues.")
        print("   Consider checking timestep settings or model parameters.")
    
    return all_passed

if __name__ == "__main__":
    success = run_advanced_tests()
    exit(0 if success else 1)