# MuJoCo Sample - Niryo Pick and Place Robots

A simple MuJoCo simulation featuring a Niryo robot positioned on a table with a conveyor belt. This demo allows you to manually control the robot using MuJoCo's built-in viewer interface.

## What is MuJoCo?

**MuJoCo** (Multi-Joint dynamics with Contact) is a powerful physics engine designed for robotics research and development. It provides:

- **High-fidelity physics simulation** with accurate contact dynamics
- **Fast computation** optimized for real-time control applications
- **Rich sensor models** including cameras, force/torque sensors, and IMUs
- **Flexible robot modeling** supporting complex kinematic chains
- **Advanced contact handling** for manipulation tasks

### Why MuJoCo for Industrial Robotics?

MuJoCo excels in simulating complex robotic systems, making it ideal for:
- Testing robot control algorithms safely before deployment
- Optimizing pick and place trajectories
- Simulating sensor feedback and vision systems
- Training reinforcement learning agents
- Validating system performance under various conditions

## Project Overview

This repository demonstrates how to use MuJoCo to simulate **Niryo robots** performing automated pick and place operations on a conveyor belt system. This is a common industrial automation scenario where robots must:

1. **Detect objects** moving on a conveyor belt
2. **Track moving targets** with precise timing
3. **Execute pick operations** with appropriate force control
4. **Navigate to placement locations** efficiently
5. **Handle dynamic environments** with varying object properties

### Niryo Robot Characteristics

Niryo robots are designed for high-precision industrial tasks:
- **6-DOF articulated arm** for flexible manipulation
- **Parallel gripper** optimized for pick and place operations
- **Vision system integration** for object detection and tracking
- **Force/torque sensors** for gentle handling of delicate objects
- **High repeatability** (±0.1mm positioning accuracy)

## Getting Started

Follow these steps to set up your MuJoCo development environment and start simulating Niryo robots.

### Prerequisites

- **Python 3.8+** (recommended: Python 3.9 or 3.10)
- **Git** for repository management
- **OpenGL support** for visualization (usually pre-installed on most systems)

### Step 1: Clone the Repository

```bash
git clone https://github.com/anissa-elias/MuJoCo_Sample.git
cd MuJoCo_Sample
```

### Step 2: Create Virtual Environment (Recommended)

```bash
# Create a new virtual environment
python -m venv mujoco_env

# Activate the environment
# On Windows:
mujoco_env\Scripts\activate
# On macOS/Linux:
source mujoco_env/bin/activate
```

## Installation

### Step 3: Install Dependencies

```bash
# Install all required packages
pip install -r requirements.txt
```

### Step 4: Verify Installation

```bash
python -c "import mujoco; print(f'MuJoCo version: {mujoco.__version__}')"
```


## Files

### Basic Demos
- `niryo_demo_simple.xml` - MuJoCo scene definition (basic)
- `simple_niryo_demo.py` - Python script to launch the simulation (basic)

### Advanced Pick & Place Simulation
- `niryo_conveyor.xml` - Complete scene with conveyor belt and movable block
- `scripts/niryo_pick_place_code_sim.py` - Automated pick and place with inverse kinematics
- `niryo_documentation.md` - Comprehensive technical documentation of the pick & place system

### Mesh Files
- `niryo-gym-main\niryo_gym\niryo_robot\meshes\ned2\stl` - Original STL file path (deprecated)
- `../../models/robots/Arms/niryo_ned2/assets/ned2/stl` - Current STL file location

## Running the Simulations

### Basic Demo (Manual Control)
```bash
python simple_niryo_demo.py
```

### Advanced Pick & Place Demo (Automated IK)
```bash
cd scripts
python niryo_pick_place_code_sim.py
```

This advanced demo features:
- **Optimization-based Inverse Kinematics** using scipy.optimize
- **Kinematic magnet system** for reliable grasping
- **9-phase trajectory execution** (home → pick → place → return)
- **Orientation control** for proper gripper alignment
- **Smooth interpolation** using smoothstep function

See `niryo_documentation.md` for detailed technical information about the magnet system, IK solver, and trajectory planning.