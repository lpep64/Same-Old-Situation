# MuJoCo-Testing

**Physics simulation and robotics examples using MuJoCo for research and education at the University of Rhode Island, Industrial Systems Engineering.**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://python.org)

## Overview

MuJoCo-Testing is a curated collection of physics simulation examples and robotics models demonstrating key concepts in dynamics, control, and manipulation. This repository serves as both a learning resource and a showcase of work in industrial systems engineering.

## Quick Start

### Installation

```bash
git clone https://github.com/URI-ISE/MuJoCo-Testing.git
cd MuJoCo-Testing
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
pip install -e ".[dev]"
```

### Running Demonstrations

```bash
# Fundamental physics
python demos/01-fundamentals/basic_tests.py

# GUI visualization
python demos/02-visualization/simple_gui.py

# Robotics arms (Franka)
python demos/03-robotics-arms/franka_panda.py

# RL training (Hanoi solver)
python demos/06-learning-rl/hanoi_solver.py
```

### Running Tests

```bash
pytest tests/
pytest -v
```

## Project Structure

The project is organized into demonstrations, models, and supporting resources:

```
MuJoCo-Testing/
├── demos/                         # Main demonstration scripts (8 categories)
│   ├── 01-fundamentals/           # Basic physics and concepts
│   ├── 02-visualization/          # GUI and interactive demos
│   ├── 03-robotics-arms/          # Arm and manipulator robots
│   ├── 04-locomotion/             # Legged locomotion
│   ├── 05-manipulation/           # Pick and place, grasping
│   ├── 06-learning-rl/            # RL training and optimal control
│   ├── 07-tools/                  # Optimization and control utilities
│   └── 08-testing/                # Integration tests
│
├── examples/                      # Legacy examples and tutorials
│   ├── basic/                     # C++ physics examples
│   ├── gui/                       # GUI implementations
│   ├── testing/                   # Test fixtures
│   ├── tools/                     # Utility scripts (CEM, DDP, MPC)
│   └── tutorial/                  # Python tutorial scripts
│
├── projects/                      # Specialized project implementations
│   ├── anissa_niryo/              # Niryo robot projects
│   ├── franka_demos/              # Franka Panda demonstrations
│   ├── hanoi/                     # Tower of Hanoi solver
│   ├── juan_manip/                # Manipulation experiments
│   ├── mujo_manipulation/         # Pick and place implementations
│   └── rl_training/               # Reinforcement learning projects
│
├── models/                        # Physics model definitions
│   ├── basic/                     # Simple demonstration models
│   ├── robots/                    # Robot model library (URDF/XML)
│   └── vendors/                   # Third-party models
│
├── tests/                         # Unit and system tests
├── docs/                          # Project documentation
├── README.md                      # This file
├── CONTRIBUTING.md                # Development guidelines
├── MIGRATION.md                   # Migration guide (old → new structure)
├── requirements.txt               # Runtime dependencies
├── pyproject.toml                 # Package configuration
└── .github/                       # CI/CD pipelines and templates
```

## Learning Paths

Choose a learning path based on your interests:

### Path 1: Physics Fundamentals
`ash
cd demos/01-fundamentals/
python basic_tests.py          # Basic physics
python pendulum.py             # Pendulum control
`

### Path 2: Visualization & GUI
`ash
cd demos/02-visualization/
python simple_gui.py           # Interactive visualization
`

### Path 3: Robotic Arms
`ash
cd demos/03-robotics-arms/
python franka_panda.py         # Franka Emika Panda
python niryo_arm.py            # Niryo One robot
`

### Path 4: Legged Locomotion
`ash
cd demos/04-locomotion/
python biped_walker.py         # Biped walking patterns
python quadruped_gait.py       # Quadruped gaits
`

### Path 5: Manipulation & Grasping
`ash
cd demos/05-manipulation/
python pick_and_place.py       # Pick and place tasks
python manipulation_tasks.py    # Advanced manipulation
`

### Path 6: Learning & Control
`ash
cd demos/06-learning-rl/
python hanoi_solver.py         # RL solver (Tower of Hanoi)
python policy_training.py      # Reinforcement learning
python control_optimization.py # Optimal control
`

## Demonstration Categories

### 01-Fundamentals
Physics basics and essential concepts:
- Basic physics simulation
- Pendulum dynamics and control
- Collision detection and response
- Multi-body dynamics

### 02-Visualization
Interactive graphical demonstrations:
- Simple GUI examples
- Real-time visualization
- Interactive control

### 03-Robotics-Arms
Robotic arm demonstrations:
- Franka Panda arm (7-DOF industrial manipulator)
- Niryo One robot (6-DOF collaborative arm)
- Arm kinematics and dynamics
- Trajectory control

### 04-Locomotion
Legged robot demonstrations:
- Biped walkers
- Quadruped gaits
- Locomotion control

### 05-Manipulation
Manipulation and grasping:
- Pick and place tasks
- Manipulation primitives
- Grasping strategies

### 06-Learning-RL
Reinforcement learning and optimal control:
- RL training examples
- Policy optimization
- Tower of Hanoi solver
- Model-based control

### 07-Tools
Utility scripts and optimization tools:
- Cross-entropy method (CEM)
- Differential dynamic programming (DDP)
- Model predictive control (MPC)
- Other control algorithms

### 08-Testing
Integration tests and validation:
- System tests
- Performance benchmarks
- Validation suite

## Dependencies

Core:
- mujoco (>=3.0.0): Physics engine
- numpy (>=1.21.0): Numerical computing
- scipy (>=1.7.0): Scientific computing
- gymnasium (>=0.26.0): RL environments
- stable-baselines3 (>=2.0.0): RL algorithms

Development:
- pytest: Testing framework
- black: Code formatting

See requirements.txt for complete list.

## Development

### Setting Up

```bash
pip install -e ".[dev]"
pytest
black demos/ models/ tests/
```

### Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on:
- Development environment setup
- Running tests and linting
- Submitting pull requests
- Code style standards

## Organization & Attribution

### Primary Contributors
- Luke Pepin: Framework architecture and integration

### Foundational Code
- mujoManipulation: Pick-and-place manipulation (Romesh Prasad, Juan Lopez)
- MuJoCo_Sample: Initial robotics examples (Anissa Elias)

### Model Library
- MuJoCo Menagerie (DeepMind): Curated robot models
  - Source: https://github.com/google-deepmind/mujoco_menagerie

All vendored assets retain original LICENSE and README files in models/vendors/.

## License

This project is licensed under Apache License 2.0. See [LICENSE](LICENSE) for details.

MuJoCo is proprietary software by DeepMind. See https://mujoco.org for licensing information.

## References

- MuJoCo: https://mujoco.org
- MuJoCo Menagerie: https://github.com/google-deepmind/mujoco_menagerie
- Gymnasium: https://gymnasium.farama.org/

---

**University of Rhode Island, Industrial Systems Engineering**  
*Last updated: January 15, 2026*
