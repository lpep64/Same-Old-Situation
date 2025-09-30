# NiryoArm MuJoCo Project

A MuJoCo simulation project for the Niryo robotic arm, including manipulation tasks, reinforcement learning, and motion planning.

## 🎯 Project Overview

This project demonstrates how to properly structure a MuJoCo robotics project with:
- Robotic arm simulation and control
- Object manipulation tasks
- Reinforcement learning integration
- Motion planning algorithms
- Data collection and analysis

## 📁 Project Structure

```
NiryoArm/
├── README.md                 # This file
├── requirements.txt          # Python dependencies
├── setup.py                 # Package setup
├── .gitignore               # Git ignore rules
├── 
├── src/                     # Source code
│   ├── __init__.py
│   ├── arm_controller.py    # Main arm control logic
│   ├── environment.py       # MuJoCo environment wrapper
│   ├── tasks/              # Manipulation tasks
│   ├── planning/           # Motion planning algorithms
│   ├── learning/           # RL components
│   └── utils/              # Utility functions
├── 
├── models/                  # MuJoCo XML models
│   ├── niryo_arm.xml       # Main arm model
│   ├── workspace.xml       # Workspace environment
│   └── objects/            # Manipulation objects
├── 
├── config/                  # Configuration files
│   ├── arm_config.yaml     # Arm parameters
│   ├── task_config.yaml    # Task settings
│   └── training_config.yaml # RL training config
├── 
├── assets/                  # Media and resources
│   ├── images/             # Screenshots, diagrams
│   ├── meshes/             # 3D mesh files
│   └── textures/           # Texture files
├── 
├── docs/                    # Documentation
│   ├── setup.md            # Setup instructions
│   ├── api.md              # API documentation
│   └── tutorials/          # Usage tutorials
├── 
├── tests/                   # Unit tests
│   ├── test_controller.py
│   ├── test_environment.py
│   └── test_tasks.py
├── 
├── scripts/                 # Utility scripts
│   ├── train_agent.py      # RL training script
│   ├── collect_data.py     # Data collection
│   └── visualize.py        # Visualization tools
├── 
└── data/                    # Generated data
    ├── trajectories/       # Recorded trajectories
    ├── models/             # Trained ML models
    └── logs/               # Training logs
```

## 🚀 Quick Start

### Installation
```bash
cd projects/NiryoArm
pip install -r requirements.txt
pip install -e .
```

### Basic Usage
```python
from src.environment import NiryoArmEnv
from src.arm_controller import ArmController

# Create environment
env = NiryoArmEnv()
controller = ArmController(env)

# Run basic arm movement
controller.move_to_position([0, 0, 0.3])
```

### Run Tests
```bash
python -m pytest tests/
```

### Train RL Agent
```bash
python scripts/train_agent.py --task pick_and_place
```

## 📋 Features (Planned)

- [ ] MuJoCo arm simulation with accurate physics
- [ ] Forward/inverse kinematics
- [ ] Collision detection and avoidance
- [ ] Pick and place tasks
- [ ] Reinforcement learning integration
- [ ] Motion planning (RRT*, trajectory optimization)
- [ ] Real robot integration (sim-to-real)
- [ ] Data collection and analysis tools

## 🔧 Development Guidelines

### Code Structure
- Use object-oriented design
- Follow PEP 8 style guidelines
- Include type hints and docstrings
- Write unit tests for all components

### Model Organization
- Keep XML models modular and reusable
- Use consistent naming conventions
- Document model parameters and assumptions

### Configuration Management
- Use YAML files for all configurations
- Separate development and production configs
- Version control all configuration changes

## 📚 Documentation

See the `docs/` folder for detailed documentation:
- [Setup Guide](docs/setup.md)
- [API Reference](docs/api.md)
- [Tutorials](docs/tutorials/)

## 🤝 Contributing

1. Fork the project
2. Create a feature branch
3. Make changes with tests
4. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

---

**Note**: This is a project template showing proper MuJoCo project structure. Implementation details will be added as the project develops.