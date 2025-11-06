# MuJoCo Development Framework

**A comprehensive, optimized MuJoCo framework for physics simulation, robotics research, and machine learning applications.**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://python.org)
[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.0+-orange.svg)](https://mujoco.org)

## 🚀 Quick Start

```bash
# Test basic functionality
python examples/basic/basic_tests.py

# Try interactive GUI demo
python examples/gui/simple_gui.py

# Explore robotics examples
python examples/robotics/franka_arm_demo.py

# Check available models
ls models/robots/Arms/
```

## 📁 **Streamlined Project Structure**

This repository is optimized for robotics projects and model usage:

```
Same-Old-Situation/              # Unified robotics framework
├── examples/                    # � Lightweight tutorials & demos
│   ├── basic/                  # Fundamental physics examples
│   ├── tutorial/               # Step-by-step learning scripts
│   ├── gui/                    # Interactive demonstrations
│   ├── tools/                  # Utilities & controllers (cem, ddp, mpc)
│   └── testing/                # Validation & testing
├── models/                     # 🤖 Comprehensive model library
│   ├── basic/                  # Simple demo models (car, pendulum, etc.)
│   ├── robots/                 # Production robot models by type
│   │   ├── Arms/              # Franka, UR5e, Niryo, etc.
│   │   ├── Humanoids/         # H1, G1, Atlas, etc.
│   │   ├── Quadrupeds/        # Spot, A1, Go1, etc.
│   │   └── [other categories]
│   └── vendors/                # Third-party assets (properly attributed)
│       ├── mujoManipulation/  # Manipulation primitives
│       └── mujoco_menagerie/  # DeepMind robot models
├── projects/                   # 🏗️ Active research projects
│   ├── mujo_manipulation/     # Pick-and-place manipulation
│   ├── rl_training/           # Reinforcement learning experiments
│   ├── hanoi_rl/              # Tower of Hanoi RL solver
│   ├── hanoi_solver/          # Tower of Hanoi demonstrations
│   ├── franka_demos/          # Franka robot examples
│   └── anissa_niryo/          # Niryo robot demonstrations
├── requirements.txt           # Essential dependencies
└── README.md                  # This documentation
```

### 🗑️ **Removed (Framework Development Only)**
- `mujoco_git/`, `mpc_git/`, `mantipulation_git/` - Now managed via pip or vendored
- `examples/robotics/` - Moved to projects/ or models/
- `examples/machine_learning/` - Moved to projects/rl_training/

## 🔧 **Key Improvements**

### ✅ **Clean Separation of Concerns**
- **examples/** - Lightweight tutorials and reference demos only
- **models/** - Centralized model library with vendor attribution
- **projects/** - Complete applications with saved state and outputs

### ✅ **Eliminated Redundancy**
- **Removed** duplicate robot models from examples/ (now in models/robots/)
- **Vendored** third-party assets properly (mujoManipulation, mujoco_menagerie)
- **Consolidated** project-like code (RL training, manipulation) into projects/

### ✅ **Professional Structure**
- **Dependencies** managed via pip (requirements.txt)
- **Attribution** preserved in vendors/ with original LICENSE/README
- **Projects** organized with clear purpose and saved artifacts

## 🎯 **Getting Started Paths**

### **New to MuJoCo?**
```bash
cd examples/tutorial/
python mujoco_base.py
```

### **Robot Manipulation?**
```bash
cd projects/mujo_manipulation/
python pnp.py  # Pick-and-place demo
```

### **Reinforcement Learning?**
```bash
cd projects/rl_training/
python quick_rl_training.py
```

### **Custom Project?**
```bash
cd projects/hanoi_rl/
# Use as template for new projects
```

## 🤖 **Robot Models Available**

- **Arms**: Franka FR3/Panda, UR5e/UR10e, Niryo NED2, Kinova Gen3, KUKA iiwa
- **Humanoids**: Unitree H1/G1, Boston Dynamics Atlas, Honda ASIMO variants  
- **Quadrupeds**: Boston Dynamics Spot, Unitree A1/Go1/Go2, ANYmal
- **Hands**: Shadow Hand, Allegro Hand, LEAP Hand
- **Mobile**: TurtleBot, Tiago, Spot + arm combinations

## 🔗 **Key Features**

- **Unified Examples**: All learning materials in one place
- **Modular Controllers**: Reusable control algorithms
- **Project Templates**: Quick start for new developments
- **Official Models**: Comprehensive robot library
- **Clean Architecture**: Separated framework from user projects

## 📖 **Documentation**

- `examples/README.md` - Comprehensive learning guide
- `projects/project_template/README.md` - Project development guide
- `doc/` - Official MuJoCo documentation
- Each directory contains specific documentation

## 🛠️ **Development Workflow**

1. **Learn**: Start with `examples/tutorial/`
2. **Explore**: Try `examples/robotics/` or `examples/machine_learning/`
3. **Develop**: Create new project in `projects/` using template
4. **Share**: Contribute improvements back to `examples/`

## � **Acknowledgements and Citations**

This repository unifies several foundational codebases and asset libraries into a single, streamlined framework for robotics research. We gratefully acknowledge the authors of the original work.

### **Primary Author**
**Luke Pepin** - Primary architecture, framework integration, and task implementation

### **Foundational Code and Examples**
This project curates and adapts code from the following repositories. We extend our thanks to the original authors for their contributions to the field:

- **mujoManipulation**: Original author credits: Romesh Prasad, Juan Lopez
- **MuJoCo_Sample**: Original author credits: Anissa Elias

### **Asset Library**
This framework utilizes robotics models from the **MuJoCo Menagerie**, a high-quality collection of models curated by Google DeepMind.

- **Repository**: [google-deepmind/mujoco_menagerie](https://github.com/google-deepmind/mujoco_menagerie)
- **Reference**: https://github.com/google-deepmind/mujoco_menagerie

All vendored assets retain their original `LICENSE` and `README.md` files within their respective directories (e.g., `models/vendors/mujoco_menagerie/franka_emika_panda/`).

## �📄 **License**

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

**MuJoCo** stands for **Mu**lti-**Jo**int dynamics with **Co**ntact. It is a general purpose physics engine that aims to facilitate research and development in robotics, biomechanics, graphics and animation, machine learning, and other areas which demand fast and accurate simulation of articulated structures interacting with their environment.