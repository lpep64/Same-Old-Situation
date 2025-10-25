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
mujoco/                          # Streamlined workspace
├── examples/                    # 🎯 Learning & reference materials
│   ├── basic/                  # Fundamental physics demos
│   ├── robotics/               # Robot-specific examples
│   ├── machine_learning/       # AI/ML integration
│   ├── tutorial/               # Step-by-step learning
│   ├── gui/                    # Interactive demonstrations
│   ├── tools/                  # Utilities & controllers (cem, ddp, mpc)
│   └── testing/                # Validation & testing
├── models/                     # 🤖 Comprehensive robot library
│   ├── basic/                  # Core physics models
│   └── robots/                 # Organized by type
│       ├── Arms/               # Franka, UR5e, Niryo, etc.
│       ├── Humanoids/          # H1, G1, Atlas, etc.
│       ├── Quadrupeds/         # Spot, A1, Go1, etc.
│       └── [other categories]
├── projects/                   # 🏗️ Your active projects
│   ├── anissa_niryo/          # Niryo robot demonstrations
│   ├── hanoi_solver/          # Tower of Hanoi RL project
│   ├── franka_demos/          # Franka robot examples
│   └── project_template/      # Template for new projects
├── requirements.txt           # Essential dependencies only
└── README.md                  # This documentation
```

### 🗑️ **Removed (Framework Development Only)**
- `cmake/`, `src/`, `include/` - C++ compilation (you use pip mujoco)
- `unity/`, `mjx/`, `plugin/` - Advanced integrations (not needed)
- `doc/`, `python/` - Available online and via pip

## 🔧 **Key Improvements**

### ✅ **Streamlined for Projects**
- **Removed** build system dependencies (cmake, src, include)
- **Removed** advanced integrations (unity, mjx, plugins) 
- **Simplified** to essential dependencies for robotics projects

### ✅ **Better Organization**
- **Merged** `examples/ml/` + `demos/ml_examples/` → `examples/machine_learning/`
- **Consolidated** `test/` → `examples/testing/`
- **Moved** shared controllers to `examples/tools/` for reference

### ✅ **Project-Focused Structure**
- **Separated** personal projects into dedicated `projects/` workspace
- **Integrated** Niryo models into official `models/robots/Arms/niryo_ned2/`
- **Created** standardized project template for consistency

## 🎯 **Getting Started Paths**

### **New to MuJoCo?**
```bash
cd examples/tutorial/
python 01_basic_simulation.py
```

### **Robot Development?**
```bash
cd examples/robotics/
python franka_arm_demo.py
```

### **AI/ML Research?**
```bash
cd examples/machine_learning/
python reinforcement_learning_basics.py
```

### **Custom Project?**
```bash
cp -r project_template/ projects/my_project/
cd projects/my_project/
# Edit README.md and start developing
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

## 📄 **License**

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

---

**MuJoCo** stands for **Mu**lti-**Jo**int dynamics with **Co**ntact. It is a general purpose physics engine that aims to facilitate research and development in robotics, biomechanics, graphics and animation, machine learning, and other areas which demand fast and accurate simulation of articulated structures interacting with their environment.