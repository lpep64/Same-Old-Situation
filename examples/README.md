# MuJoCo Examples & Learning Resources

This directory contains comprehensive examples, tutorials, and testing resources for MuJoCo physics simulation and robotics.

## 📁 Directory Structure

### `basic/` - Fundamental Examples
- Core MuJoCo functionality demonstrations
- Physics simulation basics
- Simple robot models and interactions

### `robotics/` - Robot-Specific Examples  
- Examples for specific robot platforms
- Robot arm demonstrations
- Locomotion examples
- End-effector control

### `machine_learning/` - AI/ML Integration
- Reinforcement learning examples
- Neural network training
- Policy optimization
- Imitation learning
- Combined examples from previous demos/ directory

### `tutorial/` - Learning Path
- Step-by-step tutorials
- Progressive complexity examples
- Documented learning materials

### `gui/` - Graphical Interface Examples
- Interactive demonstrations
- Viewer customization
- User interface examples

### `tools/` - Utilities & Tools
- Model conversion utilities
- Analysis scripts
- Debugging tools
- Performance benchmarks

### `testing/` - Validation & Testing
- Unit tests for core functionality
- Regression tests
- Performance benchmarks
- Model validation scripts
- Consolidated from previous test/ directory

## 🚀 Getting Started

1. **New to MuJoCo?** Start with `tutorial/`
2. **Specific robot platform?** Check `robotics/`
3. **AI/ML applications?** Explore `machine_learning/`
4. **Need debugging tools?** Look in `tools/`

## 📖 Learning Path Recommendation

1. `tutorial/` - Learn MuJoCo basics
2. `basic/` - Understand core concepts
3. `robotics/` - Apply to specific robots
4. `machine_learning/` - Advanced AI integration
5. `tools/` - Optimize and debug

## 🔗 Related Directories

- `../models/` - Official robot and environment models
- `../projects/` - User project implementations
- `../controllers/` - Shared control algorithms

## 📝 Contributing

When adding new examples:
- Place in the most appropriate subdirectory
- Include clear documentation
- Add requirements if dependencies are needed
- Follow the existing code style
- Basic model loading and simulation examples

### `gui/` - Interactive Demonstrations  
Visual examples using MuJoCo's viewer:
- `interactive_gui.py` - Interactive physics playground
- `simple_gui.py` - Basic GUI demonstration
- Camera controls and visualization examples

### `ml/` - Machine Learning Examples
Reinforcement learning and AI integration:
- `gymnasium_integration.py` - Modern Gymnasium environment setup
- `quick_rl_training.py` - Fast RL training examples
- `ant.py` - Ant robot RL environment
- `gym_test.py` - Environment testing utilities
- `ppo_robot_arm/` - PPO implementation for robot arm training

### `robotics/` - Robot Models and Examples
Robot-specific demonstrations and models:
- Various robot XML models (car, robot arms, etc.)
- Robot control examples
- Multi-robot simulations

## 🚀 Quick Start

### Basic Physics Test
```bash
cd examples/basic
python basic_tests.py
```

### Interactive GUI Demo
```bash
cd examples/gui
python interactive_gui.py
```

### Machine Learning Training
```bash
cd examples/ml
python quick_rl_training.py
```

## 📋 Prerequisites

### Required Dependencies
```bash
pip install mujoco gymnasium numpy
```

### Optional ML Dependencies
```bash
pip install torch stable-baselines3 jax
```

## 🔧 Usage Guidelines

1. **Start with Basic**: Run basic examples first to verify your installation
2. **Explore GUI**: Use interactive examples to understand MuJoCo's capabilities  
3. **Try ML**: Experiment with reinforcement learning once comfortable with basics
4. **Customize Robotics**: Modify robot examples for your specific use case

## 📚 Additional Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Gymnasium Documentation](https://gymnasium.farama.org/)
- [MuJoCo Menagerie](https://github.com/deepmind/mujoco_menagerie) - More robot models

## 🐛 Troubleshooting

### Common Issues
- **Import Errors**: Ensure all dependencies are installed
- **Display Issues**: Set `MUJOCO_GL=egl` for headless rendering
- **Performance**: Use basic examples for debugging before complex ML tasks

### Getting Help
- Check example output for expected results
- Review console logs for error messages
- Refer to individual script documentation