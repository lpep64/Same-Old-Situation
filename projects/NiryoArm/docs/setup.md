# Setup Guide - NiryoArm Project

This guide walks you through setting up the NiryoArm MuJoCo project for development and usage.

## 📋 Prerequisites

### System Requirements
- **Python**: 3.8 or newer
- **Operating System**: Windows 10/11, macOS, or Linux
- **RAM**: 8GB minimum, 16GB recommended
- **GPU**: Optional but recommended for RL training

### Required Software
- **MuJoCo**: Physics simulation engine
- **Git**: Version control
- **IDE**: VS Code, PyCharm, or similar

## 🛠️ Installation Steps

### 1. Clone the Repository
```bash
# If this is part of a larger MuJoCo project
cd /path/to/mujoco/projects/NiryoArm

# Or clone standalone
git clone <repository-url> NiryoArm
cd NiryoArm
```

### 2. Create Virtual Environment
```bash
# Create virtual environment
python -m venv venv

# Activate it
# Windows
venv\Scripts\activate
# macOS/Linux  
source venv/bin/activate
```

### 3. Install Dependencies
```bash
# Install package in development mode
pip install -e .

# Or install from requirements file
pip install -r requirements.txt

# Install development tools (optional)
pip install -e .[dev]
```

### 4. Verify Installation
```bash
# Test basic imports
python -c "import mujoco; print('✅ MuJoCo works')"
python -c "from src import NiryoArmEnv; print('✅ Package works')"

# Run tests
python -m pytest tests/ -v
```

## 🏗️ Project Configuration

### 1. Configure Environment
```bash
# Copy example config files
cp config/arm_config.example.yaml config/arm_config.yaml
cp config/task_config.example.yaml config/task_config.yaml

# Edit configurations as needed
```

### 2. Download Models (if needed)
```bash
# If using external mesh files or models
python scripts/download_assets.py
```

### 3. Set Environment Variables
```bash
# Optional: Set up environment variables
export NIRYO_PROJECT_ROOT=$(pwd)
export MUJOCO_GL=egl  # For headless rendering
```

## 🧪 Testing the Setup

### 1. Basic Environment Test
```python
from src.environment import NiryoArmEnv
from src.arm_controller import ArmController

# Create environment
env = NiryoArmEnv()
controller = ArmController(env)

# Test basic functionality
obs, info = env.reset()
print(f"Observation shape: {obs.shape}")

# Test arm controller
controller.move_to_position([0.3, 0.0, 0.2])
```

### 2. Run Demo Scripts
```bash
# Basic arm movement demo
python scripts/demo_basic.py

# RL training demo
python scripts/train_agent.py --task reach --episodes 100
```

### 3. GUI Visualization
```bash
# Open interactive viewer
python scripts/visualize.py --model models/niryo_arm.xml
```

## 🔧 Development Setup

### 1. Install Development Tools
```bash
pip install -e .[dev]

# Set up pre-commit hooks (optional)
pre-commit install
```

### 2. IDE Configuration

#### VS Code
1. Install Python extension
2. Set interpreter to virtual environment
3. Configure linting and formatting:
```json
{
    "python.linting.enabled": true,
    "python.linting.flake8Enabled": true,
    "python.formatting.provider": "black",
    "python.sortImports.args": ["--profile", "black"]
}
```

#### PyCharm
1. Set Python interpreter to virtual environment
2. Enable code inspections
3. Configure Black formatter

### 3. Testing Setup
```bash
# Run all tests
python -m pytest

# Run with coverage
python -m pytest --cov=src --cov-report=html

# Run specific test file
python -m pytest tests/test_controller.py -v
```

## 📊 Data and Model Management

### 1. Data Directory Structure
```
data/
├── trajectories/     # Recorded robot trajectories
├── models/          # Trained ML models
└── logs/            # Training and experiment logs
```

### 2. Model Versioning
- Use descriptive model names
- Include hyperparameters in filenames
- Save training configurations with models

### 3. Experiment Tracking
```bash
# Use tensorboard for visualization
tensorboard --logdir data/logs

# Or use wandb (if configured)
wandb login
python scripts/train_agent.py --use_wandb
```

## 🚨 Troubleshooting

### Common Issues

#### 1. MuJoCo Import Error
```bash
# Error: mujoco not found
pip install mujoco>=3.0.0

# Error: GL context issues
export MUJOCO_GL=osmesa  # or egl
```

#### 2. Package Import Issues
```bash
# Error: module not found
pip install -e .

# Ensure you're in the right directory
pwd  # Should show .../NiryoArm
```

#### 3. GUI/Rendering Issues
```bash
# Linux: Install required packages
sudo apt-get install mesa-utils freeglut3-dev

# Windows: Update graphics drivers
# macOS: Ensure XQuartz is installed
```

#### 4. Performance Issues
```bash
# Use CPU-only if needed
pip uninstall torch torchvision
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### Getting Help

1. Check the [API documentation](api.md)
2. Review [tutorials](tutorials/) for examples
3. Search existing issues in the repository
4. Ask questions in discussions/forums

## 🎯 Next Steps

After successful setup:

1. **Read the [API Documentation](api.md)** to understand the codebase
2. **Try the [Basic Tutorial](tutorials/01_basic_usage.md)** 
3. **Explore the [Examples](tutorials/)** for common use cases
4. **Start developing** your own robotics applications!

## 📝 Notes

- Keep your virtual environment activated when working
- Regularly update dependencies with `pip install -U -r requirements.txt`
- Use version control for your configurations and models
- Document your experiments and findings

---

**Happy coding! 🤖**