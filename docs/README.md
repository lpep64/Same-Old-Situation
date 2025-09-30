# MuJoCo Project Documentation

Welcome to the MuJoCo project documentation! This folder contains comprehensive guides and references for working with MuJoCo simulations.

## 📚 Documentation Index

### **Getting Started**
- [**📋 Getting Started Guide**](README_GETTING_STARTED.md) - Start here! Complete setup and testing instructions
- [**🎯 Project Overview**](PROJECT_OVERVIEW.md) - Project structure and organization guide

### **Technical Guides**
- [**🧠 Machine Learning Integration**](ML_INTEGRATION_GUIDE.md) - PyTorch, Gymnasium, and RL setup
- [**🔧 API Reference**](api_reference.md) - Detailed API documentation (coming soon)

### **Tutorials** 
- [**Basic Physics Simulation**](tutorials/basic_physics.md) - Non-GUI physics testing
- [**Interactive GUI Usage**](tutorials/gui_usage.md) - Using the interactive viewer  
- [**Reinforcement Learning**](tutorials/rl_training.md) - Training RL agents
- [**Custom Environments**](tutorials/custom_environments.md) - Creating your own simulations

### **Project Examples**
- [**📁 NiryoArm Project Template**](../projects/NiryoArm/) - Example of proper project structure

## 🚀 Quick Navigation

### **First Time Users**
1. Start with [Getting Started Guide](README_GETTING_STARTED.md)
2. Try basic demos: `python demos/gui/simple_gui.py`
3. Read [Project Overview](PROJECT_OVERVIEW.md)

### **Developers**
1. Review [ML Integration Guide](ML_INTEGRATION_GUIDE.md)
2. Check out [NiryoArm template](../projects/NiryoArm/)
3. Explore the `demos/` folder for examples

### **Researchers**
1. Focus on [RL Integration](ML_INTEGRATION_GUIDE.md#reinforcement-learning-workflow)
2. Use `demos/ml_examples/` for ML workflows
3. Review physics analysis in `demos/non_gui/advanced_physics.py`

## 📖 Documentation Structure

```
docs/
├── README.md                    # This file - documentation index
├── README_GETTING_STARTED.md    # Main getting started guide
├── PROJECT_OVERVIEW.md          # Project structure overview  
├── ML_INTEGRATION_GUIDE.md      # Machine learning setup
├── 
├── tutorials/                   # Step-by-step guides
│   ├── basic_physics.md
│   ├── gui_usage.md
│   ├── rl_training.md
│   └── custom_environments.md
├── 
├── api/                         # API documentation
│   ├── environment.md
│   ├── controllers.md
│   └── utilities.md
└── 
└── examples/                    # Code examples and snippets
    ├── basic_simulation.py
    ├── rl_training.py
    └── custom_models.py
```

## 🎯 Common Use Cases

### **Physics Testing**
```bash
# Non-GUI physics verification
python demos/non_gui/basic_tests.py

# Advanced physics analysis
python demos/non_gui/advanced_physics.py
```

### **Interactive Visualization**
```bash
# Basic GUI demo
python demos/gui/simple_gui.py

# Advanced physics playground
python demos/gui/interactive_gui.py
```

### **Machine Learning**
```bash
# Test RL environments
python demos/ml_examples/gymnasium_integration.py

# Quick RL training
python demos/ml_examples/quick_rl_training.py
```

### **Project Development**
```bash
# Use the NiryoArm template
cd projects/NiryoArm
pip install -e .
```

## 🔍 Finding Information

### **By Topic**
- **Setup & Installation** → [Getting Started Guide](README_GETTING_STARTED.md)
- **Project Structure** → [Project Overview](PROJECT_OVERVIEW.md)  
- **Machine Learning** → [ML Integration Guide](ML_INTEGRATION_GUIDE.md)
- **Specific Demos** → Check `demos/` folder README files

### **By User Type**
- **Beginners** → Start with Getting Started Guide
- **Researchers** → Focus on ML Integration and demos
- **Developers** → Review project templates and API docs

## 🛠️ Contributing to Documentation

### **Adding New Documentation**
1. Create markdown files in appropriate subfolder
2. Follow existing formatting and style
3. Update this index file with links
4. Add cross-references where helpful

### **Style Guidelines**
- Use clear, descriptive headings
- Include code examples where relevant
- Add emoji icons for visual organization
- Keep explanations concise but complete
- Test all code examples before documenting

### **Documentation Standards**
- **Format**: Markdown (.md files)
- **Style**: Clear, professional, beginner-friendly
- **Code**: Always test before including
- **Links**: Use relative paths for internal links
- **Images**: Store in `../assets/screenshots/` or `../assets/diagrams/`

## 📷 Screenshots and Media

Screenshots and diagrams are stored in:
- **Screenshots**: `../assets/screenshots/`
- **Diagrams**: `../assets/diagrams/`
- **Other Media**: `../assets/`

## 🔄 Keeping Documentation Updated

- Update documentation when adding new features
- Verify links and code examples regularly
- Keep version information current
- Archive outdated information appropriately

---

**Happy Learning! 📚🤖**

Need help? Check the specific guide for your use case or explore the `demos/` folder for hands-on examples.