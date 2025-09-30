# MuJoCo Getting Started Guide

Welcome to MuJoCo! This guide will help you get started with both non-GUI testing and GUI simulations, plus advanced machine learning integration.

## 📋 Prerequisites

- Python 3.7+ installed
- MuJoCo package installed (`pip install mujoco`)
- Windows PowerShell

## 🚀 Quick Verification

First, verify MuJoCo is working:

```powershell
python -c "import mujoco; print('MuJoCo version:', mujoco.__version__)"
```

## 📁 Project Structure

```
mujoco/
├── README_GETTING_STARTED.md  # This file
├── demos/                     # Demo applications
│   ├── non_gui/              # Headless simulations
│   └── gui/                  # Interactive GUI demos
├── model/                    # MuJoCo XML models
├── python/                   # Python bindings and examples
└── ...
```

## 🔧 Non-GUI Testing

Non-GUI tests run physics simulations without visual output - perfect for automated testing, batch processing, and headless servers.

### Basic Non-GUI Test

```powershell
# Navigate to project directory
cd C:\Users\lukep\Documents\mujoco

# Run basic physics test
python -c "
import mujoco
import numpy as np

# Create simple physics scene
xml = '''
<mujoco>
  <worldbody>
    <body pos='0 0 2'>
      <joint type='free'/>
      <geom size='0.1' mass='1'/>
    </body>
    <geom pos='0 0 0' size='2 2 0.1' type='box'/>
  </worldbody>
</mujoco>
'''

# Load and simulate
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

print('Starting non-GUI simulation...')
for i in range(1000):
    mujoco.mj_step(model, data)
    if i % 200 == 0:
        height = data.xpos[1][2]
        print(f'Step {i}: Ball height = {height:.3f}m')

print('Non-GUI test completed!')
"
```

### Advanced Non-GUI Tests

```powershell
# Test with built-in models
python -c "
import mujoco
import os

# Test humanoid model
if os.path.exists('model/humanoid/humanoid.xml'):
    model = mujoco.MjModel.from_xml_path('model/humanoid/humanoid.xml')
    data = mujoco.MjData(model)
    
    print(f'Humanoid model: {model.nbody} bodies, {model.nv} DOFs')
    
    # Run simulation
    for i in range(500):
        mujoco.mj_step(model, data)
    
    print('Humanoid simulation completed successfully!')
else:
    print('Humanoid model not found')
"
```

## 🖥️ GUI Testing

GUI tests provide real-time visual feedback and interaction capabilities.

### Prerequisites for GUI

Install required dependencies:

```powershell
pip install glfw PyOpenGL
```

### Basic GUI Test

```powershell
# Run simple GUI demo
python demos/gui/simple_gui.py
```

**Controls:**
- **Left Mouse + Drag:** Rotate camera
- **Right Mouse + Drag:** Pan camera  
- **Scroll Wheel:** Zoom in/out
- **ESC:** Exit

### Advanced GUI Tests

```powershell
# Interactive physics playground
python demos/gui/interactive_gui.py

# Feature-rich demo with multiple scenes
python demos/gui/mujoco_gui_demo.py
```

## 🧪 Running Test Suites

### Python Binding Tests

```powershell
# Install test dependencies
pip install pytest absl-py

# Run from outside the project directory to avoid import conflicts
cd $env:TEMP
python -c "
import mujoco

# Quick functionality test
xml = '<mujoco><worldbody><body><joint type=\"free\"/><geom size=\"0.1\"/></body></worldbody></mujoco>'
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_step(model, data)
print('✅ MuJoCo bindings test passed!')
"
```

### Model Validation Tests

```powershell
# Test all available models
python -c "
import mujoco
import os
import glob

model_files = glob.glob('C:/Users/lukep/Documents/mujoco/model/**/*.xml', recursive=True)
print(f'Found {len(model_files)} model files')

for model_file in model_files[:5]:  # Test first 5 models
    try:
        model = mujoco.MjModel.from_xml_path(model_file)
        print(f'✅ {os.path.basename(model_file)}: {model.nbody} bodies')
    except Exception as e:
        print(f'❌ {os.path.basename(model_file)}: {str(e)[:50]}...')
"
```

## 🔍 Troubleshooting

### Common Issues

1. **Import Error:** Make sure you're not in the `python/` directory when importing mujoco
2. **GUI Not Opening:** Install `pip install glfw PyOpenGL`
3. **Performance Issues:** Try reducing timestep or number of objects
4. **Model Not Found:** Check file paths are correct

### Debug Commands

```powershell
# Check MuJoCo installation
python -c "import mujoco; print('Install path:', mujoco.__file__)"

# Check dependencies  
python -c "
try:
    import glfw; print('✅ GLFW available')
except: print('❌ GLFW missing')
    
try:
    import OpenGL; print('✅ OpenGL available') 
except: print('❌ OpenGL missing')
"

# Test model loading
python -c "
import mujoco
try:
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody/></mujoco>')
    print('✅ Model loading works')
except Exception as e:
    print(f'❌ Model loading failed: {e}')
"
```

## 📈 Performance Tips

1. **Non-GUI Simulations:** Remove `time.sleep()` calls for maximum speed
2. **GUI Simulations:** Adjust timestep in XML (`<option timestep="0.01"/>`)
3. **Large Models:** Use `<option gravity="0 0 0"/>` to disable physics temporarily
4. **Batch Testing:** Use multiprocessing for parallel simulations

## 🎯 Next Steps

1. Explore the `model/` directory for example scenes
2. Try modifying XML models to create custom simulations
3. Check the `python/` directory for Jupyter notebooks
4. Install machine learning packages (PowerGym, PyTorch) for RL applications

## 📚 Resources

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Python API Reference](https://mujoco.readthedocs.io/en/latest/python.html)
- [XML Model Reference](https://mujoco.readthedocs.io/en/latest/XMLreference.html)
- [GitHub Repository](https://github.com/google-deepmind/mujoco)

---

**Happy Simulating! 🚀**