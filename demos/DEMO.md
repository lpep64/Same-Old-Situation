# MuJoCo Demo Guide

This guide provides instructions for running MuJoCo demonstrations and tests to verify your installation and explore the physics simulation capabilities.

## 🧪 Non-GUI Tests

### Basic Physics Tests
Run the comprehensive basic test suite to verify core MuJoCo functionality:

```bash
cd demos/non_gui
python basic_tests.py
```

**Expected Output:**
```
============================================================
🧪 MUJOCO NON-GUI TEST SUITE
============================================================
🔬 Running Basic Physics Test...
   ✅ Basic physics test PASSED

🔬 Running Multiple Objects Test...
   ✅ Multiple objects test PASSED

🔬 Running Model Loading Test...
   ✅ Model loading test PASSED

🔬 Running Performance Benchmark...
   ✅ Performance test PASSED

🎯 Overall: 4/4 tests passed
🎉 All tests PASSED! MuJoCo is working perfectly.
```

### Advanced Physics Analysis
Run advanced physics simulations including pendulum analysis, chaos theory, and collision detection:

```bash
cd demos/non_gui
python advanced_physics.py
```

**Expected Output:**
```
======================================================================
🧪 MUJOCO ADVANCED NON-GUI PHYSICS ANALYSIS
======================================================================
🔬 Analyzing Pendulum Motion...
   ✅ Energy conservation: GOOD

🔬 Chaos Sensitivity Analysis (Double Pendulum)...
   ✅ Chaos sensitivity: CONFIRMED

🔬 Batch Collision Testing...
   ✅ Collision detection: WORKING

🎉 All advanced physics tests PASSED!
   MuJoCo is performing accurate physics simulations.
```

## 🖥️ GUI Tests

### Simple GUI Demo
Launch a basic MuJoCo viewer with a simple falling ball simulation:

```bash
cd demos/gui
python simple_gui.py
```

**What you'll see:**
- A 3D viewer window opens
- A red ball falls and bounces on a gray platform
- Real-time physics simulation with visual feedback
- Mouse controls for camera movement
- Press `ESC` to exit

**Controls:**
- **Mouse drag**: Rotate camera
- **Mouse wheel**: Zoom in/out
- **Right mouse + drag**: Pan camera
- **ESC**: Exit the simulation

### Interactive GUI Demo
Run a sophisticated interactive physics playground with multiple objects and materials:

```bash
cd demos/gui
python interactive_gui.py
```

**What you'll see:**
- A comprehensive 3D physics playground
- 9 different objects (spheres, boxes, capsules) with various materials
- Textured ground plane and boundary walls
- Interactive platforms and obstacles
- Real-time performance monitoring and FPS counter
- Enhanced graphics with lighting and materials

**Controls:**
- **Mouse drag**: Rotate camera
- **Mouse wheel**: Zoom in/out
- **Right mouse + drag**: Pan camera
- **SPACE**: Pause/Resume simulation
- **R**: Reset simulation
- **ESC**: Exit the simulation

## 🛠️ Troubleshooting

### Common Issues

1. **ImportError: No module named 'mujoco'**
   ```bash
   pip install mujoco
   ```

2. **GUI not opening**
   - Ensure you have a display (not running in headless mode)
   - Check if OpenGL drivers are installed
   - Try running from a terminal with GUI support

3. **Model files not found**
   - Make sure you're running from the correct directory
   - Check that model files exist in the `model/` directory

4. **Performance issues**
   - Close other applications
   - Check if hardware acceleration is available
   - Reduce simulation complexity if needed

### System Requirements

- **Python**: 3.8 or higher
- **OpenGL**: 3.3 or higher (for GUI)
- **Memory**: 2GB RAM minimum
- **Display**: Required for GUI demos

## 📊 Test Results Interpretation

### Non-GUI Tests
- **Basic Physics**: Verifies gravity, collision, and basic dynamics
- **Multiple Objects**: Tests complex multi-body interactions
- **Model Loading**: Validates XML model parsing and loading
- **Performance**: Measures simulation speed (target: >1000 steps/sec)

### Advanced Physics
- **Energy Conservation**: Should be <5% variation for good results
- **Chaos Sensitivity**: Double pendulum should show divergent behavior
- **Collision Detection**: All collision scenarios should be detected

### GUI Tests
- **Visual Rendering**: Smooth 60 FPS target
- **Interactive Response**: Real-time mouse/keyboard input
- **Physics Display**: Accurate visual representation of simulation

## 🚀 Next Steps

After running these demos successfully:

1. **Explore Models**: Check out the `model/` directory for more complex examples
2. **Read Documentation**: Visit the `doc/` directory for detailed guides
3. **Try Examples**: Look in `demos/ml_examples/` for machine learning applications
4. **Build Custom**: Create your own XML models and simulations

## 📁 Demo Structure

```
demos/
├── DEMO.md              # This guide
├── non_gui/
│   ├── basic_tests.py   # Basic functionality tests
│   └── advanced_physics.py # Advanced physics analysis
└── gui/
    ├── simple_gui.py    # Basic GUI viewer
    └── interactive_gui.py # Interactive demonstrations
```

Enjoy exploring MuJoCo physics simulations! 🎯
