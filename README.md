# MuJoCo Development Framework# MuJoCo Development Framework<h1>



**A comprehensive, optimized MuJoCo framework for physics simulation, robotics research, and machine learning applications.**  <a href="#"><img alt="MuJoCo" src="banner.png" width="100%"/></a>



[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)**A comprehensive, optimized MuJoCo framework for physics simulation, robotics research, and machine learning applications.**</h1>

[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://python.org)

[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.0+-orange.svg)](https://mujoco.org)



## 🚀 Quick Start[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)<p>



```bash[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://python.org)  <a href="https://github.com/google-deepmind/mujoco/actions/workflows/build.yml?query=branch%3Amain" alt="GitHub Actions">

# Test basic functionality

python examples/basic/basic_tests.py[![MuJoCo](https://img.shields.io/badge/MuJoCo-3.0+-orange.svg)](https://mujoco.org)    <img src="https://img.shields.io/github/actions/workflow/status/google-deepmind/mujoco/build.yml?branch=main">



# Try interactive GUI demo  </a>

python examples/gui/simple_gui.py

## 🚀 Quick Start  <a href="https://mujoco.readthedocs.io/" alt="Documentation">

# List available robot models

python examples/tools/download_models.py list    <img src="https://readthedocs.org/projects/mujoco/badge/?version=latest">



# Download specific models as needed```bash  </a>

python examples/tools/download_models.py model boston_dynamics_spot

```# Test basic functionality  <a href="https://github.com/google-deepmind/mujoco/blob/main/LICENSE" alt="License">



## 📁 Project Structurepython examples/basic/basic_tests.py    <img src="https://img.shields.io/github/license/google-deepmind/mujoco">



```  </a>

mujoco/                              # Main project directory

├── examples/                        # 🎯 All demos & examples# Try interactive GUI demo</p>

│   ├── basic/                      # Physics tests & validation

│   ├── gui/                        # Interactive demonstrationspython examples/gui/simple_gui.py

│   ├── ml/                         # Machine learning examples

│   ├── robotics/                   # Robot-specific code**MuJoCo** stands for **Mu**lti-**Jo**int dynamics with **Co**ntact. It is a

│   ├── playground/                 # Experimental projects

│   └── tools/                      # Utility scripts (download_models.py)# List available robot modelsgeneral purpose physics engine that aims to facilitate research and development

├── models/                         # 🤖 Essential models only

│   ├── basic/                      # Core test models (cube, humanoid, car, etc.)python models/download_models.py listin robotics, biomechanics, graphics and animation, machine learning, and other

│   └── robots/                     # Key robots (Franka, UR5e, Unitree Go1)

├── projects/                       # 🏗️ User projectsareas which demand fast and accurate simulation of articulated structures

│   └── NiryoArm/                   # Example robotics project

├── src/                            # MuJoCo source code# Download specific models as neededinteracting with their environment.

├── python/                         # Python bindings

├── doc/                            # Official documentationpython models/download_models.py model boston_dynamics_spot

└── [other core MuJoCo files]

``````## 🚀 Unified MuJoCo Project Structure



## 🎯 Examples & Demos



### Basic Physics Testing## 📁 Project StructureThis repository now includes a comprehensive collection of MuJoCo-related content:

```bash

cd examples/basic

python basic_tests.py              # Comprehensive test suite

python advanced_physics.py         # Advanced physics analysis```### 📁 **Core MuJoCo**

```

mujoco/                              # Main project directory- Main physics engine and core functionality

### Interactive GUI Demos

```bash├── examples/                        # 🎯 All demos & examples- C++ source code, Python bindings, and documentation

cd examples/gui

python simple_gui.py               # Basic physics demo│   ├── basic/                      # Physics tests & validation- Build system and development tools

python interactive_gui.py          # Advanced physics playground

```│   ├── gui/                        # Interactive demonstrations



### Machine Learning Integration│   ├── ml/                         # Machine learning examples### 📁 **examples/** - Learning Examples

```bash

cd examples/ml│   └── robotics/                   # Robot-specific code- `gym_legacy/` - OpenAI Gym environments (legacy)

python gymnasium_integration.py    # Modern RL environment setup

python quick_rl_training.py        # Fast RL training demo├── models/                         # 🤖 Essential models only- `reinforcement_learning/` - Custom RL implementations and models

python ant.py                      # Ant robot RL environment

```│   ├── basic/                      # Core test models (cube, humanoid, car)



### Utility Tools│   ├── robots/                     # Key robots (Franka, UR5e, Unitree Go1)### 📁 **models/** - Robot Model Collection  

```bash

cd examples/tools├── examples/                        # 🎯 All demos & examples

python download_models.py list     # List all available robot models│   └── tools/                      # Utility scripts (download_models.py)- `menagerie/` - High-quality curated robot models from Google DeepMind

python download_models.py model shadow_hand  # Download specific models

```├── projects/                       # 🏗️ User projects- Includes 60+ robot models: arms, hands, quadrupeds, humanoids, sensors



## 🤖 Robot Models│   └── NiryoArm/                   # Example robotics project



### Included Models (Essential Set)├── src/                            # MuJoCo source code### 📁 **playground/** - GPU-Accelerated Environments

- **Basic Test Models**: cube, humanoid, car, and 10+ other test models (`models/basic/`)

- **Franka FR3**: Research robot arm (`models/robots/franka_fr3/`)├── python/                         # Python bindings- MJX-based environments for high-performance simulation

- **Franka Panda**: Collaborative robot (`models/robots/franka_emika_panda/`)

- **Universal Robots UR5e**: Industrial robot (`models/robots/universal_robots_ur5e/`)├── doc/                            # Official documentation- Locomotion, manipulation, and classic control tasks

- **Unitree Go1**: Quadruped robot (`models/robots/unitree_go1/`)

└── [other core MuJoCo files]- Research-ready training environments

### Download Additional Models

```bash```

# List all available models (40+ robots)

python examples/tools/download_models.py list### 📁 **projects/** - Custom Projects



# Download by category## 🎯 Examples & Demos- Your custom implementations and experiments

python examples/tools/download_models.py category humanoids    # All humanoid robots

python examples/tools/download_models.py category arms         # All robot arms- Project-specific models and configurations

python examples/tools/download_models.py category quadrupeds   # All four-legged robots

### Basic Physics Testing

# Download specific models

python examples/tools/download_models.py model boston_dynamics_spot```bash## 🎯 Quick Start Guide

python examples/tools/download_models.py model shadow_hand

python examples/tools/download_models.py model kuka_iiwa_14cd examples/basic

```

python basic_tests.py              # Comprehensive test suite### Basic Simulation

**Available Categories**: humanoids, quadrupeds, arms, hands, mobile, special

python advanced_physics.py         # Advanced physics analysis```python

## 🏗️ Project Template (NiryoArm Example)

```import mujoco

The `projects/NiryoArm/` directory demonstrates best practices for robotics projects:

import mujoco.viewer

```

NiryoArm/### Interactive GUI Demos

├── src/                            # Source code

│   ├── arm_controller.py          # Robot control logic```bash# Load a model from the menagerie

│   ├── environment.py             # MuJoCo environment wrapper

│   └── tasks/                     # Manipulation taskscd examples/guimodel = mujoco.MjModel.from_xml_path('models/menagerie/franka_emika_panda/panda.xml')

├── models/                        # MuJoCo XML models

│   ├── niryo_arm.xml             # Basic geometric model (fast)python simple_gui.py               # Basic physics demodata = mujoco.MjData(model)

│   └── niryo_arm_with_meshes.xml # Detailed STL model (realistic)

├── assets/                        # Project resourcespython interactive_gui.py          # Advanced physics playground

│   ├── meshes/niryo/             # STL/STEP files from Niryo

│   ├── images/                   # Screenshots, diagrams```# Launch interactive viewer

│   └── textures/                 # Texture files

├── scripts/                       # Utility scriptsmujoco.viewer.launch(model, data)

│   ├── gui_test.py               # Interactive model viewer

│   └── test_models.py            # Model validation### Machine Learning Integration```

├── config/                        # Configuration files

├── tests/                         # Unit tests```bash

└── docs/                         # Project documentation

```cd examples/ml### Reinforcement Learning



### Using the NiryoArm Projectpython gymnasium_integration.py    # Modern RL environment setup```python

```bash

cd projects/NiryoArmpython quick_rl_training.py        # Fast RL training demo# Using Playground environments



# Test both modelspython ant.py                      # Ant robot RL environmentimport playground

python scripts/test_models.py

```env = playground.make('dm_control_suite:cartpole-swingup')

# Interactive GUI test

python scripts/gui_test.py

```

### Robot-Specific Examples# Using legacy Gym environments  

## 📊 Optimization Results

```bashimport gym

This repository has been optimized for efficiency:

cd examples/roboticsenv = gym.make('Ant-v4')

### Space Savings

- **Before**: 2,101MB (2.1GB)# Various robot XML models and control examples```

- **After**: 466MB

- **Reduction**: 78% smaller (1.6GB saved!)```



### Improvements Made### Custom Projects

✅ **Consolidated Structure**: Merged demos/examples into logical organization  

✅ **Smart Model Management**: Essential models included, others available on-demand  ## 🤖 Robot ModelsSee `examples/` and `projects/` directories for comprehensive examples and tutorials.

✅ **Eliminated Redundancy**: Removed duplicate structures and merged `model/` and `models/`  

✅ **Better Navigation**: Clear separation by functionality  

✅ **Single Documentation**: One comprehensive README instead of 9 markdown files  

### Included Models (Essential Set)This repository is maintained by [Google DeepMind](https://www.deepmind.com/).

## 🛠️ Installation & Setup

- **Franka FR3**: Research robot arm (`models/robots/franka_fr3/`)

### Prerequisites

- Python 3.8+- **Franka Panda**: Collaborative robot (`models/robots/franka_emika_panda/`)MuJoCo has a C API and is intended for researchers and developers. The runtime

- MuJoCo 3.0+

- Git (for model downloads)- **Universal Robots UR5e**: Industrial robot (`models/robots/universal_robots_ur5e/`)simulation module is tuned to maximize performance and operates on low-level



### Quick Installation- **Unitree Go1**: Quadruped robot (`models/robots/unitree_go1/`)data structures that are preallocated by the built-in XML compiler. The library

```bash

# Clone repositoryincludes interactive visualization with a native GUI, rendered in OpenGL. MuJoCo

git clone <repository-url>

cd mujoco### Download Additional Modelsfurther exposes a large number of utility functions for computing



# Install dependencies```bashphysics-related quantities.

pip install mujoco gymnasium numpy

# List all available models (40+ robots)

# Test installation

python examples/basic/basic_tests.pypython models/download_models.py listWe also provide [Python bindings] and a plug-in for the [Unity] game engine.

```



### Development Setup

```bash# Download by category## Documentation

# Install development dependencies

pip install -e .python models/download_models.py category humanoids    # All humanoid robots



# Run comprehensive testspython models/download_models.py category arms         # All robot armsMuJoCo's documentation can be found at [mujoco.readthedocs.io]. Upcoming

python examples/basic/basic_tests.py

python examples/gui/simple_gui.pypython models/download_models.py category quadrupeds   # All four-legged robotsfeatures due for the next release can be found in the [changelog] in the

```

"latest" branch.

## 🔧 Usage Patterns

# Download specific models

### For Learning & Experimentation

1. Start with `examples/basic/` for physics fundamentalspython models/download_models.py model boston_dynamics_spot## Getting Started

2. Try `examples/gui/` for interactive exploration

3. Explore `examples/ml/` for AI/ML applicationspython models/download_models.py model shadow_hand

4. Use `examples/tools/` for downloading additional models

python models/download_models.py model kuka_iiwa_14There are two easy ways to get started with MuJoCo:

### For Research & Development

1. Use `projects/NiryoArm/` as a template```

2. Download specific models for your research using `examples/tools/download_models.py`

3. Build on the organized structure1. **Run `simulate` on your machine.**



### For Production Applications**Available Categories**: humanoids, quadrupeds, arms, hands, mobile, special[This video](https://www.youtube.com/watch?v=P83tKA1iz2Y) shows a screen capture

1. Use essential models for performance

2. Download additional models only as neededof `simulate`, MuJoCo's native interactive viewer. Follow the steps described in

3. Follow the project template structure

## 🏗️ Project Template (NiryoArm Example)the [Getting Started] section of the documentation to get `simulate` running on

## 📚 Documentation & Resources

your machine.

### Getting Started

- Run `python examples/basic/basic_tests.py` for validationThe `projects/NiryoArm/` directory demonstrates best practices for robotics projects:

- Check `examples/` subdirectories for specific use cases

- Review `projects/NiryoArm/` for project structure example2. **Explore our online IPython notebooks.**



### Official MuJoCo Resources```If you are a Python user, you might want to start with our tutorial notebooks

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)

- [MuJoCo Menagerie](https://github.com/deepmind/mujoco_menagerie) - Full model collectionNiryoArm/running on Google Colab:

- [Gymnasium Documentation](https://gymnasium.farama.org/) - RL environments

├── src/                            # Source code

### Troubleshooting

- **Import Errors**: Ensure MuJoCo and dependencies are installed│   ├── arm_controller.py          # Robot control logic - The **introductory** tutorial teaches MuJoCo basics:

- **Display Issues**: Set `MUJOCO_GL=egl` for headless rendering

- **Model Loading**: Check file paths and download missing models using `examples/tools/download_models.py`│   ├── environment.py             # MuJoCo environment wrapper   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/tutorial.ipynb)

- **Performance**: Use basic geometric models for faster simulation

│   └── tasks/                     # Manipulation tasks - The **Model Editing** tutorial shows how to create and edit models procedurally:

## 🤝 Contributing

├── models/                        # MuJoCo XML models   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/mjspec.ipynb)

1. Follow the organized directory structure

2. Add examples to appropriate subdirectories (`basic/`, `gui/`, `ml/`, `robotics/`)│   ├── niryo_arm.xml             # Basic geometric model (fast) - The **rollout** tutorial shows how to use the multithreaded `rollout` module:

3. Use `examples/tools/download_models.py` for additional robots

4. Document large assets before adding│   └── niryo_arm_with_meshes.xml # Detailed STL model (realistic)   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/rollout.ipynb)



## 📄 License├── assets/                        # Project resources - The **LQR** tutorial synthesizes a linear-quadratic controller, balancing a



This project includes:│   ├── meshes/niryo/             # STL/STEP files from Niryo   humanoid on one leg:

- **MuJoCo**: Apache 2.0 License

- **Niryo Models**: CC0 1.0 Universal (Public Domain)│   ├── images/                   # Screenshots, diagrams   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/LQR.ipynb)

- **Project Structure**: Apache 2.0 License

│   └── textures/                 # Texture files - The **least-squares** tutorial explains how to use the Python-based nonlinear

## ⚡ Performance Notes

├── scripts/                       # Utility scripts   least-squares solver:

- **Basic Models**: Use for fast prototyping and RL training

- **Detailed Models**: Use for realistic visualization and validation│   ├── gui_test.py               # Interactive model viewer   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/python/least_squares.ipynb)

- **Download Script**: Only get models you actually need

- **Optimized Structure**: 78% smaller repository for faster operations│   └── test_models.py            # Model validation - The **MJX** tutorial provides usage examples of



---├── config/                        # Configuration files   [MuJoCo XLA](https://mujoco.readthedocs.io/en/stable/mjx.html), a branch of MuJoCo written in JAX:



**Ready to explore physics simulation with MuJoCo!** 🚀├── tests/                         # Unit tests   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/tutorial.ipynb)



Start with `python examples/basic/basic_tests.py` to verify your setup, then explore the examples and create your own robotics projects.└── docs/                         # Project documentation - The **differentiable physics** tutorial trains locomotion policies with

```   analytical gradients automatically derived from MuJoCo's physics step:

   [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/google-deepmind/mujoco/blob/main/mjx/training_apg.ipynb)

### Using the NiryoArm Project

```bash## Installation

cd projects/NiryoArm

### Prebuilt binaries

# Test both models

python scripts/test_models.pyVersioned releases are available as precompiled binaries from the GitHub

[releases page], built for Linux (x86-64 and AArch64), Windows (x86-64 only),

# Interactive GUI testand macOS (universal). This is the recommended way to use the software.

python scripts/gui_test.py

### Building from source

# View setup documentation

cat docs/setup.mdUsers who wish to build MuJoCo from source should consult the [build from

```source] section of the documentation. However, note that the commit at

the tip of the `main` branch may be unstable.

## 📊 Optimization Results

### Python (>= 3.9)

This repository has been optimized for efficiency:

The native Python bindings, which come pre-packaged with a copy of MuJoCo, can

### Space Savingsbe installed from [PyPI] via:

- **Before**: 2,101MB (2.1GB)

- **After**: 466MB```bash

- **Reduction**: 78% smaller (1.6GB saved!)pip install mujoco

```

### Improvements Made

✅ **Consolidated Structure**: Merged demos/examples into logical organization  Note that Pre-built Linux wheels target `manylinux2014`, see

✅ **Smart Model Management**: Essential models included, others available on-demand  [here](https://github.com/pypa/manylinux) for compatible distributions. For more

✅ **Eliminated Redundancy**: Removed duplicate structures  information such as building the bindings from source, see the [Python bindings]

✅ **Better Navigation**: Clear separation by functionality  section of the documentation.



## 🛠️ Installation & Setup## Contributing



### PrerequisitesWe welcome community engagement: questions, requests for help, bug reports and

- Python 3.8+feature requests. To read more about bug reports, feature requests and more

- MuJoCo 3.0+ambitious contributions, please see our [contributors guide](CONTRIBUTING.md)

- Git (for model downloads)and [style guide](STYLEGUIDE.md).



### Quick Installation## Asking Questions

```bash

# Clone repositoryQuestions and requests for help are welcome as a GitHub

git clone <repository-url>["Asking for Help" Discussion](https://github.com/google-deepmind/mujoco/discussions/categories/asking-for-help)

cd mujocoand should focus on a specific problem or question.



# Install dependencies## Bug reports and feature requests

pip install mujoco gymnasium numpy

GitHub [Issues](https://github.com/google-deepmind/mujoco/issues) are reserved

# Test installationfor bug reports, feature requests and other development-related subjects.

python examples/basic/basic_tests.py

```## Related software

MuJoCo is the backbone for numerous environment packages. Below we list several

### Development Setupbindings and converters.

```bash

# Install development dependencies### Bindings

pip install -e .

These packages give users of various languages access to MuJoCo functionality:

# Run comprehensive tests

python examples/basic/basic_tests.py#### First-party bindings:

python examples/gui/simple_gui.py

```- [Python bindings](https://mujoco.readthedocs.io/en/stable/python.html)

  - [dm_control](https://github.com/google-deepmind/dm_control), Google

## 🔧 Usage Patterns    DeepMind's related environment stack, includes

    [PyMJCF](https://github.com/google-deepmind/dm_control/blob/main/dm_control/mjcf/README.md),

### For Learning & Experimentation    a module for procedural manipulation of MuJoCo models.

1. Start with `examples/basic/` for physics fundamentals- [C# bindings and Unity plug-in](https://mujoco.readthedocs.io/en/stable/unity.html)

2. Try `examples/gui/` for interactive exploration

3. Explore `examples/ml/` for AI/ML applications#### Third-party bindings:



### For Research & Development- **WebAssembly**: [mujoco_wasm](https://github.com/zalo/mujoco_wasm) by [@zalo](https://github.com/zalo) with contributions by

1. Use `projects/NiryoArm/` as a template  [@kevinzakka](https://github.com/kevinzakka), based on the [emscripten build](https://github.com/stillonearth/MuJoCo-WASM) by

2. Download specific models for your research  [@stillonearth](https://github.com/stillonearth).

3. Build on the organized structure

  :arrow_right: [Click here](https://zalo.github.io/mujoco_wasm/) for a live demo of MuJoCo running in your browser.

### For Production Applications- **MATLAB Simulink**: [Simulink Blockset for MuJoCo Simulator](https://github.com/mathworks-robotics/mujoco-simulink-blockset)

1. Use essential models for performance  by [Manoj Velmurugan](https://github.com/vmanoj1996).

2. Download additional models only as needed- **Swift**: [swift-mujoco](https://github.com/liuliu/swift-mujoco)

3. Follow the project template structure- **Java**: [mujoco-java](https://github.com/CommonWealthRobotics/mujoco-java)

- **Julia**: [MuJoCo.jl](https://github.com/JamieMair/MuJoCo.jl)

## 📚 Documentation & Resources

### Converters

### Getting Started

- Run `python examples/basic/basic_tests.py` for validation- **OpenSim**: [MyoConverter](https://github.com/MyoHub/myoconverter) converts

- Check `examples/` subdirectories for specific use cases  OpenSim models to MJCF.

- Review `projects/NiryoArm/` for project structure example- **SDFormat**: [gz-mujoco](https://github.com/gazebosim/gz-mujoco/) is a

  two-way SDFormat <-> MJCF conversion tool.

### Official MuJoCo Resources- **OBJ**: [obj2mjcf](https://github.com/kevinzakka/obj2mjcf)

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)  a script for converting composite OBJ files into a loadable MJCF model.

- [MuJoCo Menagerie](https://github.com/deepmind/mujoco_menagerie) - Full model collection- **onshape**: [Onshape to Robot](https://github.com/rhoban/onshape-to-robot)

- [Gymnasium Documentation](https://gymnasium.farama.org/) - RL environments  Converts [onshape](https://www.onshape.com/en/) CAD assemblies to MJCF.



### Troubleshooting## Citation

- **Import Errors**: Ensure MuJoCo and dependencies are installed

- **Display Issues**: Set `MUJOCO_GL=egl` for headless renderingIf you use MuJoCo for published research, please cite:

- **Model Loading**: Check file paths and download missing models

- **Performance**: Use basic geometric models for faster simulation```

@inproceedings{todorov2012mujoco,

## 🤝 Contributing  title={MuJoCo: A physics engine for model-based control},

  author={Todorov, Emanuel and Erez, Tom and Tassa, Yuval},

1. Follow the organized directory structure  booktitle={2012 IEEE/RSJ International Conference on Intelligent Robots and Systems},

2. Add examples to appropriate subdirectories (`basic/`, `gui/`, `ml/`, `robotics/`)  pages={5026--5033},

3. Document large assets before adding  year={2012},

4. Use the model download script for additional robots  organization={IEEE},

  doi={10.1109/IROS.2012.6386109}

## 📄 License}

```

This project includes:

- **MuJoCo**: Apache 2.0 License## License and Disclaimer

- **Niryo Models**: CC0 1.0 Universal (Public Domain)

- **Project Structure**: Apache 2.0 LicenseCopyright 2021 DeepMind Technologies Limited.



## ⚡ Performance NotesBox collision code ([`engine_collision_box.c`](https://github.com/google-deepmind/mujoco/blob/main/src/engine/engine_collision_box.c))

is Copyright 2016 Svetoslav Kolev.

- **Basic Models**: Use for fast prototyping and RL training

- **Detailed Models**: Use for realistic visualization and validationReStructuredText documents, images, and videos in the `doc` directory are made

- **Download Script**: Only get models you actually needavailable under the terms of the Creative Commons Attribution 4.0 (CC BY 4.0)

- **Optimized Structure**: 78% smaller repository for faster operationslicense. You may obtain a copy of the License at

https://creativecommons.org/licenses/by/4.0/legalcode.

---

Source code is licensed under the Apache License, Version 2.0. You may obtain a

**Ready to explore physics simulation with MuJoCo!** 🚀copy of the License at https://www.apache.org/licenses/LICENSE-2.0.



Start with `python examples/basic/basic_tests.py` to verify your setup, then explore the examples and create your own robotics projects.This is not an officially supported Google product.

[build from source]: https://mujoco.readthedocs.io/en/latest/programming#building-mujoco-from-source
[Getting Started]: https://mujoco.readthedocs.io/en/latest/programming#getting-started
[Unity]: https://unity.com/
[releases page]: https://github.com/google-deepmind/mujoco/releases
[mujoco.readthedocs.io]: https://mujoco.readthedocs.io
[changelog]: https://mujoco.readthedocs.io/en/latest/changelog.html
[Python bindings]: https://mujoco.readthedocs.io/en/stable/python.html#python-bindings
[PyPI]: https://pypi.org/project/mujoco/
