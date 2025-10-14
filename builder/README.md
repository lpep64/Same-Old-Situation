# MuJoCo Scene Builder

A comprehensive toolkit for building MuJoCo XML scenes with various robots and configurations. This builder system provides an interactive interface for selecting robots from the `models/robots/` directory and generating different types of scenes.

## Features

- 🤖 **Interactive Robot Selection**: Automatically discovers and lists all available robots from `models/robots/`
- 🏗️ **Multiple Scene Types**: Empty scenes, cube stacking, manipulation setups
- 📁 **Automatic Path Management**: Handles mesh directories and output paths automatically
- ✅ **Robust Error Handling**: Clear error messages and validation
- 🎯 **Modular Design**: Easy to extend with new scene types

## Quick Start

### Basic Usage

```bash
# Navigate to the builder directory
cd builder/

# Run the interactive scene builder
python robot_scene_builder.py

# Or create a cube stacking scene directly
python build_scene.py
```

### Available Scripts

1. **`robot_scene_builder.py`** - Interactive multi-scene generator
   - Empty scenes (robot + floor)
   - Cube stacking scenes
   - Manipulation scenes with table and objects

2. **`build_scene.py`** - Cube stacking scene generator
   - Specialized for cube stacking tasks
   - Includes IK target sites for grasping

3. **`test_xml_builder.py`** - Testing and validation
   - Simple scene tests
   - Feature validation

## File Structure

```
builder/
├── __init__.py                 # Package initialization
├── mujoco_xml_builder.py      # Core XML building class
├── build_scene.py             # Cube stacking scene generator
├── robot_scene_builder.py     # Interactive scene builder
├── test_xml_builder.py        # Testing utilities
└── README.md                  # This file
```

## Supported Robot Models

The builder automatically scans `models/robots/` for available robots:

- **Arms**: Franka Panda, Kinova Gen3, KUKA iiwa, etc.
- **Humanoids**: Various humanoid robots
- **Mobile Manipulators**: Combined mobile base and arm systems
- **Quadrupeds**: Four-legged robots
- **And more**: All categories in the robots directory

## Usage Examples

### Programmatic Usage

```python
from builder import MuJoCoXMLBuilder, discover_robot_models, prompt_user_for_robot

# Discover available robots
robots = discover_robot_models("../models")

# Build a custom scene
builder = MuJoCoXMLBuilder("my_scene")
builder.set_physics_options(timestep=0.002)
builder.add_floor()
builder.add_light("main_light", pos=(0, 0, 3))

# Import a robot
builder.import_robot_from_xml(robot_path, mesh_dir)

# Save the scene
builder.save_xml("output/my_scene.xml")
```

### Interactive Selection

When you run the interactive builder, you'll see:

```
🤖 Available Robot Models:
==================================================

📁 ARMS:
  1. franka_emika_panda (panda.xml)
  2. kinova_gen3 (gen3.xml)
  3. kuka_iiwa_14 (iiwa14.xml)
  ...

📁 HUMANOIDS:
  4. atlas (atlas.xml)
  5. h1 (h1.xml)
  ...

==================================================
Select a robot (1-X) or 'q' to quit:
```

## Output

Generated XML files are saved to `xmls/generated/` with descriptive names:
- `panda_empty_scene.xml`
- `gen3_cube_stacking_scene.xml`
- `iiwa14_manipulation_scene.xml`

## Features of Generated Scenes

### Cube Stacking Scenes
- 3 colored cubes (red, green, blue) for stacking tasks
- IK target sites on robot gripper for precise control
- Stacking target sites to guide cube placement
- Optimized lighting and materials

### Manipulation Scenes
- Table with objects to manipulate
- Various geometric shapes (boxes, spheres)
- Realistic physics properties
- Good lighting for vision tasks

### Empty Scenes
- Minimal setup with robot and floor
- Perfect starting point for custom additions
- Clean, uncluttered environment

## Extending the Builder

To add new scene types:

1. Create a new function in `robot_scene_builder.py`:
```python
def create_my_scene(robot_path, mesh_dir, output_path, robot_name):
    builder = MuJoCoXMLBuilder(f"{robot_name}_my_scene")
    # Add your scene elements
    builder.save_xml(output_path)
    return output_path
```

2. Add the option to the menu in `show_scene_menu()`

3. Add the case in the main loop

## Troubleshooting

### Common Issues

1. **Robot not found**: Ensure the robot XML file exists in `models/robots/`
2. **Mesh errors**: Check that mesh files are in the expected `assets/` subdirectory
3. **Permission errors**: Make sure you have write access to the output directory

### Debug Mode

Run tests to validate the builder:
```bash
python test_xml_builder.py
```

This will create simple test scenes to verify functionality.

## Dependencies

- Python 3.6+
- Standard library only (xml.etree.ElementTree, os, pathlib, glob)
- No external dependencies required

## Contributing

When adding new features:
1. Follow the existing code style
2. Add appropriate error handling
3. Update this README if adding new scripts
4. Test with multiple robot models

---

For more information about MuJoCo XML format, see the [official documentation](https://mujoco.readthedocs.io/en/latest/XMLreference.html).