# Hanoi Project - File Organization

This directory contains the organized MuJoCo Hanoi project files.

## Directory Structure

```
hanoi/
├── README.md                 # This file - project overview
├── controllers/              # Robot controllers and control logic
├── scripts/                  # Main simulation scripts
├── utils/                    # Utility modules and helper functions
├── xmls/                     # XML files organized by type
│   ├── scenes/              # Hand-crafted scene XML files
│   ├── generated/           # Auto-generated XML files (debug/temp)
│   └── components/          # Reusable XML components
└── __pycache__/             # Python cache files
```

## File Descriptions

### Controllers (`controllers/`)
- `panda_spinning_controller.py` - Basic Panda robot controller with spinning joints

### Scripts (`scripts/`)
- `panda_spinning_inscene.py` - Original simulation with combined scene/robot
- `panda_spinning_with_builder.py` - Simulation using the XML builder system
- `scene.py` - Basic scene setup script

### Utils (`utils/`)
- `mujoco_xml_builder.py` - Sustainable XML builder system for MuJoCo
- `test_xml_builder.py` - Test script for the XML builder

### XMLs (`xmls/`)
- `scenes/` - Store hand-crafted scene definitions
- `generated/` - Auto-generated XML files (temporary/debug)
- `components/` - Reusable XML components (floors, lights, obstacles)

## Usage Examples

### Running Simulations
```bash
# Run the XML builder-based simulation
python scripts/panda_spinning_with_builder.py

# Run the original combined simulation
python scripts/panda_spinning_inscene.py
```

### Using the XML Builder
```python
from utils.mujoco_xml_builder import create_scene_with_robot

# Create a scene with robot
builder = create_scene_with_robot("path/to/robot.xml")
xml = builder.build_xml()
```

### Testing
```bash
# Test the XML builder system
python utils/test_xml_builder.py
```

## Development Workflow

1. **Scene Development**: Create reusable scene components in `xmls/components/`
2. **Controller Development**: Add robot controllers in `controllers/`
3. **Simulation Scripts**: Combine scenes and controllers in `scripts/`
4. **Generated Files**: Auto-generated XMLs go to `xmls/generated/` (can be gitignored)

## Best Practices

- Use the XML builder system for new simulations (sustainable approach)
- Store reusable components in `xmls/components/`
- Keep controllers separate from scene definitions
- Document any custom XML components
- Use descriptive names for generated files