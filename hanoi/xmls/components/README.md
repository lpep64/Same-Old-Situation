# XML Components

This directory contains reusable XML components that can be imported and combined using the MuJoCo XML Builder system.

## Usage

These components are designed to be used with the `mujoco_xml_builder.py` utility to create modular, maintainable MuJoCo scenes.

## Planned Components

- `floors/` - Different floor types (textured, plain, transparent, etc.)
- `lighting/` - Lighting setups (studio, outdoor, dramatic, etc.)
- `obstacles/` - Common obstacles (boxes, spheres, walls, etc.)
- `materials/` - Material definitions (wood, metal, plastic, etc.)
- `textures/` - Texture definitions (checker, gradient, noise, etc.)

## Example Structure

```
components/
├── floors/
│   ├── textured_floor.xml
│   ├── mirror_floor.xml
│   └── grass_floor.xml
├── lighting/
│   ├── studio_lighting.xml
│   ├── outdoor_lighting.xml
│   └── dramatic_lighting.xml
└── obstacles/
    ├── box_obstacle.xml
    ├── wall_obstacle.xml
    └── hanoi_tower.xml
```

## Component Guidelines

1. **Modularity**: Each component should be self-contained
2. **Naming**: Use descriptive names with consistent conventions
3. **Documentation**: Include comments in XML explaining the component
4. **Parameterization**: Design for easy customization through the builder system