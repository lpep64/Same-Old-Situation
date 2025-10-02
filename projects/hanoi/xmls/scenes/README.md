# Scene XML Files

This directory contains complete scene definitions that can be used directly with MuJoCo or as templates for the XML builder system.

## Usage

Hand-crafted scene files that define complete environments. These can be:
1. Used directly with MuJoCo
2. Parsed by the XML builder as templates
3. Modified and extended for specific scenarios

## Planned Scenes

- `basic_hanoi_scene.xml` - Simple scene with table and towers for Hanoi puzzle
- `laboratory_scene.xml` - Clean laboratory environment
- `workshop_scene.xml` - Industrial workshop setting
- `outdoor_scene.xml` - Outdoor environment with terrain

## Scene Guidelines

1. **Complete**: Each scene should be a fully functional MuJoCo model
2. **Documented**: Include XML comments explaining key elements
3. **Modular**: Design with reusable components in mind
4. **Realistic**: Use appropriate physics parameters and materials