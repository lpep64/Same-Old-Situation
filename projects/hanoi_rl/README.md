# MuJoCo Project Template

## Project Structure

```
project_name/
├── README.md              # Project documentation
├── requirements.txt       # Python dependencies
├── configs/              # Configuration files
├── controllers/          # Control algorithms and strategies
├── models/               # Custom model variants (reference official models)
├── scripts/              # Main execution scripts and utilities
└── tests/                # Unit tests and validation scripts
```

## Getting Started

1. Copy this template directory to `projects/your_project_name/`
2. Update this README.md with your project details
3. Add your dependencies to `requirements.txt`
4. Reference official models from `../../models/robots/` when possible
5. Implement your controllers in the `controllers/` directory
6. Put main execution scripts in `scripts/`

## Model References

Use relative paths to reference official models:
```python
# Reference official robot models
model_path = "../../models/robots/Arms/franka_fr3/scene.xml"
model_path = "../../models/robots/Humanoids/unitree_h1/scene.xml"
```

## Configuration Guidelines

- Store simulation parameters in `configs/`
- Use JSON or YAML for configuration files
- Separate environment configs from robot configs

## Controller Architecture

- Implement controllers as classes in `controllers/`
- Follow the pattern: `BaseController` -> `SpecificController`
- Include documentation for control parameters

## Testing

- Add unit tests in `tests/`
- Test controllers independently
- Validate model loading and basic functionality