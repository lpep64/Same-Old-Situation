# Contributing to MuJoCo-Testing

Thank you for your interest in contributing to MuJoCo-Testing! This document provides guidelines for participation in the project.

## Development Environment Setup

### Prerequisites
- Python 3.8 or higher
- Git

### Setting Up

```bash
# Clone the repository
git clone https://github.com/URI-ISE/MuJoCo-Testing.git
cd MuJoCo-Testing

# Create a virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install in development mode with all dependencies
pip install -e ".[dev]"
```

## Running Tests

```bash
# Run all tests
pytest

# Run tests in a specific directory
pytest examples/basic/

# Run with verbose output
pytest -v

# Run with coverage report
pytest --cov=examples --cov=projects
```

## Code Style

### Formatting
We use `black` for consistent code formatting:

```bash
# Format all Python files
black demos/ tests/ models/

# Check formatting without modifying
black --check demos/ tests/ models/
```

### Python Style Guide
- Follow PEP 8 conventions
- Use descriptive variable and function names
- Add docstrings to functions and classes
- Keep lines to a reasonable length (80-100 characters)

### Example
```python
def calculate_dynamics(state, action, dt=0.01):
    """
    Calculate next state given current state and action.
    
    Args:
        state: Current system state (position, velocity)
        action: Control input
        dt: Time step (default 0.01)
    
    Returns:
        Next state after applying dynamics
    """
    # Implementation
    pass
```

## Submitting Changes

### Pull Request Process

1. **Create a branch** for your work:
   ```bash
   git checkout -b feature/your-feature-name
   ```

2. **Make your changes** and test them locally:
   ```bash
   pytest
   black .
   ```

3. **Commit with clear messages**:
   ```bash
   git add .
   git commit -m "Brief description of changes"
   ```

4. **Push and create a pull request**:
   ```bash
   git push origin feature/your-feature-name
   ```

5. **Wait for review** - maintainers will provide feedback

### What to Include in PRs
- Clear description of changes
- Reference any related issues
- Test results showing tests pass
- Updated documentation if applicable

## Reporting Issues

### Bug Reports
Include:
- Clear description of the bug
- Steps to reproduce
- Expected vs. actual behavior
- Python version and OS
- Full error traceback

### Feature Requests
Include:
- Clear description of the desired feature
- Use case and motivation
- Proposed implementation (if you have ideas)
- Any relevant references or examples

## Code of Conduct

Be respectful and professional. We welcome contributors of all experience levels.

## Questions?

Feel free to:
- Open an issue with your question
- Check existing issues and discussions
- Contact the maintainers

---

**Thank you for helping improve MuJoCo-Testing!**
