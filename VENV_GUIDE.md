# Virtual Environment Quick Reference Guide

## ⚡ Essential Commands (Windows PowerShell)

### Daily Workflow:
```powershell
# 1. Activate (ALWAYS DO THIS FIRST)
.\.venv\Scripts\Activate.ps1

# 2. Work with your projects
python projects/anissa_niryo/scripts/simple_niryo_demo.py

# 3. Install packages if needed
pip install new_package

# 4. Deactivate when done
deactivate
```

## 🔍 How to Know if Virtual Environment is Active:
- ✅ Good: `(.venv) PS C:\Users\lukep\Documents\mujoco>`
- ❌ Bad:  `PS C:\Users\lukep\Documents\mujoco>` (no .venv prefix)

## 📦 Package Management:
```powershell
pip list                    # See installed packages
pip install package_name   # Install new package
pip install -r requirements.txt  # Install from file
pip freeze > requirements.txt    # Save current packages
pip uninstall package_name # Remove package
```

## 🚨 CRITICAL RULES:
1. **ALWAYS activate venv before running Python scripts**
2. **ALWAYS activate venv before installing packages**
3. **Install packages while venv is activated, not globally**
4. **Each project can have its own venv if needed**

## 🏗️ Creating New Virtual Environments:
```powershell
# For future projects (in project directory)
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## 🔧 Troubleshooting:
- If activation fails: `Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser`
- If wrong Python version: Check `python --version` after activation
- If packages missing: Make sure venv is activated before installing