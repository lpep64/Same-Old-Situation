# Screenshots - README

This folder contains screenshots and images for documentation and project presentation.

## 📸 Screenshot Guidelines

### **Naming Convention**
- Use descriptive names: `gui_falling_objects.png`
- Include date if relevant: `training_progress_2025_09_26.png`
- Use lowercase with underscores
- PNG format for screenshots, JPG for photos

### **Organization**
```
screenshots/
├── demos/                   # Demo application screenshots
│   ├── gui/                # GUI demo screenshots
│   └── non_gui/            # Terminal/console outputs
├── ml_training/            # RL training progress, plots
├── project_examples/       # Project template examples
└── setup/                  # Installation and setup screens
```

### **Content Guidelines**
- Capture relevant windows/screens clearly
- Include window titles and relevant UI elements
- For training plots: include axes labels and legends
- For GUI demos: show physics in action
- Crop to relevant content, avoid excessive whitespace

### **Usage in Documentation**
```markdown
![Description](../assets/screenshots/folder/image_name.png)
```

## 🎯 Recommended Screenshots to Add

### **Demo Applications**
- [ ] `simple_gui_running.png` - Basic GUI demo in action
- [ ] `interactive_playground.png` - Advanced physics playground
- [ ] `terminal_physics_test.png` - Non-GUI test results

### **ML Training**
- [ ] `rl_training_progress.png` - Training reward curves
- [ ] `agent_performance.png` - Trained agent performance
- [ ] `environment_list.png` - Available RL environments

### **Project Setup**
- [ ] `project_structure.png` - File explorer view of organized project
- [ ] `niryo_arm_template.png` - NiryoArm project structure
- [ ] `successful_installation.png` - Terminal showing successful setup

### **Physics Simulations**
- [ ] `falling_objects.png` - Multi-object physics demo
- [ ] `humanoid_simulation.png` - Humanoid model in action  
- [ ] `collision_detection.png` - Objects colliding

## 📊 Adding Screenshots

### **Taking Good Screenshots**
1. **Windows**: Use Snipping Tool or Win+Shift+S
2. **macOS**: Use Cmd+Shift+4 for selection
3. **Linux**: Use gnome-screenshot or similar

### **Editing Guidelines**
- Resize to reasonable dimensions (max 1920px width)
- Add annotations if needed (arrows, labels)
- Use consistent fonts and colors for annotations
- Optimize file size without losing clarity

### **Documentation Integration**
After adding screenshots:
1. Update relevant markdown files
2. Test that images display correctly
3. Add descriptive alt text
4. Consider adding captions

---

**Note**: Remember to take screenshots that represent the current state of the project and update them as the codebase evolves.