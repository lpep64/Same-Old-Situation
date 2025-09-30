# Niryo Ned2 3D Models

This directory contains the 3D models for the Niryo Ned2 robotic arm, sourced from the official NiryoRobotics repository.

## Source
- **Repository**: https://github.com/NiryoRobotics/ned2
- **License**: CC0 1.0 Universal (Public Domain)
- **Date Retrieved**: September 29, 2025

## Directory Structure

### STL Files (`stl/`)
Ready-to-use mesh files for MuJoCo simulation:

**Main Robot Components:**
- `1- BASE - V1.STL` - Robot base/foundation
- `2- SHOULDER - V1.STL` - Shoulder joint assembly
- `3- ARM - V1.STL` - Upper arm segment
- `4- ELBOW - V1.STL` - Elbow joint assembly
- `5- FOREARM - V1.STL` - Lower arm segment
- `6- WRIST - V1.STL` - Wrist joint assembly
- `7- HAND - V1.STL` - Hand/mounting plate

**Additional Components:**
- `10- BOITIER SECURITE - NED 2.STL` - Security housing
- `11- GRIPPER 1 - NED 2.STL` - Primary gripper assembly
- `MORS GRIPPER 1 - NED 2.STL` - Gripper jaw/finger

### STEP Files (`step/`)
CAD source files for mechanical design reference:

- `NED2_STEP.step` - Complete Ned2 robot assembly
- `ADAPTATIVE_GRIPPER_NED2_STEP.STEP` - Adaptive gripper design
- `CUSTOM_GRIPPER_NED2_STEP.STEP` - Custom gripper variant
- `LARGE_GRIPPER_NED2_STEP.STEP` - Large gripper variant

## Usage in MuJoCo

The STL files can be directly referenced in MuJoCo XML models using the `<mesh>` tag:

```xml
<asset>
  <mesh name="base" file="assets/meshes/niryo/stl/1- BASE - V1.STL"/>
  <mesh name="shoulder" file="assets/meshes/niryo/stl/2- SHOULDER - V1.STL"/>
  <!-- ... other components ... -->
</asset>
```

## Notes

- STL files are ready for immediate use in MuJoCo
- STEP files provide source CAD data for modifications or conversions
- All files are released under CC0 license (public domain)
- Consider scaling factors when importing into MuJoCo (units may need adjustment)