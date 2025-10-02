# STEP to MuJoCo XML Conversion Guide

## Overview
This guide documents how to convert STEP CAD files to MuJoCo-compatible XML format based on analysis of the MuJoCo project structure, specifically studying the `niryo_ned2` and `franka_emika_panda` robot models.

## Current State Analysis

### Niryo Ned2 Model Structure
- **Location**: `models/robots/niryo_ned2/`
- **Available Assets**:
  - `NED2_STEP.step` - Original STEP file
  - `STL/` folder with individual component STL files:
    - 1- BASE - V1.STL
    - 2- SHOULDER - V1.STL
    - 3- ARM - V1.STL
    - 4- ELBOW - V1.STL
    - 5- FOREARM - V1.STL
    - 6- WRIST - V1.STL
    - 7- HAND - V1.STL
    - 10- BOITIER SECURITE - NED 2.STL
    - 11- GRIPPER 1 - NED 2.STL
    - MORS GRIPPER 1 - NED 2.STL

### Franka Panda Model Structure (Reference)
- **Location**: `models/robots/franka_emika_panda/`
- **Assets**: `assets/` folder with optimized mesh files
- **Formats**: Both STL (collision) and OBJ (visual) meshes
- **XML Structure**: Complete MuJoCo XML with proper hierarchy

## MuJoCo XML Format Requirements

### 1. Basic Structure
```xml
<mujoco model="robot_name">
  <compiler angle="radian" meshdir="assets" autolimits="true"/>
  <option integrator="implicitfast"/>
  
  <default>
    <!-- Default classes for materials, joints, etc. -->
  </default>
  
  <asset>
    <!-- Mesh and material definitions -->
  </asset>
  
  <worldbody>
    <!-- Robot body hierarchy -->
  </worldbody>
</mujoco>
```

### 2. Mesh Asset Definitions
MuJoCo supports multiple mesh formats:
- **STL**: Binary STL for collision meshes
- **OBJ**: Wavefront OBJ for visual meshes
- **PLY**: PLY format meshes
- **MSH**: Legacy MuJoCo mesh format

Example mesh definitions:
```xml
<asset>
  <!-- Collision meshes (STL) -->
  <mesh name="link0_c" file="link0.stl"/>
  <mesh name="link1_c" file="link1.stl"/>
  
  <!-- Visual meshes (OBJ) -->
  <mesh file="link0_0.obj"/>
  <mesh file="link0_1.obj"/>
</asset>
```

### 3. Body Hierarchy with Meshes
```xml
<body name="link0">
  <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
    fullinertia="0.00315 0.00388 0.004285 8.2904e-7 0.00015 8.2299e-6"/>
  
  <!-- Visual geometries -->
  <geom mesh="link0_0" material="off_white" class="visual"/>
  <geom mesh="link0_1" material="black" class="visual"/>
  
  <!-- Collision geometry -->
  <geom mesh="link0_c" class="collision"/>
  
  <!-- Child bodies -->
  <body name="link1" pos="0 0 0.333">
    <!-- ... -->
  </body>
</body>
```

## STEP to MuJoCo Conversion Packages

### Python Packages for STEP File Processing

#### 1. **FreeCAD + Python**
- **Installation**: `pip install FreeCAD`
- **Capabilities**: 
  - Read STEP files
  - Export to STL, OBJ, PLY
  - Decompose assemblies into individual parts
- **Example Usage**:
```python
import FreeCAD
import Part
import Mesh

# Load STEP file
doc = FreeCAD.newDocument()
Part.insert("model.step", doc.Name)

# Export individual objects to STL
for obj in doc.Objects:
    if hasattr(obj, 'Shape'):
        # Export to STL
        mesh = Mesh.Mesh()
        mesh.addFacets(obj.Shape.tessellate(0.1))
        mesh.write(f"{obj.Name}.stl")
```

#### 2. **PythonOCC/pythonocc-core**
- **Installation**: `pip install pythonocc-core`
- **Capabilities**: 
  - OpenCASCADE bindings for Python
  - Advanced CAD operations
  - STEP/IGES import/export
- **Example Usage**:
```python
from OCC.Core import STEPControl_Reader, TopExp_Explorer, TopAbs_SOLID
from OCC.Core import StlAPI_Writer

reader = STEPControl_Reader()
reader.ReadFile("model.step")
reader.TransferRoots()
shape = reader.OneShape()

# Export to STL
stl_writer = StlAPI_Writer()
stl_writer.Write(shape, "output.stl")
```

#### 3. **CadQuery**
- **Installation**: `pip install cadquery`
- **Capabilities**:
  - Programmatic CAD operations
  - STEP import and mesh export
  - Built on OpenCASCADE
- **Example Usage**:
```python
import cadquery as cq

# Import STEP file
result = cq.importers.importStep("model.step")

# Export to STL
cq.exporters.export(result, "output.stl")
```

#### 4. **Open3D**
- **Installation**: `pip install open3d`
- **Capabilities**:
  - Mesh processing and conversion
  - STL/OBJ/PLY support
  - Mesh optimization
- **Example Usage**:
```python
import open3d as o3d

# Load and convert mesh formats
mesh = o3d.io.read_triangle_mesh("input.stl")
o3d.io.write_triangle_mesh("output.obj", mesh)
```

#### 5. **Trimesh**
- **Installation**: `pip install trimesh`
- **Capabilities**:
  - Mesh loading and processing
  - Format conversion
  - Mesh repair and optimization
- **Example Usage**:
```python
import trimesh

# Load mesh and convert
mesh = trimesh.load("input.stl")
mesh.export("output.obj")
```

### External Tools

#### 1. **Blender (with Python scripting)**
- Free 3D software with Python API
- Excellent for STEP → OBJ conversion
- Can be automated with scripts

#### 2. **MeshLab**
- Open source mesh processing
- Good for mesh optimization and conversion

#### 3. **V-HACD**
- Convex decomposition tool
- Useful for collision meshes
- Available as Python package: `pip install pyvhacd`

## Recommended Conversion Workflow

### Step 1: STEP File Analysis
1. Load STEP file using FreeCAD or PythonOCC
2. Identify individual components/bodies
3. Extract component names and hierarchy

### Step 2: Mesh Generation
1. **For each component**:
   - Export high-quality mesh for visuals (OBJ format)
   - Export simplified mesh for collision (STL format)
   - Consider convex decomposition for complex collision shapes

### Step 3: MuJoCo XML Creation
1. Create asset definitions for all meshes
2. Define material properties
3. Build body hierarchy with proper:
   - Joint definitions
   - Inertial properties
   - Collision and visual geometries

### Step 4: Optimization
1. **Mesh optimization**:
   - Reduce polygon count for performance
   - Ensure proper scaling
   - Fix mesh issues (holes, normals)

2. **Physics tuning**:
   - Calculate/estimate inertial properties
   - Set appropriate collision margins
   - Define joint limits and dynamics

## Complete Conversion Script Template

```python
#!/usr/bin/env python3
"""
STEP to MuJoCo XML Converter
Converts STEP CAD files to MuJoCo-compatible XML format
"""

import FreeCAD
import Part
import Mesh
import os
import xml.etree.ElementTree as ET

def convert_step_to_mujoco(step_file, output_dir):
    """Convert STEP file to MuJoCo XML with mesh assets"""
    
    # Create output directories
    assets_dir = os.path.join(output_dir, "assets")
    os.makedirs(assets_dir, exist_ok=True)
    
    # Load STEP file
    doc = FreeCAD.newDocument()
    Part.insert(step_file, doc.Name)
    
    # Create XML structure
    root = ET.Element("mujoco", model=os.path.splitext(os.path.basename(step_file))[0])
    
    # Compiler settings
    compiler = ET.SubElement(root, "compiler")
    compiler.set("angle", "radian")
    compiler.set("meshdir", "assets")
    compiler.set("autolimits", "true")
    
    # Assets section
    asset = ET.SubElement(root, "asset")
    
    # World body
    worldbody = ET.SubElement(root, "worldbody")
    
    # Process each object
    for i, obj in enumerate(doc.Objects):
        if hasattr(obj, 'Shape') and obj.Shape.Volume > 0:
            # Generate mesh files
            mesh_name = f"link_{i}"
            stl_file = f"{mesh_name}.stl"
            
            # Export to STL
            mesh = Mesh.Mesh()
            mesh.addFacets(obj.Shape.tessellate(0.1))
            mesh.write(os.path.join(assets_dir, stl_file))
            
            # Add mesh asset
            mesh_elem = ET.SubElement(asset, "mesh")
            mesh_elem.set("name", mesh_name)
            mesh_elem.set("file", stl_file)
            
            # Add body (simplified)
            body = ET.SubElement(worldbody, "body")
            body.set("name", mesh_name)
            
            # Add geometry
            geom = ET.SubElement(body, "geom")
            geom.set("mesh", mesh_name)
            geom.set("type", "mesh")
    
    # Write XML file
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ", level=0)
    tree.write(os.path.join(output_dir, "robot.xml"), encoding="utf-8", xml_declaration=True)
    
    FreeCAD.closeDocument(doc.Name)
    
    print(f"Conversion complete! Files saved to {output_dir}")

if __name__ == "__main__":
    # Example usage
    step_file = "models/robots/niryo_ned2/NED2_STEP.step"
    output_dir = "converted_robot"
    convert_step_to_mujoco(step_file, output_dir)
```

## Integration with MuJoCo Project

### File Organization
Follow the MuJoCo project structure:
```
models/robots/your_robot/
├── assets/
│   ├── link0.stl        # Collision meshes
│   ├── link1.stl
│   ├── link0_0.obj      # Visual meshes (detailed)
│   ├── link0_1.obj
│   └── ...
├── robot.xml            # Main robot definition
├── scene.xml            # Scene with robot
└── README.md            # Documentation
```

### Best Practices
1. **Mesh Quality**: Balance detail vs. performance
2. **Naming Convention**: Use consistent naming (link0, link1, etc.)
3. **Scaling**: Ensure proper units (MuJoCo uses meters)
4. **Materials**: Define appropriate visual materials
5. **Collision**: Use simplified collision meshes when possible
6. **Inertia**: Calculate or estimate realistic inertial properties

## Conclusion

Converting STEP files to MuJoCo XML requires:
1. **CAD Processing**: Tools like FreeCAD, PythonOCC, or CadQuery for STEP import
2. **Mesh Generation**: Export to STL/OBJ formats with appropriate detail levels
3. **XML Structure**: Follow MuJoCo's schema for proper robot definition
4. **Optimization**: Balance visual quality with simulation performance

The niryo_ned2 model shows that STEP files can be successfully decomposed into individual STL components, which can then be integrated into a proper MuJoCo XML structure following the patterns established by models like franka_emika_panda.