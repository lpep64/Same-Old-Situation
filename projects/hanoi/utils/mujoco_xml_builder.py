#!/usr/bin/env python3
"""
MuJoCo XML Builder - A sustainable system for building MuJoCo XML files
by importing and combining specified components.

This module provides a clean, modular approach to:
1. Parse existing MuJoCo XML files
2. Extract reusable components (assets, bodies, actuators, etc.)
3. Combine multiple XML files into a unified model
4. Programmatically build complex scenes
"""

import os
import re
import xml.etree.ElementTree as ET
from typing import Dict, List, Optional, Tuple
from pathlib import Path


class MuJoCoXMLBuilder:
    """A builder class for constructing MuJoCo XML files from components."""
    
    def __init__(self, model_name: str = "combined_model"):
        """Initialize the XML builder.
        
        Args:
            model_name: Name for the final model
        """
        self.model_name = model_name
        self.compiler_attrs = {"angle": "radian"}
        self.options = {}
        self.visual_settings = {}
        self.assets = {"textures": [], "materials": [], "meshes": []}
        self.worldbody_elements = []
        self.actuators = []
        self.tendons = []
        self.equality_constraints = []
        self.keyframes = []
        self.defaults = []
        
    def set_compiler_options(self, **kwargs):
        """Set compiler options like angle, meshdir, autolimits, etc."""
        self.compiler_attrs.update(kwargs)
        return self
    
    def set_physics_options(self, timestep: float = 0.005, integrator: str = "RK4", **kwargs):
        """Set physics simulation options."""
        self.options = {"timestep": str(timestep), "integrator": integrator, **kwargs}
        return self
    
    def set_visual_options(self, **kwargs):
        """Set visual rendering options."""
        self.visual_settings.update(kwargs)
        return self
    
    def add_texture(self, name: str, texture_type: str = "2d", **attrs):
        """Add a texture asset."""
        texture = {"name": name, "type": texture_type, **attrs}
        self.assets["textures"].append(texture)
        return self
    
    def add_material(self, name: str, **attrs):
        """Add a material asset."""
        material = {"name": name, **attrs}
        self.assets["materials"].append(material)
        return self
    
    def add_mesh(self, name: str = None, file: str = None, **attrs):
        """Add a mesh asset."""
        mesh = {**attrs}
        if name:
            mesh["name"] = name
        if file:
            mesh["file"] = file
        self.assets["meshes"].append(mesh)
        return self
    
    def add_worldbody_element(self, element_type: str, **attrs):
        """Add an element to worldbody (geom, light, body, etc.)."""
        element = {"_xml_tag": element_type, **attrs}
        self.worldbody_elements.append(element)
        return self
    
    def add_floor(self, pos: Tuple[float, float, float] = (0, 0, -0.1), 
                  size: Tuple[float, float, float] = (4, 4, 0.05), 
                  material: str = None, **attrs):
        """Add a floor geom to the scene."""
        floor_attrs = {
            "name": "floor",
            "pos": f"{pos[0]} {pos[1]} {pos[2]}",
            "size": f"{size[0]} {size[1]} {size[2]}",
            "type": "box",  # This is the geom type attribute
            **attrs
        }
        if material:
            floor_attrs["material"] = material
        
        # Add as a geom element (not "box" element)
        return self.add_worldbody_element("geom", **floor_attrs)
    
    def add_light(self, name: str, pos: Tuple[float, float, float] = (0, 0, 6), 
                  directional: bool = True, **attrs):
        """Add a light to the scene."""
        light_attrs = {
            "name": name,
            "pos": f"{pos[0]} {pos[1]} {pos[2]}",
            **attrs
        }
        if directional:
            light_attrs["directional"] = "true"
        
        return self.add_worldbody_element("light", **light_attrs)
    
    def import_robot_from_xml(self, xml_path: str, mesh_dir_override: str = None):
        """Import a robot model from an existing XML file.
        
        Args:
            xml_path: Path to the robot XML file
            mesh_dir_override: Override the mesh directory path
        """
        if not os.path.exists(xml_path):
            raise FileNotFoundError(f"Robot XML file not found: {xml_path}")
        
        try:
            # Parse the XML file
            tree = ET.parse(xml_path)
            root = tree.getroot()
            
            # Extract mesh directory from compiler or use override
            compiler = root.find("compiler")
            if mesh_dir_override:
                meshdir = mesh_dir_override
            elif compiler is not None and "meshdir" in compiler.attrib:
                base_dir = os.path.dirname(xml_path)
                meshdir = os.path.join(base_dir, compiler.attrib["meshdir"])
            else:
                meshdir = os.path.dirname(xml_path)
            
            # Update compiler with correct mesh directory
            self.compiler_attrs["meshdir"] = meshdir
            if compiler is not None and "autolimits" in compiler.attrib:
                self.compiler_attrs["autolimits"] = compiler.attrib["autolimits"]
            
            # Extract assets
            self._extract_assets_from_xml(root)
            
            # Extract defaults
            self._extract_defaults_from_xml(root)
            
            # Extract worldbody content
            self._extract_worldbody_from_xml(root)
            
            # Extract actuators, tendons, etc.
            self._extract_control_elements_from_xml(root)
            
        except ET.ParseError as e:
            raise ValueError(f"Failed to parse XML file {xml_path}: {e}")
        except Exception as e:
            raise RuntimeError(f"Failed to import robot from {xml_path}: {e}")
        
        return self
    
    def _extract_assets_from_xml(self, root):
        """Extract asset definitions from XML root."""
        asset_elem = root.find("asset")
        if asset_elem is None:
            return
        
        # Extract materials
        for material in asset_elem.findall("material"):
            self.assets["materials"].append(material.attrib)
        
        # Extract meshes
        for mesh in asset_elem.findall("mesh"):
            self.assets["meshes"].append(mesh.attrib)
        
        # Extract textures
        for texture in asset_elem.findall("texture"):
            self.assets["textures"].append(texture.attrib)
    
    def _extract_defaults_from_xml(self, root):
        """Extract default definitions from XML root."""
        for default in root.findall("default"):
            self.defaults.append(ET.tostring(default, encoding='unicode'))
    
    def _extract_worldbody_from_xml(self, root):
        """Extract worldbody content from XML root."""
        worldbody = root.find("worldbody")
        if worldbody is None:
            return
        
        # Convert all child elements to our format
        for child in worldbody:
            if child.tag == "body":
                # For body elements, we need to preserve the entire structure
                body_xml = ET.tostring(child, encoding='unicode')
                self.worldbody_elements.append({
                    "_xml_tag": "body_xml",
                    "content": body_xml
                })
            else:
                # For simple elements like geom, light, etc.
                attrs = child.attrib.copy()
                self.worldbody_elements.append({
                    "_xml_tag": child.tag,
                    **attrs
                })
    
    def _extract_control_elements_from_xml(self, root):
        """Extract control-related elements (actuators, tendons, etc.)."""
        # Extract actuators
        actuator_elem = root.find("actuator")
        if actuator_elem is not None:
            self.actuators.append(ET.tostring(actuator_elem, encoding='unicode'))
        
        # Extract tendons
        tendon_elem = root.find("tendon")
        if tendon_elem is not None:
            self.tendons.append(ET.tostring(tendon_elem, encoding='unicode'))
        
        # Extract equality constraints
        equality_elem = root.find("equality")
        if equality_elem is not None:
            self.equality_constraints.append(ET.tostring(equality_elem, encoding='unicode'))
        
        # Extract keyframes
        keyframe_elem = root.find("keyframe")
        if keyframe_elem is not None:
            self.keyframes.append(ET.tostring(keyframe_elem, encoding='unicode'))
    
    def build_xml(self) -> str:
        """Build the final XML string."""
        xml_parts = [f'<mujoco model="{self.model_name}">']
        
        # Compiler section
        if self.compiler_attrs:
            attrs_str = " ".join([f'{k}="{v}"' for k, v in self.compiler_attrs.items()])
            xml_parts.append(f'  <compiler {attrs_str}/>')
        
        # Options section
        if self.options:
            attrs_str = " ".join([f'{k}="{v}"' for k, v in self.options.items()])
            xml_parts.append(f'  <option {attrs_str}>')
            xml_parts.append('    <flag warmstart="enable"/>')
            xml_parts.append('  </option>')
        
        # Visual section
        if self.visual_settings:
            xml_parts.append('  <visual>')
            for key, value in self.visual_settings.items():
                # Handle special case: global_ -> global (since global is a Python keyword)
                xml_key = "global" if key == "global_" else key
                
                if isinstance(value, dict):
                    attrs_str = " ".join([f'{k}="{v}"' for k, v in value.items()])
                    xml_parts.append(f'    <{xml_key} {attrs_str}/>')
                else:
                    xml_parts.append(f'    <{xml_key}>{value}</{xml_key}>')
            xml_parts.append('  </visual>')
        
        # Defaults section
        if self.defaults:
            for default in self.defaults:
                # Add proper indentation
                indented_default = "\n".join(["  " + line for line in default.split("\n") if line.strip()])
                xml_parts.append(indented_default)
        
        # Assets section
        if any(self.assets.values()):
            xml_parts.append('  <asset>')
            
            # Add textures
            for texture in self.assets["textures"]:
                attrs_str = " ".join([f'{k}="{v}"' for k, v in texture.items()])
                xml_parts.append(f'    <texture {attrs_str}/>')
            
            # Add materials
            for material in self.assets["materials"]:
                attrs_str = " ".join([f'{k}="{v}"' for k, v in material.items()])
                xml_parts.append(f'    <material {attrs_str}/>')
            
            # Add meshes
            for mesh in self.assets["meshes"]:
                attrs_str = " ".join([f'{k}="{v}"' for k, v in mesh.items()])
                xml_parts.append(f'    <mesh {attrs_str}/>')
            
            xml_parts.append('  </asset>')
        
        # Worldbody section
        xml_parts.append('  <worldbody>')
        
        for element in self.worldbody_elements:
            if element.get("_xml_tag") == "body_xml":
                # Insert body XML directly with proper indentation
                body_content = element["content"]
                indented_body = "\n".join(["    " + line for line in body_content.split("\n") if line.strip()])
                xml_parts.append(indented_body)
            else:
                # Handle simple elements
                element_copy = element.copy()
                # The "_xml_tag" is the XML tag name (e.g., "geom", "light")
                xml_tag = element_copy.pop("_xml_tag")
                
                if element_copy:
                    attrs_str = " ".join([f'{k}="{v}"' for k, v in element_copy.items()])
                    xml_parts.append(f'    <{xml_tag} {attrs_str}/>')
                else:
                    xml_parts.append(f'    <{xml_tag}/>')
        
        xml_parts.append('  </worldbody>')
        
        # Control elements
        for tendon in self.tendons:
            indented_tendon = "\n".join(["  " + line for line in tendon.split("\n") if line.strip()])
            xml_parts.append(indented_tendon)
        
        for equality in self.equality_constraints:
            indented_equality = "\n".join(["  " + line for line in equality.split("\n") if line.strip()])
            xml_parts.append(indented_equality)
        
        for actuator in self.actuators:
            indented_actuator = "\n".join(["  " + line for line in actuator.split("\n") if line.strip()])
            xml_parts.append(indented_actuator)
        
        for keyframe in self.keyframes:
            indented_keyframe = "\n".join(["  " + line for line in keyframe.split("\n") if line.strip()])
            xml_parts.append(indented_keyframe)
        
        xml_parts.append('</mujoco>')
        
        return "\n".join(xml_parts)
    
    def save_xml(self, filepath: str) -> str:
        """Save the built XML to a file.
        
        Args:
            filepath: Path where to save the XML file
            
        Returns:
            The XML content that was saved
        """
        xml_content = self.build_xml()
        
        # Ensure directory exists (only if there's a directory path)
        dir_path = os.path.dirname(filepath)
        if dir_path:  # Only create directories if there's a path
            os.makedirs(dir_path, exist_ok=True)
        
        with open(filepath, 'w') as f:
            f.write(xml_content)
        
        print(f"XML saved to: {filepath}")
        return xml_content


def create_scene_with_robot(robot_xml_path: str, 
                          mesh_dir_override: str = None,
                          model_name: str = "scene_with_robot") -> MuJoCoXMLBuilder:
    """Create a scene with a robot using the builder pattern.
    
    Args:
        robot_xml_path: Path to the robot XML file
        mesh_dir_override: Override mesh directory path
        model_name: Name for the combined model
        
    Returns:
        Configured MuJoCoXMLBuilder instance
    """
    builder = MuJoCoXMLBuilder(model_name)
    
    # Set up basic scene
    builder.set_physics_options(timestep=0.005, integrator="RK4")
    
    # Set up visual options
    builder.set_visual_options(
        headlight={"ambient": "0.3 0.3 0.3", "diffuse": "0.7 0.7 0.7"},
        global_={"offwidth": "1920", "offheight": "1080"},
        quality={"shadowsize": "2048", "offsamples": "8"}
    )
    
    # Add scene textures and materials
    builder.add_texture("grid", "2d", 
                       builtin="checker", 
                       rgb1="0.3 0.3 0.3", 
                       rgb2="0.1 0.1 0.1", 
                       width="300", 
                       height="300")
    
    builder.add_material("grid", texture="grid", texrepeat="5 5", reflectance="0.")
    
    builder.add_texture("skybox", "skybox",
                       builtin="gradient",
                       rgb1="0.3 0.5 0.7",
                       rgb2="0.0 0.1 0.2",
                       width="800",
                       height="800",
                       mark="random",
                       markrgb="1 1 1")
    
    # Import robot
    builder.import_robot_from_xml(robot_xml_path, mesh_dir_override)
    
    # Add floor
    builder.add_floor(material="grid")
    
    # Add lighting
    builder.add_light("scene_light", 
                     pos=(2, 2, 6), 
                     dir="-0.3 -0.3 -1",
                     ambient="0.2 0.2 0.2",
                     diffuse="0.8 0.8 0.8",
                     specular="0.3 0.3 0.3")
    
    return builder


# Example usage functions
def example_basic_scene():
    """Example: Create a basic scene with just a floor."""
    builder = MuJoCoXMLBuilder("basic_scene")
    
    builder.set_physics_options(timestep=0.002)
    builder.add_texture("wood", "2d", builtin="flat", rgb1="0.6 0.4 0.2")
    builder.add_material("wood_mat", texture="wood")
    builder.add_floor(material="wood_mat")
    builder.add_light("main_light")
    
    return builder.build_xml()


def example_robot_scene():
    """Example: Create a scene with the Panda robot."""
    robot_path = "../../models/robots/franka_emika_panda/panda.xml"
    
    if os.path.exists(robot_path):
        builder = create_scene_with_robot(robot_path)
        return builder.build_xml()
    else:
        print(f"Robot file not found: {robot_path}")
        return example_basic_scene()


if __name__ == "__main__":
    # Example usage
    print("MuJoCo XML Builder Example")
    print("=" * 40)
    
    # Create a scene with robot
    xml_content = example_robot_scene()
    
    # Save to file
    output_path = "generated_scene.xml"
    with open(output_path, 'w') as f:
        f.write(xml_content)
    
    print(f"Generated XML saved to: {output_path}")
    print("\nFirst 20 lines of generated XML:")
    print("-" * 40)
    for i, line in enumerate(xml_content.split('\n')[:20]):
        print(f"{i+1:2d}: {line}")