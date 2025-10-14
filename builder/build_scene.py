#!/usr/bin/env python3
"""
Build Scene Script
Generates a complete MuJoCo XML scene with Panda robot and cube stacking setup.
This script creates a full XML file with 3 colored cubes that can be stacked by the robot.
"""

import os
import sys

# Add parent directory to path so we can import utils
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import our XML builder
from mujoco_xml_builder import MuJoCoXMLBuilder


def build_panda_cube_stacking_scene(robot_path: str, output_path: str) -> str:
    """Build a complete MuJoCo XML scene with Panda robot and cube stacking setup.
    
    Args:
        robot_path: Path to the robot XML file
        output_path: Path where the generated XML will be saved
        
    Returns:
        Path to the generated XML file
    """
    print(f"Building Panda Cube Stacking scene...")
    print(f"Robot path: {robot_path}")
    print(f"Output path: {output_path}")
    
    # Create the builder with custom settings
    builder = MuJoCoXMLBuilder("panda_cube_stacking_scene")
    
    # Configure physics
    builder.set_physics_options(
        timestep=0.005,
        integrator="RK4"
    )
    
    # Configure visuals
    builder.set_visual_options(
        headlight={"ambient": "0.3 0.3 0.3", "diffuse": "0.7 0.7 0.7"},
        global_={"offwidth": "1920", "offheight": "1080"},
        quality={"shadowsize": "2048", "offsamples": "8"}
    )
    
    # Add custom textures
    builder.add_texture("checker_floor", "2d",
                       builtin="checker",
                       rgb1="0.2 0.3 0.4",  # Blueish
                       rgb2="0.8 0.9 1.0",  # Light blue
                       width="128",
                       height="128")
    
    builder.add_texture("gradient_sky", "skybox",
                       builtin="gradient",
                       rgb1="0.4 0.6 0.8",  # Sky blue
                       rgb2="0.1 0.2 0.4",  # Darker blue
                       width="1024",
                       height="1024",
                       mark="random",
                       markrgb="1 1 1")
    
    # Add materials
    builder.add_material("floor_material", 
                        texture="checker_floor", 
                        texrepeat="8 8", 
                        reflectance="0.1")
    
    # Import the robot
    mesh_dir = "../../../models/robots/arms/franka_emika_panda/assets"
    builder.import_robot_from_xml(robot_path, mesh_dir)
    
    # Add IK target sites to the robot gripper bodies
    builder.add_sites_to_robot_body("hand", [
        {"name": "gripper_center", "pos": "0 0 0.02", "size": "0.015", "rgba": "1 0 1 0.8"},
        {"name": "gripper_approach", "pos": "0 0 0.08", "size": "0.02", "rgba": "0 1 1 0.8"},
        {"name": "pre_grasp", "pos": "0 0 0.12", "size": "0.025", "rgba": "1 1 0 0.6"}
    ])
    
    builder.add_sites_to_robot_body("left_finger", [
        {"name": "left_finger_tip", "pos": "0 0.02 0.03", "size": "0.01", "rgba": "0 1 0 0.8"}
    ])
    
    builder.add_sites_to_robot_body("right_finger", [
        {"name": "right_finger_tip", "pos": "0 -0.02 0.03", "size": "0.01", "rgba": "1 0 0 0.8"}
    ])
    
    # Add scene elements
    builder.add_floor(pos=(0, 0, -0.05), 
                     size=(2, 2, 0.05), 
                     material="floor_material")    
    
    # Add stacking target sites for the cubes
    builder.add_worldbody_element("body_xml", content='''
    <body name="stacking_area" pos="0.5 0 0">
        <!-- Target sites for cube stacking -->
        <site name="stack_base" pos="0 0 0.025" size="0.02" rgba="0.8 0.8 0.8 0.5"/>
        <site name="stack_level1" pos="0 0 0.075" size="0.02" rgba="0.7 0.7 0.7 0.5"/>
        <site name="stack_level2" pos="0 0 0.125" size="0.02" rgba="0.6 0.6 0.6 0.5"/>
        <site name="stack_approach" pos="0 0 0.2" size="0.015" rgba="0.5 0.5 0.5 0.3"/>
    </body>''')
    # Cube 1 (red) - starts on the right side for pickup
    builder.add_worldbody_element("body_xml", content='''
    <body name="cube_body1" pos="0.7 0.3 0.025">
        <freejoint/>
        <inertial pos="0 0 0" mass="0.05" diaginertia="0.001 0.001 0.001"/>
        <!-- Red cube geometry -->
        <geom name="cube1" type="box" size="0.025 0.025 0.025" rgba="0.8 0.2 0.2 1.0"/>
        <!-- IK target sites for grasping cube 1 -->
        <site name="cube1_grasp_top" pos="0 0 0.035" size="0.015" rgba="1 1 1 0.8"/>
        <site name="cube1_grasp_front" pos="0.035 0 0" size="0.01" rgba="1 1 1 0.6"/>
        <site name="cube1_grasp_back" pos="-0.035 0 0" size="0.01" rgba="1 1 1 0.6"/>
        <site name="cube1_center" pos="0 0 0" size="0.008" rgba="1 0.5 0.5 0.7"/>
    </body>''')
    
    # Cube 2 (green) - starts in the middle for pickup
    builder.add_worldbody_element("body_xml", content='''
    <body name="cube_body2" pos="0.7 0 0.025">
        <freejoint/>
        <inertial pos="0 0 0" mass="0.05" diaginertia="0.001 0.001 0.001"/>
        <!-- Green cube geometry -->
        <geom name="cube2" type="box" size="0.025 0.025 0.025" rgba="0.2 0.8 0.2 1.0"/>
        <!-- IK target sites for grasping cube 2 -->
        <site name="cube2_grasp_top" pos="0 0 0.035" size="0.015" rgba="1 1 1 0.8"/>
        <site name="cube2_grasp_front" pos="0.035 0 0" size="0.01" rgba="1 1 1 0.6"/>
        <site name="cube2_grasp_back" pos="-0.035 0 0" size="0.01" rgba="1 1 1 0.6"/>
        <site name="cube2_center" pos="0 0 0" size="0.008" rgba="0.5 1 0.5 0.7"/>
    </body>''')

    # Cube 3 (blue) - starts on the left side for pickup
    builder.add_worldbody_element("body_xml", content='''
    <body name="cube_body3" pos="0.7 -0.3 0.025">
        <freejoint/>
        <inertial pos="0 0 0" mass="0.05" diaginertia="0.001 0.001 0.001"/>
        <!-- Blue cube geometry -->
        <geom name="cube3" type="box" size="0.025 0.025 0.025" rgba="0.2 0.2 0.8 1.0"/>
        <!-- IK target sites for grasping cube 3 -->
        <site name="cube3_grasp_top" pos="0 0 0.035" size="0.015" rgba="1 1 1 0.8"/>
        <site name="cube3_grasp_front" pos="0.035 0 0" size="0.01" rgba="1 1 1 0.6"/>
        <site name="cube3_grasp_back" pos="-0.035 0 0" size="0.01" rgba="1 1 1 0.6"/>
        <site name="cube3_center" pos="0 0 0" size="0.008" rgba="0.5 0.5 1 0.7"/>
    </body>''')

    # Add multiple lights for better illumination
    builder.add_light("main_light",
                     pos=(0, 0, 3),
                     directional=False,
                     ambient="0.3 0.3 0.3",
                     diffuse="0.9 0.9 0.9",
                     specular="0.5 0.5 0.5")
    
    builder.add_light("side_light",
                     pos=(3, 3, 2),
                     directional=True,
                     dir="-1 -1 -0.5",
                     ambient="0.1 0.1 0.2",
                     diffuse="0.6 0.7 0.8",
                     specular="0.2 0.2 0.3")
    
    # Build and save the XML
    print("Building XML...")
    xml_content = builder.build_xml()
    
    # Ensure output directory exists
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # Save the XML file
    print(f"Saving XML to: {output_path}")
    builder.save_xml(output_path)
    
    print("✅ Scene XML generated successfully!")
    return output_path


def main():
    """Main function to build the scene XML."""
    try:
        # Get script directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
        
        # Define paths
        robot_path = os.path.join(project_root, "..", "..", "models", "robots", "arms", "agilex_piper", "piper.xml")
        output_path = os.path.join(project_root)
        
        # Normalize paths
        robot_path = os.path.normpath(robot_path)
        output_path = os.path.normpath(output_path)
        
        # Check if robot file exists
        if not os.path.exists(robot_path):
            raise FileNotFoundError(f"Robot model not found at: {robot_path}")
        
        # Build the scene
        generated_xml = build_panda_cube_stacking_scene(robot_path, output_path)
        
        print(f"\n🎉 Successfully generated cube stacking scene XML!")
        print(f"📁 Location: {generated_xml}")
        print(f"📝 You can now use this XML file for cube stacking tasks")
        
        return 0
        
    except Exception as e:
        print(f"\n❌ Error occurred: {str(e)}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())