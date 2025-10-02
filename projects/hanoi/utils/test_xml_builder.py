#!/usr/bin/env python3
"""
Simple test for the XML builder to debug the geom generation issue.
"""

from mujoco_xml_builder import MuJoCoXMLBuilder

def test_simple_scene():
    """Test creating a simple scene to debug the XML generation."""
    print("Testing simple XML builder...")
    
    builder = MuJoCoXMLBuilder("test_scene")
    
    # Add a simple floor
    builder.add_floor(pos=(0, 0, -0.1), size=(2, 2, 0.05))
    
    # Add a light
    builder.add_light("test_light", pos=(0, 0, 2))
    
    # Generate XML
    xml_content = builder.build_xml()
    
    print("Generated XML:")
    print("=" * 50)
    print(xml_content)
    print("=" * 50)
    
    # Save for inspection
    with open("../xmls/generated/test_simple.xml", "w") as f:
        f.write(xml_content)
    
    print("Saved to xmls/generated/test_simple.xml")

if __name__ == "__main__":
    test_simple_scene()