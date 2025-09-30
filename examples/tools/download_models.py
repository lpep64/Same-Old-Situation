#!/usr/bin/env python3
"""
MuJoCo Model Downloader

This script allows you to download additional robot models from the MuJoCo Menagerie
when needed, instead of keeping all models locally.
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path

# Available models for download
AVAILABLE_MODELS = {
    # Humanoid robots
    "humanoids": {
        "berkeley_humanoid": "https://github.com/deepmind/mujoco_menagerie.git",
        "unitree_h1": "https://github.com/deepmind/mujoco_menagerie.git",
        "unitree_g1": "https://github.com/deepmind/mujoco_menagerie.git",
        "pal_talos": "https://github.com/deepmind/mujoco_menagerie.git",
    },
    
    # Quadrupeds
    "quadrupeds": {
        "boston_dynamics_spot": "https://github.com/deepmind/mujoco_menagerie.git",
        "unitree_a1": "https://github.com/deepmind/mujoco_menagerie.git",
        "unitree_go2": "https://github.com/deepmind/mujoco_menagerie.git",
        "anybotics_anymal_b": "https://github.com/deepmind/mujoco_menagerie.git",
        "anybotics_anymal_c": "https://github.com/deepmind/mujoco_menagerie.git",
    },
    
    # Arms and manipulators
    "arms": {
        "kuka_iiwa_14": "https://github.com/deepmind/mujoco_menagerie.git",
        "universal_robots_ur10e": "https://github.com/deepmind/mujoco_menagerie.git",
        "rethink_robotics_sawyer": "https://github.com/deepmind/mujoco_menagerie.git",
        "kinova_gen3": "https://github.com/deepmind/mujoco_menagerie.git",
        "trossen_vx300s": "https://github.com/deepmind/mujoco_menagerie.git",
    },
    
    # Grippers and hands
    "hands": {
        "shadow_hand": "https://github.com/deepmind/mujoco_menagerie.git",
        "shadow_dexee": "https://github.com/deepmind/mujoco_menagerie.git",
        "leap_hand": "https://github.com/deepmind/mujoco_menagerie.git",
        "robotiq_2f85": "https://github.com/deepmind/mujoco_menagerie.git",
        "wonik_allegro": "https://github.com/deepmind/mujoco_menagerie.git",
    },
    
    # Mobile platforms
    "mobile": {
        "hello_robot_stretch": "https://github.com/deepmind/mujoco_menagerie.git",
        "hello_robot_stretch_3": "https://github.com/deepmind/mujoco_menagerie.git",
        "pal_tiago": "https://github.com/deepmind/mujoco_menagerie.git",
        "google_robot": "https://github.com/deepmind/mujoco_menagerie.git",
    },
    
    # Special purpose
    "special": {
        "flybody": "https://github.com/deepmind/mujoco_menagerie.git",
        "realsense_d435i": "https://github.com/deepmind/mujoco_menagerie.git",
        "skydio_x2": "https://github.com/deepmind/mujoco_menagerie.git",
        "bitcraze_crazyflie_2": "https://github.com/deepmind/mujoco_menagerie.git",
    }
}

def list_available_models():
    """List all available models organized by category."""
    print("Available Models for Download")
    print("=" * 50)
    
    for category, models in AVAILABLE_MODELS.items():
        print(f"\n{category.upper()}")
        print("-" * 30)
        for i, model_name in enumerate(models.keys(), 1):
            print(f"  {i:2d}. {model_name}")
    
    print(f"\nTotal available models: {sum(len(models) for models in AVAILABLE_MODELS.values())}")

def download_model(model_name, target_dir="../../models/robots"):
    """Download a specific model from the menagerie."""
    # Find which category the model belongs to
    for category, models in AVAILABLE_MODELS.items():
        if model_name in models:
            print(f"Downloading {model_name} from {category}...")
            
            # Create temporary directory
            temp_dir = "temp_menagerie"
            if os.path.exists(temp_dir):
                try:
                    if os.name == 'nt':  # Windows
                        subprocess.run(['rmdir', '/s', '/q', temp_dir], shell=True, check=False)
                    else:
                        shutil.rmtree(temp_dir)
                except Exception:
                    pass
            
            try:
                # Clone the repository
                print("Cloning MuJoCo Menagerie...")
                subprocess.run([
                    "git", "clone", "--depth", "1", 
                    models[model_name], temp_dir
                ], check=True, capture_output=True)
                
                # Copy the specific model
                source_path = os.path.join(temp_dir, model_name)
                target_path = os.path.join(target_dir, model_name)
                
                if os.path.exists(source_path):
                    os.makedirs(target_dir, exist_ok=True)
                    shutil.copytree(source_path, target_path)
                    print(f"Successfully downloaded {model_name} to {target_path}")
                else:
                    print(f"Model {model_name} not found in repository")
                    return False
                
                # Clean up - use force remove on Windows
                try:
                    if os.name == 'nt':  # Windows
                        subprocess.run(['rmdir', '/s', '/q', temp_dir], shell=True, check=False)
                    else:
                        shutil.rmtree(temp_dir)
                except Exception as e:
                    print(f"Warning: Could not clean up temp directory: {e}")
                return True
                
            except subprocess.CalledProcessError as e:
                print(f"Error downloading model: {e}")
                if os.path.exists(temp_dir):
                    try:
                        if os.name == 'nt':  # Windows
                            subprocess.run(['rmdir', '/s', '/q', temp_dir], shell=True, check=False)
                        else:
                            shutil.rmtree(temp_dir)
                    except Exception:
                        pass
                return False
    
    print(f"Model '{model_name}' not found in available models")
    return False

def download_category(category, target_dir="../../models/robots"):
    """Download all models from a specific category."""
    if category not in AVAILABLE_MODELS:
        print(f"Category '{category}' not found")
        return False
    
    print(f"Downloading all models from category: {category}")
    
    success_count = 0
    total_count = len(AVAILABLE_MODELS[category])
    
    for model_name in AVAILABLE_MODELS[category].keys():
        if download_model(model_name, target_dir):
            success_count += 1
    
    print(f"\nDownloaded {success_count}/{total_count} models from {category}")
    return success_count == total_count

def main():
    """Main function to handle command line arguments."""
    if len(sys.argv) < 2:
        print("MuJoCo Model Downloader")
        print("=" * 40)
        print("Usage:")
        print("  python download_models.py list                    # List all available models")
        print("  python download_models.py model <model_name>      # Download specific model")
        print("  python download_models.py category <category>     # Download all models in category")
        print("\nCategories: humanoids, quadrupeds, arms, hands, mobile, special")
        return 1
    
    command = sys.argv[1].lower()
    
    if command == "list":
        list_available_models()
    
    elif command == "model" and len(sys.argv) >= 3:
        model_name = sys.argv[2]
        target_dir = sys.argv[3] if len(sys.argv) >= 4 else "models/robots"
        download_model(model_name, target_dir)
    
    elif command == "category" and len(sys.argv) >= 3:
        category = sys.argv[2].lower()
        target_dir = sys.argv[3] if len(sys.argv) >= 4 else "models/robots"
        download_category(category, target_dir)
    
    else:
        print("❌ Invalid command. Use 'list', 'model <name>', or 'category <name>'")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())