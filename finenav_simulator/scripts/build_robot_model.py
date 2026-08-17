#!/usr/bin/env python3
"""Build-time xacro → SDF processor for finenav_simulator.

Called by CMake at build time to generate the robot model.sdf from the xacro
template.  The generated SDF is installed alongside model.config as a standard
Gazebo model, making it discoverable via ``model://fine_robot``.
"""
import sys
import os

# Allow imports from the scripts/ directory (sdf_launch_utils + thirdparty)
sys.path.insert(0, os.path.dirname(__file__))
from sdf_launch_utils import generate_robot_sdf

xacro_path = sys.argv[1]
output_path = sys.argv[2]
mesh_uri = sys.argv[3] if len(sys.argv) > 3 else ""

mappings = {}
if mesh_uri:
    mappings["lidar_support_mesh_uri"] = mesh_uri

result = generate_robot_sdf(xacro_path, mappings)

os.makedirs(os.path.dirname(output_path), exist_ok=True)
with open(output_path, "w", encoding="utf-8") as f:
    f.write(result)

print(f"Generated: {output_path}")
