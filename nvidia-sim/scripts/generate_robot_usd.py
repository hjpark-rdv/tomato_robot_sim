#!/usr/bin/env python3
"""Generate self-contained URDF and convert to Isaac Sim USD for rb5_farmily robot."""

import argparse
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET


def generate_urdf(output_urdf_path: str):
    xacro_file = "/root/farmily_tomato/ros2_ws/src/rbpodo_ros2/rbpodo_description/robots/rb5_farmily.urdf.xacro"
    pkg_mesh_dir = "/root/farmily_tomato/ros2_ws/src/rbpodo_ros2/rbpodo_description"

    # Step 1: Run xacro via bash with ROS 2 sourced
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source /root/farmily_tomato/ros2_ws/install/setup.bash 2>/dev/null || true && "
        f"xacro {xacro_file}"
    )
    proc = subprocess.run(
        ["bash", "-c", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=True,
    )
    urdf_content = proc.stdout

    # Step 2: Replace package:// paths with absolute paths
    urdf_content = urdf_content.replace("package://rbpodo_description", pkg_mesh_dir)

    # Step 3: Parse XML to ensure proper inertial on links that lack them
    root = ET.fromstring(urdf_content)

    inertials_to_add = {
        "farmily_lift": {
            "mass": "25.0",
            "origin_xyz": "0.0 0.0 0.4",
            "ixx": "1.5",
            "iyy": "1.5",
            "izz": "0.5",
        },
        "farmily_lift_platform": {
            "mass": "15.0",
            "origin_xyz": "0.0 0.3 0.0",
            "ixx": "1.2",
            "iyy": "0.2",
            "izz": "1.2",
        },
        "tomato_gripper": {
            "mass": "1.2",
            "origin_xyz": "-0.08 0.01 0.0",
            "ixx": "0.005",
            "iyy": "0.005",
            "izz": "0.003",
        },
        "link0": {
            "mass": "3.5",
            "origin_xyz": "0.0 0.0 0.08",
            "ixx": "0.01",
            "iyy": "0.01",
            "izz": "0.01",
        },
    }

    for link in root.findall("link"):
        link_name = link.attrib.get("name", "")
        if link_name in inertials_to_add and link.find("inertial") is None:
            cfg = inertials_to_add[link_name]
            inertial = ET.SubElement(link, "inertial")
            ET.SubElement(inertial, "origin", {"xyz": cfg["origin_xyz"], "rpy": "0 0 0"})
            ET.SubElement(inertial, "mass", {"value": cfg["mass"]})
            ET.SubElement(
                inertial,
                "inertia",
                {
                    "ixx": cfg["ixx"],
                    "ixy": "0.0",
                    "ixz": "0.0",
                    "iyy": cfg["iyy"],
                    "iyz": "0.0",
                    "izz": cfg["izz"],
                },
            )

    os.makedirs(os.path.dirname(os.path.abspath(output_urdf_path)), exist_ok=True)
    tree = ET.ElementTree(root)
    tree.write(output_urdf_path, encoding="utf-8", xml_declaration=True)
    print(f"[OK] Generated URDF: {output_urdf_path}")


def convert_urdf_to_usd(urdf_path: str, usd_path: str):
    convert_tool = "/root/works/IsaacLab/scripts/tools/convert_urdf.py"
    cmd = [
        sys.executable,
        convert_tool,
        urdf_path,
        usd_path,
        "--headless",
        "--fix-base",
        "--merge-joints",
        "--joint-stiffness",
        "800.0",
        "--joint-damping",
        "40.0",
    ]
    print(f"Running conversion: {' '.join(cmd)}")
    subprocess.run(cmd, check=True)
    print(f"[OK] Generated Robot USD: {usd_path}")


def tune_usd_physics(usd_path: str):
    """Fine-tune linear lift joint and arm joint stiffness, damping, and force limits for PhysX."""
    code = f"""
from isaacsim import SimulationApp
app = SimulationApp({{"headless": True}})
from pxr import Usd

stage = Usd.Stage.Open("{usd_path}")
for prim in stage.Traverse():
    name = prim.GetName()
    if name == "farmily_lift_height_joint":
        print("[PHYSICS] Tuning linear lift joint drive:", prim.GetPath())
        if prim.GetAttribute("drive:linear:physics:stiffness"):
            prim.GetAttribute("drive:linear:physics:stiffness").Set(50000.0)
        if prim.GetAttribute("drive:linear:physics:damping"):
            prim.GetAttribute("drive:linear:physics:damping").Set(2500.0)
        if prim.GetAttribute("drive:linear:physics:maxForce"):
            prim.GetAttribute("drive:linear:physics:maxForce").Set(10000.0)
    elif name in ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]:
        if prim.GetAttribute("drive:angular:physics:stiffness"):
            prim.GetAttribute("drive:angular:physics:stiffness").Set(800.0)
        if prim.GetAttribute("drive:angular:physics:damping"):
            prim.GetAttribute("drive:angular:physics:damping").Set(40.0)
        if prim.GetAttribute("drive:angular:physics:maxForce"):
            prim.GetAttribute("drive:angular:physics:maxForce").Set(250.0)

stage.GetRootLayer().Save()
print("[OK] Successfully tuned USD drive stiffness, damping, and max forces!")
app.close()
"""
    subprocess.run([sys.executable, "-c", code], check=True)


def main():
    parser = argparse.ArgumentParser(description="Generate rb5_farmily USD asset for Isaac Sim")
    parser.add_argument(
        "--urdf-out",
        type=str,
        default="/root/farmily_tomato/nvidia-sim/robot_usd/rb5_farmily.urdf",
        help="Path for exported URDF",
    )
    parser.add_argument(
        "--usd-out",
        type=str,
        default="/root/farmily_tomato/nvidia-sim/robot_usd/rb5_farmily.usd",
        help="Path for generated USD",
    )
    args = parser.parse_args()

    generate_urdf(args.urdf_out)
    convert_urdf_to_usd(args.urdf_out, args.usd_out)
    tune_usd_physics(args.usd_out)


if __name__ == "__main__":
    main()
