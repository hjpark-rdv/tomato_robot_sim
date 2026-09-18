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
import os

physics_usd_path = os.path.join(
    os.path.dirname("{usd_path}"), "configuration", "rb5_farmily_physics.usd"
)
stage = Usd.Stage.Open(physics_usd_path)
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


def fix_usd_references(usd_path: str):
    """Make imported geometry visible through the robot's physics payload.

    Isaac Sim 5.1 imports visual geometry as internal references to sibling root
    prims. Those sibling prims are outside the default prim used by the physics
    payload, so they disappear in the final robot asset. Flattening the base layer
    resolves that geometry below each robot link. Physics-layer collision
    references are then made explicit against the flattened base layer.
    """
    code = f"""
from isaacsim import SimulationApp
app = SimulationApp({{"headless": True}})

from pxr import Sdf, Usd
import os

config_dir = os.path.join(os.path.dirname("{usd_path}"), "configuration")
base_usd = os.path.join(config_dir, "rb5_farmily_base.usd")
physics_usd = os.path.join(config_dir, "rb5_farmily_physics.usd")

base_stage = Usd.Stage.Open(base_usd, load=Usd.Stage.LoadAll)
if not base_stage:
    raise RuntimeError(f"Could not open base robot USD: {{base_usd}}")
for prim in list(base_stage.TraverseAll()):
    if prim.IsInstance():
        prim.SetInstanceable(False)
flattened_base = base_stage.Flatten()
if not flattened_base.Export(base_usd):
    raise RuntimeError(f"Could not export flattened base robot USD: {{base_usd}}")
print(f"[OK] Flattened robot visual geometry into {{os.path.basename(base_usd)}}")

def patch_physics_references(layer_path):
    layer = Sdf.Layer.FindOrOpen(layer_path)
    if not layer:
        return
    count = 0
    def patch_prim(prim_spec):
        nonlocal count
        new_refs = []
        for ref in prim_spec.referenceList.prependedItems:
            if ref.assetPath == '':
                new_refs.append(Sdf.Reference('./rb5_farmily_base.usd', ref.primPath))
                count += 1
            else:
                new_refs.append(ref)
        if new_refs:
            prim_spec.referenceList.prependedItems.clear()
            for r in new_refs:
                prim_spec.referenceList.prependedItems.append(r)
        for child in prim_spec.nameChildren:
            patch_prim(child)
    for root in layer.rootPrims:
        patch_prim(root)
    layer.Save()
    print(f"[OK] Patched {{count}} reference paths in {{os.path.basename(layer_path)}}")

patch_physics_references(physics_usd)
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
    fix_usd_references(args.usd_out)
    tune_usd_physics(args.usd_out)


if __name__ == "__main__":
    main()
