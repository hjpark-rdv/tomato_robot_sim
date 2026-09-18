"""Read the ROS GUI's left ready pose without requiring ROS in the simulator."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

SRDF_PATH = Path(__file__).resolve().parents[2] / 'ros2_ws/src/rbpodo_ros2/rbpodo_moveit_config/config/rbpodo.srdf'
ARM_JOINT_NAMES = ('base', 'shoulder', 'elbow', 'wrist1', 'wrist2', 'wrist3')
READY_STATE = 'PICK_READY'  # harvest_gui maps CAPTURE_LEFT to this named state.
POLICY_SCHEMA = 'greenhouse-pick-ready-lift-v1'


def read_left_pick_ready():
    root = ET.parse(SRDF_PATH).getroot()
    state = next((s for s in root.findall('group_state')
                  if s.get('name') == READY_STATE and s.get('group') == 'mainpulation'), None)
    if state is None:
        raise ValueError(f'{READY_STATE}/mainpulation not found in {SRDF_PATH}')
    values = {j.get('name'): float(j.get('value')) for j in state.findall('joint')}
    if any(name not in values or not math.isfinite(values[name]) for name in ARM_JOINT_NAMES):
        raise ValueError(f'{READY_STATE} must provide all six finite arm joint positions')
    return {name: values[name] for name in ARM_JOINT_NAMES}
