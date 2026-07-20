from glob import glob
from setuptools import setup

package_name = "tomato_moveit_teach"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.srdf") + glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
        (f"share/{package_name}/urdf", glob("urdf/*.xacro")),
    ],
    install_requires=["setuptools"],
    scripts=[
        "scripts/tomato_scene_node",
        "scripts/teach_grasp_marker",
        "scripts/grasp_pose_saver",
        "scripts/home_joint_state_publisher",
        "scripts/planned_joint_saver",
        "scripts/pose_snapshot",
        "scripts/grasp_pose_diagnostics",
        "scripts/joint_control_gui",
        "scripts/mobile_base_tf_publisher",
        "scripts/tool_depth_camera_publisher",
        "scripts/moveit_grasp_planner",
        "scripts/moveit_harvest_sequence",
        "scripts/moveit_right_tomato_pose_test",
        "scripts/trajectory_labeler_gui",
        "scripts/trajectory_selector",
        "scripts/check_trajectory_label_schema",
    ],
    zip_safe=True,
    maintainer="tomato_moveit_teach",
    maintainer_email="user@example.com",
    description="MoveIt/RViz teaching helpers for UR5 tomato grasp poses.",
    license="Apache-2.0",
    entry_points={},
)
