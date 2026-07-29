from glob import glob

from setuptools import find_packages, setup


package_name = "rbpodo_tomato_harvest"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="farmily",
    maintainer_email="user@example.com",
    description="Cartesian tomato harvesting helpers for the Farmily RB5 robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "tomato_harvest_test = rbpodo_tomato_harvest.tomato_harvest_test:main",
            "tomato_tf_generator = rbpodo_tomato_harvest.tomato_tf_generator:main",
            "fake_camera_service = rbpodo_tomato_harvest.fake_camera_service:main",
            "harvest_gui = rbpodo_tomato_harvest.harvest_gui:main",
            "harvest_report = rbpodo_tomato_harvest.harvest_report:main",
            "lift_joint_state_publisher = "
            "rbpodo_tomato_harvest.lift_joint_state_publisher:main",
        ],
    },
)
