import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'farmily_uv_lift'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rdv',
    maintainer_email='hjpark@rdv.ai',
    description='A CANopen-based lift motor controller GUI.',
    license='TODO: License declaration',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    entry_points={
        'console_scripts': [
            'lift_controller_node = farmily_uv_lift.lift_controller_node:main',
        ],
    },
)
