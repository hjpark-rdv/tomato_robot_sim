from glob import glob
from setuptools import find_packages, setup

package_name = 'arduino_linear_motor'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/arduino/mega_pin89_serial',
         glob('arduino/mega_pin89_serial/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rdv',
    maintainer_email='hjpark@rdv.ai',
    description='ROS 2 serial bridge for Arduino Mega digital pins 8 and 9.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pin89_serial_node = arduino_linear_motor.pin89_serial_node:main',
        ],
    },
)
