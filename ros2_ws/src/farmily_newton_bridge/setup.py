from setuptools import setup, find_packages

setup(name='farmily_newton_bridge', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/farmily_newton_bridge']),
                  ('share/farmily_newton_bridge', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='farmily', maintainer_email='user@example.com', license='Apache-2.0',
      description='Live Newton telemetry and RGB-D ground truth',
      entry_points={'console_scripts': ['bridge = farmily_newton_bridge.node:main',
                                       'planner = farmily_newton_bridge.planner:main',
                                       'inspect = farmily_newton_bridge.inspect:main']})
