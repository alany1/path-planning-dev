import glob
import os
from setuptools import find_packages
from setuptools import setup

package_name = 'path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/path_planning/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/path_planning/trajectories', glob.glob(os.path.join('trajectories', '*.traj')))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebastian',
    maintainer_email='sebastianag2002@gmail.com',
    description='Path Planning ROS2 Package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_builder = path_planning.trajectory_builder:main',
            'trajectory_loader = path_planning.trajectory_loader:main',
            'trajectory_planner = path_planning.path_planning:main'
        ],
    },
)
