# src/unitree_go2_nav2/setup.py

import os
from glob import glob
from setuptools import setup

package_name = 'unitree_go2_nav2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch', 'simulation'), 
            glob('launch/simulation/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.rviz')),
        # World files for Gazebo
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.world')),
        # Maps
        (os.path.join('share', package_name, 'maps'), 
            glob('maps/*')),
        (os.path.join('lib', package_name),
            glob('lib_placeholder/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Andrew O\'Shei',
    author_email='andrewoshei@gmail.com',
    maintainer='Andrew O\'Shei',
    maintainer_email='andrewoshei@gmail.com',
    description='Unitree Go2 Nav2 navigation package with simulation support',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
