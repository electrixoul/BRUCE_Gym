#!usr/bin/env python
__author__    = "Westwood Robotics Corporation"
__email__     = "info@westwoodrobotics.io"
__copyright__ = "Copyright 2023 Westwood Robotics Corporation"
__date__      = "April 1, 2023"
__version__   = "0.0.1"
__status__    = "Production"

from setuptools import setup

with open("README.md", "r") as fh:
    long_description = fh.read()

setup(
    name='gazebopy',
    version='0.0.1',
    author='Westwood Robotics Corporation',
    author_email="info@westwoodrobotics.io",
    description="An interface using ROS2 to control simulations in Gazebo",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/Westwood-Robotics/gazebopy",
    packages=['gazebopy'],
    install_requires=[
        'numpy',
    ],
)