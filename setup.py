#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['engagement_detector'],
    # scripts=['scripts'],
    package_dir={'': 'src'},
    install_requires=['numpy', 'pillow', 'tensorflow-gpu', 'keras', 'opencv-python', 'rospkg']
)

setup(**d)
