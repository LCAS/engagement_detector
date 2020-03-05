#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['engagement_detector'],
    # scripts=['scripts'],
    package_dir={'': 'src'},
    install_requires=['numpy', 'pillow', 'tensorflow-gpu==1.15.0', 'keras==2.2.4', 'opencv-python', 'rospkg']
)

setup(**d)