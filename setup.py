#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['engagement_detector'],
    scripts=['scripts'],
    package_dir={'': 'scripts'},
    install_requires=['python-numpy', 'python-imaging', 'tensorflow-gpu', 'keras', 'python-opencv']
)

setup(**d)