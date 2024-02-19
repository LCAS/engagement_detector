#!/usr/bin/env python

from setuptools import setup
from setuptools import find_packages

package_name = 'engagement_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages('src'),
    package_dir={'': 'src'},
    install_requires=['numpy', 'pillow', 'tensorflow', 'keras', 'opencv-python', 'rospkg'],
    zip_safe=True,
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache License 2.0',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Engagement Detector package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'engagement_detector_node = engagement_detector.ros_engagement_detector:main'
        ],
    },
)
