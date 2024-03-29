#!/usr/bin/env python
from os import path
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'engagement_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (path.join('share', package_name), glob(path.join('engagement_detector', '*.py'))),
        # (path.join('share', "models"), glob(path.join('models', '*.h5'))),
    ],
    install_requires=['setuptools'],
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
            'engagement_detector_node = engagement_detector.ros_engagement_detector:main',
            'webcam_node = engagement_detector.webcam:main',
        ],
    },
)
