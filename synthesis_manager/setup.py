#!/usr/bin/env python
import os
from glob import glob
from setuptools import setup

package_name = 'synthesis_manager'

setup(
    name=package_name,
    version='1.0.0',  # TODO Version change
    packages=[package_name],
    data_files=[
        # Install marker file in package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Includes launch file
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='John Wesley Hayhurst',
    author_email='johnwesley.hayhurst.12@cnu.edu',
    maintainer='CNU Robotics',
    maintainer_email='robotics@cnu.edu',
    keywords=['FlexBE'],
    classifiers=[
        'Programming Language :: Python',
    ],
    description='An action server that can create a state machine the slugs synthesizer',
    license='BSD',
    entry_points={
        'console_scripts': [
            'behavior_synthesis_server = synthesis_manager.behavior_synthesis_server:main'
            ],
        },
)
