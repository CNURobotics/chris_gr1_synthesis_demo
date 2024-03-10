#!/usr/bin/env python
import os
from setuptools import setup

package_name = 'ltl_specification'

setup(
    name=package_name,
    version='1.0.0',  # TODO Version change
    packages=[package_name],
    data_files=[
        # Install marker file in package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml file
        (os.path.join('share', package_name), ['package.xml']),
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
    description=('A service that compiles a complete LTL/GR(1) specification from various sources,'
                 ' including user input, initial conditions, system constraints, etc.'),
    license='BSD',
    entry_points={
        'console_scripts': [
            'ltl_compilation_server = '
            'ltl_specification.ltl_compilation_server:ltl_compilation_server',
        ],
    },
)
