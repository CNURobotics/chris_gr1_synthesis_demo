#!/usr/bin/env python
import os
from setuptools import setup
from glob import glob

package_name = 'file_generation'

setup(
    name=package_name,
    version='0.0.0',  # TODO Version change
    packages=[package_name],
    data_files=[
        # Install marker file in package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'),  glob(os.path.join('config', '*.yaml')))
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
    description='The file_generation package',
    # license         # TODO
    entry_points={
        'console_scripts': [
            'file_generation_server = '
            'file_generation.file_generation_server:_file_generation_server',
        ],
    }
)
