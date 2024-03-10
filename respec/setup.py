#!/usr/bin/env python
import os
from setuptools import setup
from setuptools import find_packages
from glob import glob
package_name = 'respec'

setup(
    name=package_name,
    version='0.1.0',  # TODO Version change
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file in package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        # Include config yaml files
        (('lib/python3.10/site-packages/respec/config'), glob(os.path.join(
                                                             'respec', 'config', '*.yaml'))),
        (os.path.join('share', package_name, 'additional_specs'),
         glob(os.path.join('respec', 'additional_specs', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Spyros Maniatopoulos',
    author_email='sm2296@cornell.edu',
    maintainer='Spyros Maniatopoulos',
    maintainer_email='sm2296@cornell.edu',
    # keywords        # TODO
    # classifiers     # TODO
    description=('A Python framework for generating Linear Temporal Logic formulas for'
                 ' Robotics applications'),
    license='BSD',
    # entry_points    # TODO
)
