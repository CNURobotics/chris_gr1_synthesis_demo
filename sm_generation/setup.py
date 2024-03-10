#!/usr/bin/env python
import os
from setuptools import setup
from glob import glob

package_name = 'sm_generation'

setup(
    name=package_name,
    version='1.0.0',  # TODO version change?
    packages=[package_name],
    data_files=[
        # Install marker file in package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'src', package_name, 'configs'),
         glob(os.path.join(package_name, 'configs', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Vitchyr Pong',
    author_email='vhp22@cornell.edu',
    maintainer='Vitchyr Pong',
    maintainer_email='vhp22@cornell.edu',
    # keywords      # TODO
    # classifiers   # TODO
    description=('A service that converts the output of the LTL synthesizer'
                 ' and create class parameters to generate code.'),
    license='BSD',
    # tests_require # TODO
    entry_points={
        'console_scripts': [
            'sm_generation_server = sm_generation.sm_generation_server:sm_gen_server',
        ],
    },
)
