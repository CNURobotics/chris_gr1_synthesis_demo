#!/usr/bin/env python
import os
from setuptools import setup

package_name = 'ltl_synthesizer'
submodules = 'ltl_synthesizer/StructuredSlugsParser'

setup(
    name=package_name,
    version='1.0.0',  # TODO Version change
    packages=[package_name, submodules],
    data_files=[
        # Install marker file in package index
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include package.xml file
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'synthesis_byproducts'), []),  # glob(*)
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
    description=('An interface between the Behavior Synthesis Manager'
                 ' and the LTL Synthesizer (slugs)'),
    license='BSD',
    entry_points={
        'console_scripts': [
            'ltl_synthesis_server = ltl_synthesizer.ltl_synthesis_server:ltl_synthesis_server',
        ],
    },
)
