#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['fake_world', 'fake_world_ros'],
 package_dir={'fake_world': 'common/src/fake_world', 'fake_world_ros': 'ros/src/fake_world_ros'}
)

setup(**d)
