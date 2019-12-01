#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['endekf', 'endekf_ros'],
 package_dir={'endekf': 'common/src/endekf', 'endekf_ros': 'ros/src/endekf_ros'}
)

setup(**d)
