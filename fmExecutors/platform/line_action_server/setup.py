#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/lineGoalActionServer.py'],
  packages=['line_control','line_smach'],
  package_dir={'':'src'}
)

setup(**d)