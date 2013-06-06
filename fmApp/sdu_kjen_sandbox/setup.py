#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['src/remote_control_mission.py','wptnavi_pichi/robot_track_map_node.py'],
)

setup(**d)
