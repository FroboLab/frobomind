#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['src/remote_control_mission.py','src/robot_track_map_node.py', 'waypoint_navigation/waypoint_navigation_node.py', 'polygon_map/polygon_map_node.py'],
)

setup(**d)
