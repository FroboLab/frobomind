from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['mission/keyboard_mission_node.py',
  'waypoint_navigation/waypoint_navigation_node.py'],)
setup(**d)

