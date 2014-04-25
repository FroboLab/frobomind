from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['mission/keyboard_mission_node.py',
  'mission/wiimote_mission_node.py',
  'mission/wiimote_mission_node_rtk_fixed.py',
  'navigation/waypoint_navigation_node.py',
  'navigation/area_search_node.py'],)

setup(**d)

