from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['mission_planners/simple_mission.py',
  'waypoint_navigation/waypoint_navigation_node.py',
  'robot_track_map/robot_track_map_node.py'],
)

setup(**d)


