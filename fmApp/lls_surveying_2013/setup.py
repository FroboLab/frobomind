from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['mission/keyboard_mission.py',
  'mission/wiimote_mission.py',
  'navigation/waypoint_navigation_node.py',
  'survey/survey_log.py',
  'monitor/plot_propulsion_feedback_node.py',
  'monitor/plot_lidar_node.py',
  'monitor/plot_lidar_line_node.py',
  'monitor/show_pose_node.py',
  'monitor/show_imu_node.py',
  'monitor/show_navigation_status_node.py',
  'monitor/robot_track_map_node.py',
  'scripts/cmd_vel_publish.py'],)

setup(**d)

