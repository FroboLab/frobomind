from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['src/show_imu_node.py',
	'src/show_pose_node.py'],
)

setup(**d)

