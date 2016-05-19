from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['src/publish_cmd_vel_node.py'],)
setup(**d)

