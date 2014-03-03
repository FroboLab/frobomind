from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/differential_odometry_reset_node.py'],
  packages=['differential_odometry'],
  package_dir={'':'src'}
  )
setup(**d)
