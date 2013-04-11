from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/planner.py'],
  packages=['simple_2d_math'],
  package_dir={'':'src'}
  )
setup(**d)