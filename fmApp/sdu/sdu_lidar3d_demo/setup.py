from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/print_pitch_roll.py',
  'src/dmm_tech_abs_node.py'],
)

setup(**d)

