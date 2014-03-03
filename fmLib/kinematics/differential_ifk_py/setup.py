from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=[''],
  packages=['differential_ifk_py'],
  package_dir={'':'src'}
  )
setup(**d)
