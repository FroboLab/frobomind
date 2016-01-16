from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['scripts/utm_test.py', 'scripts/kp2000_test.py', 'scripts/dktm_test.py'],
  packages=['transverse_mercator_py'],
  package_dir={'':'src'}
  )
setup(**d)
