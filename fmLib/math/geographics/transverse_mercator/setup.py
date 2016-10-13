from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
  scripts=['src/transverse_mercator_py/utm_test.py', 'src/transverse_mercator_py/kp2000_test.py', 'src/transverse_mercator_py/dktm_test.py'],
  packages=['transverse_mercator_py'],
  package_dir={'':'src'}
  )
setup(**d)
