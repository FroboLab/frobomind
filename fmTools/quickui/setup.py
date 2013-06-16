#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   scripts=['scripts/quickui_example.py'],
   packages=['quickui'],
   package_dir={'': 'src'}
)

setup(**d)