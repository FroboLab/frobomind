#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
	scripts=['src/drive_forward.py', 'src/line_follow_action.py', 'src/make_turn.py', 'src/navigate_in_row_simple.py' , 'src/timed_turn_action.py'])

setup(**d)
