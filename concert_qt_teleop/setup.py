#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_qt_teleop'],
    package_dir={'': 'src'},
    scripts=['scripts/concert_teleop'],
)
setup(**d)
