#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_remocon'],
    package_dir={'': 'src'},
    scripts=['scripts/rocon_remocon', 'scripts/rocon_remocon_sub', 'scripts/rocon_remocon_check_up'],
)
setup(**d)
