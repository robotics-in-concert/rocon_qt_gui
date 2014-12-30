#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_qt_library', 'rocon_qt_library.interfaces', 'rocon_qt_library.utils', 'rocon_qt_library.views', 'rocon_qt_library.widgets'],
    package_dir={'': 'src'},
)
setup(**d)
