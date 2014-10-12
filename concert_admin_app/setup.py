#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_admin_app'],
    package_dir={'': 'src'},
    scripts=['scripts/concert_admin_app'],
    requires=['qt_gui_py_common', 'rqt_gui', 'rqt_gui_py', 'rospy', 'rospkg']
)
setup(**d)
