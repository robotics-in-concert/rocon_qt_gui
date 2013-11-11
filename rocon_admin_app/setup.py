#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rocon_admin_app'],
    package_dir={'': 'src'},
    scripts=['scripts/rocon_admin_app'],
    requires=['qt_gui_py_common', 'rqt_gui', 'rqt_gui_py', 'rospy', 'rospkg']
)
setup(**d)
