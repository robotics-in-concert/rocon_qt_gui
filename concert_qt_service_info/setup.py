#!/user/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['concert_qt_service_info'],
    package_dir={'': 'src'},
)
setup(**d)

