#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rospkg

##############################################################################
# Methods
##############################################################################


def setup_home_dirs():
    if not os.path.isdir(get_home()):
        os.makedirs(get_home())
    if not os.path.isdir(get_icon_cache_home()):
        os.makedirs(get_icon_cache_home())
    if not os.path.isdir(get_settings_cache_home()):
        os.makedirs(get_settings_cache_home())


def get_home():
    '''
      Retrieve the location of the home directory for the rocon remocon's
      temporary storage needs

      @return the rocon remocon home directory (path object).
      @type str
    '''
    return os.path.join(rospkg.get_ros_home(), 'rocon', 'remocon')


def get_icon_cache_home():
    '''
      Retrieve the location of the directory used for storing icons.

      @return the rocon remocon icons directory (path object).
      @type str
    '''
    return os.path.join(get_home(), 'icons')


def get_settings_cache_home():
    '''
      Retrieve the location of the directory used for storing qt settings.

      @return the rocon remocon qt settings directory (path object).
      @type str
    '''
    return os.path.join(get_home(), 'cache')
