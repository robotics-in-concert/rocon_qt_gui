#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#

##############################################################################
# TeleopAppInfo
##############################################################################


class TeleopAppInfo(object):
    def __init__(self):
        self._event_callback = None
        # should make use of concert_msgs/Strings here.

    def _reg_event_callback(self, func):
        self._event_callback = func
