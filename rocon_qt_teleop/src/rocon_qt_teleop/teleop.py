#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division
import os
import math

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, pyqtSlot
from python_qt_binding.QtGui import QWidget

from qt_gui.plugin import Plugin
import rospkg
import rospy

from rocon_qt_library.widgets import QVideoTeleop

##############################################################################
# Teleop
##############################################################################


class Teleop(Plugin):
    def __init__(self, context):
        self._context = context
        super(Teleop, self).__init__(context)
        # I'd like these to be also configurable via the gui
        self._widget = QVideoTeleop()
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        self._default_cmd_vel_topic = 'cmd_vel'
        self._default_compressed_image_topic = 'compressed_image'
        self._widget.init_teleop_interface(self._default_cmd_vel_topic, self._default_compressed_image_topic)

    def shutdown_plugin(self):
        self._widget.shutdown_plugin()
