#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division

from python_qt_binding.QtCore import Qt, pyqtSlot, QUrl
from python_qt_binding.QtDeclarative import QDeclarativeView

import os
import rospkg
import sensor_msgs.msg as sensor_msgs

##############################################################################
# Class
##############################################################################


class QVirtualJoystickView(QDeclarativeView):
    """
    Accepts an image of a teleop compressed image type and draws that in the
    scene/view.
    """

    def __init__(self, parent=None):
        super(QVirtualJoystickView, self).__init__(parent)
        virtual_joystick_path = os.path.join(rospkg.RosPack().get_path('rocon_qt_library'), 'ui', 'virtual_joystick.qml')
        self.setSource(QUrl(virtual_joystick_path))
        self.setResizeMode(QDeclarativeView.SizeRootObjectToView)

    def joystick_feedback(self):
        return self.rootObject().feedback

    def mouse_pressed(self):
        return self.rootObject().mousePressed

    def mouse_released(self):
        return self.rootObject().mouseReleased
