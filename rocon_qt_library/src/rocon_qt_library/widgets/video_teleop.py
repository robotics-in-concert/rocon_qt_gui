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

from rocon_qt_library.interfaces import VideoTeleopInterface
from rocon_qt_library.views import QCameraView, QVirtualJoystickView

##############################################################################
# Teleop
##############################################################################

class QVideoTeleop(QWidget):

    degrees_to_radians = 3.141592 / 180

    def __init__(self, parent=None):
        super(QVideoTeleop, self).__init__()
        self.maximum_linear_velocity = rospy.get_param('~maximum_linear_velocity', 2.0)                         
        self.maximum_angular_velocity = rospy.get_param('~maximum_angular_velocity', 90 * QVideoTeleop.degrees_to_radians)
        self._teleop_interface = None

        self._load_ui()
        self._init_events()
        #self._init_callbacks()

    def _load_ui(self):
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_qt_library'), 'ui', 'video_teleop.ui')
        loadUi(ui_file, self, {'QCameraView': QCameraView, 'QVirtualJoystickView': QVirtualJoystickView})


    def _init_events(self):
        # virtual joystick signals
        self.virtual_joystick_view.joystick_feedback().connect(self.on_joystick_feedback)
        self.virtual_joystick_view.mouse_released().connect(self.on_mouse_released)
        self.on_compressed_image_received = self.camera_view.on_compressed_image_received

        #keyboard control
        for k in self.children():
            try:
                k.keyPressEvent = self.on_key_press
                k.keyReleaseEvent = self.on_key_release
            except:
                pass

    def reset(self):
        self.camera_view.load_default_image()

        if self._teleop_interface:
            self._teleop_interface.shutdown()
            self._teleop_interface = None
    
    def init_teleop_interface(self, cmd_vel_topic_name, compressed_image_topic_name):
        self._teleop_interface = VideoTeleopInterface(image_received_slot=self.on_compressed_image_received, cmd_vel_topic_name=cmd_vel_topic_name, compressed_image_topic_name=compressed_image_topic_name)

    def shutdown_plugin(self):
        if self._teleop_interface:
            self._teleop_interface.shutdown()

    def on_key_release(self, e):
        if e.isAutoRepeat():
            return
        else:
            if self._teleop_interface:
                self._teleop_interface.cmd_vel = (0.0, 0.0)

    def on_key_press(self, e):
        '''
        Currently a very crude implementation - no smoothing, no acceleration, just min, max.
        Need to replace this with something like our non-gui keyboard teleop in python.
        '''
        linear_command = 0
        angular_command = 0
        if Qt.Key_Up == e.key():
            linear_command = self.maximum_linear_velocity
        elif Qt.Key_Down == e.key():
            linear_command = -self.maximum_linear_velocity
        elif Qt.Key_Right == e.key():
            angular_command = -self.maximum_angular_velocity
        elif Qt.Key_Left == e.key():
            angular_command = self.maximum_angular_velocity
        if self._teleop_interface:
            self._teleop_interface.cmd_vel = (linear_command, angular_command)

    @pyqtSlot(float, float)
    def on_joystick_feedback(self, x, y):
        '''
        Takes a normalised double pair coming in with which we can apply to our
        velocity bounds.

        We also use a narrow dead zone along each x and y axis.

        :param double x: normalised (-1.0, 1.0) left to right position
        :param double y: normalised (-1.0, 1.0) bottom to top position
        '''
        dead_zone_radius = 0.05
        if math.fabs(x) < dead_zone_radius:
            angular_command = 0.0
        else:
            angular_command = -x * self.maximum_angular_velocity
        if  math.fabs(y) < dead_zone_radius:
            linear_command = 0.0
        else:
            linear_command = y * self.maximum_linear_velocity
        
        if self._teleop_interface:
            self._teleop_interface.cmd_vel = (linear_command, angular_command)

    @pyqtSlot()
    def on_mouse_released(self):
        if self._teleop_interface:
            self._teleop_interface.cmd_vel = (0.0, 0.0)
