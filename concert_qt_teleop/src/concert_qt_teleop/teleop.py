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
from python_qt_binding.QtCore import Qt, Signal, pyqtSlot, SIGNAL
from python_qt_binding.QtGui import QWidget, QMessageBox, QTreeWidgetItem
from python_qt_binding.QtGui import QBrush, QColor

import rospkg
import rospy
from teleop_app_info import TeleopManager
from rocon_qt_library.views import QCameraView, QVirtualJoystickView

from qt_gui.plugin import Plugin

##############################################################################
# Teleop App
##############################################################################


class Teleop(Plugin):
    _update_robot_list_signal = Signal()

    degrees_to_radians = 3.141592 / 180

    def __init__(self, context):
        self._context = context
        super(Teleop, self).__init__(context)
        # I'd like these to be also configurable via the gui
        self.maximum_linear_velocity = rospy.get_param('~maximum_linear_velocity', 2.0)
        self.maximum_angular_velocity = rospy.get_param('~maximum_angular_velocity', 90 * Teleop.degrees_to_radians)
        rospy.loginfo("Rocon Teleop : maximum velocities [%s, %s]" % (self.maximum_linear_velocity, self.maximum_angular_velocity))

        self.initialised = False

        self.is_setting_dlg_live = False
        self._widget = QWidget()
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_qt_teleop'), 'ui', 'concert_teleop.ui')
        loadUi(ui_file, self._widget, {'QCameraView': QCameraView, 'QVirtualJoystickView': QVirtualJoystickView})
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        #list item click event
        self._widget.robot_list_tree_widget.itemClicked.connect(self._select_robot_list_tree_item)
        self._widget.robot_list_tree_widget.itemDoubleClicked.connect(self._dbclick_robot_list_item)

        #button event connection

        self._widget.capture_teleop_btn.pressed.connect(self._capture_teleop)
        self._widget.release_teleop_btn.pressed.connect(self._release_teleop)
        #signal event connection

        self._update_robot_list_signal.connect(self._update_robot_list)
        self.connect(self, SIGNAL("capture"), self._show_capture_teleop_message)
        self.connect(self, SIGNAL("release"), self._show_release_teleop_message)
        self.connect(self, SIGNAL("error"), self._show_error_teleop_message)

        context.add_widget(self._widget)

        self._widget.release_teleop_btn.setEnabled(False)
        self.teleop_app_info = TeleopManager(
            image_received_slot=self._widget.camera_view.on_compressed_image_received,
            event_callback=self._refresh_robot_list,
            capture_event_callback=self._capture_event_callback,
            release_event_callback=self._release_event_callback,
            error_event_callback=self._error_event_callback,
        )

        self.robot_item_list = {}
        self.current_robot = None
        self.current_captured_robot = None

        #virtual joystick
        self._widget.virtual_joystick_view.joystick_feedback().connect(self.on_joystick_feedback)
        self._widget.virtual_joystick_view.mouse_released().connect(self.on_mouse_released)

        #keyboard control
        for k in self._widget.children():
            try:
                k.keyPressEvent = self.on_key_press
                k.keyReleaseEvent = self.on_key_release
            except:
                pass

    def shutdown_plugin(self):
        if self.current_captured_robot:
            self.teleop_app_info._release_teleop(self.current_captured_robot["rocon_uri"])
        self.teleop_app_info.shutdown()

    def on_key_release(self, e):
        if e.isAutoRepeat():
            return
        else:
            self.teleop_app_info.publish_cmd_vel(0.0, 0.0)

    def on_key_press(self, e):
        linear_command = 0.0
        angular_command = 0.0
        if Qt.Key_Up == e.key():
            linear_command = self.maximum_linear_velocity
        elif Qt.Key_Down == e.key():
            linear_command = -self.maximum_linear_velocity
        elif Qt.Key_Right == e.key():
            angular_command = -self.maximum_angular_velocity
        elif Qt.Key_Left == e.key():
            angular_command = self.maximum_angular_velocity
        self.teleop_app_info.publish_cmd_vel(linear_command, angular_command)

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
        self.teleop_app_info.publish_cmd_vel(linear_command, angular_command)

    @pyqtSlot()
    def on_mouse_released(self):
        self.teleop_app_info.publish_cmd_vel(0.0, 0.0)

    def _show_capture_teleop_message(self, rtn):
        if rtn:
            QMessageBox.warning(self._widget, 'SUCCESS', "CAPTURE!!!!", QMessageBox.Ok | QMessageBox.Ok)
            for k in self.robot_item_list.keys():
                if self.robot_item_list[k] == self.current_robot:
                    k.setBackground(0, QBrush(Qt.SolidPattern))
                    k.setBackgroundColor(0, QColor(0, 255, 0, 255))
                    robot_name = k.text(0)
                    k.setText(0, str(robot_name) + " (captured)")
                else:
                    k.setBackground(0, QBrush(Qt.NoBrush))
                    k.setBackgroundColor(0, QColor(0, 0, 0, 0))
                    robot_name = k.text(0)
            self._widget.capture_teleop_btn.setEnabled(False)
            self._widget.release_teleop_btn.setEnabled(True)
            self.current_captured_robot = self.current_robot
        else:
            QMessageBox.warning(self._widget, 'FAIL', "FAIURE CAPTURE!!!!", QMessageBox.Ok | QMessageBox.Ok)
        self._widget.setDisabled(False)

    def _show_release_teleop_message(self, rtn):
        if rtn:
            QMessageBox.warning(self._widget, 'SUCCESS', "RELEASE!!!!", QMessageBox.Ok | QMessageBox.Ok)
            for k in self.robot_item_list.keys():
                if self.robot_item_list[k] == self.current_captured_robot:
                    k.setBackground(0, QBrush(Qt.NoBrush))
                    k.setBackgroundColor(0, QColor(0, 0, 0, 0))
                    robot_name = k.text(0)
                    k.setText(0, robot_name[:robot_name.find(" (captured)")])
        else:
            QMessageBox.warning(self._widget, 'FAIL', "FAIURE RELEASE!!!!", QMessageBox.Ok | QMessageBox.Ok)

        self._widget.setDisabled(False)
        self._widget.capture_teleop_btn.setEnabled(True)
        self._widget.release_teleop_btn.setEnabled(False)
        self.current_captured_robot = None

    def _show_error_teleop_message(self, err):
        QMessageBox.warning(self._widget, 'ERROR', err, QMessageBox.Ok | QMessageBox.Ok)
        self._widget.setDisabled(False)
        self._widget.capture_teleop_btn.setEnabled(True)
        self._widget.release_teleop_btn.setEnabled(True)

    def _capture_event_callback(self, rtn):
        try:
            self.emit(SIGNAL("capture"), rtn)
        except:
            pass

    def _release_event_callback(self, rtn):
        try:
            self.emit(SIGNAL("release"), rtn)
        except:
            pass

    def _error_event_callback(self, err):
        try:
            self.emit(SIGNAL("error"), err)
        except:
            pass

    def _release_teleop(self):
        if self.current_robot != self.current_captured_robot:
            print "NO Capture robot (captured: %s)" % self.current_captured_robot['name'].string
            return
        self.teleop_app_info._release_teleop(self.current_robot["rocon_uri"])
        self._widget.setDisabled(True)

    def _capture_teleop(self):
        if self.current_robot == None:
            print "NO Select robot"
            return
        elif self.current_captured_robot:
            print "Already captured robot"
            return
        self.teleop_app_info._capture_teleop(self.current_robot["rocon_uri"])
        self._widget.setDisabled(True)
        pass

    def _dbclick_robot_list_item(self):
        if self.current_captured_robot == None:
            self._capture_teleop()
        else:
            self._release_teleop()

    def _update_robot_list(self):
        self._widget.robot_list_tree_widget.clear()
        robot_list = self.teleop_app_info.robot_list

        for k in robot_list.values():
            robot_item = QTreeWidgetItem(self._widget.robot_list_tree_widget)
            robot_item.setText(0, k["name"].string)
            self.robot_item_list[robot_item] = k

    def _select_robot_list_tree_item(self, Item):
        if not Item in self.robot_item_list.keys():
            print "HAS NO KEY"
        else:
            self.current_robot = self.robot_item_list[Item]
            if self.current_robot == self.current_captured_robot:
                self._widget.release_teleop_btn.setEnabled(True)
            else:
                self._widget.release_teleop_btn.setEnabled(False)

    def _refresh_robot_list(self):
        self._update_robot_list_signal.emit()
        pass
