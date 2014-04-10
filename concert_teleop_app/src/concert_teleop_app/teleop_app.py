#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
#system
from __future__ import division
import os
import math
#pyqt
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, QAbstractListModel, pyqtSignal
from python_qt_binding.QtCore import pyqtSlot, SIGNAL,SLOT, QRectF , QTimer, QEvent, QUrl
from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget, QLabel, QComboBox
from python_qt_binding.QtGui import QSizePolicy,QTextEdit, QCompleter, QBrush, QDialog, QColor, QPen, QPushButton
from python_qt_binding.QtGui import QTabWidget, QPlainTextEdit,QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox
from python_qt_binding.QtGui import QTreeWidgetItem, QPixmap, QGraphicsScene
from python_qt_binding.QtDeclarative import QDeclarativeView
from python_qt_binding.QtSvg import QSvgGenerator
#ros
import rospkg
from teleop_app_info import TeleopAppInfo
#rqt
from qt_gui.plugin import Plugin

import rospy

##############################################################################
# Teleop App
##############################################################################


class TeleopApp(Plugin):
    _update_robot_list_signal = Signal()

    CAMERA_FPS = (1000 / 20)
    CONTROL_INTERVAL = 10
    D2R = 3.141592 / 180
    R2D = 180 / 3.141592
    LINEAR_V = 2.0
    ANGULAR_V = 90 * D2R
    UP_KEY = 16777235
    DOWN_KEY = 16777237
    LEFT_KEY = 16777234
    RIGHT_KEY = 16777236

    def __init__(self, context):
        self._context = context
        super(TeleopApp, self).__init__(context)
        self.initialised = False
        self.setObjectName('Teleop App')

        self.is_setting_dlg_live = False
        self._widget = QWidget()
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_teleop_app'), 'ui', 'teleop_app.ui')
        self._widget.setObjectName('TeleopAppUi')
        loadUi(ui_file, self._widget, {})
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        #list item click event
        self._widget.robot_list_tree_widget.itemClicked.connect(self._select_robot_list_tree_item)
        self._widget.robot_list_tree_widget.itemDoubleClicked.connect(self._dbclick_robot_list_item)

        #button event connection

        self._widget.capture_teleop_btn.pressed.connect(self._capture_teleop)
        self._widget.release_teleop_btn.pressed.connect(self._release_teleop)
        #signal event connection

        self._widget.destroyed.connect(self._exit)

        self._update_robot_list_signal.connect(self._update_robot_list)
        self.connect(self, SIGNAL("capture"), self._show_capture_teleop_message)
        self.connect(self, SIGNAL("release"), self._show_release_teleop_message)
        self.connect(self, SIGNAL("error"), self._show_error_teleop_message)

        context.add_widget(self._widget)

        #init
        self.scene = QGraphicsScene(self._widget)
        self._widget.camera_view.setScene(self.scene)
        self.timer_image = QTimer(self._widget)
        self.timer_image.timeout.connect(self._display_image)
        self.timer_image.start(self.CAMERA_FPS)

        self.timer_contorl = QTimer(self._widget)
        self.timer_contorl.timeout.connect(self._on_move)

        self._widget.release_teleop_btn.setEnabled(False)
        self.teleop_app_info = TeleopAppInfo()
        self.teleop_app_info._reg_event_callback(self._refresh_robot_list)
        self.teleop_app_info._reg_capture_event_callback(self._capture_event_callback)
        self.teleop_app_info._reg_release_event_callback(self._release_event_callback)
        self.teleop_app_info._reg_error_event_callback(self._error_event_callback)

        self.robot_item_list = {}
        self.current_robot = None
        self.current_captured_robot = None

        #virtual joystick controll
        self.last_linear_command = 0.0
        self.last_angular_command = 0.0
        vj_path = os.path.join(rospack.get_path('concert_teleop_app'), 'ui', 'virtual_joystick.qml')
        self._widget.vj_view.setSource(QUrl(vj_path))
        self._widget.vj_view.setResizeMode(QDeclarativeView.SizeRootObjectToView)
        rootObject = self._widget.vj_view.rootObject()
        rootObject.feedback.connect(self.joystick_feedback_event)
        rootObject.pressedHoverChanged.connect(self.pressed_hover_changed_event)
        #keyboard control
        for k in self._widget.children():
            try:
                k.keyPressEvent = self.on_key_press
                k.keyReleaseEvent = self.on_key_release
            except:
                pass

    def on_key_release(self, e):
        if e.isAutoRepeat():
            return
        else:
            self._stop()

    def on_key_press(self, e):
        self.last_linear_command = 0
        self.last_angular_command = 0
        if self.UP_KEY == e.key():
            self.last_linear_command = self.LINEAR_V
        if self.DOWN_KEY == e.key():
            self.last_linear_command = -self.LINEAR_V
        if self.RIGHT_KEY == e.key():
            self.last_angular_command = -self.ANGULAR_V
        if self.LEFT_KEY == e.key():
            self.last_angular_command = self.ANGULAR_V
        self._set_cmf_vel(self.last_linear_command, self.last_angular_command)

    def joystick_feedback_event(self, x, y):
        '''
        Takes a normalised double pair coming in with which we can apply to our
        velocity bounds.

        We also use a narrow dead zone along each x and y axis.

        :param double x: normalised (-1.0, 1.0) left to right position
        :param double y: normalised (-1.0, 1.0) bottom to top position
        '''
        dead_zone_radius = 0.05
        if math.fabs(x) < dead_zone_radius:
            self.last_angular_command = 0.0
        else:
            self.last_angular_command = -x * self.ANGULAR_V
        if  math.fabs(y) < dead_zone_radius:
            self.last_linear_command = 0.0
        else:
            self.last_linear_command = y * self.LINEAR_V

    def pressed_hover_changed_event(self, ishover):
        if ishover:
            self.timer_contorl.start(self.CONTROL_INTERVAL)
        else:
            self._stop()
            self.timer_contorl.stop()

    def _on_move(self):
        self._set_cmf_vel(self.last_linear_command, self.last_angular_command)

    def _exit(self):
        if self.current_captured_robot:
            self.teleop_app_info._release_teleop(self.current_captured_robot["rocon_uri"])

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

    def _set_cmf_vel(self, linear, angular):
        self.teleop_app_info._request_teleop_cmd_vel(linear, angular)

    def _stop(self):
        self.teleop_app_info._request_teleop_cmd_vel(0, 0)

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

    def _check_image_format(self, image_format):
        formats = ['JPG', 'PNG', 'BMP', 'JPEG']
        for f in formats:
            if f in image_format.upper():
                return f

    def _display_image(self):
        image = self.teleop_app_info.image_data
        if image:
            if len(self.scene.items()) > 1:
                self.scene.removeItem(self.scene.items()[0])
            image_data = self.teleop_app_info.image_data.data
            image_format = self._check_image_format(self.teleop_app_info.image_data.format)
            pixmap = QPixmap()
            pixmap.loadFromData(image_data, format=image_format)
            self._widget.camera_view.fitInView(QRectF(0, 0, pixmap.width(), pixmap.height()), Qt.KeepAspectRatio)
            self.scene.addPixmap(pixmap)
            self.scene.update()
        else:
            self.scene.clear()
