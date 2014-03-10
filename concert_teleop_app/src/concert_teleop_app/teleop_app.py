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
import time
#pyqt
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, QAbstractListModel, pyqtSignal, pyqtSlot, SIGNAL,SLOT, QRectF , QTimer
from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget,QLabel, QComboBox
from python_qt_binding.QtGui import QSizePolicy,QTextEdit, QCompleter, QBrush,QDialog, QColor, QPen, QPushButton
from python_qt_binding.QtGui import QTabWidget, QPlainTextEdit,QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox
from python_qt_binding.QtGui import QTreeWidgetItem, QPixmap, QGraphicsScene
from python_qt_binding.QtSvg import QSvgGenerator
#ros
import rospkg
from teleop_app_info import TeleopAppInfo
#rqt
from qt_gui.plugin import Plugin

##############################################################################
# Teleop App
##############################################################################


class TeleopApp(Plugin):
    _update_robot_list_signal = Signal()
    _captured_teleop_signal = Signal()

    CAMERA_FPS = (1000 / 20)
    D2R = 3.141592 / 180
    R2D = 180 / 3.141592
    LINEAR_V = 1.5
    ANGULAR_V = 60 * D2R

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
        #concert item click event

        self._widget.robot_list_tree_widget.itemClicked.connect(self._select_robot_list_tree_item)
        #button event connection
        self._widget.backward_btn.setAutoRepeat(True)
        self._widget.forward_btn.setAutoRepeat(True)
        self._widget.left_turn_btn.setAutoRepeat(True)
        self._widget.right_turn_btn.setAutoRepeat(True)

        self._widget.backward_btn.clicked.connect(self._backward)
        self._widget.forward_btn.clicked.connect(self._forward)
        self._widget.left_turn_btn.clicked.connect(self._left_turn)
        self._widget.right_turn_btn.clicked.connect(self._right_turn)

        self._widget.backward_btn.released.connect(self._stop)
        self._widget.forward_btn.released.connect(self._stop)
        self._widget.left_turn_btn.released.connect(self._stop)
        self._widget.right_turn_btn.released.connect(self._stop)

        self._widget.test_capture_teleop_btn.pressed.connect(self._capture_teleop)
        self._update_robot_list_signal.connect(self._update_robot_list)
        self._captured_teleop_signal.connect(self._show_capture_teleop_message)
        context.add_widget(self._widget)

        #init
        self.scene = QGraphicsScene()
        self._widget.camera_view.setScene(self.scene)
        self.timer = QTimer(self._widget)
        self.timer.timeout.connect(self._display_image)
        self.timer.start(self.CAMERA_FPS)

        self.teleop_app_info = TeleopAppInfo()
        self.teleop_app_info._reg_event_callback(self._refresh_robot_list)
        self.teleop_app_info._reg_capture_event_callback(self._capture_event_callback)

        self.robot_item_list = {}
        self.current_robot = None

    def _show_capture_teleop_message(self):
        QMessageBox.warning(self._widget, 'SUCCESS', "CAPTURE!!!!", QMessageBox.Ok | QMessageBox.Ok)
        self._widget.test_capture_teleop_btn.setEnabled(False)
        self._widget.setDisabled(False)
        pass

    def _capture_event_callback(self):
        self._captured_teleop_signal.emit()

    def _capture_teleop(self):
        if self.current_robot == None:
                print "NO Select robot"
                return
        print ("_capture teleop: %s" % self.current_robot["rocon_uri"])
        self.teleop_app_info._capture_teleop(self.current_robot["rocon_uri"])
        self._widget.setDisabled(True)
        pass

    def _update_robot_list(self):
        self._widget.robot_list_tree_widget.clear()
        robot_list = self.teleop_app_info.robot_list

        for k in robot_list.values():
            robot_item = QTreeWidgetItem(self._widget.robot_list_tree_widget)
            robot_item.setText(0, k["name"].string)
            self.robot_item_list[robot_item] = k

    def _backward(self):
        if self._widget.backward_btn.isDown():
            self.teleop_app_info._request_teleop_cmd_vel(-self.LINEAR_V, 0)
        else:
            self._stop()

    def _forward(self):
        if self._widget.forward_btn.isDown():
            self.teleop_app_info._request_teleop_cmd_vel(self.LINEAR_V, 0)
        else:
            self._stop()

    def _left_turn(self):
        if self._widget.left_turn_btn.isDown():
            self.teleop_app_info._request_teleop_cmd_vel(0, self.ANGULAR_V)
        else:
            self._stop()

    def _right_turn(self):
        if self._widget.right_turn_btn.isDown():
            self.teleop_app_info._request_teleop_cmd_vel(0, -self.ANGULAR_V)
        else:
            self._stop()

    def _stop(self):
        self.teleop_app_info._request_teleop_cmd_vel(0, 0)

    def _select_robot_list_tree_item(self, Item):
        print '_select_robot: ' + Item.text(0)
        self.current_robot = self.robot_item_list[Item]
        pass

    def _refresh_robot_list(self):
        self._update_robot_list_signal.emit()
        pass

    def _display_image(self):
        image = self.teleop_app_info.image_data
        if image:
            if len(self.scene.items()) > 1:
                self.scene.removeItem(self.scene.items()[0])

            image_data = self.teleop_app_info.image_data.data
            pixmap = QPixmap()
            pixmap.loadFromData(image_data, format="PNG",)
            self._widget.camera_view.fitInView(QRectF(0, 0, pixmap.width(), pixmap.height()), Qt.KeepAspectRatio)
            self.scene.addPixmap(pixmap)
            self.scene.update()
