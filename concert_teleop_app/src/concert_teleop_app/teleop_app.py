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
    CAMERA_FPS = (1000 / 20)

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
        #button event connection
        #concert item click event
        self._widget.robot_list_tree_widget.itemClicked.connect(self._select_robot_list_tree_item)
        self._widget.test_btn.pressed.connect(self.test_func)
        self._widget.test_add_robot_btn.pressed.connect(self._add_robot)
        self._widget.test_delete_robot_btn.pressed.connect(self._delete_robot)

        self._widget.backward_btn.pressed.connect(self._backward)
        self._widget.forward_btn.pressed.connect(self._forward)
        self._widget.left_turn_btn.pressed.connect(self._left_turn)
        self._widget.right_turn_btn.pressed.connect(self._right_turn)
        self._widget.test_capture_teleop_btn.pressed.connect(self._capture_teleop)
        self._update_robot_list_signal.connect(self._update_robot_list)
        context.add_widget(self._widget)
        #init
        self.scene = QGraphicsScene()
        self._widget.camera_view.setScene(self.scene)

        self.timer = QTimer(self._widget)
        self.timer.timeout.connect(self._display_image)
        self.timer.start(self.CAMERA_FPS)

        self.teleop_app_info = TeleopAppInfo()
        self.teleop_app_info._reg_event_callback(self._refresh_robot_list)

        self.robot_item_list = {}
        self.current_robot = None

        pass

    def _capture_teleop(self):
        if self.current_robot == None:
                print "NO Select robot"
                return
        print ("_capture teleop: %s"%self.current_robot["rocon_uri"])
        self.teleop_app_info._capture_teleop(self.current_robot["rocon_uri"])
        pass

    def _update_robot_list(self):
        self._widget.robot_list_tree_widget.clear()
        robot_list = self.teleop_app_info.robot_list

        for k in robot_list.values():
            robot_item = QTreeWidgetItem(self._widget.robot_list_tree_widget)
            robot_item.setText(0, k["name"].string)
            self.robot_item_list[robot_item] = k
        pass

    def _add_robot(self):
        print "add robot"
        robot_list = self.robot_list
        robot_name = "robot_" + str(len(robot_list))
        self.robot_list[robot_name] = {}
        self.robot_list[robot_name]['name'] = robot_name
        self._update_robot_list()
        pass

    def _delete_robot(self):
        print "delete robot"
        pass

    def _backward(self):
        print "%s: backward robot" % self.current_robot
        pass

    def _forward(self):
        print "%s: forward robot" % self.current_robot
        pass

    def _left_turn(self):
        print "%s: left_turn robot" % self.current_robot
        pass

    def _right_turn(self):
        print "%s: right_turn robot" % self.current_robot
        pass

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

            image = self.teleop_app_info.image_data
            pixmap = QPixmap(QImage(image.data, image.width, image.height, image.step, QImage.Format_RGB888))
            self._widget.camera_view.fitInView(QRectF(0, 0, image.width, image.height), Qt.KeepAspectRatio)
            self.scene.addPixmap(pixmap)
            self.scene.update()

    def test_func(self):
        print "it is test"