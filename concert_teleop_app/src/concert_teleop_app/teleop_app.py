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
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, QAbstractListModel, pyqtSignal, pyqtSlot, SIGNAL,SLOT, QRectF
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

    _display_image_signal = Signal()

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
        self._display_image_signal.connect(self._display_image)

        context.add_widget(self._widget)
        #init
        self.scene = QGraphicsScene()
        self._widget.camera_view.setScene(self.scene)
         #
        self.robot_list = {}
        self.current_robot = ""
        self.teleop_image_width = 320
        self.teleop_image_height = 240
         #
        self.teleop_app_info = TeleopAppInfo()
        self.teleop_app_info._reg_event_callback(self._refresh)
        pass

    def _update_robot_list(self):
        self._widget.robot_list_tree_widget.clear()
        for k in self.robot_list.values():
            robot_item = QTreeWidgetItem(self._widget.robot_list_tree_widget)
            robot_item.setText(0, k['name'])
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
        self.current_robot = Item.text(0)
        pass
    
    def _refresh(self):
        self._display_image_signal.emit()
        pass
    
    def _display_image(self):
        image = self.teleop_app_info.image_data
        pixmap = QPixmap(QImage(image.data, image.width, image.height, image.step, QImage.Format_RGB888))
        #self._widget.fitInView (self, QRectF rect, Qt.AspectRatioMode mode = Qt.IgnoreAspectRatio)
        self.scene.addPixmap(pixmap)
        self.scene.update()
        pass
            

    def test_func(self):
        print "it is test"
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_teleop_app'), 'resources/images', 'rocon_logo.png')
        print ui_file
        pixmap = QPixmap(ui_file)
        self.scene.addPixmap(pixmap)
        self.scene.update()
