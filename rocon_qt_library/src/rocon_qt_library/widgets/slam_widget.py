#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
#
# Imports
#

from __future__ import division
import os
import math

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, SIGNAL, pyqtSlot, QPointF
from python_qt_binding.QtGui import QWidget, QPen, QBrush, QColor, QPolygonF, QMatrix, QMessageBox

from qt_gui.plugin import Plugin
import rospkg
import rospy
import tf
from tf.transformations import quaternion_from_euler

from rocon_qt_library.interfaces import SlamWidgetInterface
from rocon_qt_library.views import QMapView

#
# SlamWidget
#


class QSlamWidget(QWidget):

    def __init__(self, parent=None):
        super(QSlamWidget, self).__init__()
        self._parent = parent
        self._slam_widget_interface = None

        self._load_ui()
        self._init_events()
        self.destroyed.connect(self.close)

        self._scene = self.map_view._scene
        self._robot_pose = None
        self._robot_pose_item = None
        self._robot_polygon = QPolygonF([QPointF(-4, 4), QPointF(-4, -4), QPointF(12, 0)])

        self._scan = None
        self._scan_items = []

        self._callback={}
        self._callback['save_map'] = None

    def _load_ui(self):
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_qt_library'), 'ui', 'slam_widget.ui')
        loadUi(ui_file, self, {'QMapView': QMapView})

    def _init_events(self):
        self.on_map_received = self.map_view.map_cb
        self.save_map_btn.pressed.connect(self.save_map)
        self.connect(self, SIGNAL("map_saved"), self.show_saved_map_message)

    def init_slam_widget_interface(self, map_topic, scan_received_slot, scan_topic, robot_pose_received_slot, robot_pose_topic, wc_namespace, map_saved_callbacks):
        self._slam_widget_interface = SlamWidgetInterface(map_received_slot=self.on_map_received,
                                                          map_topic=map_topic,
                                                          scan_received_slot=scan_received_slot,
                                                          scan_topic=scan_topic,
                                                          robot_pose_received_slot=robot_pose_received_slot,
                                                          robot_pose_topic=robot_pose_topic,
                                                          wc_namespace=wc_namespace,
                                                          map_saved_callbacks=map_saved_callbacks)
        self._callback['save_map'] = self._slam_widget_interface.save_map

    def unset_slam_interface(self):
        self._slam_widget_interface = None
    
    def map_saved_callback(self, msg):
        try:
            self.emit(SIGNAL("map_saved"), msg)
        except: 
            pass

    def show_saved_map_message(self, rtn):
        (success, message) = rtn
        if success:
            QMessageBox.warning(self, 'SUCCESS', "SAVE!!!!", QMessageBox.Ok | QMessageBox.Ok)           
        else:
            QMessageBox.warning(self, 'FAIL', "FAIURE CAPTURE[%s]"%str(message), QMessageBox.Ok | QMessageBox.Ok)
        self.setDisabled(False)
    
    def save_map(self):
        if self._slam_widget_interface:
            self.setDisabled(True)
            map_name = str(self.map_name_txt.toPlainText()).lower().replace(' ', '_')
            world_name = str(self.world_name_txt.toPlainText()).lower().replace(' ', '_')
            self.map_name_txt.setPlainText(map_name)
            self.world_name_txt.setPlainText(world_name)
            self._callback['save_map'](world=world_name, map_name=map_name)
        else:
            QMessageBox.warning(self, 'FAIL', "No map has created",QMessageBox.Ok | QMessageBox.Ok)

    @pyqtSlot(list)
    def draw_scan(self, data):
        old_items = []
        if len(self._scan_items):
            for item in self._scan_items:
                old_items.append(item)

        for pt in data:
            scan_item = self._scene.addRect(pt[0], pt[1], 1, 1, pen=QPen(QColor(0, 255, 0)), brush=QBrush(QColor(0, 255, 0)))
            # Everything must be mirrored
            self._mirror(scan_item)
            self._scan_items.append(scan_item)

        if len(old_items):
            for item in old_items:
                self._scene.removeItem(item)
                self._scan_items.remove(item)

    @pyqtSlot(dict)
    def draw_robot_pose(self, data):
        old_item = None
        if self._robot_pose_item:
            old_item = self._robot_pose_item

        robot_pose = QMatrix().rotate(data['yaw']).map(self._robot_polygon).translated(data['x'], data['y'])
        self._robot_pose_item = self._scene.addPolygon(robot_pose, pen=QPen(QColor(255, 0, 0)), brush=QBrush(QColor(255, 0, 0)))
        # Everything must be mirrored
        self._mirror(self._robot_pose_item)
        if old_item:
            self._scene.removeItem(old_item)

    def _mirror(self, item):
        item.scale(-1, 1)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
