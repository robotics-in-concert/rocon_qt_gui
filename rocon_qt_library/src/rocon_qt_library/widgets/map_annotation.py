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
from python_qt_binding.QtGui import QLabel, QTextEdit, QWidget, QPen, QBrush, QColor, QPolygonF, QMatrix, QMessageBox, QFont, QGraphicsView, QDialog, QPushButton, QVBoxLayout, QHBoxLayout

from qt_gui.plugin import Plugin
import rospkg
import rospy
import tf
from tf.transformations import quaternion_from_euler

from rocon_qt_library.interfaces import MapAnnotationInterface
from rocon_qt_library.views import QMapView

#
# Map Annotation
#


class QMapAnnotation(QWidget):

    def __init__(self, parent=None):
        super(QMapAnnotation, self).__init__()
        self._parent = parent
        self.destroyed.connect(self.close)
        self._load_ui()

        self._scene = self.map_view._scene
        self._viz_markers_pose = None
        self._viz_marker_items = []
        self._viz_marker_polygon = QPolygonF([QPointF(-4, 4), QPointF(-4, -4), QPointF(12, 0)])

        self._callback = {}
        self._callback['save_annotation'] = None

        self.annotating = False
        self.annotation = {}
        self.annotation_item = None
        self.point_x = 0
        self.point_y = 0

        self._init_events()

    def _load_ui(self):
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_qt_library'), 'ui', 'map_annotation.ui')
        loadUi(ui_file, self, {'QMapView': QMapView})

    def _init_events(self):
        self.on_map_received = self.map_view.map_cb

        self.map_view._scene.mousePressEvent = self._mousePressEvent
        self.map_view._scene.mouseReleaseEvent = self._mouseReleaseEvent
        self.map_view._scene.mouseMoveEvent = self._mouseMoveEvent

        self.ar_marker_cbox.stateChanged.connect(self._check_ar_marker_cbox)
        self.table_cbox.stateChanged.connect(self._check_table_cbox)
        self.save_annotation_btn.clicked.connect(self._save_annotation)

        self.x_txt.currentCharFormatChanged.connect(self.test)

    def test(self):
        print 'name changed'

    def _save_annotation(self):
        if self.annotation:
            self._get_annotating_info()
            self._callback['save_annotation'](self.annotation)
            self.annotation = None

            if self.annotation_item:
                self._scene.removeItem(self.annotation_item)
                self.annotation_item = None
        else:
            print "Annotating first"

    def _check_ar_marker_cbox(self, data):
        if data == Qt.Checked:
            self.table_cbox.setCheckState(Qt.Unchecked)
            self.map_view.setDragMode(QGraphicsView.NoDrag)
            if self.annotation_item:
                self._scene.removeItem(self.annotation_item)
                self.annotation_item = None
        else:
            self.map_view.setDragMode(QGraphicsView.ScrollHandDrag)

    def _check_table_cbox(self, data):
        if data == Qt.Checked:
            self.ar_marker_cbox.setCheckState(Qt.Unchecked)
            self.map_view.setDragMode(QGraphicsView.NoDrag)
            if self.annotation_item:
                self._scene.removeItem(self.annotation_item)
                self.annotation_item = None
        else:
            self.map_view.setDragMode(QGraphicsView.ScrollHandDrag)

    def _mouseReleaseEvent(self, e):
        if self.ar_marker_cbox.checkState() != Qt.Unchecked or self.table_cbox.checkState() != Qt.Unchecked:
            self.annotating = False
            self.table_cbox.setCheckState(Qt.Unchecked)
            self.ar_marker_cbox.setCheckState(Qt.Unchecked)

    def _mousePressEvent(self, e):
        if self.ar_marker_cbox.checkState() != Qt.Unchecked or self.table_cbox.checkState() != Qt.Unchecked:
            self.annotating = True
            self.annotation = {}
            self.point_x = e.scenePos().x()
            self.point_y = e.scenePos().y()
            anno_type = None
            if self.ar_marker_cbox.checkState() == Qt.Checked:
                anno_type = self.ar_marker_cbox.objectName()
            elif self.table_cbox.checkState() == Qt.Checked:
                anno_type = self.table_cbox.objectName()
            else:
                return

            self._set_annotating_info(anno_type=anno_type, x=self.point_x, y=self.point_y, radius=1)
            self.draw_annotations(self.annotation)

    def _mouseMoveEvent(self, e):
        if self.ar_marker_cbox.checkState() != Qt.Unchecked or self.table_cbox.checkState() != Qt.Unchecked:
            anno_type = None
            if self.ar_marker_cbox.checkState() == Qt.Checked:
                anno_type = self.ar_marker_cbox.objectName()
            elif self.table_cbox.checkState() == Qt.Checked:
                anno_type = self.table_cbox.objectName()
            else:
                return

            dx = e.scenePos().x() - self.point_x
            dy = e.scenePos().y() - self.point_y
            dist = math.hypot(dx, dy)
            yaw = math.degrees(math.atan2(dy, dx))

            self._set_annotating_info(anno_type=anno_type, x=self.point_x, y=self.point_y, radius=dist, yaw=yaw)
            self.draw_annotations(self.annotation)

    def _set_annotating_info(self, anno_type='', name='', x=0, y=0, height=0, radius=1, roll=90, pitch=0, yaw=0):
        self.annotation['type'] = anno_type
        self.annotation['x'] = x
        self.annotation['y'] = y
        self.annotation['height'] = height
        self.annotation['yaw'] = yaw
        self.annotation['roll'] = roll
        self.annotation['pitch'] = pitch
        self.annotation['scale'] = (radius, radius, radius)
        self.annotation['radius'] = radius
        self.annotation['name'] = ''

        self.name_txt.setText(str(self.annotation['name']))
        self.x_txt.setText(str(round(self.annotation['x'], 2)))
        self.y_txt.setText(str(round(self.annotation['y'], 2)))
        self.height_txt.setText(str(round(self.annotation['height'], 2)))
        self.radius_txt.setText(str(round(self.annotation['radius'], 2)))
        self.roll_txt.setText(str(round(self.annotation['roll'])))
        self.pitch_txt.setText(str(round(self.annotation['pitch'], 2)))
        self.yaw_txt.setText(str(round(self.annotation['yaw'], 2)))

    def _get_annotating_info(self):

        self.annotation['name'] = str(self.name_txt.toPlainText())

        self.annotation['x'] = float(self.x_txt.toPlainText())
        self.annotation['y'] = float(self.y_txt.toPlainText())
        self.annotation['height'] = float(self.height_txt.toPlainText())

        self.annotation['yaw'] = float(self.yaw_txt.toPlainText())
        self.annotation['roll'] = float(self.roll_txt.toPlainText())
        self.annotation['pitch'] = float(self.pitch_txt.toPlainText())

        radius = float(self.radius_txt.toPlainText())
        self.annotation['scale'] = (radius, radius, radius)
        self.annotation['radius'] = radius

    def init_map_annotation_interface(self, map_topic, viz_markers_received_slot, viz_markers_topic, scene_update_slot):
        self._map_annotation_interface = MapAnnotationInterface(map_received_slot=self.on_map_received,
                                                                map_topic=map_topic,
                                                                viz_markers_received_slot=viz_markers_received_slot,
                                                                viz_markers_topic=viz_markers_topic,
                                                                scene_update_slot=scene_update_slot)
        self._callback['save_annotation'] = self._map_annotation_interface.save_annotation

    @pyqtSlot(dict)
    def draw_scene(self, data):
        self.draw_map(data['map'])
        self.draw_viz_markers(data['viz_markers'])

    def draw_annotations(self, data):
        if not self.annotating:
            return
        old_item = None
        if self.annotation_item:
            old_item = self.annotation_item
        annotation_item = None

        if data['type'] == self.table_cbox.objectName():

            annotation_item = self._scene.addEllipse(data['x'] - data['scale'][0] / 2,
                                                     data['y'] - data['scale'][1] / 2,
                                                     data['scale'][0], data['scale'][1],
                                                     pen=QPen(QColor(0, 0, 255)),
                                                     brush=QBrush(QColor(0, 0, 255, 125)))

        elif data['type'] == self.ar_marker_cbox.objectName():

            viz_marker_pose = QMatrix().rotate(data['yaw']).map(self._viz_marker_polygon).translated(data['x'], data['y'])
            annotation_item = self._scene.addPolygon(viz_marker_pose, pen=QPen(QColor(0, 0, 255)), brush=QBrush(QColor(0, 0, 255, 125)))

        if old_item:
            self._scene.removeItem(old_item)
        self.annotation_item = annotation_item

    def draw_map(self, data):
        self.map_view.map_cb(data)

    @pyqtSlot(list)
    def draw_viz_markers(self, data):
        old_items = []
        if len(self._viz_marker_items):
            for item in self._viz_marker_items:
                old_items.append(item)

        for viz_marker in data:
            if viz_marker['type'] is 9:
                # text
                viz_marker_text = viz_marker['text']
                viz_marker_pose_item = self._scene.addSimpleText(viz_marker_text, font=QFont())
                # Everything must be mirrored
                viz_marker_pose_item.translate(-viz_marker['x'], viz_marker['y'])
            elif viz_marker['type'] is 3:
                # table
                viz_marker_pose_item = self._scene.addEllipse(viz_marker['x'] - viz_marker['scale'][0] / 2, viz_marker['y'] - viz_marker['scale'][1] / 2, viz_marker['scale'][0], viz_marker['scale'][1], pen=QPen(QColor(255, 0, 0)), brush=QBrush(QColor(255, 0, 0)))
                # Everything must be mirrored
                self._mirror(viz_marker_pose_item)
            else:
                # marker
                viz_marker_pose = QMatrix().rotate(viz_marker['yaw']).map(self._viz_marker_polygon).translated(viz_marker['x'], viz_marker['y'])
                viz_marker_pose_item = self._scene.addPolygon(viz_marker_pose, pen=QPen(QColor(255, 0, 0)), brush=QBrush(QColor(255, 0, 0)))
                # Everything must be mirrored
                self._mirror(viz_marker_pose_item)

            self._viz_marker_items.append(viz_marker_pose_item)
        if len(old_items):
            for item in old_items:
                self._scene.removeItem(item)
                self._scan_items.remove(item)

    def _mirror(self, item):
        item.scale(-1, 1)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
