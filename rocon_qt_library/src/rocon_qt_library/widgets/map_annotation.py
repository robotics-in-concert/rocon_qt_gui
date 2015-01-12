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
import copy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, SIGNAL, pyqtSlot, QPointF
from python_qt_binding.QtGui import QWidget, QPen, QBrush, QColor, QPolygonF, QMatrix, QFont, QGraphicsView, QDialog, QPushButton, QVBoxLayout, QHBoxLayout

from qt_gui.plugin import Plugin
import rospkg
import rospy
import tf
from tf.transformations import quaternion_from_euler

import rocon_qt_library.utils as utils
from rocon_qt_library.interfaces import MapAnnotationInterface
from rocon_qt_library.views import QMapView
from world_canvas_client import AnnotationCollection
from visualization_msgs.msg import Marker

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
        self._viz_marker_items = {}
        self._viz_marker_items["annotations"] = [] 
        self._viz_marker_items["new_annotations"] = [] 
        self._viz_marker_polygon = QPolygonF([QPointF(-4, 4), QPointF(-4, -4), QPointF(12, 0)])

        self._callback = {}
        self._callback['save_annotation'] = None
        self._callback['load_world'] = None
        self._callback['add_annotation'] = None
        self._wc = None
        self._init_events()
        self._drawing_objects = {}

        self._init_variables_for_drawing()
        self._init_variableS_for_list()

    def _init_variableS_for_list(self):
        self._annotation_name_list = []
        self._new_annotation_name_list = []
        self._map_name_list = []

    def _init_variables_for_drawing(self):
        self.annotating = False
        self.annotation = {}
        self.annotation_item = None
        self.point_x = 0
        self.point_y = 0
        self.map_resolution = 1

        self.anno_type = None

        self._selected_map = None


    def _load_ui(self):
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_qt_library'), 'ui', 'map_annotation.ui')
        loadUi(ui_file, self, {'QMapView': QMapView})

        world_name =  rospy.get_param('~default_world_name',None)
        self._enable_buttons(False)

    def _init_events(self):
        self.on_map_received = self.map_view.map_cb

        self.map_view._scene.mousePressEvent = self._mousePressEvent
        self.map_view._scene.mouseReleaseEvent = self._mouseReleaseEvent
        self.map_view._scene.mouseMoveEvent = self._mouseMoveEvent

        self.ar_marker_cbox.stateChanged.connect(self._check_ar_marker_cbox)
        self.location_cbox.stateChanged.connect(self._check_location_cbox)
        self.add_annotation_btn.clicked.connect(self._add_annotation)
        self.remove_annotation_btn.clicked.connect(self._remove_annotation)
        self.save_annotation_btn.clicked.connect(self._save_annotation)
        self.load_world_btn.clicked.connect(self._load_world)
        self.map_select_combobox.currentIndexChanged.connect(self._select_map_item_clicked)
        self.annotations_list_widget.itemClicked.connect(self._annotation_list_item_clicked)

        self.connect(self, SIGNAL("show_message"), utils.show_message)
        self.connect(self, SIGNAL("draw_scene"), self.draw_scene)
        self.connect(self, SIGNAL("update_annotation_list"), self._update_annotation_list)
        self.connect(self, SIGNAL("update_map_selector"), self._update_map_selector)

    def init_map_annotation_interface(self, scene_update_slot, wc_namespace):
        self._map_annotation_interface = MapAnnotationInterface(scene_update_slot=self.update_scene, wc_namespace=wc_namespace)
        self._callback['list_world'] = self._map_annotation_interface.get_list_world
        self._callback['load_world'] = self._map_annotation_interface.load_world
        self._callback['load_map'] = self._map_annotation_interface.load_map
        self._callback['add_annotation']  = self._map_annotation_interface.add_annotation
        self._callback['remove_annotation']  = self._map_annotation_interface.remove_annotation
        self._callback['save_annotation'] = self._map_annotation_interface.save_annotations

        self.load_world_list()

    def load_world_list(self):
        success, message, self._world_list = self._callback['list_world']()
        
        if success:        
            self._update_world_list()
        else:
            self.emit(SIGNAL("show_message"), self, "Failed", message)

    def _update_world_list(self): 
        self.list_world_combobox.clear()
    
        for name in self._world_list:
            self.list_world_combobox.addItem(name)

    def _load_world(self):
        #world_name = self.world_name_text.toPlainText()
        world_name = str(self.list_world_combobox.currentText())

        success, message, self._map_name_list, self._annotation_name_list = self._callback['load_world'](world_name)

        if success:
            self._world_name = world_name
            self._selected_map = self._map_name_list[0]
            self.emit(SIGNAL("update_map_selector"))
            self._new_annotation_name_list = []
            self.emit(SIGNAL("update_annotation_list"))
            self._enable_buttons(True)
        else:
            self.emit(SIGNAL("show_message"), self, "Failed", message)


    def _enable_buttons(self, enable):
        self.add_annotation_btn.setEnabled(enable)
        self.remove_annotation_btn.setEnabled(enable)
        self.save_annotation_btn.setEnabled(enable)

    def _update_map_selector(self):
        self.map_select_combobox.clear()
        
        for name in self._map_name_list:
            self.map_select_combobox.addItem(name)

        if self._selected_map:
            idx = self.map_select_combobox.findText(self._selected_map)
            self.map_select_combobox.setCurrentIndex(idx)

    def _update_annotation_list(self):
        self.annotations_list_widget.clear()
        
        for n in self._annotation_name_list:
            self.annotations_list_widget.addItem(n)
            
        for n in self._new_annotation_name_list:
            self.annotations_list_widget.addItem('(new) '+n)

    def _add_annotation(self):
        if self.annotation:
            self._get_annotating_info()
            anno = copy.deepcopy(self.annotation)

            if anno['name'] == '':
                self.emit(SIGNAL('show_message'), self, "Failed", "Name is empty!")
            elif self.anno_type == 'ar_track_alvar_msgs/AlvarMarker' and not anno['name'].isdigit():
                self.emit(SIGNAL('show_message'), self, "Failed", "Marker ID should be int!")
            else:
                self._new_annotation_name_list = self._callback['add_annotation'](anno, self.anno_type)
                self.emit(SIGNAL("update_annotation_list"))
                self._clear_on_annotation()
                self.ar_marker_cbox.setCheckState(Qt.Unchecked)
                self.location_cbox.setCheckState(Qt.Unchecked)
                self._clear_edit_annotation_box()

    def _remove_annotation(self):
        if self._selected_annotation:
            ann_list, names = self._callback['remove_annotation'](self._selected_annotation)

            if ann_list == 'annotations':
                self._annotation_name_list = names
            elif ann_list == 'new_annotations':
                self._new_annotation_name_list = names
            self.emit(SIGNAL("update_annotation_list"))

    def _save_annotation(self):
        success, message = self._callback['save_annotation']()

        if success:
            success, message, self._map_name_list, self._annotation_name_list = self._callback['load_world'](self._world_name)
    
            if success:
                self.emit(SIGNAL("update_map_selector"))
                self._new_annotation_name_list = []
                self.emit(SIGNAL("update_annotation_list"))

                self._enable_buttons(True)
            else:
                self.emit(SIGNAL("show_message"), self, "Failed", message)
        else:
            self.emit(SIGNAL("show_message"), self, "Failed", message)

    def _set_annotating_info(self, anno_type='', name='', x=0, y=0, yaw=0, radius=1, roll=None, pitch=None, height=None):
        resolution = self._drawing_objects['map'].info.resolution
        origin = self._drawing_objects['map'].info.origin
        w = self._drawing_objects['map'].info.width
        h = self._drawing_objects['map'].info.height
        
        self.annotation['type'] = anno_type
        self.annotation['x'] = (-x + w) * resolution + origin.position.x
        self.annotation['y'] = y * resolution + origin.position.y
        self.annotation['yaw'] = yaw 
        self.annotation['radius'] = 0.2 #fixed value

        if height != None:
            self.annotation['height'] = height
        if roll != None:
            self.annotation['roll'] = roll
        if pitch != None:
            self.annotation['pitch'] = pitch

        radius = self.annotation['radius']
        self.annotation['scale'] = (radius, radius, radius)
        self.annotation['name'] = name

    def _clear_edit_annotation_box(self):
        self.name_txt.clear()
        self.x_txt.clear()
        self.y_txt.clear()
        self.height_txt.clear()
        self.radius_txt.clear()
        self.roll_txt.clear()
        self.pitch_txt.clear()
        self.yaw_txt.clear()

    def _update_edit_annotation_box(self):
        self.name_txt.setText(str(self.annotation['name']))
        self.x_txt.setText(str(round(self.annotation['x'], 3)))
        self.y_txt.setText(str(round(self.annotation['y'], 3)))
        self.height_txt.setText(str(round(self.annotation['height'], 3)))
        self.radius_txt.setText(str(round(self.annotation['radius'], 3)))
        self.roll_txt.setText(str(round(self.annotation['roll'])))
        self.pitch_txt.setText(str(round(self.annotation['pitch'], 3)))
        self.yaw_txt.setText(str(round(self.annotation['yaw'], 3)))

    def _set_edit_annotation_box(self, name="", x=0, y=0, height=0, radius=0, roll=0, pitch=0, yaw=0):
        self.name_txt.setText(str(name))
        self.x_txt.setText(str(round(x, 3)))
        self.y_txt.setText(str(round(y, 3)))
        self.height_txt.setText(str(round(height, 3)))
        self.radius_txt.setText(str(round(radius, 3)))
        self.roll_txt.setText(str(round(roll)))
        self.pitch_txt.setText(str(round(pitch, 3)))
        self.yaw_txt.setText(str(round(yaw, 3)))

    def _get_annotating_info(self):
        self.annotation['name'] = str(self.name_txt.toPlainText().strip())

        self.annotation['x'] = float(self.x_txt.toPlainText())
        self.annotation['y'] = float(self.y_txt.toPlainText())
        self.annotation['height'] = float(self.height_txt.toPlainText())

        self.annotation['yaw'] = float(self.yaw_txt.toPlainText())
        self.annotation['roll'] = float(self.roll_txt.toPlainText())
        self.annotation['pitch'] = float(self.pitch_txt.toPlainText())

        radius = float(self.radius_txt.toPlainText())
        self.annotation['scale'] = (radius, radius, radius)
        self.annotation['radius'] = radius

    @pyqtSlot(str, dict)
    def update_scene(self, object_name, data):
        if object_name == 'map':
            self._drawing_objects['map'] = data['map']
        elif object_name == 'annotations':
            self._drawing_objects['annotations'] = data['annotations']
        elif object_name == 'new_annotations':
            self._drawing_objects['new_annotations'] = data['new_annotations']
        elif object_name == 'on_annotation':
            self._drawing_objects['on_annotation'] = data
        self.emit(SIGNAL("draw_scene"), object_name)

    @pyqtSlot(str)
    def draw_scene(self, key):
        if key == 'map':
            self.draw_map(self._drawing_objects['map'])
        if key == 'annotations' or key == 'new_annotations':
            self.draw_viz_markers(self._drawing_objects[key], key)
        if key == 'on_annotation':
            self._draw_annotation(self._drawing_objects['on_annotation'])

    def _draw_annotation(self, data):
        old_item = None
        if self.annotation_item:
            old_item = self.annotation_item
        annotation_item = None

        if not data == {}:
            resolution = self._drawing_objects['map'].info.resolution
            origin = self._drawing_objects['map'].info.origin
            w = self._drawing_objects['map'].info.width
            h = self._drawing_objects['map'].info.height                         

            x = -((data['x'] - origin.position.x) / resolution - w)
            y = (data['y'] - origin.position.y) / resolution
            if data['type'] == "yocs_msgs/Waypoint":
                #annotation_item = self._scene.addEllipse(x - (data['scale'][0] / resolution), y -(data['scale'][1] / resolution), data['scale'][0] / resolution * 2, data['scale'][1] / resolution * 2, pen=QPen(QColor(0, 0, 255)), brush=QBrush(QColor(0, 0, 255, 125)))
                viz_marker_pose = QMatrix().rotate(-data['yaw']+180).map(self._viz_marker_polygon).translated(x, y)
                annotation_item = self._scene.addPolygon(viz_marker_pose, pen=QPen(QColor(0, 0, 255)), brush=QBrush(QColor(0, 0, 255, 125)))
            elif data['type'] == "ar_track_alvar_msgs/AlvarMarker": 
                viz_marker_pose = QMatrix().rotate(-(data['yaw']-90)+180).map(self._viz_marker_polygon).translated(x, y)
                annotation_item = self._scene.addPolygon(viz_marker_pose, pen=QPen(QColor(0, 0, 255)), brush=QBrush(QColor(0, 0, 255, 125)))
            annotation_item.setZValue(2)
        if old_item:
            self._scene.removeItem(old_item)
        self.annotation_item = annotation_item

    def draw_map(self, data):
        self.map_view.map_cb(data)

    @pyqtSlot(list)
    def draw_viz_markers(self, data, key):
        old_items = []
        if len(self._viz_marker_items[key]):
            for item in self._viz_marker_items[key]:
                old_items.append(item)
        self._viz_marker_items[key] = []
        for viz_marker in data:
            if viz_marker['type'] is Marker.TEXT_VIEW_FACING:
                # text
                viz_marker_text = viz_marker['text']
                viz_marker_pose_item = self._scene.addSimpleText(viz_marker_text, font=QFont())
                # Everything must be mirrored
                viz_marker_pose_item.translate(-viz_marker['x'], viz_marker['y'])
            elif viz_marker['type'] is Marker.CYLINDER:
                # # waypoint 
                # viz_marker_pose_item = self._scene.addEllipse(viz_marker['x'] - viz_marker['scale'][0] / 2, viz_marker['y'] - viz_marker['scale'][1] / 2, viz_marker['scale'][0], viz_marker['scale'][1], pen=QPen(QColor(255, 0, 0)), brush=QBrush(QColor(255, 0, 0)))
                # # Everything must be mirrored
                # self._mirror(viz_marker_pose_item)
                viz_marker_pose = QMatrix().scale(1,-1).rotate(viz_marker['yaw']+180).map(self._viz_marker_polygon).translated(-viz_marker['x'], viz_marker['y'])
                viz_marker_pose_item = self._scene.addPolygon(viz_marker_pose, pen=QPen(QColor(0, 255, 0)), brush=QBrush(QColor(0, 255, 0)))
            elif viz_marker['type'] is Marker.ARROW:
                # marker
                # Everything must be mirrored
                viz_marker_pose = QMatrix().scale(1,-1).rotate((viz_marker['yaw']-90)+180).map(self._viz_marker_polygon).translated(-viz_marker['x'], viz_marker['y'])
                viz_marker_pose_item = self._scene.addPolygon(viz_marker_pose, pen=QPen(QColor(255, 0, 0)), brush=QBrush(QColor(255, 0, 0)))
            else:
                rospy.logerr("Unknown Marker type : %s"%(viz_marker['type']))
            viz_marker_pose_item.setZValue(1)
            self._viz_marker_items[key].append(viz_marker_pose_item)
        if len(old_items):
            for item in old_items:
                self._scene.removeItem(item)
                #self._scan_items.remove(item)

    def _mirror(self, item):
        item.scale(-1, 1)

################################################################
# Moust Events
################################################################
    def _mouseReleaseEvent(self, e):
        if self.annotating:
            self.annotating = False
            #self.anno_type = None

            # canceling
            if self.point_x == e.scenePos().x() and self.point_y == e.scenePos().y():
                self.annotation = {}
                self.update_scene("on_annotation", self.annotation)

    def _mousePressEvent(self, e):
        if not self.annotating:
            if self.anno_type:
                self.annotating = True
                self.annotation = {}
                self.point_x = e.scenePos().x()
                self.point_y = e.scenePos().y()

                height = 0.36
                radius = 1
                roll = 90.0 if self.anno_type == 'ar_track_alvar_msgs/AlvarMarker' else 0.0
                self._set_annotating_info(anno_type=self.anno_type, x=self.point_x, y=self.point_y, yaw=0.0, radius=radius, roll=roll, pitch=0.0, height=height)
                self._update_edit_annotation_box()
                
                self.update_scene("on_annotation", self.annotation)

    def _mouseMoveEvent(self, e):
        if self.annotating:
            dx = e.scenePos().x() - self.point_x
            dy = e.scenePos().y() - self.point_y
            dist = math.hypot(dx, dy)
            yaw = -math.degrees(math.atan2(dy, dx))
            if self.anno_type == 'ar_track_alvar_msgs/AlvarMarker':
                yaw += 90 
            self._set_annotating_info(anno_type=self.anno_type, x=self.point_x, y=self.point_y, yaw=yaw, radius=dist, roll=None, pitch=None, height=None)
            self._update_edit_annotation_box()
            self.update_scene("on_annotation", self.annotation)

###############################################################
# Annotation List Events
###############################################################
    def _annotation_list_item_clicked(self, item):
        
        self._selected_annotation = str(item.text())
        if self._selected_annotation.startswith('(new)'):
            self._selected_annotation = self._selected_annotation[6:]
        annotation_info = self._map_annotation_interface.get_annotation_info(self._selected_annotation)

        if annotation_info:
            self._set_edit_annotation_box()
            
            self._set_edit_annotation_box(name=annotation_info[1], 
                                            x=annotation_info[2], 
                                            y=annotation_info[3], 
                                            yaw=annotation_info[4], 
                                            radius=annotation_info[5], 
                                            roll=annotation_info[6], 
                                            pitch=annotation_info[7], 
                                            height=annotation_info[8])

###############################################################
# Map ItemClicked 
###############################################################
    def _select_map_item_clicked(self, item_index):
        # if item_index is -1, combobox got reset
        if item_index >= 0 and self.map_select_combobox.count() > 1: # this is to avoid being selected when the first item is added in the combobox
            self._selected_map = self.map_select_combobox.itemText(item_index)
            print(str("in item clicked : " + self._selected_map))
            success, message = self._callback['load_map'](self._selected_map)

            if not success:
                self.emit(SIGNAL("show_message"), self, "Failed", message)

        

###############################################################
# Check box Events
###############################################################
    def _check_ar_marker_cbox(self, data):
        if data == Qt.Checked:
            self.location_cbox.setCheckState(Qt.Unchecked)
            self.map_view.setDragMode(QGraphicsView.NoDrag)
            self.anno_type = "ar_track_alvar_msgs/AlvarMarker"
        else:
            self.map_view.setDragMode(QGraphicsView.ScrollHandDrag)
            if self.location_cbox.checkState() == Qt.Unchecked:
                self.anno_type = None
        self._clear_on_annotation()
        self._deselect_list_annotation()

    def _check_location_cbox(self, data):
        if data == Qt.Checked:
            self.ar_marker_cbox.setCheckState(Qt.Unchecked)
            self.map_view.setDragMode(QGraphicsView.NoDrag)
            self.anno_type = "yocs_msgs/Waypoint" 
        else:
            self.map_view.setDragMode(QGraphicsView.ScrollHandDrag)

            if self.ar_marker_cbox.checkState() == Qt.Unchecked:
                self.anno_type = None
        self._clear_on_annotation()
        self._deselect_list_annotation()

    def _clear_on_annotation(self):
        if self.annotation_item:
            self._scene.removeItem(self.annotation_item)
            self.annotation_item = None
            self.annotation = None
            self.annotation = {}
        self._clear_edit_annotation_box()

    def _deselect_list_annotation(self):
        listwidget = self.annotations_list_widget
        self._selected_annotation = None
    
        for i in range(listwidget.count()):
            item = listwidget.item(i)
            listwidget.setItemSelected(item, False)

################################################################
# QT Events
################################################################
    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
