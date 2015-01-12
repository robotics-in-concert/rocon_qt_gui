# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy
import random
from math import sqrt, atan, pi, degrees, radians

import rospy
import tf
import unique_id
from tf.transformations import quaternion_from_euler

import nav_msgs.msg as nav_msgs
import visualization_msgs.msg as visualization_msgs


from python_qt_binding.QtCore import Signal, QObject, pyqtSlot
import rocon_qt_library.utils as utils
from world_canvas_client import AnnotationCollection, WCFError
from visualization_msgs.msg import Marker


class MapAnnotationInterface(QObject):

    scene_update = Signal(str, dict, name='scene_update')

    def __init__(self, scene_update_slot=None, wc_namespace='world_canvas', _tf=None):
        super(MapAnnotationInterface, self).__init__()
        if scene_update_slot is not None:
            self.scene_update.connect(scene_update_slot)
        self.destroyed.connect(self.close)

        if _tf is None:
            self._tf = tf.TransformListener()
        else:
            self._tf = _tf

        self.w = 0
        self.h = 0
        self.ori_x = 0
        self.ori_y = 0
        self.resolution = 1
        self.map_frame = None

        self._annotations = None
        self.map_msg = None
        self.wc_namespace = wc_namespace
        self.ac_handler_map    = AnnotationCollection(srv_namespace=self.wc_namespace)
        self.ac_handler_others = AnnotationCollection(srv_namespace=self.wc_namespace)

        self._new_annotations = []
        self._new_annotations_data = []

    def get_list_world(self):
        try:
            world_names = self.ac_handler_map.get_world_list()
        except WCFError as e:
            return False, str(e), []

        return True, "Success", world_names
         

    def load_world(self, world):
        '''
        loading world information from world canvas server

        :returns: Map annotation
        '''
        # Loading map.
        # Load the first map on map view
        # return list of available maps
        message, map_name_list = self._load_map(world)
        
        if len(map_name_list) > 0:
            message, annotation_name_list = self._load_annotations(world)
        else:
            return False, message, [], []

        self._world = world
        self._new_annotations      = []
        self._new_annotations_data = []
        return True, message, map_name_list, annotation_name_list

    def load_map(self, selected_name):
        try:
            map_name, map_id = selected_name.split('-')
            for annot in self._map_annotations:
                if map_name == annot.name and map_id == str(annot.id):
                    map_msg = self.ac_handler_map.get_data(annot)
                    self._update_map(map_msg)
                    annotations_tmp = self.ac_handler_others.get_annotations()
                    annotations = [a for a in annotations_tmp if not a.type == 'nav_msgs/OccupancyGrid']
                    self._annotations = annotations
                    self._update_annotations('annotations',annotations)
                    self._update_annotations('new_annotations', self._new_annotations)
        except WCFError as e:
            return False, str(e)

        return True, "Success"

    def _load_map(self, world):
        message = "Success"
        try:
            self.ac_handler_map.filter_by(world=world, types=['nav_msgs/OccupancyGrid'])
            self.ac_handler_map.load_data()
            map_annotations = self.ac_handler_map.get_annotations(type='nav_msgs/OccupancyGrid')
            self._map_annotations = map_annotations
        except WCFError as e:
            message = str(e)
            return message, []

        if len(map_annotations) < 1:
            message = "No map available"
            return message, []
        
        map_msg = self.ac_handler_map.get_data(self._map_annotations[0])
        self._update_map(map_msg)
        map_names = [a.name+"-"+str(a.id) for a in self._map_annotations]
        return message, map_names

    def _update_map(self,map_msg):
        draw_data = {}
        self._map = map_msg

        # map data
        self.map_frame = self._map.header.frame_id
        self.resolution = self._map.info.resolution
        self.w = self._map.info.width
        self.h = self._map.info.height
        self.ori_x = self._map.info.origin.position.x
        self.ori_y = self._map.info.origin.position.y

        draw_data["map"] = self._map
        self.scene_update.emit('map', draw_data)
        
    def _load_annotations(self, world):
        message = "Success"
        try: 
            self.ac_handler_others.filter_by(world=world)
            self.ac_handler_others.load_data()
            annotations_tmp = self.ac_handler_others.get_annotations()
            annotations = [a for a in annotations_tmp if not a.type == 'nav_msgs/OccupancyGrid']
            self._annotations = annotations
        except WCFError as e:
            message = str(e)

        if len(annotations) < 1:
            message = "No annotations available"
            return message, []

        self._update_annotations('annotations',annotations)
        self._update_annotations('new_annotations', self._new_annotations)

        names = [a.name for a in annotations]
        return message, names

    def _update_annotations(self, key,  annotations):
        viz_marker_items = []
        markers = utils.annotations_to_viz_markers(annotations)
        for marker in markers.markers:
            viz_marker = {}
            
            # This is to support markers which are not in map frame. since map annotation is offlince tool. We don't know if we have to suport annotation in different frame
            #trans_pose = None
            #if not (marker.header.frame_id == self.map_frame or marker.header.frame_id == ''):
            #    try:
            #        self._tf.waitForTransform(marker.header.frame_id, self.map_frame, rospy.Time(), rospy.Duration(10))
            #        trans_pose = self._tf.transformPose(self.map_frame, marker.pose)
            #    except tf.Exception:
            #        rospy.logerr("TF Error")
            #        trans_pose = None
            #else:
            trans_pose = marker

            if trans_pose:
                dx = (trans_pose.pose.position.x - self.ori_x) / self.resolution - self.w
                dy = (trans_pose.pose.position.y - self.ori_y) / self.resolution
                (droll, dpitch, dyaw) = tf.transformations.euler_from_quaternion([trans_pose.pose.orientation.x,
                                                                                 trans_pose.pose.orientation.y,
                                                                                 trans_pose.pose.orientation.z,
                                                                                 trans_pose.pose.orientation.w])
                viz_marker['x'] = dx
                viz_marker['y'] = dy
                viz_marker['yaw'] = degrees(dyaw)
            else:
                viz_marker['x'] = None
                viz_marker['y'] = None
                viz_marker['yaw'] = None

            viz_marker['scale'] = (marker.scale.x / self.resolution, marker.scale.y / self.resolution, marker.scale.z / self.resolution)
            viz_marker['type'] = marker.type
            viz_marker['color'] = (marker.color.r, marker.color.g, marker.color.b, marker.color.a)
            viz_marker['text'] = marker.text
            viz_marker_items.append(viz_marker)

        draw_data = {}
        draw_data[key] = viz_marker_items
        self.scene_update.emit(key, draw_data)

    def add_annotation(self, annotation_info, annotation_type):
        if annotation_type == "ar_track_alvar_msgs/AlvarMarker": 
            anno, data = utils.create_alvar_marker_from_info(annotation_info, self._world, self._map.header.frame_id)
        elif annotation_type == "yocs_msgs/Waypoint":
            anno, data = utils.create_waypoint_from_info(annotation_info, self._world, self._map.header.frame_id)

        self._new_annotations.append(anno)
        self._new_annotations_data.append(data)

        self._update_annotations('new_annotations', self._new_annotations)

        self.ac_handler_others.add(anno, data)
        new_annotation_names = [a.name for a in self._new_annotations]

        return new_annotation_names

    def remove_annotation(self, annotation_name):
        for a in self._annotations:
            if a.name == annotation_name:
                self.ac_handler_others.remove(a.id)
                self._annotations.remove(a)
                self._update_annotations('annotations', self._annotations)
                return 'annotations', [a.name for a in self._annotations]

        for a in self._new_annotations:
            if a.name == annotation_name:
                self.ac_handler_others.remove(a.id)
                self._new_annotations.remove(a)
                self._update_annotations('new_annotations', self._new_annotations)
                return 'new_annotations', [a.name for a in self._new_annotations]

    def save_annotations(self):
        try:
            self.ac_handler_others.save()
            for a in self._new_annotations:
                self.ac_handler_others.remove(a.id)
            self._new_annotations = []
            self._new_annotations_data = []
            
        except WCFError as e:
            message = str(e)
            return False, message
        return True, "Success" 

    def get_annotation_info(self, annotation_name):
        annotations = []
        for a in self._annotations:
            if a.name == annotation_name:
                annotations = self.ac_handler_others.get_annotations(annotation_name)
                break

        for a in self._new_annotations:
            if a.name == annotation_name:
                annotations = self.ac_handler_others.get_annotations(annotation_name)
                break
        if annotations:
            annotation = annotations[0]
            name = annotation.name
            anno_type = annotation.shape
            x = annotation.pose.pose.pose.position.x
            y = annotation.pose.pose.pose.position.y
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([annotation.pose.pose.pose.orientation.x,
                                                                                     annotation.pose.pose.pose.orientation.y,
                                                                                     annotation.pose.pose.pose.orientation.z,
                                                                                     annotation.pose.pose.pose.orientation.w])
            height = annotation.pose.pose.pose.position.z
            radius = annotation.size.x
            return (anno_type, name, x, y, yaw, radius, roll, pitch, height)
        else:
            return ()

    def close(self):
        super(MapAnnotationInterface, self).close()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
