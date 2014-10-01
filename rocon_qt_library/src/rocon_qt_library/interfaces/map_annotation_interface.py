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
from tf.transformations import quaternion_from_euler

import nav_msgs.msg as nav_msgs
import visualization_msgs.msg as visualization_msgs


from python_qt_binding.QtCore import Signal, QObject, pyqtSlot
import rocon_qt_library.utils as utils


class MapAnnotationInterface(QObject):

    scene_update = Signal(dict, name='scene_update')

    def __init__(self, map_received_slot=None,
                 map_topic='map',
                 viz_markers_received_slot=None,
                 viz_markers_topic='viz_markers',
                 scene_update_slot=None,
                 save_annotation_srv='save_annotation',
                 _tf=None):

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

        self.viz_markers_msg = None
        self.map_msg = None

        self.sub_map = rospy.Subscriber(map_topic, nav_msgs.OccupancyGrid, self.map_cb)
        self.sub_viz_makers = rospy.Subscriber(viz_markers_topic, visualization_msgs.MarkerArray, self.viz_markers_cb)
        #self.srv_save_annotation = rospy.ServiceProxy(save_annotation, visualization_msgs.Marker)

    def save_annotation(self, data):
        viz_marker = visualization_msgs.Marker()
        viz_marker.header = self.map_frame

        viz_marker.pose.position.x = (-data['x'] + self.w) * self.resolution + self.ori_x
        viz_marker.pose.position.y = (data['y']) * self.resolution + self.ori_y
        viz_marker.pose.position.z = data['height'] * self.resolution

        (viz_marker.pose.orientation.x, viz_marker.pose.orientation.y, viz_marker.pose.orientation.z, viz_marker.pose.orientation.w) = tf.transformations.quaternion_from_euler(radians(data['roll']), radians(data['pitch']), radians(data['yaw']))

        viz_marker.scale.x = data['scale'][0] * self.resolution
        viz_marker.scale.y = data['scale'][1] * self.resolution
        viz_marker.scale.z = data['scale'][2] * self.resolution
        viz_marker.text = data['name']
        rospy.loginfo(viz_marker)

    def update_scene(self):
        draw_data = {}

        if self.map_msg and self.viz_markers_msg:
            # map data
            self.map_frame = self.map_msg.header.frame_id
            self.resolution = self.map_msg.info.resolution
            self.w = self.map_msg.info.width
            self.h = self.map_msg.info.height
            self.ori_x = self.map_msg.info.origin.position.x
            self.ori_y = self.map_msg.info.origin.position.y
            # viz_marker_data
            viz_marker_items = []
            for marker in self.viz_markers_msg.markers:
                viz_marker = {}
                trans_pose = None
                if not (marker.header.frame_id == self.map_frame or marker.header.frame_id == ''):
                    try:
                        self._tf.waitForTransform(marker.header.frame_id, self.map_frame, rospy.Time(), rospy.Duration(10))
                        trans_pose = self._tf.transformPose(self.map_frame, marker.pose)
                    except tf.Exception:
                        rospy.logerr("TF Error")
                        trans_pose = None
                else:
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

            draw_data["map"] = self.map_msg
            draw_data["viz_markers"] = viz_marker_items
            draw_data["map_resolution"] = self.resolution
            self.scene_update.emit(draw_data)

    def map_cb(self, msg):
        """
        Update the map 
        
        :param nav_msgs.OccupancyGrid msg: a map from system
        """
        self.map_msg = msg
        self.update_scene()

    def viz_markers_cb(self, msg):
        """
        Update the visualization makers
        
        :param visualization_msgs.MarkerArray msg: Visualization makers on the map
        """
        self.viz_markers_msg = msg
        self.update_scene()

    def close(self):
        if self.sub_map:
            self.sub_map.unregister()

        if self.sub_scan:
            self.sub_scan.unregister()

        if self.sub_robot_pose:
            self.sub_robot_pose.unregister()

        super(MapAnnotationInterface, self).close()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
