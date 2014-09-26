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

import rospy
import tf
from tf.transformations import quaternion_from_euler

import numpy
import random
from math import sqrt, atan, pi, degrees, cos, sin

import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
import rocon_qt_library.utils as utils
from python_qt_binding.QtCore import Signal, Slot,  pyqtSlot, Qt, SIGNAL, QPointF
from python_qt_binding.QtGui import QPixmap, QImage, QGraphicsView, QGraphicsScene, qRgb, QPen, QBrush, QColor, QPolygonF, QMatrix
from rocon_qt_library.views import QMapView
from rqt_py_common.topic_helpers import get_field_type


class QSlamView(QMapView):

    scan_changed = Signal()
    robot_pose_changed = Signal()

    def __init__(self, parent=None):
        super(QSlamView, self).__init__()
        self._parent = parent
        self._tf = tf.TransformListener()
        
        self.scan_changed.connect(self._update_scan)
        self.robot_pose_changed.connect(self._update_robot_pose)

        self.destroyed.connect(self.close)

        self._robot_pose = None
        self._robot_pose_item = None

        self._scan = None
        self._scan_items = []
        self._colors = [(238, 34, 116), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]
        self._robot_polygon = QPolygonF([QPointF(-4, 4), QPointF(-4, -4), QPointF(12, 0)])


    def laser_scan_to_point_cloud(self, msg):
        point_cloud = sensor_msgs.PointCloud()
        point_cloud.header.frame_id = msg.header.frame_id

        for idx in range(0, len(msg.ranges), 10):
            point = geometry_msgs.Point32()
            a = msg.angle_min + msg.angle_increment * idx
            point.x = msg.ranges[idx] * cos(a)
            point.y = msg.ranges[idx] * sin(a)
            point_cloud.points.append(point)
        return point_cloud

    @pyqtSlot(sensor_msgs.LaserScan, name='scan_received')
    def scan_cb(self, msg):
        if not self._map:
            return
        trans_ptc = None
        if not (msg.header.frame_id == self.map_frame or msg.header.frame_id == ''):
            try:
                self._tf.waitForTransform(msg.header.frame_id, self.map_frame, rospy.Time(), rospy.Duration(10))
                point_cloud = utils.laser_scan_to_point_cloud(msg)
                trans_ptc = self._tf.transformPointCloud(
                    self.map_frame, point_cloud)
            except tf.Exception:
                rospy.logerr("TF Error")
                trans_ptc = None
        else:
            trans_ptc = utils.laser_scan_to_point_cloud(msg)

        if trans_ptc:
            self._scan = trans_ptc
            self.scan_changed.emit()

    @pyqtSlot(geometry_msgs.PoseStamped, name='robot_pose_received')
    def robot_pose_cb(self, msg):
        if not self._map:
            return
        # Transform everything in to the map frame
        trans_pose = None
        if not (msg.header.frame_id == self.map_frame or msg.header.frame_id == ''):
            try:
                self._tf.waitForTransform(msg.header.frame_id, self.map_frame, rospy.Time(), rospy.Duration(10))
                trans_pose = self._tf.transformPose(self.map_frame, msg)
            except tf.Exception:
                rospy.logerr("TF Error")
                trans_pose = None
        else:
            trans_pose = msg

        if trans_pose:
            self._robot_pose = trans_pose
            self.robot_pose_changed.emit()

    def _update_scan(self):
        old_items = []
        if len(self._scan_items):
            for item in self._scan_items:
                old_items.append(item)

        for pt in self._scan.points:
            dx = (pt.x - self.ori_x) / self.resolution
            dy = (pt.y - self.ori_y) / self.resolution
            scan_item = self._scene.addRect(dx, dy, 1, 1, pen=QPen(
                QColor(0, 255, 0)), brush=QBrush(QColor(0, 255, 0)))
            # Everything must be mirrored
            self._mirror(scan_item)
            self._scan_items.append(scan_item)

        if len(old_items):
            for item in old_items:
                self._scene.removeItem(item)
                self._scan_items.remove(item)

    def _update_robot_pose(self):
        old_item = None
        if self._robot_pose_item:
            old_item = self._robot_pose_item
        dx = (self._robot_pose.pose.position.x - self.ori_x) / self.resolution
        dy = (self._robot_pose.pose.position.y - self.ori_y) / self.resolution
        (droll, dpitch, dyaw) = tf.transformations.euler_from_quaternion([self._robot_pose.pose.orientation.x,
                                                                         self._robot_pose.pose.orientation.y,
                                                                         self._robot_pose.pose.orientation.z,
                                                                         self._robot_pose.pose.orientation.w])
        translated_robot_pose = QMatrix().rotate(degrees(dyaw)).map(self._robot_polygon).translated(dx, dy)
        self._robot_pose_item = self._scene.addPolygon(translated_robot_pose, pen=QPen(QColor(255, 0, 0)), brush=QBrush(QColor(255, 0, 0)))
        # Everything must be mirrored
        self._mirror(self._robot_pose_item)
        if old_item:
            self._scene.removeItem(old_item)

    def _mirror(self, item):
        item.scale(-1, 1)
        item.translate(-self.w, 0)

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
