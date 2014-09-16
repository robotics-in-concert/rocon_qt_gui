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

from python_qt_binding.QtCore import Signal, Slot,  pyqtSlot, Qt, SIGNAL , QPointF
from python_qt_binding.QtGui import QPixmap, QImage, QGraphicsView, QGraphicsScene, qRgb, QPen, QBrush, QColor, QPolygonF, QMatrix

from rqt_py_common.topic_helpers import get_field_type

class QSlamView(QGraphicsView):
    map_changed = Signal()
    scan_changed = Signal()
    robot_pose_changed = Signal()

    def __init__(self, parent=None):
        super(QSlamView, self).__init__()
        self._parent = parent
        self._tf = tf.TransformListener()
        self._scene = QGraphicsScene()

        self.map_changed.connect(self._update_map)
        self.scan_changed.connect(self._update_scan)
        self.robot_pose_changed.connect(self._update_robot_pose)

        self.destroyed.connect(self.close)

        #ScrollHandDrag
        self.setDragMode(QGraphicsView.ScrollHandDrag)

        self.w = 0
        self.h = 0
        self.ori_x = 0
        self.ori_y = 0
        self.map_frame = None
        self._map = None
        self._map_item = {}
        self._robot_pose = None
        self._robot_pose_item = None

        self._scan = None
        self._scan_items = []
        self._colors = [(238, 34, 116), (68, 134, 252), (236, 228, 46), (102, 224, 18), (242, 156, 6), (240, 64, 10), (196, 30, 250)]
        self._robot_polygon = QPolygonF([QPointF(-4, 4), QPointF(-4, -4), QPointF(12, 0)])

        self.setScene(self._scene)
    def add_dragdrop(self, item):
        # Add drag and drop functionality to all the items in the view
        def c(x, e): 
            self.dragEnterEvent(e)
        def d(x, e): 
            self.dropEvent(e)
        item.setAcceptDrops(True)
        item.dragEnterEvent = c 
        item.dropEvent = d 

    def dragEnterEvent(self, e): 
        if self._parent:
            self._parent.dragEnterEvent(e)

    def dropEvent(self, e): 
        if self._parent:
            self._parent.dropEvent(e)

    def wheelEvent(self, event):
        event.ignore()
        if event.delta() > 0:
            self.scale(1.15, 1.15)
        else:
            self.scale(0.85, 0.85)

    @pyqtSlot(nav_msgs.OccupancyGrid, name='map_received') 
    def map_cb(self, msg):
        self.map_frame = msg.header.frame_id
        self.resolution = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height
        self.ori_x = msg.info.origin.position.x
        self.ori_y = msg.info.origin.position.y
        a = numpy.array(msg.data, dtype=numpy.uint8, copy=False, order='C')
        a = a.reshape((self.h, self.w))
        if self.w % 4:
            e = numpy.empty((self.h, 4 - self.w % 4), dtype=a.dtype, order='C')
            a = numpy.append(a, e, axis=1)
        image = QImage(a.reshape((a.shape[0] * a.shape[1])), self.w, self.h, QImage.Format_Indexed8)

        for i in reversed(range(101)):
            image.setColor(100 - i, qRgb(i* 2.55, i * 2.55, i * 2.55))
        image.setColor(101, qRgb(255, 0, 0))  # not used indices
        image.setColor(255, qRgb(200, 200, 200))  # color for unknown value -1
        self._map = image
        self.setSceneRect(0, 0, self.w, self.h)
        self.map_changed.emit()
    
    def laser_scan_to_point_cloud(self, msg):
        point_cloud = sensor_msgs.PointCloud()
        point_cloud.header.frame_id = msg.header.frame_id

        for idx in range(0, len(msg.ranges), 10):
            point = geometry_msgs.Point32()
            a = msg.angle_min + msg.angle_increment*idx
            point.x = msg.ranges[idx]*cos(a)
            point.y = msg.ranges[idx]*sin(a)
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
                point_cloud = self.laser_scan_to_point_cloud(msg)
                trans_ptc = self._tf.transformPointCloud(self.map_frame, point_cloud)
            except tf.Exception:
                rospy.logerr("TF Error")
                trans_ptc = None
        else:
            trans_ptc = self.laser_scan_to_point_cloud(msg)

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

    def _update_map(self):
        if self._map_item:
            self._scene.removeItem(self._map_item)

        pixmap = QPixmap.fromImage(self._map)
        self._map_item = self._scene.addPixmap(pixmap)

        # Everything must be mirrored
        self._mirror(self._map_item)

        # Add drag and drop functionality
        self.add_dragdrop(self._map_item)

        self.centerOn(self._map_item)
        self.show()

    def _update_scan(self):
        old_items = []
        if len(self._scan_items):
            for item in self._scan_items:
                old_items.append(item)

        for pt in self._scan.points:
            dx = (pt.x - self.ori_x)/self.resolution
            dy = (pt.y - self.ori_y)/self.resolution
            scan_item = self._scene.addRect(dx, dy, 1, 1, pen = QPen(QColor(0,255,0)), brush = QBrush(QColor(0,255,0)))
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
        dx = (self._robot_pose.pose.position.x - self.ori_x)/self.resolution
        dy = (self._robot_pose.pose.position.y - self.ori_y)/self.resolution
        (droll, dpitch, dyaw) = tf.transformations.euler_from_quaternion([self._robot_pose.pose.orientation.x,
                                                                      self._robot_pose.pose.orientation.y,
                                                                      self._robot_pose.pose.orientation.z,
                                                                      self._robot_pose.pose.orientation.w])
        translated_robot_pose = QMatrix().rotate(degrees(dyaw)).map(self._robot_polygon).translated(dx, dy)
        self._robot_pose_item = self._scene.addPolygon(translated_robot_pose, pen = QPen(QColor(255,0,0)), brush = QBrush(QColor(255,0,0)))
        # Everything must be mirrored
        self._mirror(self._robot_pose_item)
        if old_item:
            self._scene.removeItem(old_item)

    def _mirror(self, item):
        item.scale(-1, 1)
        item.translate(-self.w, 0)
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
