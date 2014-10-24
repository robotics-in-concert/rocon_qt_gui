#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################


import numpy
import random
from math import sqrt, atan, pi, degrees

import rospy
import tf
from tf.transformations import quaternion_from_euler
from world_canvas_client import AnnotationCollection, WCFError

import nav_msgs.msg as nav_msgs
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs

from python_qt_binding.QtCore import Signal, QObject, pyqtSlot
import rocon_qt_library.utils as utils


class SlamWidgetInterface(QObject):

    map_received = Signal(nav_msgs.OccupancyGrid, name='map_received')
    scan_received = Signal(list)
    robot_pose_received = Signal(dict)

    def __init__(self, map_received_slot=None, map_topic='map', scan_received_slot=None, scan_topic='scan', robot_pose_received_slot=None, robot_pose_topic='robot_pose', wc_namespace='world_canvas', map_saved_callbacks=None,_tf=None):
        super(SlamWidgetInterface, self).__init__()
        if map_received_slot is not None:
            self.map_received.connect(map_received_slot)
        if scan_received_slot is not None:
            self.scan_received.connect(scan_received_slot)
        if robot_pose_received_slot is not None:
            self.robot_pose_received.connect(robot_pose_received_slot)
        
        self.map_saved_callbacks=[]
        if map_saved_callbacks is not None:
            self.map_saved_callbacks = map_saved_callbacks

        self.destroyed.connect(self.close)

        if _tf is None:
            self._tf = tf.TransformListener()
        else:
            self._tf = _tf
        self.wc_namespace = wc_namespace
        self.ac_handler = AnnotationCollection(srv_namespace=self.wc_namespace)
        
        self.sub_map = rospy.Subscriber(map_topic, nav_msgs.OccupancyGrid, self.map_cb)
        self.sub_scan = rospy.Subscriber(scan_topic, sensor_msgs.LaserScan, self.scan_cb)
        self.sub_robot_pose = rospy.Subscriber(robot_pose_topic, geometry_msgs.PoseStamped, self.robot_pose_cb)

        self.w = 0
        self.h = 0
        self.ori_x = 0
        self.ori_y = 0
        self.map_frame = None

    def save_map(self, world='world', map_name='map'):
        annotation = utils.create_map_annotation(world, map_name, self.map_msg)
        success = True
        message = "Success"
        try:
            self.ac_handler.add(annotation, self.map_msg)
            self.ac_handler.save()
        except WCFError as e:
            success = False
            message = str(e)
            
        for callback in self.map_saved_callbacks:
            callback((success, message))

    def map_cb(self, msg):
        """
        Update the map 
        
        :param nav_msgs.OccupancyGrid msg: a map from system
        """
        self.map_frame = msg.header.frame_id
        self.resolution = msg.info.resolution
        self.w = msg.info.width
        self.h = msg.info.height
        self.ori_x = msg.info.origin.position.x
        self.ori_y = msg.info.origin.position.y
        self.map_msg = msg
        self.map_received.emit(msg)

    def scan_cb(self, msg):
        """
        Update the scan

        :param sensor_msgs.LaserScan msg: scans data from system
        """
        scans = []
        trans_ptc = None
        if self.map_frame:
            if not (msg.header.frame_id == self.map_frame or msg.header.frame_id == ''):
                try:
                    self._tf.waitForTransform(msg.header.frame_id, self.map_frame, rospy.Time(), rospy.Duration(10))
                    point_cloud = utils.laser_scan_to_point_cloud(msg,10)
                    trans_ptc = self._tf.transformPointCloud(self.map_frame, point_cloud)
                except tf.Exception:
                    rospy.logerr("TF Error")
                    trans_ptc = None
            else:
                trans_ptc = utils.laser_scan_to_point_cloud(msg)

        if trans_ptc:
            for pt in trans_ptc.points:
                dx = (pt.x - self.ori_x) / self.resolution - self.w
                dy = (pt.y - self.ori_y) / self.resolution
                scans.append((dx, dy))
        self.scan_received.emit(scans)

    def robot_pose_cb(self, msg):
        """
        Update the robot_pose

        :param geometry_msgs.PoseStamped msg: robot pose data from system
        """
        # Transform everything in to the map frame
        trans_pose = None
        if self.map_frame:
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
            dx = (trans_pose.pose.position.x - self.ori_x) / self.resolution - self.w
            dy = (trans_pose.pose.position.y - self.ori_y) / self.resolution
            (droll, dpitch, dyaw) = tf.transformations.euler_from_quaternion([trans_pose.pose.orientation.x,
                                                                             trans_pose.pose.orientation.y,
                                                                             trans_pose.pose.orientation.z,
                                                                             trans_pose.pose.orientation.w])
            translated_robot_pose = {}
            translated_robot_pose['x'] = dx
            translated_robot_pose['y'] = dy
            translated_robot_pose['yaw'] = degrees(dyaw)

            self.robot_pose_received.emit(translated_robot_pose)

    def close(self):
        if self.sub_map:
            self.sub_map.unregister()

        if self.sub_scan:
            self.sub_scan.unregister()

        if self.sub_robot_pose:
            self.sub_robot_pose.unregister()

        super(SlamWidgetInterface, self).close()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
