#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
import rospy
import copy
from math import cos, sin
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs
from visualization_msgs.msg import MarkerArray, Marker

##############################################################################
# converter Function
##############################################################################
def laser_scan_to_point_cloud(laser_scan_msg, density=100):
    """
        Change type 

        :param sensor_msgs.LaserScan laser_scan_msg: laser scans data (r, theta) type
        :param int intensity: laser scans data (r, theta) type

    """
    point_cloud = sensor_msgs.PointCloud()
    point_cloud.header.frame_id = laser_scan_msg.header.frame_id

    for idx in range(0, len(laser_scan_msg.ranges), 100/density):
        point = geometry_msgs.Point32()
        a = laser_scan_msg.angle_min + laser_scan_msg.angle_increment * idx
        point.x = laser_scan_msg.ranges[idx] * cos(a)
        point.y = laser_scan_msg.ranges[idx] * sin(a)
        point_cloud.points.append(point)
    return point_cloud

def annotations_to_viz_markers(annotations):
    '''
        world_canvas_msgs.Annotations to visualization_msgs.MarkerArray
    '''
    markers_list = MarkerArray()
    marker_id = 1
    for a in annotations:
        marker, label = anntation_to_viz_marker(a, marker_id)
        markers_list.markers.append(marker)
        markers_list.markers.append(label)
        marker_id = marker_id + 1

    return markers_list

def anntation_to_viz_marker(a, marker_id):
    # Marker
    marker = Marker()
    marker.id = marker_id
    marker.header = a.pose.header
    marker.type = a.shape
    marker.ns = a.type
    marker.action = Marker.ADD
    marker.lifetime = rospy.Duration.from_sec(0)
    marker.pose = copy.deepcopy(a.pose.pose.pose)
    marker.scale = a.size
    marker.color = a.color

    # view facing text
    label = copy.deepcopy(marker)
    label.id = label.id + 1000000 # marker id must be unique
    label.type = Marker.TEXT_VIEW_FACING
    label.text = a.name  #+ ' [' + a.type + ']'
    label.pose.position.z = label.pose.position.z + label.scale.z/2.0 + 0.1 # just above the visual
    label.scale.x = label.scale.y = label.scale.z = 0.3
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0

    return marker, label
