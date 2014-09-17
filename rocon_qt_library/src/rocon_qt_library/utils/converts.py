#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
from math import cos, sin
import sensor_msgs.msg as sensor_msgs
import geometry_msgs.msg as geometry_msgs

##############################################################################
# converter Function
##############################################################################
def laser_scan_to_point_cloud(laser_scan_msg):
    point_cloud = sensor_msgs.PointCloud()
    point_cloud.header.frame_id = laser_scan_msg.header.frame_id

    for idx in range(0, len(laser_scan_msg.ranges)):
        point = geometry_msgs.Point32()
        a = laser_scan_msg.angle_min + laser_scan_msg.angle_increment * idx
        point.x = laser_scan_msg.ranges[idx] * cos(a)
        point.y = laser_scan_msg.ranges[idx] * sin(a)
        point_cloud.points.append(point)
    return point_cloud
