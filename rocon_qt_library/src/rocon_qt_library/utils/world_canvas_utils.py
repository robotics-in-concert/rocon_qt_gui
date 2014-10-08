#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################

import rospy
from world_canvas_msgs.msg import Annotation
import unique_id
from rospy_message_converter import message_converter

def create_map_annotation(world, map_name, map_msg):
    ann = Annotation()
    ann.timestamp = rospy.Time.now()
    ann.world = world
    ann.name = map_name
    ann.type = 'nav_msgs/OccupancyGrid'
    ann.keywords.append(map_name)
    ann.shape = 1 # CUBE
    ann.color.r = 0.2
    ann.color.g = 0.2
    ann.color.b = 0.2
    ann.color.a = 0.01
    ann.size.x = map_msg.info.width * map_msg.info.resolution
    ann.size.y = map_msg.info.height * map_msg.info.resolution
    ann.size.z = 0.000001
    ann.pose.header.frame_id = '/map'
    ann.pose.header.stamp = rospy.Time.now()
    ann.pose.pose.pose = message_converter.convert_dictionary_to_ros_message('geometry_msgs/Pose', map_msg.info.origin)
    return ann
