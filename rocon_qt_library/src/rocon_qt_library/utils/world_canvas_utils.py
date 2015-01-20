#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################

import tf
import rospy
from world_canvas_msgs.msg import Annotation
import unique_id
from rospy_message_converter import message_converter
from tf.transformations import quaternion_from_euler
from math import radians
import ar_track_alvar_msgs.msg as ar_msgs
import yocs_msgs.msg as yocs_msgs

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
    ann.pose.pose.pose = map_msg.info.origin
    return ann

def create_alvar_marker_from_info(annotation_info, world, frame_id):
    ann = Annotation()
    ann.timestamp = rospy.Time.now()
    ann.id = unique_id.toMsg(unique_id.fromRandom())
    ann.world = world
    ann.name = "marker " + str(annotation_info['name'])
    ann.type = 'ar_track_alvar_msgs/AlvarMarker'
    ann.keywords.append(str(world))
    ann.shape = 0 # Cylinder 
    ann.color.r = 1.0
    ann.color.g = 1.0
    ann.color.b = 1.0
    ann.color.a = 1.0
    ann.size.x = 0.18 
    ann.size.y = 0.18
    ann.size.z = 0.01
    ann.pose.header.frame_id = frame_id
    ann.pose.header.stamp = rospy.Time.now()
    ann.pose.pose.pose.position.x = annotation_info['x']
    ann.pose.pose.pose.position.y = annotation_info['y']
    ann.pose.pose.pose.position.z = annotation_info['height']
    (ann.pose.pose.pose.orientation.x, ann.pose.pose.pose.orientation.y, ann.pose.pose.pose.orientation.z, ann.pose.pose.pose.orientation.w) = tf.transformations.quaternion_from_euler(radians(annotation_info['roll']), radians(annotation_info['pitch']), radians(annotation_info['yaw']))

    obj = ar_msgs.AlvarMarker()
    obj.id = int(annotation_info['name'])
    obj.confidence = 80
    obj.pose.header = ann.pose.header
    obj.pose.pose = ann.pose.pose.pose

    return ann, obj

def create_waypoint_from_info(annotation_info, world, frame_id):
    ann = Annotation()
    ann.timestamp = rospy.Time.now()
    ann.id = unique_id.toMsg(unique_id.fromRandom())
    ann.world = world
    ann.name = annotation_info['name']
    ann.type = 'yocs_msgs/Waypoint'
    ann.keywords.append(str(world))
    ann.shape = 3 # Arrow
    ann.color.r = 0.2
    ann.color.g = 0.2
    ann.color.b = 0.8
    ann.color.a = 0.5
    ann.size.x = float(annotation_info['radius']) * 2
    ann.size.y = float(annotation_info['radius']) * 2
    ann.size.z = float(annotation_info['height'])
    ann.pose.header.frame_id = frame_id
    ann.pose.header.stamp = rospy.Time.now()
    ann.pose.pose.pose.position.x = annotation_info['x']
    ann.pose.pose.pose.position.y = annotation_info['y']
    ann.pose.pose.pose.position.z = 0.0#annotation_info['height']
    (ann.pose.pose.pose.orientation.x, ann.pose.pose.pose.orientation.y, ann.pose.pose.pose.orientation.z, ann.pose.pose.pose.orientation.w) = tf.transformations.quaternion_from_euler(radians(annotation_info['roll']), radians(annotation_info['pitch']), radians(annotation_info['yaw']))

    obj = yocs_msgs.Waypoint()
    obj.name = annotation_info['name'] 
    obj.header.frame_id = frame_id
    obj.header.stamp = rospy.Time.now()
    obj.pose.position.x = annotation_info['x']
    obj.pose.position.y = annotation_info['y']
    obj.pose.position.z = 0.0
    (obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w) = tf.transformations.quaternion_from_euler(radians(annotation_info['roll']), radians(annotation_info['pitch']), radians(annotation_info['yaw']))

    # waypoints are assumed to lay on the floor, so z coordinate is zero;
    # but WCF assumes that the annotation pose is the center of the object
    ann.pose.pose.pose.position.z += ann.size.z/2.0

    return ann, obj
