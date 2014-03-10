#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

#ros
import rospy
from sensor_msgs.msg import CompressedImage

#rocon
import rocon_uri
import rocon_python_comms
from rocon_std_msgs.msg import StringArray
from geometry_msgs.msg import Twist

import rocon_service_msgs.msg as rocon_service_msgs

##############################################################################
# TeleopAppInfo
##############################################################################


class TeleopAppInfo(object):
    def __init__(self):
        self._capture_event_callback = None
        self._event_callback = None
        self.image_data = None
        self.robot_list = {}
        self.pre_robot_list = {}
        self.service_pair_msg_q = []
        self.captured_teleop_rocon_uri = None
        self.captured_teleop_cmd_vel_pub = None
        self.captured_teleop_compressed_image_sub = None
        rospy.Subscriber("/usb_cam/image_raw", CompressedImage, self._update_teleop_image)
        rospy.Subscriber("/services/teleop/available_teleops", StringArray, self._update_robot_list)
        self.capture_teleop = rocon_python_comms.ServicePairClient('/services/teleop/capture_teleop', rocon_service_msgs.CaptureTeleopPair)

    def _update_robot_list(self, data):
        """
        Update the available teleop robot list

        @param data: information of robot list
        @type
        """

        for k in data.strings:
            uri = rocon_uri.parse(k)
            self.robot_list[k] = {}
            self.robot_list[k]["rocon_uri"] = k
            self.robot_list[k]["name"] = uri.name
            self.robot_list[k]["hardware_platform"] = uri.hardware_platform
            self.robot_list[k]["application_framework"] = uri.application_framework
            self.robot_list[k]["operating_system"] = uri.operating_system

        if len(self.pre_robot_list):
            if not self._compare_list():
                self._event_callback()
                self.pre_robot_list = self.robot_list
        else:
            print "first update"
            self.pre_robot_list = self.robot_list
            self._event_callback()

    def _compare_list(self):
        if self.robot_list == self.pre_robot_list:
            return True
        else:
            return False

    def _update_teleop_image(self, data):
        """
        Update the teleop image

        @param data: compressed image
        @type
        """
        print "update the teleop image"
        self.image_data = data

    def _request_teleop_cmd_vel(self, linear, angular):
        """
        Update the teleop image

        @param linear: linear velocity. Unit is m/s
        @type float

        @param angular: angular velocity. Unit is rad/s
        @type float
        """
        if self.captured_teleop_cmd_vel_pub:
            cmd_vel = Twist()
            cmd_vel.linear.x = linear
            cmd_vel.angular.z = angular
            self.captured_teleop_cmd_vel_pub.publish(cmd_vel)

    def _capture_teleop(self, rocon_uri):
        """
        caputre the robot with rocon uri

        @param rocon_uri: robot information as uri type
        @type rocon_uri class
        """
        print "call capture Teleop"
        request = rocon_service_msgs.CaptureTeleopRequest()
        request.rocon_uri = rocon_uri
        self.captured_teleop_rocon_uri = rocon_uri
        msg_id = self.capture_teleop(request, timeout=rospy.Duration(7.0), callback=self.callback, error_callback=self.error_callback)
        self.service_pair_msg_q.append(msg_id)

    def callback(self, msg_id, msg):
        """ User callback to feed into non-blocking requests.

        @param msg_id : id of the request-response pair.
        @type uuid_msgs.UniqueID

        @param msg : message response received
        @type <name>Response
         """
        if msg_id in self.service_pair_msg_q:
            self.service_pair_msg_q.remove(msg_id)
            print " msg_id: %s" % msg_id
            print " msg: %s" % msg
            if msg.result == True:
                self._init_teleop(self.captured_teleop_rocon_uri)
                self._capture_event_callback()

    def _init_teleop(self, captured_teleop_rocon_uri):
        """ After capturing teleop, intialization with teleop information.

        @param teleop_name : captured teleop name
        @type string
         """
        uri = rocon_uri.parse(captured_teleop_rocon_uri)
        print ("init teleop: %s" % uri.name.string)
        captured_name = uri.name.string

        self.captured_teleop_cmd_vel_pub = rospy.Publisher(captured_name + "/cmd_vel", Twist, latch=True)
        rospy.Subscriber(captured_name + "/compressed_image", CompressedImage, self._update_teleop_image)

    def error_callback(self, msg_id, error_message):
        """ User callback to feed into non-blocking requests.

          @param msg_id : id of the request-response pair.
          @type uuid_msgs.UniqueID

          @param error_message : error string received
          @type str
        """
        rospy.loginfo("Error Callback: %s" % error_message)

    def _reg_event_callback(self, func):
        self._event_callback = func

    def _reg_capture_event_callback(self, func):
        self._capture_event_callback = func
