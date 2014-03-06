#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#

#ros
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image

#rocon
import rocon_uri
import rocon_python_comms
from rocon_console import console
from rocon_std_msgs.msg import StringArray
import rocon_service_pair_msgs.msg as rocon_service_pair_msgs
import rocon_service_msgs.msg as rocon_service_msgs

##############################################################################
# TeleopAppInfo
##############################################################################


class TeleopAppInfo(object):
    def __init__(self):
        self._event_image_callback = None
        self._event_callback = None
        self.image_data = None
        self.robot_list = {}
        self.pre_robot_list = {}
        rospy.Subscriber("/usb_cam/image_raw", Image, self._update_teleop_image)
        rospy.Subscriber("/services/teleop/available_teleops", StringArray, self._update_robot_list)
        self.capture_teleop = rocon_python_comms.ServicePairClient('concert_teleop_app', rocon_service_msgs.CaptureTeleopPair)

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
        self.image_data = data
        pass

    def _request_teleop_cmd_vel(self, cmd_vel):
        """
        Update the teleop image

        @param cmd_vel: command of velocity
        @type
        """
        pass

    def _capture_teleop(self, rocon_uri):
        """
        caputre the robot with rocon uri

        @param rocon_uri: robot information as uri type
        @type rocon_uri class
        """
        print "call capture Teleop"
        request = rocon_service_msgs.CaptureTeleopRequest()
        request.rocon_uri = rocon_uri
        msg_id = self.capture_teleop(request, timeout=rospy.Duration(10.0), callback=self.callback, error_callback=self.error_callback)
        pass

    def callback(self, msg_id, msg):

        """ User callback to feed into non-blocking requests.

        @param msg_id : id of the request-response pair.
        @type uuid_msgs.UniqueID

        @param msg : message response received
        @type <name>Response
        """
        print " msg_id: %s" % msg_id
        print " msg: %s" % msg
        pass

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
