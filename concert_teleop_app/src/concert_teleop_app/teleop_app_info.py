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
from rocon_console import console


##############################################################################
# TeleopAppInfo
##############################################################################


class TeleopAppInfo(object):
    def __init__(self):

        self._event_callback = None
        self.image_data = None
        self.robot_list = {}
        # should make use of concert_msgs/Strings here.
        #rospy.Subscriber("/available_teleop", String, self.update_robot_list)
        #rospy.Subscriber("/available_teleop", String, self.update_robot_list)

        rospy.Subscriber("/usb_cam/image_raw", Image, self._update_teleop_image)
        print "register sub"

    def _update_robot_list(self, data):
        """
        Update the available teleop robot list

        @param data: information of robot list
        @type
        """
        pass

    def _update_teleop_image(self, data):
        """
        Update the teleop image

        @param data: compressed image
        @type
        """
        self.image_data = data
        self._event_callback()
        pass

    def _request_teleop_cmd_vel(self, cmd_vel):
        """
        Update the teleop image

        @param cmd_vel: command of velocity
        @type
        """
        pass

    def _reg_event_callback(self, func):
        self._event_callback = func
