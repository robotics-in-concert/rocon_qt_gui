#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import concert_service_msgs.msg as concert_service_msgs
import rocon_console.console as console
import rocon_python_comms
from rocon_qt_library.interfaces.teleop_interface import TeleopInterface
from rocon_std_msgs.msg import StringArray
import rocon_uri
import rospy
import threading

##############################################################################
# TeleopManager
##############################################################################


class TeleopManager(object):
    def __init__(self,
                 image_received_slot,
                 event_callback,
                 capture_event_callback,
                 release_event_callback,
                 error_event_callback
                 ):
        self._lock = threading.Lock()
        self._capture_timeout = rospy.get_param('~capture_timeout', 15.0)
        rospy.loginfo("Concert Teleop : capture timeout [%s]" % self._capture_timeout)
        self.teleop_interface = None
        self._image_received_slot = image_received_slot
        self._capture_event_callback = capture_event_callback
        self._release_event_callback = release_event_callback
        self._error_event_callback = error_event_callback
        self._event_callback = event_callback
        self.robot_list = {}
        self.pre_robot_list = {}
        self.service_pair_msg_q = []
        self.captured_teleop_rocon_uri = None
        rospy.Subscriber("/services/teleop/available_teleops", StringArray, self._update_robot_list)
        self.capture_teleop = rocon_python_comms.ServicePairClient('/services/teleop/capture_teleop', concert_service_msgs.CaptureTeleopPair)

    def shutdown(self):
        """
        Final curtains, signifies the end of existence for this rqt plugin.
        """
        with self._lock:
            if self.teleop_interface:
                self.teleop_interface.shutdown()
                self.teleop_interface = None

    def publish_cmd_vel(self, linear, angular):
        """
        Relay to the underlying teleop interface if there is a
        teleopable robot currently captured.
        """
        with self._lock:
            if self.teleop_interface:
                self.teleop_interface.cmd_vel = (linear, angular)

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
            self.pre_robot_list = self.robot_list
            self._event_callback()

    def _compare_list(self):
        if self.robot_list == self.pre_robot_list:
            return True
        else:
            return False

    def _capture_teleop(self, rocon_uri):
        """
        Initiate a request to capture a teleoppable robot.

        :param str rocon_uri: robot information as a rocon uri
        """
        request = concert_service_msgs.CaptureTeleopRequest()
        request.rocon_uri = rocon_uri
        request.release = False
        self.captured_teleop_rocon_uri = rocon_uri
        msg_id = self.capture_teleop(request, timeout=rospy.Duration(15.0), callback=self._capture_callback, error_callback=self.error_callback)
        self.service_pair_msg_q.append(msg_id)

    def _release_teleop(self, rocon_uri):
        """
        caputre the robot with rocon uri

        @param rocon_uri: robot information as uri type
        @type rocon_uri class
        """
        request = concert_service_msgs.CaptureTeleopRequest()
        request.rocon_uri = rocon_uri
        request.release = True
        self.captured_teleop_rocon_uri = rocon_uri
        msg_id = self.capture_teleop(request, timeout=rospy.Duration(15.0), callback=self._release_callback, error_callback=self.error_callback)
        self.service_pair_msg_q.append(msg_id)

    def _capture_callback(self, msg_id, msg):
        """
        Handle the response from a capture service pair request.

        :param uuid_msgs.UniqueID msg_id: id of the request-response pair.
        :param concert_service_msgs.CaptureTeleopPairResponse msg: message response received
         """
        if msg_id in self.service_pair_msg_q:
            self.service_pair_msg_q.remove(msg_id)
            if msg.result == True:
                self._init_teleop(self.captured_teleop_rocon_uri)
            self._capture_event_callback(msg.result)

    def _release_callback(self, msg_id, msg):
        """ User callback to feed into non-blocking requests.

        @param msg_id : id of the request-response pair.
        @type uuid_msgs.UniqueID

        @param msg : message response received
        @type <name>Response
         """
        if msg_id in self.service_pair_msg_q:
            self.service_pair_msg_q.remove(msg_id)
            if msg.result == True:
                self._uninit_teleop(self.captured_teleop_rocon_uri)
            self._release_event_callback(msg.result)

    def error_callback(self, msg_id, error_message):
        """ User callback to feed into non-blocking requests.

          @param msg_id : id of the request-response pair.
          @type uuid_msgs.UniqueID

          @param error_message : error string received
          @type str
        """
        rospy.logerr("Concert Teleop : triggered the error_callback [%s]" % error_message)
        self._error_event_callback(error_message)

    def _init_teleop(self, captured_teleop_rocon_uri):
        """
        Initialise the teleop interface for a newly captured robot.

        :param str captured_teleop_rocon_uri: captured rocon uri
        """
        captured_name = rocon_uri.parse(captured_teleop_rocon_uri).name.string
        with self._lock:
            self.teleop_interface = TeleopInterface(
                image_received_slot=self._image_received_slot,
                cmd_vel_topic_name=captured_name + "/cmd_vel",
                compressed_image_topic_name=captured_name + "/compressed_image"
            )

    def _uninit_teleop(self, captured_teleop_rocon_uri):
        """
        Shutdown the teleop interface for the captured robot.

        :param str captured_teleop_rocon_uri: captured rocon uri
        """
        unused_uri = rocon_uri.parse(captured_teleop_rocon_uri)
        with self._lock:
            if self.teleop_interface:
                self.teleop_interface.shutdown()
                self.teleop_interface = None
