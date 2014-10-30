#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
#
# Imports
#
from python_qt_binding.QtCore import QObject

import rospy
import threading
import rocon_uri
import rocon_python_comms
import concert_service_msgs.msg as concert_service_msgs
from rocon_std_msgs.msg import StringArray

class ResourceChooserInterface(QObject):

    def __init__(self, capture_timeout=15.0, available_resource_topic='avaialble_resource', capture_resource_pair_topic='capture_resource', capture_resource_callbacks=[], release_resource_callbacks=[], error_resource_callbacks=[], refresh_resource_list_callbacks=[]):
        super(ResourceChooserInterface, self).__init__()
        self.destroyed.connect(self.close)

        self._capture_timeout = capture_timeout
        self._resource_list = []
        self._service_pair_msg_q = []

        self.available_resource_topic = available_resource_topic
        self.capture_resource_pair_topic = capture_resource_pair_topic

        self._callback = {}
        self._callback['capture'] = capture_resource_callbacks
        self._callback['release'] = release_resource_callbacks
        self._callback['error'] = error_resource_callbacks
        self._callback['refresh_list'] = refresh_resource_list_callbacks

        self._init_ros_api()

    def _init_ros_api(self):
        self._sub_avail_resource = rospy.Subscriber(self.available_resource_topic, StringArray, self._update_resource_list)
        self._pair_capture_resource = rocon_python_comms.ServicePairClient(self.capture_resource_pair_topic, concert_service_msgs.CaptureResourcePair)

    def capture_resource(self, uri):
        """
        Initiate a request to capture an available robot.

        :param str rocon_uri: robot information as a rocon uri
        """
        request = concert_service_msgs.CaptureResourceRequest()
        request.rocon_uri = uri
        request.release = False

        self.captured_resource_uri = request.rocon_uri
        msg_id = self._pair_capture_resource(request, timeout=rospy.Duration(self._capture_timeout), callback=self._capture_resource_callback, error_callback=self._error_resource_callback)
        self._service_pair_msg_q.append(msg_id)

    def _capture_resource_callback(self, msg_id, msg):
        """
        Handle the response from a capture service pair request.

        :param uuid_msgs.UniqueID msg_id: id of the request-response pair.
        :param concert_service_msgs.CaptureTeleopPairResponse msg: message response received
        """
        if msg_id in self._service_pair_msg_q:
            self._service_pair_msg_q.remove(msg_id)
            for callback in self._callback['capture']:
                callback(self.captured_resource_uri, msg)

    def _error_resource_callback(self, msg_id, error_message):
        """ User callback to feed into non-blocking requests.

          @param msg_id : id of the request-response pair.
          @type uuid_msgs.UniqueID
            
          @param error_message : error string received
          @type str
        """
        for callback in self._callback['error']:
            callback(error_message)

    def release_resource(self, uri):
        """
        caputre the resource with rocon uri

        @param rocon_uri: resource information as uri type
        @type rocon_uri class
        """
        request = concert_service_msgs.CaptureResourceRequest()
        request.rocon_uri = uri
        request.release = True
        self.captured_resource_uri = request.rocon_uri
        msg_id = self._pair_capture_resource(request, timeout=rospy.Duration(self._capture_timeout), callback=self._release_resource_callback, error_callback=self._error_resource_callback)
        self._service_pair_msg_q.append(msg_id)

    def _release_resource_callback(self, msg_id, msg):
        """ User callback to feed into non-blocking requests.

        @param msg_id : id of the request-response pair.
        @type uuid_msgs.UniqueID

        @param msg : message response received
        @type <name>Response
         """
        if msg_id in self._service_pair_msg_q:
            self._service_pair_msg_q.remove(msg_id)
            self.captured_resource_uri = None
            for callback in self._callback['release']:
                callback(self.captured_resource_uri, msg)

    def _update_resource_list(self, msg):
        ''' 
        Receives available resource list from resource pimp.
        '''
        diff = lambda l1, l2: [x for x in l1 if x not in [l for l in l2]]
        new_resources = diff(msg.strings, self._resource_list)
        gone_resources = diff(self._resource_list, msg.strings)

        if new_resources or gone_resources:
            self._resource_list = msg.strings
            for callback in self._callback['refresh_list']:
                callback(msg.strings)

    def close(self):
        if self.captured_resource_uri:
            self.release_resource(self.captured_resource_uri)
        super(ResourceChooserInterface, self).close()
