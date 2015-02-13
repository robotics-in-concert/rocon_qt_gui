#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
import rospy
import rocon_python_comms

import concert_msgs.msg as concert_msgs
import rocon_std_msgs.msg as rocon_std_msgs

from concert_msgs.srv import EnableService
from concert_msgs.srv import UpdateServiceConfig

from rocon_python_comms.exceptions import NotFoundException
##############################################################################
# Graph
##############################################################################


class AdminAppInterface(object):

    def __init__(self):
        self._event_callback = None
        self.full_params_path = ""
        self.service_list = {}
        self.ros_subscribers = {}
        self.ros_services = {}

    def _init_admin_app_interface(self):
        self._init_ros_subscriber()
        self._init_ros_service()

    def _init_ros_subscriber(self):
        try:
            subscriber_topic_name = rocon_python_comms.find_topic('concert_msgs/Services', timeout=rospy.rostime.Duration(5.0), unique=True)
            self.ros_subscribers['service_list'] = rospy.Subscriber(subscriber_topic_name, concert_msgs.Services, self.update_service_list)
        except NotFoundException as e:
            raise e

    def _init_ros_service(self):
        try:
            enable_service = rocon_python_comms.find_service('concert_msgs/EnableService', timeout=rospy.rostime.Duration(5.0), unique=True)
            self.ros_services['enable_service'] = rospy.ServiceProxy(enable_service, EnableService)
            update_service_config = rocon_python_comms.find_service('concert_msgs/UpdateServiceConfig', timeout=rospy.rostime.Duration(5.0), unique=True)
            self.ros_services['update_service_config'] = rospy.ServiceProxy(update_service_config, UpdateServiceConfig)
        except NotFoundException as e:
            raise e

    def _reg_event_callback(self, func):
        self._event_callback = func

    def eable_service(self, srv_name):
        # todo
        call_result = self.ros_services['enable_service'](srv_name, True)
        return call_result.success, call_result.error_message

    def disable_service(self, srv_name):
        # todo
        call_result = self.ros_services['enable_service'](srv_name, False)
        return call_result.success, call_result.error_message

    def update_service_config(self, srv_profile):
        # todo
        call_result = self.ros_services['update_service_config'](srv_profile)
        return (call_result.success, call_result.error_message)

    def get_srv_parameters(self, parameters_detail):
        params = None
        params = {}
        for param in parameters_detail:
            params[param.key] = param.value
        return params

    def set_srv_parameters(self, srv_name, param_data):
        result = False
        message = ""
        try:
            srv_profile_msg = concert_msgs.ServiceProfile()
            srv_profile_msg.name = srv_name
            parameter_detail = []
            for key in param_data.keys():
                parameter = rocon_std_msgs.KeyValue(key, param_data[key])
                parameter_detail.append(parameter)
            srv_profile_msg.parameters_detail = parameter_detail
            (result, message) = self.update_service_config(srv_profile_msg)
        except Exception, e:
            rospy.loginfo(e)
            message = str(e)            
            result = False
        return (result, message)

    def update_service_list(self, data):

        self.service_list = {}
        for k in data.services:
            service_name = k.name
            self.service_list[service_name] = {}
            self.service_list[service_name]['name'] = service_name
            self.service_list[service_name]['resource_name'] = k.resource_name
            self.service_list[service_name]['description'] = k.description

            self.service_list[service_name]['author'] = k.author
            self.service_list[service_name]['priority'] = k.priority
            self.service_list[service_name]['launcher_type'] = k.launcher_type
            self.service_list[service_name]['launcher'] = k.launcher
            self.service_list[service_name]['interactions'] = k.interactions
            self.service_list[service_name]['parameters'] = k.parameters
            self.service_list[service_name]['parameters_detail'] = k.parameters_detail
            self.service_list[service_name]['uuid'] = k.uuid
            self.service_list[service_name]['status'] = k.status
            self.service_list[service_name]['enabled'] = k.enabled

            # html
            service_context = "<html>"
            for key in self.service_list[service_name].keys():
                service_context += "<p><b>" + key + ": </b>" + str(self.service_list[service_name][key]) + "</p>"
            service_context += "</html>"
            self.service_list[service_name]['context'] = service_context

        if self._event_callback != None:
            self._event_callback()
