#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
import yaml
import rospy
import rocon_python_utils
import concert_msgs.msg as concert_msgs
from concert_msgs.srv import EnableService

##############################################################################
# Graph
##############################################################################


class AdminAppInterface(object):
    def __init__(self):
        self._event_callback = None
        self.full_params_path  = ""
        self.service_list = {}
        # should make use of concert_msgs/Strings here.
        rospy.Subscriber("concert/services/list", concert_msgs.Services, self.update_service_list)

    def _reg_event_callback(self, func):
        self._event_callback = func

    
    def eable_service(self, srv_name):
        service = "/concert/services/enable"
        service_handle=rospy.ServiceProxy(service, EnableService)
        call_result=service_handle(srv_name,True)
        print call_result


    def disable_service(self, srv_name):
        service = "/concert/services/enable"
        service_handle=rospy.ServiceProxy(service, EnableService)
        call_result=service_handle(srv_name,False)
        print call_result

    def get_srv_parameters(self, parameters_path):
        params = None
        if parameters_path:
            self.full_params_path = rocon_python_utils.ros.resources.find_resource_from_string(parameters_path+".parameters")
            try:
                stream = open(self.full_params_path, 'r')
                params = yaml.load(stream)
                
            except Exception, e:
                rospy.loginfo(e)
            else:
                return params                

    def set_srv_parameters(self, data):
        print params
        try:
            if not self.full_params_path:
                return False
            params = file(self.full_params_path, 'w')
            yaml.dump(data, params, default_flow_style=False) 
        except Exception, e:
            rospy.loginfo(e)
            return False
        else:
            return True
        
    def update_service_list(self, data):
        print "update_service_list start"
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
            self.service_list[service_name]['uuid'] = k.uuid
            self.service_list[service_name]['status'] = k.status
            self.service_list[service_name]['enabled'] = k.enabled
            # not implementation
            self.service_list[service_name]['client_list'] = {}

            #html
            service_context = "<html>"
            service_context += "<p>-------------------------------------------</p>"
            service_context += "<p><b>name: </b>" + k.name + "</p>"
            service_context += "<p><b>resource_name: </b>" + k.resource_name + "</p>"
            service_context += "<p><b>description: </b>" + k.description + "</p>"
            service_context += "<p><b>author: </b>" + k.author + "</p>"
            service_context += "<p><b>priority: </b>" + str(k.priority) + "</p>"
            service_context += "<p><b>launcher_type: </b>" + k.launcher_type + "</p>"
            service_context += "<p><b>launcher: </b>" + k.launcher + "</p>"
            service_context += "<p><b>interactions: </b>" + k.interactions + "</p>"
            service_context += "<p><b>parameters: </b>" + k.parameters + "</p>"
            
            service_context += "<p><b>uuid: </b>" + str(k.uuid) + "</p>"
            service_context += "<p><b>status: </b>" + str(k.status) + "</p>"
            service_context += "<p><b>enabled: </b>" + str(k.enabled) + "</p>"
            service_context += "</html>"

            self.service_list[service_name]['context'] = service_context

        if self._event_callback != None:
            self._event_callback()
