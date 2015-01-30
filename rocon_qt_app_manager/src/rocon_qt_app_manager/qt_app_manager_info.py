#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
#system
import os
#ros
import rospy
import rocon_python_comms
#rocon
from rocon_std_msgs.msg import Remapping, KeyValue
from rocon_app_manager_msgs.msg import Status, RappList
from rocon_app_manager_msgs.srv import StartRapp, StopRapp

##############################################################################
# QtRappManagerInfo
##############################################################################

def list_rapp_msg_to_dict(list_rapp):
    """
    convert msg to dict
    """
    dict_rapp = {}
    for rapp in list_rapp:
        name = rapp.name
        dict_rapp[name] = {}
        dict_rapp[name]["status"] = rapp.status
        dict_rapp[name]["name"] = rapp.name
        dict_rapp[name]["display_name"] = rapp.display_name
        dict_rapp[name]["description"] = rapp.description
        dict_rapp[name]["compatibility"] = rapp.compatibility
        dict_rapp[name]["preferred"] = rapp.preferred
        dict_rapp[name]["icon"] = rapp.icon
        dict_rapp[name]["implementations"] = rapp.implementations
        dict_rapp[name]["public_interface"] = rapp.public_interface
        dict_rapp[name]["public_parameters"] = rapp.public_parameters
    return dict_rapp


RAPP_LIST_TOPIC = 'rapp_list'

class QtRappManagerInfo(object):


    def __init__(self, update_rapp_callback):
        self._available_rapps = {}
        self._running_rapps = {}
        self._update_rapps_callback = update_rapp_callback

        self._current_namespace = ''
        self._current_subscriber = None

    def _update_rapps(self, data):
        """
        Update the available rapp list

        @param data: information of rapps
        @type rocon_app_manager_msgs/RappList
        """
        self._available_rapps = list_rapp_msg_to_dict(data.available_rapps)
        self._running_rapps = list_rapp_msg_to_dict(data.running_rapps)


    def _process_rapp_list_msg(self, msg):
        """
        Update the available rapp list
                                      
        @param data: information of rapps
        @type rocon_app_manager_msgs/RappList
        """
        
        self._available_rapps = list_rapp_msg_to_dict(msg.available_rapps)
        self._running_rapps = list_rapp_msg_to_dict(msg.running_rapps)
        self._update_rapps_callback()

    def select_rapp_manager(self, namespace):
        if self._current_subscriber and self._current_namespace != namespace:
            self._current_subscriber.unregister()
            self._current_subscriber = None
        self._current_subscriber = rospy.Subscriber(namespace + RAPP_LIST_TOPIC, RappList, self._process_rapp_list_msg)
        self._current_namespace = namespace

    def get_available_rapps(self):
        return self._available_rapps

    def _get_namespaces(self):
        """
        Getting the name space with running the rapp
        @return name space list
        @type list
        """
        get_rapp_list_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/GetRappList', timeout=rospy.rostime.Duration(5.0), unique=False)
        return get_rapp_list_service_names

    def _get_rapps(self, namespace):
        """
        Getting the rapp list using service calling
        @param namespace: namespace of running rapp
        @type String

        @param service_name: service_name
        @type String
        """
        service_handle = rospy.ServiceProxy(namespace + 'list_rapps', GetRappList)
        self._update_rapps(service_handle())

    def _get_rapp_info(self, rapp):
        """
        Getting the rapp information to html type
        @param rapp: information of rapp
        @type dict

        @return the rapp information
        @type String
        """

        rapp_info = "<html>"
        rapp_info += "<p>-------------------------------------------</p>"
        for info_key in rapp.keys():
            if info_key is 'icon':
                continue
            rapp_info += "<p><b>%s: </b>%s</p>" % (info_key, str(rapp[info_key]))
        rapp_info += "</html>"
        return rapp_info

    def _start_rapp(self, namespace, rapp_name, parameters):
        """
        Start rapp

        :param rapp_name: rapp to start
        :type rapp_name: str 
        :param namespace: name space of running rapp
        :type namespace: str
        :param parameters: public parameters
        :type parameters: dict
        """
        #not yet
        remapping = Remapping()
        params = [KeyValue(key, value) for key, value in parameters.items()]
        print('Starting %s with %s'%(rapp_name, str(params)))

        service_handle = rospy.ServiceProxy(namespace + 'start_rapp', StartRapp)
        call_result = service_handle(rapp_name, [], params) 
        call_result_html = "<html>"
        call_result_html += "<p>-------------------------------------------</p>"
        call_result_html += "<p><b>started: </b>" + str(call_result.started) + "</p>"
        call_result_html += "<p><b>error_code: </b>" + str(call_result.error_code) + "</p>"
        call_result_html += "<p><b>message: </b>" + call_result.message + "</p>"
        call_result_html += "</html>"
        return call_result_html

    def _stop_rapp(self, namespace):
        """
        Stop rapp

        @param namespace: name space of running rapp
        @type String
        """
        service_handle = rospy.ServiceProxy(namespace + 'stop_rapp', StopRapp)
        call_result = service_handle()
        call_result_html = "<html>"
        call_result_html += "<p>-------------------------------------------</p>"
        call_result_html += "<p><b>stopped: </b>" + str(call_result.stopped) + "</p>"
        call_result_html += "<p><b>error_code: </b>" + str(call_result.error_code) + "</p>"
        call_result_html += "<p><b>message: </b>" + call_result.message + "</p>"
        call_result_html += "</html>"
        return call_result_html

    def _get_icon(self):
        rapps = self.running_rapps
        for k in rapps.values():
            return k['icon']

    def _reg_update_rapps_callback(self, func):
        self._update_rapps_callback = func
