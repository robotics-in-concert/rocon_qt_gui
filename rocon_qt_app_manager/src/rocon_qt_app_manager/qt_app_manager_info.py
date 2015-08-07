#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rospy
import rocon_python_comms
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


class QtRappManagerInfo(object):

    def __init__(self, update_rapp_callback):
        self._available_rapps = {}
        self._running_rapp = None
        self._update_rapps_callback = update_rapp_callback

        self._current_namespace = ''
        self._rapp_list_subscriber = None
        self._status_subscriber = None

#     def _update_rapps(self, data):
#         """
#         Update the available rapp list
# 
#         @param data: information of rapps
#         @type rocon_app_manager_msgs/RappList
#         """
#         self._available_rapps = list_rapp_msg_to_dict(data.available_rapps)
#         self._running_rapps = list_rapp_msg_to_dict(data.running_rapps)

    def _process_rapp_list_msg(self, msg):
        """
        Update the available rapp list

        @param data: information of rapps
        @type rocon_app_manager_msgs/RappList
        """
        self._available_rapps = list_rapp_msg_to_dict(msg.available_rapps)
        self._update_rapps_callback()

    def _process_status_msg(self, msg):
        """
        Update the available rapp list

        @param data: information of rapps
        @type rocon_app_manager_msgs/RappList
        """
        self._running_rapp = msg.rapp.name if msg.rapp_status != 'stopped' else None
        self._update_rapps_callback()

    def select_rapp_manager(self, namespace):
        if self._rapp_list_subscriber and self._current_namespace != namespace:
            self._status_subscriber.unregister()
            self._status_subscriber = None
            self._rapp_list_subscriber.unregister()
            self._rapp_list_subscriber = None
        self._rapp_list_subscriber = rospy.Subscriber(namespace + 'rapp_list', RappList, self._process_rapp_list_msg)
        self._status_subscriber = rospy.Subscriber(namespace + 'status', Status, self._process_status_msg)
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

#     def _get_rapps(self, namespace):
#         """
#         Getting the rapp list using service calling
#         @param namespace: namespace of running rapp
#         @type String
# 
#         @param service_name: service_name
#         @type String
#         """
#         service_handle = rospy.ServiceProxy(namespace + 'list_rapps', GetRappList)
#         self._update_rapps(service_handle())

    def start_rapp(self, rapp_name, remappings, parameters):
        """
        Start rapp

        :param rapp_name: rapp to start
        :type rapp_name: str
        :param remappings: remapping rules
        :type remappings: list
        :param parameters: public parameters
        :type parameters: list
        """
        remaps = [Remapping(key, value) for key, value in remappings]
        params = [KeyValue(k, v) for k, v in parameters]

        print('Starting %s with %s, %s' % (rapp_name, str(params), str(remaps)))
        service_handle = rospy.ServiceProxy(self._current_namespace + 'start_rapp', StartRapp)
        call_result = service_handle(rapp_name, remaps, params)
        if call_result.started:
            self._current_rapp = rapp_name
        return call_result

    def stop_rapp(self):
        """
        Stop rapp

        @param namespace: name space of running rapp
        @type String
        """
        service_handle = rospy.ServiceProxy(self._current_namespace + 'stop_rapp', StopRapp)
        call_result = service_handle()
        return call_result

    def get_running_rapps(self):
        running_rapps = {}
        try:
            running_rapps[self._running_rapp] = self._available_rapps[self._running_rapp]
        except KeyError:
            pass
        return running_rapps

    def is_running_rapp(self, rapp):
        if self._running_rapp == rapp['name']:
            return True
        else:
            return False
