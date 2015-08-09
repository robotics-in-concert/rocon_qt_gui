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
import rocon_std_msgs.msg as rocon_std_msgs
from rocon_std_msgs.msg import Remapping, KeyValue
from rocon_app_manager_msgs.msg import Status, RappList
from rocon_app_manager_msgs.srv import StartRapp, StopRapp

##############################################################################
# QtRappManagerInfo
##############################################################################


def rapp_msg_to_dict(msg):
    rapp = {}
    rapp["status"] = msg.status
    rapp["name"] = msg.name
    rapp["display_name"] = msg.display_name
    rapp["description"] = msg.description
    rapp["compatibility"] = msg.compatibility
    rapp["preferred"] = msg.preferred
    rapp["icon"] = msg.icon
    rapp["implementations"] = msg.implementations
    rapp["public_interface"] = msg.public_interface
    rapp["public_parameters"] = []
    for parameter in msg.public_parameters:
        if parameter.value.lower() == "false":
            value = False
        elif parameter.value.lower() == "true":
            value = True
        else:
            value = parameter.value
        rapp["public_parameters"].append(rocon_std_msgs.KeyValue(parameter.key, value))
    return rapp


def list_rapp_msg_to_dict(list_rapp):
    """
    convert msg to dict
    """
    dict_rapp = {}
    for rapp in list_rapp:
        name = rapp.name
        dict_rapp[name] = rapp_msg_to_dict(rapp)
    return dict_rapp


class QtRappManagerInfo(object):

    def __init__(self, update_rapp_callback):
        self._available_rapps = {}
        self._running_rapp = None
        self._update_rapps_callback = update_rapp_callback

        self._current_namespace = ''
        self._rapp_list_subscriber = None
        self._status_subscriber = None

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
        self._running_rapp = None
        running_rapp_name = msg.rapp.name if msg.rapp_status != 'stopped' else None
        if running_rapp_name is not None:
            self._running_rapp = rapp_msg_to_dict(msg.rapp)
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

    def start_rapp(self, rapp_name, remappings, parameters):
        """
        Start rapp

        :param rapp_name: rapp to start
        :type rapp_name: str
        :param remappings: remapping rules
        :type remappings: list
        :param parameters: public parameters
        :type parameters: list of rocon_std_msgs.KeyValue
        """
        remaps = [Remapping(key, value) for key, value in remappings]

        print('Starting %s with %s, %s' % (rapp_name, str(parameters), str(remaps)))
        service_handle = rospy.ServiceProxy(self._current_namespace + 'start_rapp', StartRapp)
        call_result = service_handle(rapp_name, remaps, parameters)
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
        if self._running_rapp is None:
            return {}
        running_rapps = {}
        try:
            if self._running_rapp in self._available_rapps.keys():
                running_rapps[self._running_rapp] = self._available_rapps[self._running_rapp]
            else:
                for rapp in self._available_rapps.values():
                    if self._running_rapp['name'] in rapp['implementations']:
                        running_rapps[rapp['name']] = rapp
                        break
        except KeyError:
            pass
        return running_rapps

    def is_running_rapp(self, rapp):
        result = False
        if self._running_rapp is not None:
            if self._running_rapp['name'] == rapp['name']:
                result = True
            elif self._running_rapp['name'] in rapp['implementations']:
                result = True
        return result
