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
from rocon_std_msgs.msg import Remapping
from rocon_app_manager_msgs.msg import Status
from rocon_app_manager_msgs.srv import StartRapp
from rocon_app_manager_msgs.srv import StopRapp
from rocon_app_manager_msgs.srv import GetRappList

##############################################################################
# QtAppManagerInfo
##############################################################################


class QtAppManagerInfo(object):
    def __init__(self):
        self.apps = {}
        self.running_apps = {}
        self._update_apps_callback = None
        self.current_namespace = ''
        self.current_subscriber = None

    def _update_apps(self, data):
        """
        Update the available app list

        @param data: information of apps
        @type rocon_app_manager_msgs/AppList
        """
        self.apps = {}
        self.running_apps = {}
        for app in data.available_rapps:
            self.apps[app] = {}
            self.apps[app]["status"] = app.status
            self.apps[app]["name"] = app.name
            self.apps[app]["display_name"] = app.display_name
            self.apps[app]["description"] = app.description
            self.apps[app]["compatibility"] = app.compatibility
            self.apps[app]["icon"] = app.icon
            self.apps[app]["required_capabilities"] = app.required_capabilities
        for app in data.running_rapps:
            self.running_apps[app] = {}
            self.running_apps[app]["status"] = app.status
            self.running_apps[app]["name"] = app.name
            self.running_apps[app]["display_name"] = app.display_name
            self.running_apps[app]["description"] = app.description
            self.running_apps[app]["compatibility"] = app.compatibility
            self.running_apps[app]["icon"] = app.icon
            self.running_apps[app]["required_capabilities"] = app.required_capabilities
        #Call update callback
        self._update_apps_callback()

    def _update_app_status(self, data):
        self._get_apps(self.current_namespace)

    def _set_update_status(self, namespace):
        if self.current_subscriber:
            self.current_subscriber.unregister()
            self.current_subscriber = None
        self.current_subscriber = rospy.Subscriber(namespace + 'status', Status, self._update_app_status)
        self.current_namespace = namespace

    def _get_namespaces(self):
        """
        Getting the name space with running the app
        @return name space list
        @type list
        """
        get_app_list_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/GetRappList', timeout=rospy.rostime.Duration(5.0), unique=False)
        return get_app_list_service_names

    def _get_apps(self, namespace):
        """
        Getting the app list using service calling
        @param namespace: namespace of running app
        @type String

        @param service_name: service_name
        @type String
        """
        service_handle = rospy.ServiceProxy(namespace + 'list_rapps', GetRappList)
        self._update_apps(service_handle())

    def _get_app_info(self, app):
        """
        Getting the app information to html type
        @param app: information of app
        @type dict

        @return the app information
        @type String
        """

        app_info = "<html>"
        app_info += "<p>-------------------------------------------</p>"
        for info_key in app.keys():
            if info_key is 'icon':
                continue
            app_info += "<p><b>%s: </b>%s</p>" % (info_key, str(app[info_key]))
        app_info += "</html>"
        return app_info

    def _start_app(self, namespace, app_name):
        """
        Start App

        @param app_name: app to start
        @type String

        @param namespace: name space of running app
        @type String
        """
        #not yet
        remapping = Remapping()

        service_handle = rospy.ServiceProxy(namespace + 'start_rapp', StartRapp)
        call_result = service_handle(app_name, [remapping, ])
        call_result_html = "<html>"
        call_result_html += "<p>-------------------------------------------</p>"
        call_result_html += "<p><b>started: </b>" + str(call_result.started) + "</p>"
        call_result_html += "<p><b>error_code: </b>" + str(call_result.error_code) + "</p>"
        call_result_html += "<p><b>message: </b>" + call_result.message + "</p>"
        call_result_html += "</html>"
        return call_result_html

    def _stop_app(self, namespace):
        """
        Stop App

        @param namespace: name space of running app
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

    def _reg_update_apps_callback(self, func):
        self._update_apps_callback = func
