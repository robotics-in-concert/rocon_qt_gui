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
from rocon_app_manager_msgs.msg import AppList
from rocon_app_manager_msgs.srv import StartApp
from rocon_app_manager_msgs.srv import StopApp
from rocon_app_manager_msgs.srv import GetAppList

##############################################################################
# QtAppManagerInfo
##############################################################################


class QtAppManagerInfo(object):
    def __init__(self):
        self.apps = {}
        self.running_apps = {}
        self._update_apps_callback = None
        #rospy.Subscriber('/app_manager/app_list', AppList, self._update_apps)

    def _update_apps(self, data):
        """
        Update the available app list

        @param data: information of apps
        @type rocon_app_manager_msgs/AppList
        """
        self.apps = {}
        self.running_apps = {}
        for app in data.available_apps:
            self.apps[app] = {}
            self.apps[app]["running"] = False
            self.apps[app]["name"] = app.name
            self.apps[app]["display_name"] = app.display_name
            self.apps[app]["description"] = app.description
            self.apps[app]["compatibility"] = app.compatibility
            self.apps[app]["icon"] = app.icon
            self.apps[app]["pairing_clients"] = app.pairing_clients
            self.apps[app]["required_capabilities"] = app.required_capabilities
        for app in data.running_apps:
            self.running_apps[app] = {}
            self.running_apps[app]["running"] = True
            self.running_apps[app]["name"] = app.name
            self.running_apps[app]["display_name"] = app.display_name
            self.running_apps[app]["description"] = app.description
            self.running_apps[app]["compatibility"] = app.compatibility
            self.running_apps[app]["icon"] = app.icon
            self.running_apps[app]["pairing_clients"] = app.pairing_clients
            self.running_apps[app]["required_capabilities"] = app.required_capabilities
        #Call update callback
        self._update_apps_callback()

    def _get_namespace(self):
        get_app_list_service_names = rocon_python_comms.find_service('rocon_app_manager_msgs/GetAppList', timeout=rospy.rostime.Duration(5.0), unique=False)
        return get_app_list_service_names

    def _get_apps(self, namespace, service_name):
        """
        Getting the app list using service calling
        @param namespace: namespace of running app
        @type String

        @param service_name: service_name
        @type String
        """
        service_handle = rospy.ServiceProxy(namespace + service_name, GetAppList)
        self._update_apps(service_handle())
        pass

    def _get_app_info(self, app):
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

        service_handle = rospy.ServiceProxy(namespace + 'start_app', StartApp)
        call_result = service_handle(app_name, [remapping, ])
        call_result_html = "<html>"
        call_result_html += "<p>-------------------------------------------</p>"
        call_result_html += "<p><b>started: </b>" + str(call_result.started) + "</p>"
        call_result_html += "<p><b>error_code: </b>" + str(call_result.error_code) + "</p>"
        call_result_html += "<p><b>message: </b>" + call_result.message + "</p>"
        call_result_html += "<p><b>app_namespace: </b>" + call_result.app_namespace + "</p>"
        call_result_html += "</html>"
        return call_result_html

    def _stop_app(self, namespace):
        """
        Stop App

        @param namespace: name space of running app
        @type String
        """
        service_handle = rospy.ServiceProxy(namespace + 'stop_app', StopApp)
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
