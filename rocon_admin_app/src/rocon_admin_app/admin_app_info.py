#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
import rospy

from concert_msgs.msg import ConcertServices

##############################################################################
# Graph
##############################################################################


class AdminAppInfo(object):
    def __init__(self):
        self._event_callback = None

        self.service_list = {}
        # should make use of concert_msgs/Strings here.
        rospy.Subscriber("/concert/services/list", ConcertServices, self.update_service_list)

    def _reg_event_callback(self, func):
        self._event_callback = func


    def update_service_list(self, data):   
        print "update_service_list start"
        self.service_list = {}
        for k in data.services:
            service_name = k.name
            self.service_list[service_name] = {}
            self.service_list[service_name]['name'] = service_name
            self.service_list[service_name]['description'] = k.description
            
            self.service_list[service_name]['author'] = k.author
            self.service_list[service_name]['priority'] = k.priority
            self.service_list[service_name]['launcher_type'] = k.launcher_type
            self.service_list[service_name]['launcher'] = k.launcher
            self.service_list[service_name]['uuid'] = k.uuid
            self.service_list[service_name]['status'] = k.status
            self.service_list[service_name]['enabled'] = k.enabled
            # not implementation
            self.service_list[service_name]['client_list'] = {}
                
            #html
            service_context = "<html>"
            service_context += "<p>-------------------------------------------</p>"
            service_context += "<p><b>name: </b>" +k.name+"</p>"
            service_context += "<p><b>description: </b>" +k.description+"</p>"
            service_context += "<p><b>author: </b>" +k.author+"</p>"
            service_context += "<p><b>launcher_type: </b>" +k.launcher_type+"</p>"
            service_context += "<p><b>launcher: </b>" +k.launcher+"</p>"
            service_context += "<p><b>uuid: </b>" +str(k.uuid)+"</p>"
            service_context += "<p><b>status: </b>" +str(k.status)+"</p>"
            service_context += "<p><b>enabled: </b>" +str(k.enabled)+"</p>"
            service_context +="</html>"
            
            self.service_list[service_name]['context'] = service_context
            
        if self._event_callback != None:
            self._event_callback()
     
