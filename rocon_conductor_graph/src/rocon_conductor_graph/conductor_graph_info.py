#!/usr/bin/env python
#


import rospy
import rosgraph
from rosgraph.impl.graph import Edge, EdgeList
from concert_msgs.msg import ConcertClients
from rocon_std_msgs.srv import GetPlatformInfo
from rocon_app_manager_msgs.srv import Status, Invite, StartApp, StopApp
#from rocon_app_manager_msgs.msg import PlatformInfo

import random
from std_msgs.msg import String
##############################################################################
# Graph
##############################################################################

class ConductorGraphInfo(object):

    def __init__(self):
        '''
        Creates the polling topics necessary for updating statistics
        about the running gateway-hub network.
        '''
        self._last_update = 0
        self._gateway_namespace = None
        self._concert_conductor_name = "concert_conductor"
        self.gateway_nodes = []  # Gateway nodes
        self.gateway_edges = []  # Gateway-Gateway edges
        
        # Rubbish to clear out once rocon_gateway_graph is integrated
        self.bad_nodes = []
        
        rospy.Subscriber("/concert/list_concert_clients", ConcertClients, self.update_client_list)
        self._client_info_list = {}
        
       

    def update_client_list(self, data):
    
        print "update_client_list"
        
        #update dotgraph info
        self.gateway_nodes = []
        self.gateway_nodes.append(self._concert_conductor_name)
        self.gateway_edges = EdgeList()
        
        for k in data.clients:
            self.gateway_nodes.append(k.name)
            if k.client_status == 'connected':
                self.gateway_edges.add(Edge(self._concert_conductor_name,k.name,k.client_status))
        
        #update app widget info
        for k in data.clients:
            #temp function for assigning connection strength
            #connection_strength = self.set_random_connection_strength()
            connection_strength = "very_strong"
            
            #uuid
            uuid='None'
            
            client_name = k.name            
            
            if self._client_info_list.has_key(client_name):
                self._client_info_list[client_name]["isNew"] = False
            else:
                self._client_info_list[client_name]={}
                self._client_info_list[client_name]["isNew"] = True
            self._client_info_list[client_name]["isCheck"]=True
            
            self._client_info_list[client_name]["name"]=k.name
            self._client_info_list[client_name]["gateway_name"]=k.gateway_name
            self._client_info_list[client_name]["os"]=k.os
            self._client_info_list[client_name]["version"]=k.version
            self._client_info_list[client_name]["system"]=k.system
            self._client_info_list[client_name]["platform"]=k.platform
            self._client_info_list[client_name]["client_status"]=k.client_status
            
            self._client_info_list[client_name]["last_connection_timestamp"]=k.last_connection_timestamp
            self._client_info_list[client_name]["apps"]={}
            
            for L in self._client_info_list[client_name]["apps"].values():
                print L
                self._client_info_list[client_name]["apps"]['name'] = L.name
                self._client_info_list[client_name]["apps"]['display_name'] = L.display_name
                self._client_info_list[client_name]["apps"]['description'] = L.description
                self._client_info_list[client_name]["apps"]['platform'] = L.platform
                self._client_info_list[client_name]["apps"]['status'] = L.status

            self._client_info_list[client_name]["uuid"]= uuid
           
            #html
            app_context = "<html>"
            app_context += "<p>-------------------------------------------</p>"
            app_context += "<p><b>name: </b>" +k.name+"</p>"
            app_context += "<p><b>gateway_name: </b>" +k.gateway_name+"</p>"
            app_context += "<p>-------------------------------------------</p>"
            app_context += "<p><b>os: </b>" +k.os+"</p>"
            app_context += "<p><b>version: </b>" +k.version+"</p>"
            app_context += "<p><b>system: </b>" +k.system+"</p>"
            app_context += "<p><b>platform: </b>" +k.platform+"</p>"
            app_context += "<p>-------------------------------------------</p>"
            app_context += "<p><b>client_status: </b>" +k.client_status+"</p>"
            app_context += "<p><b>app_status: </b>" +k.app_status+"</p>"
            #app_context += "<p><b>last_connection_timestamp: </b>" +str(k.last_connection_timestamp.time)+"</p>"
            
            for L in self._client_info_list[client_name]["apps"].values():
                app_context += "<p>-------------------------------------------</p>"
                app_context += "<p><b>app_name: </b>" +L['name']+"</p>"
                app_context += "<p><b>app_display_name: </b>" +L['display_name']+"</p>"
                app_context += "<p><b>app_description: </b>" +L['description']+"</p>"
                app_context += "<p><b>app_platform: </b>" +L['platform']+"</p>"
                app_context += "<p><b>app_status: </b>" +L['status']+"</p>"
                             
            app_context +="</html>"
            
            self._client_info_list[client_name]["connection_strength"]=connection_strength
            self._client_info_list[client_name]["app_context"]=app_context
  
        for k in self._client_info_list.keys():
            if self._client_info_list[k]["isCheck"] == True:
                self._client_info_list[k]["isCheck"] = False
            else:
                del self._client_info_list[k]


    def set_random_connection_strength(self):
        connection_strength = random.randrange(1,6)
        if connection_strength == 1:
            return 'very_strong'
        elif connection_strength == 2:
            return 'strong'
        elif connection_strength == 3:
            return 'normal'
        elif connection_strength == 4:
            return 'weak'
        elif connection_strength == 5:
            return 'very_weak'
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
