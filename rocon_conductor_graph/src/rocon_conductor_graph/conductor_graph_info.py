#!/usr/bin/env python
#


import rospy
import rosgraph
from rosgraph.impl.graph import Edge, EdgeList
from concert_msgs.msg import ConcertClients
from rocon_app_manager_msgs.srv import GetPlatformInfo, Status, Invite, StartApp, StopApp
from rocon_app_manager_msgs.msg import PlatformInfo

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
            connection_strength = self.set_random_connection_strength()
            
            #uuid
            service = k.gateway_name+'/'+'platform_info'
            service_handle = rospy.ServiceProxy(service, GetPlatformInfo)
            call_result = service_handle()
            if k.gateway_name.count(call_result.platform_info.name)>0:                    
                uuid = k.gateway_name.replace(call_result.platform_info.name,'')
            else:
                uuid ="None"
                
            #html
            app_context = "<html>"
            app_context += "<p>-------------------------------------------</p>"
            app_context += "<p><b>name: </b>" +k.name+"</p>"
            app_context += "<p><b>gateway_name: </b>" +k.gateway_name+"</p>"
            app_context += "<p><b>platform: </b>" +k.platform+"</p>"
            app_context += "<p><b>system: </b>" +k.system+"</p>"
            app_context += "<p><b>robot: </b>" +k.robot+"</p>"
            app_context += "<p><b>client_status: </b>" +k.client_status+"</p>"
            app_context += "<p><b>connection_strengh: </b>" +connection_strength+"</p>"
            app_context +="</html>"
            app_name = k.name            
           
            if self._client_info_list.has_key(app_name):
                self._client_info_list[app_name]["isNew"] = False
            else:
                self._client_info_list[app_name]={}
                self._client_info_list[app_name]["isNew"] = True

            self._client_info_list[app_name]["isCheck"]=True
            self._client_info_list[app_name]["app_name"]=app_name
            self._client_info_list[app_name]["gateway_name"]=k.gateway_name
            self._client_info_list[app_name]["app_context"]=app_context
            self._client_info_list[app_name]["connection_strength"]=connection_strength  
            self._client_info_list[app_name]["uuid"]= uuid
  
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
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
