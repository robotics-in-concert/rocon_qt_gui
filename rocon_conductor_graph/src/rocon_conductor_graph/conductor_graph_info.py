#!/usr/bin/env python
#


import rospy
import rosgraph
from rosgraph.impl.graph import Edge, EdgeList
from concert_msgs.msg import ConcertClients
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
        
        #update tab widget info
        self._client_info_list = {}
        for k in data.clients:
            #temp function
            connection_strength = self.set_random_connection_strength()
        
            #html
            tab_context = "<html>"
            tab_context += "<p>-------------------------------------------</p>"
            tab_context += "<p><b>name: </b>" +k.name+"</p>"
            tab_context += "<p><b>gateway_name: </b>" +k.gateway_name+"</p>"
            tab_context += "<p><b>platform: </b>" +k.platform+"</p>"
            tab_context += "<p><b>system: </b>" +k.system+"</p>"
            tab_context += "<p><b>robot: </b>" +k.robot+"</p>"
            tab_context += "<p><b>client_status: </b>" +k.client_status+"</p>"
            tab_context += "<p><b>connection_strengh: </b>" +connection_strength+"</p>"
            tab_context +="</html>"
            tab_name = k.name            
           
            self._client_info_list[tab_name]={}
            self._client_info_list[tab_name]["tab_name"]=tab_name
            self._client_info_list[tab_name]["tab_context"]=tab_context
            self._client_info_list[tab_name]["connection_strength"]=connection_strength  
        
    def update(self):
        print "update info"
        pass       
        # Gateways
    
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
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
