#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import copy
import time

import rospy
from rosgraph.impl.graph import Edge, EdgeList
import concert_msgs.msg as concert_msgs
from gateway_msgs.msg import ConnectionStatistics
import rocon_python_comms

from .concert_client import ConcertClient

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
        self._conductor_name = "conductor"
        self.gateway_nodes = []
        self.gateway_nodes.append(self._conductor_name)
        self.gateway_edges = []  # Gateway-Gateway edges
        self.bad_nodes = []  # Gateway nodes
        self.pending_nodes = []
        self.uninvited_nodes = []
        self.joining_nodes = []
        self.available_nodes = []
        self.missing_nodes = []
        self.blocking_nodes = []
        self.gone_nodes = []

        #Rubbish to clear out once rocon_gateway_graph is integrated
        self._event_callback = None
        self._period_callback = None
        self.is_first_update = False

        while not rospy.is_shutdown():
            try:
                graph_topic_name = rocon_python_comms.find_topic('concert_msgs/ConductorGraph', timeout=rospy.rostime.Duration(0.1), unique=True)
                (namespace, unused_topic_name) = graph_topic_name.rsplit('/', 1)
                clients_topic_name = namespace + "/concert_clients"  # this is assuming they didn't remap this bugger.
                break
            except rocon_python_comms.NotFoundException:
                pass  # just loop around
        rospy.logwarn("Setting up subscribers inside %s" % namespace)
        # get data on all clients, even those not connected
        rospy.Subscriber(graph_topic_name, concert_msgs.ConductorGraph, self._update_callback)
        # get the periodic data of connected clients
        rospy.Subscriber(clients_topic_name, concert_msgs.ConcertClients, self.update_client_list)

        self._client_info_list = {}
        self._pre_client_info_list = {}

    def _update_callback(self, data):
        if self._event_callback != None:
            self._event_callback()

    def update_client_list(self, data):
        print("[conductor_graph_info]: update_client_list")

        if self.is_first_update == False:
            if self._event_callback != None:
                self._event_callback()
            if self._period_callback != None:
                self._period_callback()

            self.is_first_update = True

        #update dotgraph info
        self.gateway_edges = EdgeList()

        for msg in data.clients:
            if msg.name in self._client_info_list.keys():
                self._client_info_list[msg.name].is_new = False
            else:
                self._client_info_list[msg.name] = ConcertClient(msg)
                self.gateway_nodes.append(msg.name)
            # an alias to the variable for working with
            concert_client = self._client_info_list[msg.name]

            #graph info
            if msg.conn_stats.gateway_available == True:
                self.gateway_edges.add(Edge(self._conductor_name, concert_client.concert_alias, concert_client.link_type))

        #new node check - DJS this was broken for me, what did it do?
#         for concert_client_name in self._client_info_list.keys():
#             if self._client_info_list[concert_client_name].is_checked:
#                 self._client_info_list[concert_client_name].is_checked = False
#             else:
#                 del self._client_info_list[concert_client_name]
        #update check
        if not self._compare_client_info_list():
            self._event_callback()

        self._pre_client_info_list = copy.deepcopy(self._client_info_list)
        #call period callback function
        if self._period_callback != None:
            self._period_callback()

    def _register_event_callback(self, func):
        self._event_callback = func

    def _register_period_callback(self, func):
        self._period_callback = func

    def _compare_client_info_list(self):
        result = True
        pre = self._pre_client_info_list
        cur = self._client_info_list
        for k in cur.values():
            client_name = k.msg.name
            if not client_name in pre.keys():
                continue
            if pre[client_name].msg.state != cur[client_name].msg.state:
                result = False
            elif pre[client_name].get_connection_strength() != cur[client_name].get_connection_strength():
                result = False
            elif pre[client_name].msg.conn_stats.gateway_available != cur[client_name].msg.conn_stats.gateway_available:
                result = False

        return result
