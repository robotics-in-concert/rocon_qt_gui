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
import concert_msgs.msg as concert_msgs
from gateway_msgs.msg import ConnectionStatistics
import rocon_python_comms
import rocon_console.console as console
from .concert_client import ConcertClient

##############################################################################
# Graph
##############################################################################


class ConductorGraphInfo(object):
    def __init__(self, change_callback, periodic_callback):
        '''
        Creates the polling topics necessary for updating statistics
        about the running gateway-hub network.
        '''
        self._last_update = 0
        self.concert_clients = {}  # dictionary of .concert_client.ConcertClient objects keyed by concert alias

        #Rubbish to clear out once rocon_gateway_graph is integrated
        self._change_callback = change_callback
        self._periodic_callback = periodic_callback
        self.is_first_update = False

        print(console.green + "Searching for the conductor's ros topics..." + console.reset)
        while not rospy.is_shutdown():
            try:
                graph_topic_name = rocon_python_comms.find_topic('concert_msgs/ConductorGraph', timeout=rospy.rostime.Duration(0.1), unique=True)
                (namespace, unused_topic_name) = graph_topic_name.rsplit('/', 1)
                clients_topic_name = namespace + "/concert_clients"  # this is assuming they didn't remap this bugger.
                break
            except rocon_python_comms.NotFoundException:
                pass  # just loop around
#         if rospy.is_shutdown():
#             return

        rospy.logwarn("Setting up subscribers inside %s" % namespace)
        # get data on all clients, even those not connected
        rospy.Subscriber(graph_topic_name, concert_msgs.ConductorGraph, self._update_clients_callback)
        # get the periodic data of connected clients
        rospy.Subscriber(clients_topic_name, concert_msgs.ConcertClients, self.update_connection_statistics)

    def _update_clients_callback(self, msg):
        '''
        Update the cached list of concert clients when a client comes, goes or
        changes its state. This update happens rather infrequently with every
        message supplied by the conductor's latched graph publisher.
        '''
        self._graph = msg
        # sneaky way of getting all the states and the lists
        visible_concert_clients_by_name = []
        for state in msg.__slots__:
            concert_clients = getattr(msg, state)  # by state
            for concert_client in concert_clients:
                visible_concert_clients_by_name.append(concert_client.name)
                if concert_client.name in self.concert_clients.keys():
                    self.concert_clients[concert_client.name].is_new = False
                else:
                    self.concert_clients[concert_client.name] = ConcertClient(concert_client)  # create extended ConcertClient class from msg
        # remove any that are no longer visible
        lost_clients_by_name = [concert_alias for concert_alias in self.concert_clients.keys() if concert_alias not in visible_concert_clients_by_name]
        for concert_alias in lost_clients_by_name:
            del self.concert_clients[concert_alias]
        self._change_callback()

    def update_connection_statistics(self, msg):
        '''
        Update the current list of concert clients' connection statistics. This
        happens periodically with every message supplied by the conductor's periodic
        publisher.

        :param msg concert_msgs.ConcertClients : graph of concert connected/connectable clients.
        '''
        print("[conductor_graph_info]: update_connection_statistics")
        for state in msg.__slots__:
            concert_clients = getattr(msg, state)  # by state
            for concert_client in concert_clients:  # concert_msgs.ConcertClient
                if concert_client.name in self.concert_clients.keys():
                    # pick up the changing information in the msg and pass it to our class
                    self.concert_clients[concert_client.name].update(concert_client)
                else:
                    pass  # just ignore it, the changes topic will pick up and drop new/old clients
        self._change_callback()
        self._periodic_callback()

    def _compare_client_info_list(self):
        """
          Not currently using this, but would be a more optimal way of indentifying when the graph
          changes so we can tell the drawing functions when to update rather than at every step.

          You would insert this logic into the update_connection_statistics to flag when you would
          want a change and/or a period callback.
        """
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
