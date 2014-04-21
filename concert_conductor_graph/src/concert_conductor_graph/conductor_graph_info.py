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
        self._conductor_name = "conductor"
        self.gateway_nodes = []  # Gateway nodes
        self.gateway_edges = []  # Gateway-Gateway edges
        self.bad_nodes = []  # Gateway nodes

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
                time.sleep(0.1)  # just loop around
        rospy.logwarn("Setting up subscribers inside %s" % namespace)
        rospy.Subscriber(graph_topic_name, concert_msgs.ConductorGraph, self._update_callback)
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
        self.gateway_nodes = []
        self.gateway_nodes.append(self._conductor_name)
        self.gateway_edges = EdgeList()

        client_list = []

        for k in data.clients:
            client_list.append((k, True))
        for k in data.uninvited_clients:
            client_list.append((k, False))

        for client in client_list:
            k = client[0]
            client_name = client[0].name

            if client_name in self._client_info_list.keys():
                self._client_info_list[client_name]["is_new"] = False
            else:
                self._client_info_list[client_name] = {}
                self._client_info_list[client_name]["is_new"] = True

            self._client_info_list[client_name]["is_check"] = True

            self._client_info_list[client_name]["is_invite"] = client[1]
            self._client_info_list[client_name]["name"] = client[0].name
            self._client_info_list[client_name]["gateway_name"] = client[0].gateway_name
            self._client_info_list[client_name]["platform_info"] = client[0].platform_info

            self._client_info_list[client_name]["is_local_client"] = client[0].is_local_client
            self._client_info_list[client_name]["state"] = client[0].state

            self._client_info_list[client_name]["conn_stats"] = client[0].conn_stats
            self._client_info_list[client_name]["gateway_available"] = client[0].conn_stats.gateway_available
            self._client_info_list[client_name]["time_since_last_seen"] = client[0].conn_stats.time_since_last_seen
            self._client_info_list[client_name]["ping_latency_min"] = client[0].conn_stats.ping_latency_min
            self._client_info_list[client_name]["ping_latency_max"] = client[0].conn_stats.ping_latency_max
            self._client_info_list[client_name]["ping_latency_avg"] = client[0].conn_stats.ping_latency_avg
            self._client_info_list[client_name]["ping_latency_mdev"] = client[0].conn_stats.ping_latency_mdev

            self._client_info_list[client_name]["network_info_available"] = client[0].conn_stats.network_info_available
            self._client_info_list[client_name]["network_type"] = client[0].conn_stats.network_type
            self._client_info_list[client_name]["wireless_bitrate"] = client[0].conn_stats.wireless_bitrate
            self._client_info_list[client_name]["wireless_link_quality"] = client[0].conn_stats.wireless_link_quality
            self._client_info_list[client_name]["wireless_signal_level"] = client[0].conn_stats.wireless_signal_level
            self._client_info_list[client_name]["wireless_noise_level"] = client[0].conn_stats.wireless_noise_level

            self._client_info_list[client_name]["rapps"] = {}

            for l in client[0].rapps:
                app_name = l.name
                self._client_info_list[client_name]["rapps"][app_name] = {}
                self._client_info_list[client_name]["rapps"][app_name]['name'] = l.name
                self._client_info_list[client_name]["rapps"][app_name]['display_name'] = l.display_name
                self._client_info_list[client_name]["rapps"][app_name]['description'] = l.description
                self._client_info_list[client_name]["rapps"][app_name]['compatibility'] = l.compatibility
                self._client_info_list[client_name]["rapps"][app_name]['status'] = l.status

            #text info
            app_context = "<html>"
            app_context += "<p>-------------------------------------------</p>"
            app_context += "<p><b>concert_alias: </b>" + client[0].name + "</p>"
            app_context += "<p><b>gateway_name: </b>" + client[0].gateway_name + "</p>"
            app_context += "<p><b>rocon_uri: </b>" + client[0].platform_info.uri + "</p>"
            app_context += "<p><b>state: </b>" + client[0].state + "</p>"
            for l in self._client_info_list[client_name]["rapps"].values():
                app_context += "<p>-------------------------------------------</p>"
                app_context += "<p><b>app_name: </b>" + l['name'] + "</p>"
                app_context += "<p><b>app_display_name: </b>" + l['display_name'] + "</p>"
                app_context += "<p><b>app_description: </b>" + l['description'] + "</p>"
                app_context += "<p><b>app_compatibility: </b>" + l['compatibility'] + "</p>"
                app_context += "<p><b>app_status: </b>" + l['status'] + "</p>"
            app_context += "</html>"
            self._client_info_list[client_name]["app_context"] = app_context
            #How to set the strength range???
            connection_strength = self.get_connection_strength(client[0].is_local_client, client[0].conn_stats.wireless_link_quality)
            self._client_info_list[client_name]["connection_strength"] = connection_strength

            #graph info
            self.gateway_nodes.append(client[0].name)
            if client[1] == False:  # uninvited client has only node,no link.
                continue
            if client[0].conn_stats.gateway_available == True:
                if client[0].is_local_client == True:
                    self.gateway_edges.add(Edge(self._conductor_name, client[0].name, "local"))
                elif client[0].conn_stats.network_type == ConnectionStatistics.WIRED:
                    self.gateway_edges.add(Edge(self._conductor_name, client[0].name, "wired"))
                elif client[0].conn_stats.network_type == ConnectionStatistics.WIRELESS:
                    self.gateway_edges.add(Edge(self._conductor_name, client[0].name, "wireless"))
                else:
                    print "[conductor_graph_info]: Unknown network type"
            else:
                print "[conductor_graph_info]:No network connection"
        #new node check
        for k in self._client_info_list.keys():
            if self._client_info_list[k]["is_check"] == True:
                self._client_info_list[k]["is_check"] = False
            else:
                del self._client_info_list[k]
        #update check
        if self._compare_client_info_list():
            pass
        else:
            self._event_callback()
        self._pre_client_info_list = copy.deepcopy(self._client_info_list)
        #call period callback function
        if self._period_callback != None:
            self._period_callback()

    def _reg_event_callback(self, func):
        self._event_callback = func

    def _reg_period_callback(self, func):
        self._period_callback = func

    def _compare_client_info_list(self):
        result = True
        pre = self._pre_client_info_list
        cur = self._client_info_list
        for k in cur.values():
            client_name = k["name"]
            if not client_name in pre.keys():
                continue
            if pre[client_name]["state"] != cur[client_name]["state"]:
                result = False
            elif pre[client_name]["connection_strength"] != cur[client_name]["connection_strength"]:
                result = False
            elif pre[client_name]["gateway_available"] != cur[client_name]["gateway_available"]:
                result = False

        return result
        pass

    def get_connection_strength(self, is_local_client, link_quality):
        if is_local_client == True:
            return 'very_strong'
        else:
            link_quality_percent = (float(link_quality) / 70) * 100
            if 80 < link_quality_percent and 100 >= link_quality_percent:
                return 'very_strong'
            elif 60 < link_quality_percent and 80 >= link_quality_percent:
                return 'strong'
            elif 40 < link_quality_percent and 60 >= link_quality_percent:
                return 'normal'
            elif 20 < link_quality_percent and 40 >= link_quality_percent:
                return 'weak'
            elif 0 <= link_quality_percent and 20 >= link_quality_percent:
                return 'very_weak'
            else:
                return None
