#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rocon_console.console as console
import rocon_interactions
import rocon_interaction_msgs.msg as interaction_msgs
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_python_comms
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri
import rosgraph
import rospy
import uuid

from python_qt_binding.QtCore import QObject, Signal

from . import utils

##############################################################################
# Methods
##############################################################################


def get_namespaces():
    """
    Get the name space where an interactions manager is running.
    @return name space
    @rtype str
    """
    try:
        service_names = rocon_python_comms.find_service('rocon_interaction_msgs/GetInteractions',
                                                        timeout=rospy.rostime.Duration(5.0),
                                                        unique=False
                                                        )
    except rocon_python_comms.NotFoundException:
        rospy.logwarn("Remocon : failed to find an interactions manager")
        return []
    return [rosgraph.names.namespace(service_name) for service_name in service_names]


def get_pairings(interactions_namespace):
    if interactions_namespace is None:
        return rocon_interactions.PairingsTable()
    get_pairings = rospy.ServiceProxy(interactions_namespace + "get_pairings", interaction_srvs.GetPairings)
    request = interaction_srvs.GetPairingsRequest()
    response = get_pairings(request)
    pairings_table = rocon_interactions.PairingsTable()
    pairings_table.load(response.pairings)
    return pairings_table


def get_interactions(interactions_namespace, platform_rocon_uri):
    if interactions_namespace is None:
        return rocon_interactions.InteractionsTable()
    get_interactions = rospy.ServiceProxy(interactions_namespace + "get_interactions", interaction_srvs.GetInteractions)
    request = interaction_srvs.GetInteractionsRequest(groups=[], uri=platform_rocon_uri)
    response = get_interactions(request)
    interactions_table = rocon_interactions.InteractionsTable()
    interactions_table.load(response.interactions)
    return interactions_table


##############################################################################
# Interactions Manager
##############################################################################


class InteractionsRemocon(QObject):

    # PySide signals are always defined as class attributes (GPL Pyqt4 Signals use pyqtSignal)
    signal_pairing_updated = Signal()

    def __init__(self):
        super(InteractionsRemocon, self).__init__()
        self.key = uuid.uuid4()
        self.name = "rqt_remocon_" + self.key.hex
        self.namespaces = get_namespaces()
        self.active_namespace = None if not self.namespaces else self.namespaces[0]
        self.rocon_uri = rocon_uri.parse(
            rocon_uri.generate_platform_rocon_uri('pc', self.name) + "|" + utils.get_web_browser_codename()
        )
        self.active_pairing = None
        self.launched_interactions = {}  # dict of interaction hash : launch_list

        # be also great to have a configurable icon...with a default
        self.platform_info = rocon_std_msgs.MasterInfo(version=rocon_std_msgs.Strings.ROCON_VERSION,
                                                       rocon_uri=str(self.rocon_uri),
                                                       icon=rocon_std_msgs.Icon(),
                                                       description=""
                                                       )

        # Load Data
        self.pairings_table = get_pairings(self.active_namespace)
        self.interactions_table = get_interactions(self.active_namespace, "rocon:/")

        self.service_proxies = rocon_python_comms.utils.ServiceProxies(
            [
                (self.active_namespace + "request_interaction", interaction_srvs.RequestInteraction),
                (self.active_namespace + "start_pairing", interaction_srvs.StartPairing),
                (self.active_namespace + "stop_pairing", interaction_srvs.StopPairing)
            ]
        )
        self.remocon_status_publisher = rospy.Publisher("/remocons/" + self.name, interaction_msgs.RemoconStatus, latch=True, queue_size=5)
        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                (self.active_namespace + "pairing_status", interaction_msgs.PairingStatus, self._subscribe_pairing_status_callback)
            ]
        )
        self._publish_remocon_status()

    def connect(self, refresh_slot):
        self.signal_pairing_updated.connect(refresh_slot)

    def start_pairing(self, name):
        request = interaction_srvs.StartPairingRequest(name)
        response = self.service_proxies.start_pairing(request)
        return response

    def stop_pairing(self, name):
        request = interaction_srvs.StopPairingRequest(name)
        response = self.service_proxies.stop_pairing(request)
        return response

    def start_interaction(self):
        print("Starting interaction")

    def stop_interaction(self):
        print("Stop interaction")

    def _subscribe_pairing_status_callback(self, msg):
        if msg.active_pairing:
            self.active_pairing = self.pairings_table.find(msg.active_pairing)
            print("Pairing status: %s" % self.active_pairing.name)
        else:
            print("Pairing status: none")
            self.active_pairing = None
        self.signal_pairing_updated.emit()

    def _publish_remocon_status(self):
        remocon_status = interaction_msgs.RemoconStatus()
        remocon_status.platform_info = self.platform_info
        remocon_status.uuid = str(self.key.hex)
        remocon_status.version = rocon_std_msgs.Strings.ROCON_VERSION
        running_interactions = []
        for interaction_hash, launch_list in self.launched_interactions.iteritems():
            for unused_process_name in launch_list.keys():
                running_interactions.append(interaction_hash)
        remocon_status.running_interactions = running_interactions
        console.logdebug("Remocon : publishing remocon status")
        self.remocon_status_publisher.publish(remocon_status)
