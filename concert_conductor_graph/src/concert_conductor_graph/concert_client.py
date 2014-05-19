#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import concert_msgs.msg as concert_msgs
import gateway_msgs.msg as gateway_msgs

##############################################################################
# Classes
##############################################################################


class ConcertClient(object):

#     __slots__ = [
#        ' msg',        # gateway_msgs/ConcertClient
#        'is_new',      # seen for the first time, we react to this in the gui
#        'is_checked',  # whether the client is checked in the higher level
#        'link_type',   # used to show on the gui what link type it is
#     ]

    def __init__(self, msg):
        self.msg = msg
        self.is_new = True
        self.is_checked = True
        self.link_type = self._set_link_string()

    def _set_link_string(self):
        '''
        Returns the link type in string format from those in
        gateway_msgs.ConnectionStatus constants or 'unknown'.

        :returns: link type string
        :rtype: str
        '''
        if self.msg.is_local_client:
            return 'local'
        elif self.msg.conn_stats.network_type == gateway_msgs.ConnectionStatistics.WIRED:
            return 'wired'
        elif self.msg.conn_stats.network_type == gateway_msgs.ConnectionStatistics.WIRELESS:
            return 'wireless'
        else:
            return 'unknown'

    def get_rapp_context(self):
        '''
        Generate a html string used to display the rapp context of this client.
        '''
        # TODO check that it is a client that has rapp info first.
        context = "<html>"
        context += "<p>-------------------------------------------</p>"
        context += "<p><b>concert_alias: </b>" + self.msg.name + "</p>"
        context += "<p><b>gateway_name: </b>" + self.msg.gateway_name + "</p>"
        context += "<p><b>rocon_uri: </b>" + self.msg.platform_info.uri + "</p>"
        context += "<p><b>state: </b>" + self.msg.state + "</p>"
        for rapp in self.msg.rapps:
            context += "<p>-------------------------------------------</p>"
            context += "<p><b>app_name: </b>" + rapp.name + "</p>"
            context += "<p><b>app_display_name: </b>" + rapp.display_name + "</p>"
            context += "<p><b>app_description: </b>" + rapp.description + "</p>"
            context += "<p><b>app_compatibility: </b>" + rapp.compatibility + "</p>"
            context += "<p><b>app_status: </b>" + rapp.status + "</p>"
        context += "</html>"
        return context

    def get_connection_strength(self):
        if self.msg.is_local_client == True:
            return 'very_strong'
        elif self.msg.state == concert_msgs.ConcertClientState.MISSING:
            return 'missing' 
        else:
            link_quality_percent = (float(self.msg.conn_stats.wireless_link_quality) / 70) * 100
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

    def update(self, msg):
        '''
        Update with new information that gets periodically published by the concert
        conductor. This currently includes the platform info (which can highlight
        the rapp that is running), the connection statistics and the state.
        '''
        # could pull the individual bits, but just easy to drop the new msg in place
        old_msg = self.msg
        self.msg = msg
        if (
                old_msg.is_local_client != msg.is_local_client or
                old_msg.conn_stats.network_type != msg.conn_stats.network_type
           ):
            self.link_type = self._set_link_string()

    ##############################################################################
    # Conveniences
    ##############################################################################

    @property
    def concert_alias(self):
        return self.msg.name

    @concert_alias.setter
    def concert_alias(self, value):
        self.msg.name = value

    @property
    def state(self):
        return self.msg.state

    @property
    def ip(self):
        return self.msg.ip.replace('.', '_')

    @property
    def gateway_name(self):
        return self.msg.gateway_name

    @gateway_name.setter
    def gateway_name(self, value):
        self.msg.gateway_name = value
