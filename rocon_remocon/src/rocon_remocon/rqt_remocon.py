#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
# system
from __future__ import division
import os

# rqt
from qt_gui.plugin import Plugin
import rocon_qt_library.utils as utils

# rocon
from rocon_console import console
from rocon_remocon.interactive_client_ui import InteractiveClientUI

##############################################################################
# Rqt Remocon
##############################################################################


class RqtRemocon(Plugin):

    def __init__(self, context):
        self._context = context
        super(RqtRemocon, self).__init__(context)
        # Process standalone plugin command-line arguments
        self.rocon_master_uri = 'localhost'
        self.host_name = 'localhost'

        try:
            self.rocon_master_uri = os.environ["ROS_MASTER_URI"]
            self.host_name = os.environ["ROS_HOSTNAME"]
        except KeyError as e:
            console.logerror("Rqt Remocon: %s " % str(e))

        self.setObjectName('Rqt Remocon')
        self._rqt_remocon = InteractiveClientUI(None, "Rqt remocon", None, self.rocon_master_uri, self.host_name, True)
        context.add_widget(self._rqt_remocon.get_main_ui_handle())

    def shutdown_plugin(self):
        self._rqt_remocon.shutdown()
