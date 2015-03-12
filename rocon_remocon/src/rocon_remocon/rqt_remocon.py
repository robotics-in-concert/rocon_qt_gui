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
# pyqt
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal, SIGNAL, SLOT, QSize, QEvent
from python_qt_binding.QtGui import QWidget, QIcon, QColor, QMessageBox

# ros
import rospkg

# rocon
from rocon_console import console
# rqt
from qt_gui.plugin import Plugin
import rocon_qt_library.utils as utils

from rocon_python_comms.exceptions import NotFoundException

from rocon_remocon.interactive_client_ui import InteractiveClientUI
import rocon_interactions.web_interactions as web_interactions
from . import utils

##############################################################################
# Rocon Qt Remocon
##############################################################################


class RqtRemocon(Plugin):

    # pyqt signals are always defined as class attributes
    signal_interactions_updated = Signal()

    def __init__(self, context):
        self._context = context
        super(RqtRemocon, self).__init__(context)
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument('--ip', dest='ip', help='ros master ip', default='localhost')
        parser.add_argument('--host', dest='host', help='ros host address', default='localhost')
        parser.add_argument('--index', dest='index', help='ros master index', default=0)
        parser.add_argument('--name', dest='name', help='ros host address', default='rocon concert')
        args, unknowns = parser.parse_known_args(context.argv())
        
        self.rocon_master_index = args.index
        self.rocon_master_uri = args.ip
        self.rocon_master_name = args.name
        self.host_name = args.host

        rqt_remocon = InteractiveClientUI(None, "Rqt remocon", None, self.rocon_master_uri, self.host_name, False)
        context.add_widget(rqt_remocon.get_main_ui_handle())

        