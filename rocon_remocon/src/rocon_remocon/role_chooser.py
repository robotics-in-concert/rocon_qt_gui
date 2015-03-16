#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rospkg
#from PyQt4 import uic
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, QSize
from python_qt_binding.QtGui import QWidget

from rocon_console import console
from rocon_remocon.interactive_client_interface import InteractiveClientInterface
from . import utils

##############################################################################
# Remocon
##############################################################################


class QRoleChooser():
    # pyqt signals are always defined as class attributes
    # signal_interactions_updated = Signal()

    def __init__(self, interactive_client_interface=None, with_rqt=False):

        self.interactive_client_interface = interactive_client_interface
        self.with_rqt = with_rqt
        self.binded_function = {}
        self.cur_selected_role = ''
        self.roles_widget = QWidget()
        # load ui
        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'role_list.ui')
        loadUi(path, self.roles_widget)

        # connect ui event role list widget
        self.roles_widget.role_list_widget.setIconSize(QSize(50, 50))
        self.roles_widget.role_list_widget.itemDoubleClicked.connect(self._select_role)
        self.roles_widget.refresh_btn.pressed.connect(self.refresh_role_list)
        self.roles_widget.back_btn.pressed.connect(self._back)
        self.roles_widget.stop_all_interactions_button.pressed.connect(self._stop_all_interactions)
        self.roles_widget.closeEvent = self._close_event

        self._init()
        console.logdebug('init QRoleChooser')

    def _init(self):
        """
        todo
        """
        if self.with_rqt:
            self.roles_widget.back_btn.setEnabled(False)
        role_list = self.refresh_role_list()
        if len(role_list) == 1:
            self.cur_selected_role = role_list[0]
            self.interactive_client_interface.select_role(self.cur_selected_role)

    def _back(self):
        """
        Public method to enable shutdown of the script - this function is primarily for
        shutting down the Role chooser from external signals (e.g. CTRL-C on the command
        line).
        """
        console.logdebug("Role Chooser : Back")
        if 'back' in self.binded_function.keys() and self.binded_function['back'] is not None:
            self.binded_function['back']()

    def _close_event(self, event):
        """
        Re-implementation of close event handlers for the interaction chooser's children
        (i.e. role and interactions list widgets).
        """
        console.logdebug("Role Chooser : Role Chooser shutting down.")
        self._back()

    def _select_role(self, Item):
        """
        Todo
        Take the selected role and switch to an interactions view of that role.
        """
        console.logdebug("Role Chooser : switching to the interactions list")
        self.cur_selected_role = str(Item.text())
        if 'select_role' in self.binded_function.keys() and self.binded_function['select_role'] is not None:
            self.binded_function['select_role']()

    def _stop_all_interactions(self):
        """
        Todo
        """
        console.logdebug("Role Chooser : stopping all running interactions")
        self.interactive_client_interface.stop_all_interactions()
        self.roles_widget.stop_all_interactions_button.setEnabled(False)

    def bind_function(self, name, function_handle):
        """
        Todo
        """
        self.binded_function[name] = function_handle

    def show(self, pos=None):
        """
        Todo
        """
        self.roles_widget.show()
        if pos is not None:
            self.roles_widget.move(pos)
        self.refresh_role_list()

    def hide(self):
        """
        Todo
        """
        self.roles_widget.hide()

    def pos(self):
        """
        todo
        """
        return self.roles_widget.pos()

    def close(self):
        """
        todo
        """
        self.roles_widget.close()

    def refresh_role_list(self):
        """
        Todo
        """
        if self.interactive_client_interface.has_running_interactions():
            self.roles_widget.stop_all_interactions_button.setEnabled(True)
        else:
            self.roles_widget.stop_all_interactions_button.setEnabled(False)

        self.roles_widget.role_list_widget.clear()
        role_list = self.interactive_client_interface.get_role_list()
        # set list widget item (reverse order because we push them on the top)
        for role in reversed(role_list):
            self.roles_widget.role_list_widget.insertItem(0, role)
            # setting the list font
            font = self.roles_widget.role_list_widget.item(0).font()
            font.setPointSize(13)
            self.roles_widget.role_list_widget.item(0).setFont(font)

        return role_list
