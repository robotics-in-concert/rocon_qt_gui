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
from python_qt_binding.QtCore import Signal, Qt, QSize, QEvent
from python_qt_binding.QtGui import QIcon, QWidget, QColor, QMainWindow, QMessageBox

from rocon_console import console
import rocon_interactions.web_interactions as web_interactions

from rocon_remocon.interactive_client_interface import InteractiveClientInterface
from . import utils
##############################################################################
# Remocon
##############################################################################


class QInteractionsChooser(QMainWindow):

    def __init__(self, interactive_client_interface=None):
        self.binded_function = {}
        self.cur_selected_role = ''
        self.cur_selected_interaction = None
        self.interactions = {}
        self.interactive_client_interface = interactive_client_interface
        self.interactions_widget = QWidget()
        # load ui
        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'interactions_list.ui')
        loadUi(path, self.interactions_widget)

        # interactions list widget
        self.interactions_widget.interactions_list_widget.setIconSize(QSize(50, 50))
        self.interactions_widget.interactions_list_widget.itemDoubleClicked.connect(self._start_interaction)
        self.interactions_widget.back_btn.pressed.connect(self._back)
        self.interactions_widget.interactions_list_widget.itemClicked.connect(self._display_interaction_info)  # rocon master item click event
        self.interactions_widget.stop_interactions_button.pressed.connect(self._stop_interaction)
        self.interactions_widget.stop_interactions_button.setDisabled(True)
        self.interactions_widget.closeEvent = self._close_event

        console.logdebug('init QInteractionsChooser')

    def _back(self):
        if 'back' in self.binded_function.keys() and self.binded_function['back'] is not None:
            self.binded_function['back']()
        else:
            console.logdebug("Interactions Chooser : None binded functione: shutdown")

    def _close_event(self, event):
        """
        Re-implementation of close event handlers for the interaction chooser's children
        (i.e. role and interactions list widgets).
        """
        console.logdebug("Interactions Chooser : remocon shutting down.")
        if 'shutdown' in self.binded_function.keys() and self.binded_function['shutdown'] is not None:
            self.binded_function['shutdown']()
        else:
            console.logdebug("Interactions Chooser : None binded functione: shutdown")

    ######################################
    # Interactions List Widget
    ######################################
    def _display_interaction_info(self, Item):
        """
        Display the currently selected interaction's information. Triggered
        when single-clicking on it in the interactions list view.
        """
        list_widget = Item.listWidget()
        cur_index = list_widget.count() - list_widget.currentRow() - 1
        for k in self.interactions.values():
            if(k.index == cur_index):
                self.cur_selected_interaction = k
                break
        self.interactions_widget.app_info.clear()
        info_text = "<html>"
        info_text += "<p>-------------------------------------------</p>"
        web_interaction = web_interactions.parse(self.cur_selected_interaction.name)
        name = self.cur_selected_interaction.name if web_interaction is None else web_interaction.url
        info_text += "<p><b>name: </b>" + name + "</p>"
        info_text += "<p><b>  ---------------------</b>" + "</p>"
        info_text += "<p><b>compatibility: </b>" + self.cur_selected_interaction.compatibility + "</p>"
        info_text += "<p><b>display name: </b>" + self.cur_selected_interaction.display_name + "</p>"
        info_text += "<p><b>description: </b>" + self.cur_selected_interaction.description + "</p>"
        info_text += "<p><b>namespace: </b>" + self.cur_selected_interaction.namespace + "</p>"
        info_text += "<p><b>max: </b>" + str(self.cur_selected_interaction.max) + "</p>"
        info_text += "<p><b>  ---------------------</b>" + "</p>"
        info_text += "<p><b>remappings: </b>" + str(self.cur_selected_interaction.remappings) + "</p>"
        info_text += "<p><b>parameters: </b>" + str(self.cur_selected_interaction.parameters) + "</p>"
        info_text += "</html>"

        self.interactions_widget.app_info.appendHtml(info_text)
        self._set_stop_interactions_button()

    ######################################
    # Gui Updates/Refreshes
    ######################################

    def _set_stop_interactions_button(self):
        """
          Disable or enable the stop button depending on whether the
          selected interaction has any currently launched processes,
        """
        if not self.interactions:
            console.logwarn("No interactions")
            return
        if self.cur_selected_interaction.launch_list:
            console.logdebug("Interactions Chooser : enabling stop interactions button [%s]" % self.cur_selected_interaction.display_name)
            self.interactions_widget.stop_interactions_button.setEnabled(True)
        else:
            console.logdebug("Interactions Chooser : disabling stop interactions button [%s]" % self.cur_selected_interaction.display_name)
            self.interactions_widget.stop_interactions_button.setEnabled(False)

    ######################################
    # Start/Stop Interactions
    ######################################

    def _start_interaction(self):
        """
        Start selected interactions when user hits start button and does doubleclicking interaction item.
        The interactions can be launched in duplicate.
        """
        console.logdebug("Interactions Chooser : starting interaction [%s]" % str(self.cur_selected_interaction.name))
        (result, message) = self.interactive_client_interface.start_interaction(self.cur_selected_role,
                                                                                self.cur_selected_interaction.hash)
        if result:
            if self.cur_selected_interaction.is_paired_type():
                self.refresh_interactions_list()  # make sure the highlight is working
            self.interactions_widget.stop_interactions_button.setDisabled(False)
        else:
            QMessageBox.warning(self.interactions_widget, 'Start Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
            console.logwarn("Interactions Chooser : start interaction failed [%s]" % message)

    def _stop_interaction(self):
        """
        Stop running interactions when user hits `stop` or 'all stop interactions button` button.
        If no interactions is running, buttons are disabled.
        """
        console.logdebug("Interactions Chooser : stopping interaction %s " % str(self.cur_selected_interaction.name))
        (result, message) = self.interactive_client_interface.stop_interaction(self.cur_selected_interaction.hash)
        if result:
            if self.cur_selected_interaction.is_paired_type():
                self.refresh_interactions_list()  # make sure the highlight is disabled
            self._set_stop_interactions_button()
            # self.interactions_widget.stop_interactions_button.setDisabled(True)
        else:
            QMessageBox.warning(self.interactions_widget, 'Stop Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
            console.logwarn("Interactions Chooser : stop interaction failed [%s]" % message)

    def bind_function(self, name, function_handle):
        """
        Binding external function to map with ui button
        """
        self.binded_function[name] = function_handle

    def show(self, pos=None):
        """
        Showing the interactions chooser
        """
        self.interactions_widget.show()
        if pos is not None:
            self.interactions_widget.move(pos)

        if self.interactive_client_interface.has_running_interactions():
            self.interactions_widget.stop_interactions_button.setEnabled(True)
        else:
            self.interactions_widget.stop_interactions_button.setEnabled(False)

    def hide(self):
        """
        Hiding the interactions chooser
        """
        self.interactions_widget.hide()

    def select_role(self, role):
        """
        Take the selected role to get a list of interaction.

        :param role: role name from role chooser.
        :type role: string
        """
        self.cur_selected_role = role
        self.interactive_client_interface.select_role(self.cur_selected_role)
        self.refresh_interactions_list()

    def refresh_interactions_list(self):
        """
        This just does a complete redraw of the interactions with the
        currently selected role. It's a bit brute force doing this
        every time the interactions' 'state' changes, but this suffices for now.
        """
        console.logdebug("Interactions Chooser : refreshing the interactions list")
        self.interactions = self.interactive_client_interface.interactions(self.cur_selected_role)
        self.interactions_widget.interactions_list_widget.clear()
        index = 0
        for interaction in self.interactions.values():
            interaction.index = index
            index = index + 1

            self.interactions_widget.interactions_list_widget.insertItem(0, interaction.display_name)

            # is it a currently running pairing
            if self.interactive_client_interface.pairing == interaction.hash:
                self.interactions_widget.interactions_list_widget.item(0).setBackground(QColor(100, 100, 150))

            # setting the list font
            font = self.interactions_widget.interactions_list_widget.item(0).font()
            font.setPointSize(13)
            self.interactions_widget.interactions_list_widget.item(0).setFont(font)

            # setting the icon
            icon = interaction.icon
            if icon == "unknown.png":
                icon = QIcon(self.icon_paths['unknown'])
                self.interactions_widget.interactions_list_widget.item(0).setIcon(icon)
            elif len(icon):
                icon = QIcon(os.path.join(utils.get_icon_cache_home(), icon))
                self.interactions_widget.interactions_list_widget.item(0).setIcon(icon)
            else:
                console.logdebug("%s : No icon" % str(self.rocon_master_name))
