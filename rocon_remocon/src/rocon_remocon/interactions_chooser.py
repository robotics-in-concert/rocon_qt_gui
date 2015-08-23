#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
from rocon_console import console
import rocon_interactions.web_interactions as web_interactions
import rospkg
import rospy

from python_qt_binding import loadUi
from python_qt_binding.QtCore import pyqtSlot, Qt, QSize, QEvent
from python_qt_binding.QtGui import QListView, QWidget, QStandardItemModel  # QIcon, QColor, QMainWindow, QMessageBox

from . import utils
from . import icon
from .interactions_remocon import InteractionsRemocon
from .pairing_dialog import PairingDialog
from .interaction_dialog import InteractionDialog

##############################################################################
# Remocon
##############################################################################


class InteractionsChooserUI():

    def __init__(self, rocon_master_uri='localhost', host_name='localhost', with_rqt=False):
        self.rocon_master_uri = rocon_master_uri
        self.host_name = host_name
        self.with_rqt = with_rqt
        self.widget = QWidget()
        self.pairings_view_model = QStandardItemModel()
        self.interactions_view_model = QStandardItemModel()
        self.interactions_remocon = InteractionsRemocon()

        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'interactions_chooser.ui')
        loadUi(ui_file, self.widget, {})

        # create a few directories for caching icons and ...
        utils.setup_home_dirs()

        self.init_ui()
        self.init_events()

    def init_ui(self):
        self.widget.pairings_grid.setViewMode(QListView.IconMode)
        self.widget.pairings_grid.setModel(self.pairings_view_model)
        self.widget.pairings_grid.setWrapping(True)
        self.widget.pairings_grid.setIconSize(QSize(60, 60))
        self.widget.pairings_grid.setSpacing(10)
        self.widget.interactions_grid.setViewMode(QListView.IconMode)
        self.widget.interactions_grid.setModel(self.interactions_view_model)
        self.widget.interactions_grid.setWrapping(True)
        self.widget.interactions_grid.setIconSize(QSize(60, 60))
        self.widget.interactions_grid.setSpacing(10)
        for ns in self.interactions_remocon.namespaces:
            self.widget.namespace_checkbox.addItem(ns)
        self.refresh_grids()
        # TODO namespace checkbox to self.interactions_remocon.active_namespace

    def init_events(self):
        self.widget.namespace_checkbox.currentIndexChanged.connect(self.event_change_namespace)
        self.widget.pairings_grid.clicked.connect(self.pairing_single_click)
        self.widget.interactions_grid.clicked.connect(self.interaction_single_click)
        # self.widget.interactions_grid.doubleClicked.connect(self.interaction_double_click)

    def event_change_namespace(self):
        rospy.logwarn("Remocon : changing interaction managers is currently not supported.")
        # self.interactions_remocon.change_interactions_manager()

    def pairing_single_click(self, index):
        pairing_item = self.pairings_view_model.item(index.row())
        pairing = pairing_item.implementation
        self.create_pairing_dialog(pairing)

    def create_pairing_dialog(self, pairing):
        # is_running = self._qt_rapp_manager_info.is_running_rapp(rapp)
        is_running = False
        self.selected_pairing = pairing
        self.dialog = PairingDialog(self.widget, pairing, self.interactions_remocon.start_pairing, self.interactions_remocon.stop_pairing, is_running)
        self.dialog.show()

    def interaction_single_click(self, index):
        interaction_item = self.interactions_view_model.item(index.row())
        interaction = interaction_item.implementation
        self.create_interaction_dialog(interaction)

    def create_interaction_dialog(self, interaction):
        # is_running = self._qt_rapp_manager_info.is_running_rapp(rapp)
        is_running = False
        self.selected_interaction = interaction
        self.dialog = InteractionDialog(self.widget, interaction, self.interactions_remocon.start_interaction, self.interactions_remocon.stop_interaction, is_running)
        self.dialog.show()

    def refresh_grids(self):
        """
        This just does a complete redraw of the interactions with the
        currently selected role. It's a bit brute force doing this
        every time the interactions' 'state' changes, but this suffices for now.
        """
        console.logdebug("Remocon : refreshing the interactions grid")

#         self.interactions = self.interactive_client_interface.interactions(self.cur_selected_role)
#         self.interactions_widget.interactions_list_widget.clear()
#         index = 0
        self.pairings_view_model.clear()
        self.interactions_view_model.clear()

        for i in self.interactions_remocon.interactions_table.interactions:
            item = icon.QModelIconItem(i, running=False)
            self.interactions_view_model.appendRow(item)

        for p in self.interactions_remocon.pairings_table.pairings:
            item = icon.QModelIconItem(p, running=False)
            self.pairings_view_model.appendRow(item)

            # setting the list font
            # font = self.interactions_widget.interactions_list_widget.item(0).font()
            # font.setPointSize(13)
            # self.interactions_widget.interactions_list_widget.item(0).setFont(font)

#         self.binded_function = {}
#         self.cur_selected_role = ''
#         self.cur_selected_interaction = None
#         self.interactions = {}
#         self.interactive_client_interface = interactive_client_interface
#         self.interactions_widget = QWidget()
#
#         # interactions list widget
#         self.interactions_widget.interactions_list_widget.setIconSize(QSize(50, 50))
#         self.interactions_widget.interactions_list_widget.itemDoubleClicked.connect(self._start_interaction)
#         self.interactions_widget.back_btn.pressed.connect(self._back)
#         self.interactions_widget.interactions_list_widget.itemClicked.connect(self._display_interaction_info)  # rocon master item click event
#         self.interactions_widget.stop_interactions_button.pressed.connect(self._stop_interaction)
#         self.interactions_widget.stop_interactions_button.setDisabled(True)
#         self.interactions_widget.closeEvent = self._close_event
#
#         console.logdebug('init QInteractionsChooser')
#
#     def _back(self):
#         if 'back' in self.binded_function.keys() and self.binded_function['back'] is not None:
#             self.binded_function['back']()
#         else:
#             console.logdebug("Interactions Chooser : None binded functione: shutdown")
#
#     def _close_event(self, event):
#         """
#         Re-implementation of close event handlers for the interaction chooser's children
#         (i.e. role and interactions list widgets).
#         """
#         console.logdebug("Interactions Chooser : remocon shutting down.")
#         if 'shutdown' in self.binded_function.keys() and self.binded_function['shutdown'] is not None:
#             self.binded_function['shutdown']()
#         else:
#             console.logdebug("Interactions Chooser : None binded functione: shutdown")
#
#     ######################################
#     # Interactions List Widget
#     ######################################
#     def _display_interaction_info(self, Item):
#         """
#         Display the currently selected interaction's information. Triggered
#         when single-clicking on it in the interactions list view.
#         """
#         list_widget = Item.listWidget()
#         cur_index = list_widget.count() - list_widget.currentRow() - 1
#         for k in self.interactions.values():
#             if(k.index == cur_index):
#                 self.cur_selected_interaction = k
#                 break
#         self.interactions_widget.app_info.clear()
#         info_text = "<html>"
#         info_text += "<p>-------------------------------------------</p>"
#         web_interaction = web_interactions.parse(self.cur_selected_interaction.name)
#         name = self.cur_selected_interaction.name if web_interaction is None else web_interaction.url
#         info_text += "<p><b>name: </b>" + name + "</p>"
#         info_text += "<p><b>  ---------------------</b>" + "</p>"
#         info_text += "<p><b>compatibility: </b>" + self.cur_selected_interaction.compatibility + "</p>"
#         info_text += "<p><b>display name: </b>" + self.cur_selected_interaction.display_name + "</p>"
#         info_text += "<p><b>description: </b>" + self.cur_selected_interaction.description + "</p>"
#         info_text += "<p><b>namespace: </b>" + self.cur_selected_interaction.namespace + "</p>"
#         info_text += "<p><b>max: </b>" + str(self.cur_selected_interaction.max) + "</p>"
#         info_text += "<p><b>  ---------------------</b>" + "</p>"
#         info_text += "<p><b>remappings: </b>" + str(self.cur_selected_interaction.remappings) + "</p>"
#         info_text += "<p><b>parameters: </b>" + str(self.cur_selected_interaction.parameters) + "</p>"
#         info_text += "</html>"
# 
#         self.interactions_widget.app_info.appendHtml(info_text)
#         self._set_stop_interactions_button()
# 
#     ######################################
#     # Gui Updates/Refreshes
#     ######################################
# 
#     def _set_stop_interactions_button(self):
#         """
#           Disable or enable the stop button depending on whether the
#           selected interaction has any currently launched processes,
#         """
#         if not self.interactions:
#             console.logwarn("No interactions")
#             return
#         if self.cur_selected_interaction.launch_list:
#             console.logdebug("Interactions Chooser : enabling stop interactions button [%s]" % self.cur_selected_interaction.display_name)
#             self.interactions_widget.stop_interactions_button.setEnabled(True)
#         else:
#             console.logdebug("Interactions Chooser : disabling stop interactions button [%s]" % self.cur_selected_interaction.display_name)
#             self.interactions_widget.stop_interactions_button.setEnabled(False)
# 
#     ######################################
#     # Start/Stop Interactions
#     ######################################
# 
#     def _start_interaction(self):
#         """
#         Start selected interactions when user hits start button and does doubleclicking interaction item.
#         The interactions can be launched in duplicate.
#         """
#         console.logdebug("Interactions Chooser : starting interaction [%s]" % str(self.cur_selected_interaction.name))
#         (result, message) = self.interactive_client_interface.start_interaction(self.cur_selected_role,
#                                                                                 self.cur_selected_interaction.hash)
#         if result:
#             if self.cur_selected_interaction.is_paired_type():
#                 self.refresh_interactions_list()  # make sure the highlight is working
#             self.interactions_widget.stop_interactions_button.setDisabled(False)
#         else:
#             QMessageBox.warning(self.interactions_widget, 'Start Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
#             console.logwarn("Interactions Chooser : start interaction failed [%s]" % message)
# 
#     def _stop_interaction(self):
#         """
#         Stop running interactions when user hits `stop` or 'all stop interactions button` button.
#         If no interactions is running, buttons are disabled.
#         """
#         console.logdebug("Interactions Chooser : stopping interaction %s " % str(self.cur_selected_interaction.name))
#         (result, message) = self.interactive_client_interface.stop_interaction(self.cur_selected_interaction.hash)
#         if result:
#             if self.cur_selected_interaction.is_paired_type():
#                 self.refresh_interactions_list()  # make sure the highlight is disabled
#             self._set_stop_interactions_button()
#             # self.interactions_widget.stop_interactions_button.setDisabled(True)
#         else:
#             QMessageBox.warning(self.interactions_widget, 'Stop Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
#             console.logwarn("Interactions Chooser : stop interaction failed [%s]" % message)
# 
#     def bind_function(self, name, function_handle):
#         """
#         Binding external function to map with ui button
#         """
#         self.binded_function[name] = function_handle
# 
#     def show(self, pos=None):
#         """
#         Showing the interactions chooser
#         """
#         self.interactions_widget.show()
#         if pos is not None:
#             self.interactions_widget.move(pos)
# 
#         if self.interactive_client_interface.has_running_interactions():
#             self.interactions_widget.stop_interactions_button.setEnabled(True)
#         else:
#             self.interactions_widget.stop_interactions_button.setEnabled(False)
# 
#     def hide(self):
#         """
#         Hiding the interactions chooser
#         """
#         self.interactions_widget.hide()
# 
#     def is_hidden(self):
#         return self.interactions_widget.is_hidden()
# 
#     def select_role(self, role):
#         """
#         Take the selected role to get a list of interaction.
# 
#         :param role: role name from role chooser.
#         :type role: string
#         """
#         self.cur_selected_role = role
#         self.refresh_interactions_list()
# 
#     def refresh_interactions_list(self):
#         """
#         This just does a complete redraw of the interactions with the
#         currently selected role. It's a bit brute force doing this
#         every time the interactions' 'state' changes, but this suffices for now.
#         """
#         console.logdebug("Interactions Chooser : refreshing the interactions list")
#         self.interactions = self.interactive_client_interface.interactions(self.cur_selected_role)
#         self.interactions_widget.interactions_list_widget.clear()
#         index = 0
#         for interaction in self.interactions.values():
#             interaction.index = index
#             index = index + 1
# 
#             self.interactions_widget.interactions_list_widget.insertItem(0, interaction.display_name)
# 
#             # is it a currently running pairing
#             for interaction_hash in self.interactive_client_interface.currently_pairing_interaction_hashes:
#                 if interaction_hash == interaction.hash:
#                     self.interactions_widget.interactions_list_widget.item(0).setBackground(QColor(100, 100, 150))
#                     break
# 
#             # setting the list font
#             font = self.interactions_widget.interactions_list_widget.item(0).font()
#             font.setPointSize(13)
#             self.interactions_widget.interactions_list_widget.item(0).setFont(font)
# 
#             # setting the icon
#             icon = interaction.icon
#             if icon == "unknown.png":
#                 icon = QIcon(self.icon_paths['unknown'])
#                 self.interactions_widget.interactions_list_widget.item(0).setIcon(icon)
#             elif len(icon):
#                 icon = QIcon(os.path.join(utils.get_icon_cache_home(), icon))
#                 self.interactions_widget.interactions_list_widget.item(0).setIcon(icon)
#             else:
#                 console.logdebug("%s : No icon" % str(self.rocon_master_name))

##############################################################################
# Graveyard
##############################################################################

# class QInteractionsChooser(QMainWindow):
# 
#     def __init__(self, interactive_client_interface=None):
#         self.binded_function = {}
#         self.cur_selected_role = ''
#         self.cur_selected_interaction = None
#         self.interactions = {}
#         self.interactive_client_interface = interactive_client_interface
#         self.interactions_widget = QWidget()
#         # load ui
#         rospack = rospkg.RosPack()
#         path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'interactions_list.ui')
#         loadUi(path, self.interactions_widget)
# 
#         # interactions list widget
#         self.interactions_widget.interactions_list_widget.setIconSize(QSize(50, 50))
#         self.interactions_widget.interactions_list_widget.itemDoubleClicked.connect(self._start_interaction)
#         self.interactions_widget.back_btn.pressed.connect(self._back)
#         self.interactions_widget.interactions_list_widget.itemClicked.connect(self._display_interaction_info)  # rocon master item click event
#         self.interactions_widget.stop_interactions_button.pressed.connect(self._stop_interaction)
#         self.interactions_widget.stop_interactions_button.setDisabled(True)
#         self.interactions_widget.closeEvent = self._close_event
# 
#         console.logdebug('init QInteractionsChooser')
# 
#     def _back(self):
#         if 'back' in self.binded_function.keys() and self.binded_function['back'] is not None:
#             self.binded_function['back']()
#         else:
#             console.logdebug("Interactions Chooser : None binded functione: shutdown")
# 
#     def _close_event(self, event):
#         """
#         Re-implementation of close event handlers for the interaction chooser's children
#         (i.e. role and interactions list widgets).
#         """
#         console.logdebug("Interactions Chooser : remocon shutting down.")
#         if 'shutdown' in self.binded_function.keys() and self.binded_function['shutdown'] is not None:
#             self.binded_function['shutdown']()
#         else:
#             console.logdebug("Interactions Chooser : None binded functione: shutdown")
# 
#     ######################################
#     # Interactions List Widget
#     ######################################
#     def _display_interaction_info(self, Item):
#         """
#         Display the currently selected interaction's information. Triggered
#         when single-clicking on it in the interactions list view.
#         """
#         list_widget = Item.listWidget()
#         cur_index = list_widget.count() - list_widget.currentRow() - 1
#         for k in self.interactions.values():
#             if(k.index == cur_index):
#                 self.cur_selected_interaction = k
#                 break
#         self.interactions_widget.app_info.clear()
#         info_text = "<html>"
#         info_text += "<p>-------------------------------------------</p>"
#         web_interaction = web_interactions.parse(self.cur_selected_interaction.name)
#         name = self.cur_selected_interaction.name if web_interaction is None else web_interaction.url
#         info_text += "<p><b>name: </b>" + name + "</p>"
#         info_text += "<p><b>  ---------------------</b>" + "</p>"
#         info_text += "<p><b>compatibility: </b>" + self.cur_selected_interaction.compatibility + "</p>"
#         info_text += "<p><b>display name: </b>" + self.cur_selected_interaction.display_name + "</p>"
#         info_text += "<p><b>description: </b>" + self.cur_selected_interaction.description + "</p>"
#         info_text += "<p><b>namespace: </b>" + self.cur_selected_interaction.namespace + "</p>"
#         info_text += "<p><b>max: </b>" + str(self.cur_selected_interaction.max) + "</p>"
#         info_text += "<p><b>  ---------------------</b>" + "</p>"
#         info_text += "<p><b>remappings: </b>" + str(self.cur_selected_interaction.remappings) + "</p>"
#         info_text += "<p><b>parameters: </b>" + str(self.cur_selected_interaction.parameters) + "</p>"
#         info_text += "</html>"
# 
#         self.interactions_widget.app_info.appendHtml(info_text)
#         self._set_stop_interactions_button()
# 
#     ######################################
#     # Gui Updates/Refreshes
#     ######################################
# 
#     def _set_stop_interactions_button(self):
#         """
#           Disable or enable the stop button depending on whether the
#           selected interaction has any currently launched processes,
#         """
#         if not self.interactions:
#             console.logwarn("No interactions")
#             return
#         if self.cur_selected_interaction.launch_list:
#             console.logdebug("Interactions Chooser : enabling stop interactions button [%s]" % self.cur_selected_interaction.display_name)
#             self.interactions_widget.stop_interactions_button.setEnabled(True)
#         else:
#             console.logdebug("Interactions Chooser : disabling stop interactions button [%s]" % self.cur_selected_interaction.display_name)
#             self.interactions_widget.stop_interactions_button.setEnabled(False)
# 
#     ######################################
#     # Start/Stop Interactions
#     ######################################
# 
#     def _start_interaction(self):
#         """
#         Start selected interactions when user hits start button and does doubleclicking interaction item.
#         The interactions can be launched in duplicate.
#         """
#         console.logdebug("Interactions Chooser : starting interaction [%s]" % str(self.cur_selected_interaction.name))
#         (result, message) = self.interactive_client_interface.start_interaction(self.cur_selected_role,
#                                                                                 self.cur_selected_interaction.hash)
#         if result:
#             if self.cur_selected_interaction.is_paired_type():
#                 self.refresh_interactions_list()  # make sure the highlight is working
#             self.interactions_widget.stop_interactions_button.setDisabled(False)
#         else:
#             QMessageBox.warning(self.interactions_widget, 'Start Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
#             console.logwarn("Interactions Chooser : start interaction failed [%s]" % message)
# 
#     def _stop_interaction(self):
#         """
#         Stop running interactions when user hits `stop` or 'all stop interactions button` button.
#         If no interactions is running, buttons are disabled.
#         """
#         console.logdebug("Interactions Chooser : stopping interaction %s " % str(self.cur_selected_interaction.name))
#         (result, message) = self.interactive_client_interface.stop_interaction(self.cur_selected_interaction.hash)
#         if result:
#             if self.cur_selected_interaction.is_paired_type():
#                 self.refresh_interactions_list()  # make sure the highlight is disabled
#             self._set_stop_interactions_button()
#             # self.interactions_widget.stop_interactions_button.setDisabled(True)
#         else:
#             QMessageBox.warning(self.interactions_widget, 'Stop Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
#             console.logwarn("Interactions Chooser : stop interaction failed [%s]" % message)
# 
#     def bind_function(self, name, function_handle):
#         """
#         Binding external function to map with ui button
#         """
#         self.binded_function[name] = function_handle
# 
#     def show(self, pos=None):
#         """
#         Showing the interactions chooser
#         """
#         self.interactions_widget.show()
#         if pos is not None:
#             self.interactions_widget.move(pos)
# 
#         if self.interactive_client_interface.has_running_interactions():
#             self.interactions_widget.stop_interactions_button.setEnabled(True)
#         else:
#             self.interactions_widget.stop_interactions_button.setEnabled(False)
# 
#     def hide(self):
#         """
#         Hiding the interactions chooser
#         """
#         self.interactions_widget.hide()
# 
#     def is_hidden(self):
#         return self.interactions_widget.is_hidden()
# 
#     def select_role(self, role):
#         """
#         Take the selected role to get a list of interaction.
# 
#         :param role: role name from role chooser.
#         :type role: string
#         """
#         self.cur_selected_role = role
#         self.refresh_interactions_list()
# 
#     def refresh_interactions_list(self):
#         """
#         This just does a complete redraw of the interactions with the
#         currently selected role. It's a bit brute force doing this
#         every time the interactions' 'state' changes, but this suffices for now.
#         """
#         console.logdebug("Interactions Chooser : refreshing the interactions list")
#         self.interactions = self.interactive_client_interface.interactions(self.cur_selected_role)
#         self.interactions_widget.interactions_list_widget.clear()
#         index = 0
#         for interaction in self.interactions.values():
#             interaction.index = index
#             index = index + 1
# 
#             self.interactions_widget.interactions_list_widget.insertItem(0, interaction.display_name)
# 
#             # is it a currently running pairing
#             for interaction_hash in self.interactive_client_interface.currently_pairing_interaction_hashes:
#                 if interaction_hash == interaction.hash:
#                     self.interactions_widget.interactions_list_widget.item(0).setBackground(QColor(100, 100, 150))
#                     break
# 
#             # setting the list font
#             font = self.interactions_widget.interactions_list_widget.item(0).font()
#             font.setPointSize(13)
#             self.interactions_widget.interactions_list_widget.item(0).setFont(font)
# 
#             # setting the icon
#             icon = interaction.icon
#             if icon == "unknown.png":
#                 icon = QIcon(self.icon_paths['unknown'])
#                 self.interactions_widget.interactions_list_widget.item(0).setIcon(icon)
#             elif len(icon):
#                 icon = QIcon(os.path.join(utils.get_icon_cache_home(), icon))
#                 self.interactions_widget.interactions_list_widget.item(0).setIcon(icon)
#             else:
#                 console.logdebug("%s : No icon" % str(self.rocon_master_name))
