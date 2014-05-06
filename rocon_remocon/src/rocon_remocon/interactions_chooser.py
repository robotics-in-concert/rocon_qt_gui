#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os

#from PyQt4 import uic
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtCore import Qt, QSize
from python_qt_binding.QtGui import QIcon, QWidget
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtGui import QMainWindow

from python_qt_binding.QtGui import QMessageBox
#from PyQt4.QtSvg import QSvgGenerator

from rocon_console import console
import rocon_interactions.web_interactions as web_interactions

from rocon_remocon.interactive_client import InteractiveClient
from . import utils
from .master_chooser import QMasterChooser

##############################################################################
# Remocon
##############################################################################


class QInteractionsChooser(QMainWindow):

    # pyqt signals are always defined as class attributes
    signal_interactions_updated = Signal()

    def __init__(self, parent, title, application, rocon_master_index="", rocon_master_name="", rocon_master_uri='localhost', host_name='localhost'):
        self.rocon_master_index = rocon_master_index
        self.rocon_master_uri = rocon_master_uri
        self.rocon_master_name = rocon_master_name
        self.host_name = host_name
        self._context = parent
        self.application = application

        super(QInteractionsChooser, self).__init__(parent)
        self.initialised = False

        self.interactions_widget = QWidget()
        self.roles_widget = QWidget()

        self.cur_selected_role = 0

        self.interactions = {}
        self.cur_selected_interaction = None

        self.interactive_client = InteractiveClient(stop_interaction_postexec_fn=self.interactions_updated_relay)

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../ui/interactions_list.ui")
        loadUi(path, self.interactions_widget)

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../ui/role_list.ui")
        loadUi(path, self.roles_widget)

        utils.setup_home_dirs()

        # role list widget
        self.roles_widget.role_list_widget.setIconSize(QSize(50, 50))
        self.roles_widget.role_list_widget.itemDoubleClicked.connect(self._select_role_list)
        self.roles_widget.back_btn.pressed.connect(self._back_role_list)
        self.roles_widget.refresh_btn.pressed.connect(self._refresh_role_list)
        # interactions list widget
        self.interactions_widget.interactions_list_widget.setIconSize(QSize(50, 50))
        self.interactions_widget.interactions_list_widget.itemDoubleClicked.connect(self._start_interaction)
        self.interactions_widget.back_btn.pressed.connect(self._uninit_interactions_list)
        self.interactions_widget.interactions_list_widget.itemClicked.connect(self._select_app_list)  # rocon master item click event
        self.interactions_widget.stop_interactions_button.pressed.connect(self._stop_interaction)
        self.interactions_widget.stop_interactions_button.setDisabled(True)

        # signals
        self.signal_interactions_updated.connect(self._refresh_interactions_list, Qt.QueuedConnection)
        self.signal_interactions_updated.connect(self._set_stop_interactions_button, Qt.QueuedConnection)

        #init
        self._init()

    def _init(self):
        (result, message) = self._init_role_list()
        if not result:
            QMessageBox.warning(self, 'Connection Failed', "%s." % message.capitalize(), QMessageBox.Ok)
            self._back_role_list()
            return
        # Ugly Hack : our window manager is not graying out the button when an interaction closes itself down and the appropriate
        # callback (_set_stop_interactions_button) is fired. It does otherwise though so it looks like the window manager
        # is getting confused when the original program doesn't have the focus.
        #
        # Taking control of it ourselves works...
        self.interactions_widget.stop_interactions_button.setStyleSheet("QPushButton:disabled { color: gray }")
        self.roles_widget.show()
        self.initialised = True

    ######################################
    # Roles List Widget
    ######################################

    def _init_role_list(self):
        (result, message) = self.interactive_client._connect(self.rocon_master_name, self.rocon_master_uri, self.host_name)
        if not result:
            return (False, message)
        self._refresh_role_list()
        return (True, "success")

    def _uninit_role_list(self):

        self.interactive_client.shutdown()
        self.cur_selected_role = 0

    def _select_role_list(self, Item):

        self.cur_selected_role = str(Item.text())

        self.interactive_client._select_role(self.cur_selected_role)

        self.interactions_widget.show()
        self.interactions_widget.move(self.roles_widget.pos())
        self.roles_widget.hide()
        self._init_interactions_list()

    def _back_role_list(self):
        self._uninit_role_list()
        self.initialised = False
        os.execv(QMasterChooser.rocon_remocon_script, ['', self.host_name])

    def _refresh_role_list(self):
        self.roles_widget.role_list_widget.clear()

        role_list = self.interactive_client.get_role_list()

        #set list widget item (reverse order because we push them on the top)
        for role in reversed(role_list):
            self.roles_widget.role_list_widget.insertItem(0, role)
            #setting the list font
            font = self.roles_widget.role_list_widget.item(0).font()
            font.setPointSize(13)
            self.roles_widget.role_list_widget.item(0).setFont(font)

    ######################################
    # Interactions List Widget
    ######################################

    def _init_interactions_list(self):
        self._refresh_interactions_list()

    def _uninit_interactions_list(self):
        self.roles_widget.show()
        self.roles_widget.move(self.interactions_widget.pos())
        self.interactions_widget.hide()

    def _select_app_list(self, Item):
        list_widget = Item.listWidget()
        cur_index = list_widget.count() - list_widget.currentRow() - 1
        for k in self.interactions.values():
            if(k['index'] == cur_index):
                self.cur_selected_interaction = k
                break
        self.interactions_widget.app_info.clear()
        info_text = "<html>"
        info_text += "<p>-------------------------------------------</p>"
        web_interaction = web_interactions.parse(self.cur_selected_interaction['name'])
        name = self.cur_selected_interaction['name'] if web_interaction is None else web_interaction.url
        info_text += "<p><b>name: </b>" + name + "</p>"
        info_text += "<p><b>  ---------------------</b>" + "</p>"
        info_text += "<p><b>compatibility: </b>" + self.cur_selected_interaction['compatibility'] + "</p>"
        info_text += "<p><b>display name: </b>" + self.cur_selected_interaction['display_name'] + "</p>"
        info_text += "<p><b>description: </b>" + self.cur_selected_interaction['description'] + "</p>"
        info_text += "<p><b>namespace: </b>" + self.cur_selected_interaction['namespace'] + "</p>"
        info_text += "<p><b>max: </b>" + str(self.cur_selected_interaction['max']) + "</p>"
        info_text += "<p><b>  ---------------------</b>" + "</p>"
        info_text += "<p><b>remappings: </b>" + str(self.cur_selected_interaction['remappings']) + "</p>"
        info_text += "<p><b>parameters: </b>" + str(self.cur_selected_interaction['parameters']) + "</p>"
        info_text += "</html>"

        self.interactions_widget.app_info.appendHtml(info_text)
        self._set_stop_interactions_button()

    ######################################
    # Gui Updates/Refreshes
    ######################################

    def interactions_updated_relay(self):
        """
        Called by the underlying interactive client whenever the gui needs to be updated with
        fresh information. Using this relay saves us from having to embed qt functions in the
        underlying class but makes sure we signal across threads so the gui can update things
        in its own thread.

        Currently this only handles updates caused by termination of an interaction. If we wished to
        handle additional situations, we should use an argument here indicating what kind of interaction
        update occurred.
        """
        self.signal_interactions_updated.emit()
        # this connects to:
        #  - self._refresh_interactions_list()
        #  - self._set_stop_interactions_button()

    def _refresh_interactions_list(self):
        """
        This just does a complete redraw of the interactions list with the current state.
        It's a bit brute force doing this every time the interactions' 'state' changes,
        but this suffices for now.
        """
        self.interactions = {}
        self.interactions = self.interactive_client.interactions
        self.interactions_widget.interactions_list_widget.clear()

        index = 0
        for interaction in self.interactions.values():
            interaction['index'] = index
            index = index + 1

            self.interactions_widget.interactions_list_widget.insertItem(0, interaction['display_name'])

            # is it a currently running pairing
            if self.interactive_client.pairing == interaction['hash']:
                self.interactions_widget.interactions_list_widget.item(0).setBackground(QColor(100, 100, 150))

            #setting the list font
            font = self.interactions_widget.interactions_list_widget.item(0).font()
            font.setPointSize(13)
            self.interactions_widget.interactions_list_widget.item(0).setFont(font)

            #setting the icon
            app_icon = interaction['icon']
            if app_icon == "unknown.png":
                icon = QIcon(self.icon_paths['unknown'])
                self.interactions_widget.interactions_list_widget.item(0).setIcon(icon)
            elif len(app_icon):
                icon = QIcon(os.path.join(utils.get_icon_cache_home(), app_icon))
                self.interactions_widget.interactions_list_widget.item(0).setIcon(icon)
            else:
                console.logdebug("%s : No icon" % str(self.rocon_master_name))

    def _set_stop_interactions_button(self):
        '''
          Disable or enable the stop button depending on whether the
          selected interaction has any currently launched processes,
        '''
        if not self.interactions:
            return
        try:
            if self.cur_selected_interaction["launch_list"]:
                console.logdebug("Remocon : enabling stop interactions button [%s]" % self.cur_selected_interaction['display_name'])
                self.interactions_widget.stop_interactions_button.setDisabled(False)
            else:
                console.logdebug("Remocon : disabling stop interactions button [%s]" % self.cur_selected_interaction['display_name'])
                self.interactions_widget.stop_interactions_button.setEnabled(False)
        except KeyError:
            pass  # do nothing

    ######################################
    # Start/Stop Interactions
    ######################################

    def _start_interaction(self):
        console.logdebug("Remocon : starting interaction [%s]" % str(self.cur_selected_interaction['name']))
        (result, message) = self.interactive_client.start_interaction(self.cur_selected_interaction['hash'])
        if result:
            if self.cur_selected_interaction['pairing']:
                self._refresh_interactions_list()  # make sure the highlight is working
            self.interactions_widget.stop_interactions_button.setDisabled(False)
        else:
            QMessageBox.warning(self, 'Start Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
            console.logwarn("Remocon : start interaction failed [%s]" % message)

    def _stop_interaction(self):
        console.logdebug("Remocon : stopping interaction %s " % str(self.cur_selected_interaction['name']))
        (result, message) = self.interactive_client.stop_interaction(self.cur_selected_interaction['hash'])
        if result:
            if self.cur_selected_interaction['pairing']:
                self._refresh_interactions_list()  # make sure the highlight is disabled
            self._set_stop_interactions_button()
            #self.interactions_widget.stop_interactions_button.setDisabled(True)
        else:
            QMessageBox.warning(self, 'Stop Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
            console.logwarn("Remocon : stop interaction failed [%s]" % message)
