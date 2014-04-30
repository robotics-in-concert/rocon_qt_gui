#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import sys

from PyQt4 import uic
from PyQt4.QtCore import QString  # pyqtSlot, SIGNAL, SLOT, QPoint, QEvent
from PyQt4.QtCore import Qt, QSize  # QFile, QIODevice, QAbstractListModel, pyqtSignal, QStringList
from PyQt4.QtGui import QIcon, QWidget, QLabel  # QFileDialog, QGraphicsScene, QImage, QPainter, QComboBox
from PyQt4.QtGui import QSizePolicy, QTextEdit, QPushButton, QDialog, QColor  # QCompleter, QBrush, QPen
from PyQt4.QtGui import QMainWindow, QCheckBox

from PyQt4.QtGui import QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox  #QTabWidget, QPlainTextEdit
#from PyQt4.QtSvg import QSvgGenerator

import rospkg
import rocon_python_utils
from rocon_console import console
import rocon_interactions.web_interactions as web_interactions

from rocon_remocon.interactive_client import InteractiveClient
from . import utils
from .rocon_masters import RoconMasters

##############################################################################
# Remocon
##############################################################################


class RemoconSub(QMainWindow):

    def __init__(self, parent, title, application, rocon_master_index="", rocon_master_name="", rocon_master_uri='localhost', host_name='localhost'):
        self.rocon_master_index = rocon_master_index
        self.rocon_master_uri = rocon_master_uri
        self.rocon_master_name = rocon_master_name
        self.host_name = host_name
        self._context = parent
        self.application = application

        super(RemoconSub, self).__init__(parent)
        self.initialised = False

        self.interactions_widget = QWidget()
        self.roles_widget = QWidget()

        self.cur_selected_role = 0

        self.interactions = {}
        self.cur_selected_interaction = None

        self.interactive_client = InteractiveClient(stop_interaction_postexec_fn=self.interactions_updated_handler)

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../ui/interactions_list.ui")
        uic.loadUi(path, self.interactions_widget)

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../ui/role_list.ui")
        uic.loadUi(path, self.roles_widget)

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
        self.interactions_widget.refresh_btn.pressed.connect(self._refresh_interactions_list)
        self.interactions_widget.stop_interactions_button.setDisabled(True)

        #init
        self._init()

    def _init(self):
        self._init_role_list()
        # Ugly Hack : our window manager is not graying out the button when an interaction closes itself down and the appropriate
        # callback (_set_stop_interactions_button) is fired. It does otherwise though so it looks like the window manager
        # is getting confused when the original program doesn't have the focus.
        #
        # Taking control of it ourselves works...
        self.interactions_widget.stop_interactions_button.setStyleSheet(QString.fromUtf8("QPushButton:disabled { color: gray }"))
        self.roles_widget.show()
        self.initialised = True

    ######################################
    # Roles List Widget
    ######################################

    def _init_role_list(self):

        if not self.interactive_client._connect(self.rocon_master_name, self.rocon_master_uri, self.host_name):
            return False
        self._refresh_role_list()
        return True

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
        os.execv(RemoconMain.rocon_remocon_script, ['', self.host_name])

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

    def interactions_updated_handler(self):
        """
        Called by the underlying interactive client whenever the gui needs to be updated with
        fresh information.

        Currently this only handles updates caused by termination of an interaction. If we wished to
        handle additional situations, we should use an argument here indicating what kind of interaction
        update occurred.
        """
        self._refresh_interactions_list()     # redraw the list
        self._set_stop_interactions_button()  # toggle the stop button if necessary

    def _refresh_interactions_list(self):
        self.interactions = {}
        self.interactions = self.interactive_client.interactions
        self.interactions_widget.interactions_list_widget.clear()

        index = 0
        for interaction in self.interactions.values():
            interaction['index'] = index
            index = index + 1

            self.interactions_widget.interactions_list_widget.insertItem(0, interaction['display_name'])

            # is it a currently running pairing
            console.logwarn("Looking for pairing [%s][%s]" % (self.interactive_client.pairing, interaction['display_name']))
            if self.interactive_client.pairing == interaction['hash']:
                self.interactions_widget.interactions_list_widget.item(0).setBackground(QColor(100, 100, 150))
                console.logwarn("THIS IS A PAIRING [%s]" % interaction['display_name'])

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
                self._refresh_interactions_list()  # highlight that it is a pairing running
            self.interactions_widget.stop_interactions_button.setDisabled(False)
        else:
            QMessageBox.warning(self, 'Start Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
            console.logwarn("Remocon : start interaction failed [%s]" % message)

    def _stop_interaction(self):
        console.logdebug("Remocon : stopping interaction %s " % str(self.cur_selected_interaction['name']))
        (result, message) = self.interactive_client.stop_interaction(self.cur_selected_interaction['hash'])
        if result:
            self._set_stop_interactions_button()
            #self.interactions_widget.stop_interactions_button.setDisabled(True)
        else:
            QMessageBox.warning(self, 'Stop Interaction Failed', "%s." % message.capitalize(), QMessageBox.Ok)
            console.logwarn("Remocon : stop interaction failed [%s]" % message)

#################################################################
##Remocon Main
#################################################################


class RemoconMain(QMainWindow):

    rocon_remocon_script = utils.find_rocon_remocon_script('rocon_remocon')
    rocon_remocon_sub_script = utils.find_rocon_remocon_script('rocon_remocon_sub')

    def __init__(self, parent, title, application):
        self._context = parent

        super(RemoconMain, self).__init__()
        self.initialised = False
        self.setObjectName('Remocon')

        self.host_name = "localhost"
        self.master_uri = "http://%s:11311" % (self.host_name)

        self.env_host_name = os.getenv("ROS_HOSTNAME")
        self.env_master_uri = os.getenv("ROS_MASTER_URI")
        if self.env_host_name == None:
            self.env_host_name = 'localhost'
        if self.env_master_uri == None:
            self.env_master_uri = "http://%s:11311" % (self.env_host_name)

        self.application = application
        self._widget_main = QWidget()

        self.rocon_masters = RoconMasters()
        self.cur_selected_rocon_master = None
        self.is_init = False

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "../../ui/remocon.ui")
        uic.loadUi(path, self._widget_main)

        utils.setup_home_dirs()

        self.icon_paths = {}
        try:
            self.icon_paths['unknown'] = rocon_python_utils.ros.find_resource_from_string('rocon_icons/unknown', extension='png')
        except (rospkg.ResourceNotFound, ValueError):
            console.logerror("Remocon : couldn't find icons on the ros package path (install rocon_icons and rocon_bubble_icons")
            sys.exit(1)

        #main widget
        self._widget_main.list_widget.setIconSize(QSize(50, 50))
        self._widget_main.list_widget.itemDoubleClicked.connect(self._connect_rocon_master)  # list item double click event
        self._widget_main.list_widget.itemClicked.connect(self._select_rocon_master)  # list item double click event

        self._widget_main.add_concert_btn.pressed.connect(self._set_add_rocon_master)  # add button event
        self._widget_main.delete_btn.pressed.connect(self._delete_rocon_master)  # delete button event
        self._widget_main.delete_all_btn.pressed.connect(self._delete_all_rocon_masters)  # delete all button event
        self._widget_main.refresh_btn.pressed.connect(self._refresh_all_rocon_master_list)  # refresh all button event

        #init
        self._init()
        self._widget_main.show()
        self._widget_main.activateWindow()  # give it the focus
        self._widget_main.raise_()          # make sure it is on top

    def __del__(self):
        console.loginfo("RemoconMain: Destroy")

    def _init(self):

        self._connect_dlg_isValid = False
        self.cur_selected_rocon_master = None
        self._refresh_all_rocon_master_list()
        self.is_init = True

    def _delete_all_rocon_masters(self):
        self.rocon_masters.clear()
        self._update_rocon_master_list()
        self._widget_main.list_info_widget.clear()

    def _delete_rocon_master(self):
        if self.cur_selected_rocon_master in self.rocon_masters.keys():
            self.rocon_masters.delete(self.cur_selected_rocon_master)
        self._update_rocon_master_list()
        self._widget_main.list_info_widget.clear()

    def _add_rocon_master(self, uri_text_widget, host_name_text_widget):
        rocon_master = self.rocon_masters.add(uri_text_widget.toPlainText(), host_name_text_widget.toPlainText())
        self._refresh_rocon_master(rocon_master)

    def _set_add_rocon_master(self):

        if self._connect_dlg_isValid:
            console.logdebug("Dialog is live!!")
            self._connect_dlg.done(0)

        #dialog
        self._connect_dlg = QDialog(self._widget_main)
        self._connect_dlg.setWindowTitle("Add Ros Master")
        self._connect_dlg.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
        self._connect_dlg.setMinimumSize(350, 0)
        # dlg_rect = self._connect_dlg.geometry()

        #dialog layout
        ver_layout = QVBoxLayout(self._connect_dlg)
        ver_layout.setContentsMargins(9, 9, 9, 9)

        #param layout
        text_grid_sub_widget = QWidget()
        text_grid_layout = QGridLayout(text_grid_sub_widget)
        text_grid_layout.setColumnStretch(1, 0)
        text_grid_layout.setRowStretch(2, 0)

        #param 1
        title_widget1 = QLabel("MASTER_URI: ")
        context_widget1 = QTextEdit()
        context_widget1.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
        context_widget1.setMinimumSize(0, 30)
        context_widget1.append(self.master_uri)

        #param 2
        title_widget2 = QLabel("HOST_NAME: ")
        context_widget2 = QTextEdit()
        context_widget2.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
        context_widget2.setMinimumSize(0, 30)
        context_widget2.append(self.host_name)

        #add param
        text_grid_layout.addWidget(title_widget1)
        text_grid_layout.addWidget(context_widget1)
        text_grid_layout.addWidget(title_widget2)
        text_grid_layout.addWidget(context_widget2)

        #add param layout
        ver_layout.addWidget(text_grid_sub_widget)

        #button layout
        button_hor_sub_widget = QWidget()
        button_hor_layout = QHBoxLayout(button_hor_sub_widget)

        uri_text_widget = context_widget1
        host_name_text_widget = context_widget2

        #check box
        use_env_var_check = QCheckBox("Use environment variables")
        use_env_var_check.setCheckState(Qt.Unchecked)

        def set_use_env_var(data, text_widget1, text_widget2):
            if data == Qt.Unchecked:
                text_widget1.setText(self.master_uri)
                text_widget2.setText(self.host_name)
            elif data == Qt.Checked:
                self.master_uri = str(text_widget1.toPlainText())
                self.host_name = str(text_widget2.toPlainText())
                text_widget1.setText(self.env_master_uri)
                text_widget2.setText(self.env_host_name)

        def check_event(data):
            set_use_env_var(data, context_widget1, context_widget2)

        use_env_var_check.stateChanged.connect(check_event)
        ver_layout.addWidget(use_env_var_check)

        #button
        btn_call = QPushButton("Add")
        btn_cancel = QPushButton("Cancel")

        btn_call.clicked.connect(lambda: self._connect_dlg.done(0))
        btn_call.clicked.connect(lambda: self._add_rocon_master(uri_text_widget, host_name_text_widget))

        btn_cancel.clicked.connect(lambda: self._connect_dlg.done(0))

        #add button
        button_hor_layout.addWidget(btn_call)
        button_hor_layout.addWidget(btn_cancel)

        #add button layout
        ver_layout.addWidget(button_hor_sub_widget)
        self._connect_dlg.setVisible(True)
        self._connect_dlg.finished.connect(self._destroy_connect_dlg)
        self._connect_dlg_isValid = True

    def _refresh_rocon_master(self, rocon_master):
        rocon_master.check()
        self._widget_main.list_info_widget.clear()
        self._update_rocon_master_list()

    def _refresh_all_rocon_master_list(self):
        if self.is_init:
            self.rocon_masters.check()
        self._widget_main.list_info_widget.clear()
        self._update_rocon_master_list()

    def _update_rocon_master_list(self):
        self._widget_main.list_widget.clear()
        for rocon_master in self.rocon_masters.values():
            self._add_rocon_master_list_item(rocon_master)
        self.rocon_masters.dump()

    def _add_rocon_master_list_item(self, rocon_master):

        rocon_master.current_row = str(self._widget_main.list_widget.count())

        display_name = str(rocon_master.name) + "\n" + "[" + str(rocon_master.uri) + "]"
        self._widget_main.list_widget.insertItem(self._widget_main.list_widget.count(), display_name)

        #setting the list font

        font = self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).font()
        font.setPointSize(13)
        self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).setFont(font)

        #setToolTip
        rocon_master_info = ""
        rocon_master_info += "rocon_master_index: " + str(rocon_master.index) + "\n"
        rocon_master_info += "rocon_master_name: " + str(rocon_master.name) + "\n"
        rocon_master_info += "master_uri:  " + str(rocon_master.uri) + "\n"
        rocon_master_info += "host_name:  " + str(rocon_master.host_name) + "\n"
        rocon_master_info += "description:  " + str(rocon_master.description)
        self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).setToolTip(rocon_master_info)

        #set icon
        if rocon_master.icon == "unknown.png":
            icon = QIcon(self.icon_paths['unknown'])
            self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).setIcon(icon)
        elif len(rocon_master.icon):
            icon = QIcon(os.path.join(utils.get_icon_cache_home(), rocon_master.icon))
            self._widget_main.list_widget.item(self._widget_main.list_widget.count() - 1).setIcon(icon)
        else:
            console.logdebug("%s : No icon" % rocon_master.name)
        pass

    def _select_rocon_master(self, Item):
        list_widget = Item.listWidget()
        for k in self.rocon_masters.values():
            if k.current_row == str(list_widget.currentRow()):
                self.cur_selected_rocon_master = k.index
                break
        self._widget_main.list_info_widget.clear()
        info_text = "<html>"
        info_text += "<p>-------------------------------------------</p>"
        info_text += "<p><b>name: </b>" + str(self.rocon_masters[self.cur_selected_rocon_master].name) + "</p>"
        info_text += "<p><b>master_uri: </b>" + str(self.rocon_masters[self.cur_selected_rocon_master].uri) + "</p>"
        info_text += "<p><b>host_name: </b>" + str(self.rocon_masters[self.cur_selected_rocon_master].host_name) + "</p>"
        info_text += "<p><b>description: </b>" + str(self.rocon_masters[self.cur_selected_rocon_master].description) + "</p>"
        info_text += "<p>-------------------------------------------</p>"
        info_text += "</html>"
        self._widget_main.list_info_widget.appendHtml(info_text)

    def _destroy_connect_dlg(self):
        self._connect_dlg_isValid = False

    def _connect_rocon_master(self):
        rocon_master_name = str(self.rocon_masters[self.cur_selected_rocon_master].name)
        rocon_master_uri = str(self.rocon_masters[self.cur_selected_rocon_master].uri)
        rocon_master_host_name = str(self.rocon_masters[self.cur_selected_rocon_master].host_name)

        rocon_master_index = str(self.cur_selected_rocon_master)
        self.rocon_masters[rocon_master_index].check()
        if self.rocon_masters[rocon_master_index].flag == '0':
            QMessageBox.warning(self, 'Rocon Master Connection Error', "You selected no concert", QMessageBox.Ok)
            return

        self._widget_main.hide()
        arguments = ["", rocon_master_index, rocon_master_name, rocon_master_uri, rocon_master_host_name]
        os.execv(RemoconMain.rocon_remocon_sub_script, arguments)
        console.logdebug("Spawning: %s with args %s" % (RemoconMain.rocon_remocon_sub_script, arguments))
