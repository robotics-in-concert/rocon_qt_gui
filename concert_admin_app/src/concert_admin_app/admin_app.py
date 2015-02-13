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
from python_qt_binding.QtCore import Qt, Signal, SIGNAL, SLOT
from python_qt_binding.QtGui import QWidget, QTreeWidgetItem, QTextEdit, QPushButton, QPlainTextEdit, QGridLayout, QLabel

# ros
import rospkg
# rqt
from qt_gui.plugin import Plugin
import rocon_qt_library.utils as utils

from .admin_app_interface import AdminAppInterface
from rocon_python_comms.exceptions import NotFoundException


##############################################################################
# Admin App
##############################################################################

class AdminApp(Plugin):

    _refresh_service_list_signal = Signal()

    def __init__(self, context):
        self._context = context
        super(AdminApp, self).__init__(context)
        self.initialised = False
        self.setObjectName('Admin App')

        self.is_setting_dlg_live = False
        self._widget = QWidget()
        

        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_admin_app'), 'ui', 'admin_app.ui')
        self._widget.setObjectName('AdminApphUi')
        loadUi(ui_file, self._widget, {})

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self.current_service = None

        self.params_layout = None
        self.params_layout_items = []

        self._init_admin_app_interface()
        self._init_event()
        self._init_widget()
        context.add_widget(self._widget)

    def _init_admin_app_interface(self):
        self.admin_app_interface = AdminAppInterface()
        try:
            self.admin_app_interface._init_admin_app_interface()
            self.admin_app_interface._reg_event_callback(self._refresh_service)
        except NotFoundException as e:
            self.emit(SIGNAL("show_message"), self._widget, "Failed", e)

    def _init_event(self):
        self._widget.enable_disable_btn.pressed.connect(self._toggle_service)
        self._widget.refresh_btn.pressed.connect(self._refresh_service)
        self._widget.save_configuration_btn.pressed.connect(self._save_parmeters)
        self._widget.service_tree_widget.itemClicked.connect(self._select_service_tree_item)  # concert item click event
        self._refresh_service_list_signal.connect(self._update_service_list)
        self.connect(self, SIGNAL("show_message"), utils.show_message)

    def _init_widget(self):
        self.params_layout = self._widget.findChildren(QGridLayout, "params_layout")[0]
        self.params_layout_items = []

        self._update_service_list()

        if not self.current_service:
            self._widget.enable_disable_btn.setDisabled(True)

    def _save_parmeters(self):
        params = {}
        result = False
        msg = ""
        if not self.current_service:
            msg = "Not selected service"
        elif self.current_service['enabled']:
            msg = "service is already enabled!"
        elif self.params_layout_items:
            for item in self.params_layout_items:
                params[item[0].text()] = item[1].toPlainText().strip()
            (result, msg) = self.admin_app_interface.set_srv_parameters(self.current_service['name'], params)
        else:
            msg = "No params infomation"

        if result:
            self.emit(SIGNAL("show_message"), self._widget, "Success", "Saved Parameters")
        else:
            self.emit(SIGNAL("show_message"), self._widget, "Failed", msg)

    def _toggle_service(self):
        if self.current_service['enabled']:
            print "Disable Service: %s" % self.current_service['name']
            (success, message) = self.admin_app_interface.disable_service(self.current_service['name'])
            self._set_enable_params_layout(True)
        else:
            print "Enable Service: %s" % self.current_service['name']
            (success, message) = self.admin_app_interface.eable_service(self.current_service['name'])
            self._set_enable_params_layout(False)
        
        if success:
            self.emit(SIGNAL("show_message"), self._widget, "Success", message)
        else:
            self.emit(SIGNAL("show_message"), self._widget, "Failure", message)

        self.current_service = None
        self._widget.enable_disable_btn.setDisabled(True)
        self._widget.enable_disable_btn.setText("Enable/Disable")

    def _refresh_service(self):
        self._refresh_service_list_signal.emit()

    def _update_service_list(self):
        self._widget.service_tree_widget.clear()
        self._widget.service_info_text.clear()
        self._widgetitem_service_pair = {}
        service_list = self.admin_app_interface.service_list

        for k in service_list.values():
            # Top service
            service_item = QTreeWidgetItem(self._widget.service_tree_widget)
            # service_item=QTreeWidgetItem()
            service_item.setText(0, k['name'])

            # set Top Level Font
            font = service_item.font(0)
            font.setPointSize(20)
            font.setBold(True)
            service_item.setFont(0, font)

            self._widgetitem_service_pair[service_item] = k

    def _set_service_info(self, service_name):
        service_list = self.admin_app_interface.service_list
        self._widget.service_info_text.clear()
        self._widget.service_info_text.appendHtml(service_list[service_name]['context'])

    def _select_service_tree_item(self, item):
        if item.parent() == None:
            selected_service = self._widgetitem_service_pair[item]

            print '_select_service: ' + selected_service['name']
            self._set_service_info(selected_service['name'])
            self.current_service = selected_service
            self._set_parameter_layout()

            if self.current_service['enabled']:
                self._widget.enable_disable_btn.setText("Disable")
                self._widget.enable_disable_btn.setDisabled(False)
            else:
                self._widget.enable_disable_btn.setText("Enable")
                self._widget.enable_disable_btn.setDisabled(False)

    def _set_parameter_layout(self):

        params = self.admin_app_interface.get_srv_parameters(self.current_service['parameters_detail'])
        is_enabled = self.current_service['enabled']

        if self.params_layout_items:
            for item in self.params_layout_items:
                self.params_layout.removeWidget(item[0])
                self.params_layout.removeWidget(item[1])
                item[0].setParent(None)
                item[1].setParent(None)
            self.params_layout_items = []

        if params:
            self.params_layout.setColumnStretch(1, 0)
            self.params_layout.setRowStretch(2, 0)
            for param in params.keys():
                label = QLabel(param)
                self.params_layout.addWidget(label)
                value = QTextEdit(str(params[param]))
                value.setMaximumHeight(30)
                self.params_layout.addWidget(value)
                self.params_layout_items.append((label, value))

        self._set_enable_params_layout(not is_enabled)

    def _set_enable_params_layout(self, enable):
        if self.params_layout_items:
            self._widget.save_configuration_btn.setEnabled(enable)
            for item in self.params_layout_items:
                item[0].setEnabled(enable)
                item[1].setEnabled(enable)
