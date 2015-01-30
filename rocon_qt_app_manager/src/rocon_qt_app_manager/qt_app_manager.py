#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
#system
from __future__ import division
import os

#pyqt
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, QSize
from python_qt_binding.QtGui import QListView, QWidget, QStandardItemModel, QStandardItem, QIcon, QPixmap 
#ros
import rospkg
from qt_app_manager_info import QtRappManagerInfo
#rqt
from qt_gui.plugin import Plugin

##############################################################################
# Utils
##############################################################################

def create_label_textedit_pair(key, value):

    param_layout = QHBoxLayout()

    name_widget = QLabel(key)
    textedit_widget = QTextEdit() 
    textedit_widget.setSizePolicy(QSizePolicy.MinimumExpanding, QSizePolicy.Ignored)
    textedit_widget.setMinimumSize(0,30)
    textedit_widget.append(str(value))

    param_layout.addWidget(name_widget)
    param_layout.addWidget(textedit_widget)

    return param_layout
 

def clear_layout(layout):
    for i in reversed(range(layout.count())):
        item = layout.itemAt(i)

        if isinstance(item, QWidgetItem):
            item.widget().close()
        else:
            clear_layout(item.layout())

        # remove the item from layout
        layout.removeItem(item)    


##############################################################################
# QtAppManager
##############################################################################


class QtRappManager(Plugin):
    _update_rapps_signal = Signal()

    def __init__(self, context):
        self._context = context
        super(QtRappManager, self).__init__(context)


        self._init_ui(context)
        self._init_events()
        self._init_variables()
        self._start()

    def _init_ui(self, context):
        self._widget = QWidget()
        self.setObjectName('QtRappManger')

        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_qt_app_manager'), 'ui', 'qt_rapp_manager.ui')
        self._widget.setObjectName('QtRappManger')
        loadUi(ui_file, self._widget, {})

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Set view mode
        self._widget.rapp_grid.setViewMode(QListView.IconMode)
        #self._widget.rapp_grid.setViewMode(QListView.ListMode)

    def _init_events(self):
        #combo box change event
        self._widget.namespace_cbox.currentIndexChanged.connect(self._change_namespace)

    def _init_variables(self):
        self.initialised = False

        #init
        self._qt_rapp_manager_info = QtRappManagerInfo(self._refresh_rapps)
        self._update_rapps_signal.connect(self._update_rapp_list)

        self._rapp_view_model = QStandardItemModel()
        self._widget.rapp_grid.setModel(self._rapp_view_model)
        self._widget.rapp_grid.setWrapping(True)
        self._widget.rapp_grid.setIconSize(QSize(90,90))
        self._widget.rapp_grid.setSpacing(10)


    def _start(self):
        self._get_appmanager_namespaces()

    def _change_namespace(self, event):
        self._cleanup_rapps()
        self._qt_rapp_manager_info.select_rapp_manager(self._widget.namespace_cbox.currentText())

    def _cleanup_rapps(self):
        pass 

    def _get_appmanager_namespaces(self):
        namespaces = self._qt_rapp_manager_info._get_namespaces()
        for namespace in namespaces:
            ns = namespace[:namespace.find('list_rapps')]
            self._widget.namespace_cbox.addItem(ns)

    def _update_rapp_list(self):
        rapps = self._qt_rapp_manager_info.get_available_rapps()

        for r, v in rapps.items():
            item = QStandardItem(v['display_name'])
            item.setSizeHint(QSize(100,100))
            icon = self._get_qicon(v['icon'])
            item.setIcon(icon)
            self._rapp_view_model.appendRow(item)


    def _select_rapp_tree_item(self, item):
        if not item in self.rapps.keys():
            print "HAS NO KEY"
        else:
            self.current_rapp = self.rapps[item]
        self._widget.rapp_info_text.clear()
        rapp_info = self.qt_rapp_manager_info._get_rapp_info(self.current_rapp)
        self._widget.rapp_info_text.appendHtml(rapp_info)
        self._update_rapp_parameter_layout(self.current_rapp)
        self._update_implementation_tree(self.current_rapp)

    def _update_rapp_parameter_layout(self, rapp):
        parameters_layout = self._widget.rapp_parameter_layout
        clear_layout(parameters_layout)

        for param in rapp['public_parameters']:
            one_param_layout = create_label_textedit_pair(param.key, param.value)
            parameters_layout.addLayout(one_param_layout)

    def _update_implementation_tree(self, rapp):
        self._widget.implementation_tree_widget.clear()
        self.selected_impl = None        
        self.impls = {}
        for impl in rapp['implementations']:
            impl_item = QTreeWidgetItem(self._widget.implementation_tree_widget)                      
            impl_item.setText(0, impl)
            self.impls[impl_item] = impl

    def _select_implementation_tree_item(self, item):
        if not item in self.impls.keys():
            print "HAS NO KEY"
        else:
            self.selected_impl = self.impls[item]

    def _get_public_parameters(self):
        public_parameters = {}
        parameters_layout = self._widget.rapp_parameter_layout
        for i in reversed(range(parameters_layout.count())):
            item = parameters_layout.itemAt(i)
            key_label = item.itemAt(0).widget()
            value_textbox = item.itemAt(1).widget()
            public_parameters[key_label.text()] = str(value_textbox.toPlainText())
        return public_parameters

    def _manage_buttons(self):
        rapps = self.qt_rapp_manager_info.running_rapps
        if(rapps == {}):
            self._widget.icon_label.clear()
            self._widget.stop_rapp_btn.setEnabled(False)
            self._widget.start_rapp_btn.setEnabled(True)
        else:
            self._widget.stop_rapp_btn.setEnabled(True)
            self._widget.start_rapp_btn.setEnabled(False)
            self._widget.icon_label.clear()
            self._set_icon()

    def _get_qicon(self, icon):
        pixmap = QPixmap()
        pixmap.loadFromData(icon.data, format=icon.format)
        
        return QIcon(pixmap)
        
    def _refresh_rapps(self):
        self._update_rapps_signal.emit()

    def _start_rapp(self):
        ns = self._widget.namespace_cbox.currentText()
        
        if not self.selected_impl and not self.current_rapp:
            print "No rapp has been selected yet" 
            return
            
        rapp = self.selected_impl if self.selected_impl else self.current_rapp['name']
        parameters = self._get_public_parameters()

        result = self.qt_rapp_manager_info._start_rapp(ns, rapp, parameters)
        self._widget.service_result_text.appendHtml(result)
        self._widget.icon_label.clear()

    def _stop_rapp(self):
        ns = self._widget.namespace_cbox.currentText()
        result = self.qt_rapp_manager_info._stop_rapp(ns)
        self._widget.service_result_text.appendHtml(result)


    def _exit(self):
        pass
