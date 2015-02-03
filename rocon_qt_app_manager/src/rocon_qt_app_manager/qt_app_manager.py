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
from python_qt_binding.QtCore import Signal, QSize, SIGNAL
from python_qt_binding.QtGui import QListView, QWidget, QStandardItemModel, QStandardItem, QIcon, QPixmap 
#ros
import rospkg
from qt_app_manager_info import QtRappManagerInfo
#rqt
from qt_gui.plugin import Plugin


from rocon_qt_library.utils import show_message
from .utils import QRappItem
from .qt_rapp_status_dialog import QtRappDialog


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
        self.spin()

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

        # Rapp single click event
        self._widget.rapp_grid.clicked.connect(self._rapp_single_click)
        # Rapp double click event
        self._widget.rapp_grid.doubleClicked.connect(self._rapp_double_click)

    def _init_variables(self):
        self.initialised = False

        #init
        self._qt_rapp_manager_info = QtRappManagerInfo(self._refresh_rapps)
        self._update_rapps_signal.connect(self._update_rapp_list)

        self._rapp_view_model = QStandardItemModel()
        self._widget.rapp_grid.setModel(self._rapp_view_model)
        self._widget.rapp_grid.setWrapping(True)
        self._widget.rapp_grid.setIconSize(QSize(60,60))
        self._widget.rapp_grid.setSpacing(10)

        self._selected_rapp = None

    def spin(self):
        self._get_appmanager_namespaces()

    def _cleanup_rapps(self):
        self._rapp_view_model.clear()
        pass 

    def _get_appmanager_namespaces(self):
        namespaces = self._qt_rapp_manager_info._get_namespaces()
        for namespace in namespaces:
            ns = namespace[:namespace.find('list_rapps')]
            self._widget.namespace_cbox.addItem(ns)

    def _exit(self):
        pass

###################################################################
# Events
###################################################################
    def _change_namespace(self, event):
        self._qt_rapp_manager_info.select_rapp_manager(self._widget.namespace_cbox.currentText())

    def _refresh_rapps(self):
        """
        Updates from qt_app_manager_info.
        """
        self._update_rapps_signal.emit()

    def _update_rapp_list(self):
        """
        Rapp manager namespace event
        """
        self._cleanup_rapps()
        available_rapps = self._qt_rapp_manager_info.get_available_rapps()
        running_rapps = self._qt_rapp_manager_info.get_running_rapps()

        rapp_items = []
        for r, v in available_rapps.items():
            if r in running_rapps.keys():
                item = QRappItem(v, running=True)
            else:
                item = QRappItem(v, running=False)
            self._rapp_view_model.appendRow(item)


    def _rapp_single_click(self, index):
        qrapp = self._rapp_view_model.item(index.row())
        rapp = qrapp.getRapp()
        self._create_rapp_dialog(rapp)

    def _create_rapp_dialog(self, rapp):
        is_running = self._qt_rapp_manager_info.is_running_rapp(rapp)
        self._selected_rapp = rapp
        self._dialog = QtRappDialog(self._widget,rapp, self._qt_rapp_manager_info.start_rapp, self._qt_rapp_manager_info.stop_rapp, is_running)
        self._dialog.show()

    def _rapp_double_click(self, item):
        running_rapps = self._qt_rapp_manager_info.get_running_rapps()
        if len(running_rapps) > 0:
            names = [r['display_name'] for r in running_rapps.values()]
            show_message(self._widget, "Error", "Rapp %s are already running"%names)
        else:
            self._start_rapp()

    def _start_rapp(self):
        result = self._qt_rapp_manager_info.start_rapp(self._selected_rapp['name'], [], self._selected_rapp['public_parameters'])
        show_message(self._widget, str(result.started), result.message)
        self._selected_rapp = None
        return result

    def _stop_rapp(self):
        result = self._qt_rapp_manager_info.stop_rapp()
        show_message(self._widget, str(result.stopped), result.message)
        self._selected_rapp = None
        return result

########################################
# Legacy
########################################
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
