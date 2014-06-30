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
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, QAbstractListModel, pyqtSignal
from python_qt_binding.QtCore import pyqtSlot, SIGNAL,SLOT, QRectF , QTimer, QEvent, QUrl
from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget, QLabel, QComboBox
from python_qt_binding.QtGui import QSizePolicy,QTextEdit, QCompleter, QBrush, QDialog, QColor, QPen, QPushButton
from python_qt_binding.QtGui import QTabWidget, QPlainTextEdit,QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox, QWidgetItem, QSpacerItem
from python_qt_binding.QtGui import QTreeWidgetItem, QPixmap, QGraphicsScene
from python_qt_binding.QtDeclarative import QDeclarativeView
from python_qt_binding.QtSvg import QSvgGenerator
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
        self.initialised = False
        self.setObjectName('QtRappManger')

        self._widget = QWidget()
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_qt_app_manager'), 'ui', 'qt_rapp_manager.ui')
        self._widget.setObjectName('QtRappManger')
        loadUi(ui_file, self._widget, {})
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        #list item click event
        self._widget.rapp_tree_widget.itemClicked.connect(self._select_rapp_tree_item)
        #button event connection
        self._widget.start_rapp_btn.pressed.connect(self._start_rapp)
        self._widget.stop_rapp_btn.pressed.connect(self._stop_rapp)
        #signal event connection
        self._widget.destroyed.connect(self._exit)
        self._update_rapps_signal.connect(self._update_rapp_list)
        #combo box change event
        self._widget.namespace_cbox.currentIndexChanged.connect(self._change_namespace)
        context.add_widget(self._widget)

        #init
        self.qt_rapp_manager_info = QtRappManagerInfo()
        self.qt_rapp_manager_info._reg_update_rapps_callback(self._refresh_rapps)

        self.rapps = {}
        self.current_rapp = None
        self.current_captured_robot = None

        self._get_name_spaces()

    def _change_namespace(self, event):
        self.qt_rapp_manager_info._get_rapps(self._widget.namespace_cbox.currentText())
        self.qt_rapp_manager_info._set_update_status(self._widget.namespace_cbox.currentText())
        pass

    def _get_name_spaces(self):
        for namespace in self.qt_rapp_manager_info._get_namespaces():
            ns = namespace[:namespace.find('list_rapps')]
            self._widget.namespace_cbox.addItem(ns)
        self.qt_rapp_manager_info._get_rapps(self._widget.namespace_cbox.currentText())
        self.qt_rapp_manager_info._set_update_status(self._widget.namespace_cbox.currentText())

    def _exit(self):
        pass

    def _start_rapp(self):
        ns = self._widget.namespace_cbox.currentText()

        parameters = self._get_public_parameters()

        result = self.qt_rapp_manager_info._start_rapp(ns, self.current_rapp['name'], parameters)
        self._widget.service_result_text.appendHtml(result)
        pass

    def _stop_rapp(self):
        ns = self._widget.namespace_cbox.currentText()
        result = self.qt_rapp_manager_info._stop_rapp(ns)
        self._widget.service_result_text.appendHtml(result)
        pass

    def _update_rapp_list(self):
        self._widget.rapp_info_text.clear()
        self._widget.rapp_tree_widget.clear()
        self.rapps = {}
        rapps = self.qt_rapp_manager_info.rapps
        for k in rapps.values():
            rapp = QTreeWidgetItem(self._widget.rapp_tree_widget)
            rapp.setText(0, k["display_name"])
            self.rapps[rapp] = k

        self._widget.running_rapp_tree_widget.clear()
        rapps = self.qt_rapp_manager_info.running_rapps
        for k in rapps.values():
            rapp = QTreeWidgetItem(self._widget.running_rapp_tree_widget)
            rapp.setText(0, k["display_name"])
            self.rapps[rapp] = k

    def _select_rapp_tree_item(self, Item):
        if not Item in self.rapps.keys():
            print "HAS NO KEY"
        else:
            self.current_rapp = self.rapps[Item]
        self._widget.rapp_info_text.clear()
        rapp_info = self.qt_rapp_manager_info._get_rapp_info(self.current_rapp)
        self._widget.rapp_info_text.appendHtml(rapp_info)
        self._update_rapp_parameter_layout(self.current_rapp)

    def _update_rapp_parameter_layout(self, rapp):
        parameters_layout = self._widget.rapp_parameter_layout
        clear_layout(parameters_layout)

        for param in rapp['public_parameters']:
            one_param_layout = create_label_textedit_pair(param.key, param.value)
            parameters_layout.addLayout(one_param_layout)

    def _get_public_parameters(self):
        public_parameters = {}
        parameters_layout = self._widget.rapp_parameter_layout
        for i in reversed(range(parameters_layout.count())):
            item = parameters_layout.itemAt(i)
            key_label = item.itemAt(0).widget()
            value_textbox = item.itemAt(1).widget()
            public_parameters[key_label.text()] = str(value_textbox.toPlainText())
        return public_parameters
        
    def _refresh_rapps(self):
        self._update_rapps_signal.emit()
