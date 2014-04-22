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
from python_qt_binding.QtGui import QTabWidget, QPlainTextEdit,QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox
from python_qt_binding.QtGui import QTreeWidgetItem, QPixmap, QGraphicsScene
from python_qt_binding.QtDeclarative import QDeclarativeView
from python_qt_binding.QtSvg import QSvgGenerator
#ros
import rospkg
from qt_app_manager_info import QtAppManagerInfo
#rqt
from qt_gui.plugin import Plugin

##############################################################################
# QtAppManager
##############################################################################


class QtAppManager(Plugin):
    _update_apps_signal = Signal()

    def __init__(self, context):
        self._context = context
        super(QtAppManager, self).__init__(context)
        self.initialised = False
        self.setObjectName('QtAppManger')

        self._widget = QWidget()
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_qt_app_manager'), 'ui', 'qt_app_manager.ui')
        self._widget.setObjectName('QtAppManger')
        loadUi(ui_file, self._widget, {})
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        #list item click event
        self._widget.app_tree_widget.itemClicked.connect(self._select_app_tree_item)
        #button event connection
        self._widget.start_app_btn.pressed.connect(self._start_app)
        self._widget.stop_app_btn.pressed.connect(self._stop_app)
        #signal event connection
        self._widget.destroyed.connect(self._exit)
        self._update_apps_signal.connect(self._update_app_list)
        #combo box change event
        self._widget.namespace_cbox.currentIndexChanged.connect(self._change_namespace)
        context.add_widget(self._widget)

        #init
        self.qt_app_manager_info = QtAppManagerInfo()
        self.qt_app_manager_info._reg_update_apps_callback(self._refresh_apps)

        self.apps = {}
        self.current_app = None
        self.current_captured_robot = None

        self._get_name_spaces()

    def _change_namespace(self, event):
        self.qt_app_manager_info._get_apps(self._widget.namespace_cbox.currentText())
        self.qt_app_manager_info._set_update_status(self._widget.namespace_cbox.currentText())
        pass

    def _get_name_spaces(self):
        for namespace in self.qt_app_manager_info._get_namespaces():
            ns = namespace[:namespace.find('list_rapps')]
            self._widget.namespace_cbox.addItem(ns)
        self.qt_app_manager_info._get_apps(self._widget.namespace_cbox.currentText())
        self.qt_app_manager_info._set_update_status(self._widget.namespace_cbox.currentText())

    def _exit(self):
        pass

    def _start_app(self):
        ns = self._widget.namespace_cbox.currentText()
        result = self.qt_app_manager_info._start_app(ns, self.current_app['name'])
        self._widget.service_result_text.appendHtml(result)
        pass

    def _stop_app(self):
        ns = self._widget.namespace_cbox.currentText()
        result = self.qt_app_manager_info._stop_app(ns)
        self._widget.service_result_text.appendHtml(result)
        pass

    def _update_app_list(self):
        self._widget.app_info_text.clear()
        self._widget.app_tree_widget.clear()
        self.apps = {}
        apps = self.qt_app_manager_info.apps
        for k in apps.values():
            app = QTreeWidgetItem(self._widget.app_tree_widget)
            app.setText(0, k["display_name"])
            self.apps[app] = k

        self._widget.running_app_tree_widget.clear()
        apps = self.qt_app_manager_info.running_apps
        for k in apps.values():
            app = QTreeWidgetItem(self._widget.running_app_tree_widget)
            app.setText(0, k["display_name"])
            self.apps[app] = k

    def _select_app_tree_item(self, Item):
        if not Item in self.apps.keys():
            print "HAS NO KEY"
        else:
            self.current_app = self.apps[Item]
        self._widget.app_info_text.clear()
        app_info = self.qt_app_manager_info._get_app_info(self.current_app)
        self._widget.app_info_text.appendHtml(app_info)

    def _refresh_apps(self):
        self._update_apps_signal.emit()
        pass
