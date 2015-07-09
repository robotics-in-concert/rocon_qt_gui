#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
from __future__ import division
import os

import rospy
import rospkg
import rocon_uri
import threading
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, Signal, SIGNAL
from python_qt_binding.QtGui import QWidget, QMessageBox, QTreeWidgetItem, QBrush, QColor

##############################################################################
# Class
##############################################################################

class QResourceChooser(QWidget):
    '''
      Captures resource from concert
    '''
    def __init__(self, parent=None):
        super(QResourceChooser, self).__init__()
        self._load_ui()
        self._init_events()
        self._init_callbacks()
        self.current_captured_resource = None
        self.current_resource = None
        self.resource_item_list = {}
        self._lock = threading.Lock()


    def _load_ui(self): 
        rospack = rospkg.RosPack() 
        ui_file = os.path.join(rospack.get_path('rocon_qt_library'), 'ui', 'resource_chooser.ui')
        loadUi(ui_file, self)

    def set_callbacks(self, capture_resource_event_callbacks=[], release_resource_event_callbacks=[]):
        self._callback['capture_event'] = capture_resource_event_callbacks
        self._callback['release_event'] = release_resource_event_callbacks
        
    def _init_events(self):
        # List item click event
        self.resource_list_tree_widget.itemClicked.connect(self._resource_selected)
        self.resource_list_tree_widget.itemDoubleClicked.connect(self._resource_double_clicked)

        # Button event connect
        self.capture_resource_btn.pressed.connect(self._capture_resource)
        self.release_resource_btn.pressed.connect(self._release_resource)

        # Event emitter
        self.connect(self, SIGNAL("capture"), self._show_capture_resource_message)
        self.connect(self, SIGNAL("release"), self._show_release_resource_message)
        self.connect(self, SIGNAL("error"), self._show_error_resource_message)
        self.connect(self, SIGNAL("refresh_resource_list"), self._refresh_resource_list)

    def _init_callbacks(self):
        self._callback = {}
        self._callback['capture_event'] = []
        self._callback['release_event'] = []

    def _resource_selected(self, item):
        self._lock.acquire()
        if not item in self.resource_item_list.keys():
            print "HAS NO KEY"
        else:
            self.current_resource = self.resource_item_list[item]
            if self.current_resource == self.current_captured_resource:
                self.release_resource_btn.setEnabled(True)
            else:
                self.release_resource_btn.setEnabled(False)
        self._lock.release()

    def _resource_double_clicked(self):
        if self.current_captured_resource == None:
            self._capture_resource()
        else:
            self._release_resource()

    def _capture_resource(self):
        if self.current_resource == None:
            print "No resource is selected"
        elif self.current_captured_resource:
            print "Already one robot has been captured"
        else:
            for callback in self._callback['capture_event']:
                callback(self.current_resource)

            self.setDisabled(True)

    def _release_resource(self):

        if self.current_captured_resource == None:
            print "No resource has been captured yet!"
        elif self.current_captured_resource != self.current_resource:
            print "cannot release not captured robot.(Captured: %s)" % self.current_captured_resource['name'].string
        else:
            for callback in self._callback['release_event']:
                callback(self.current_captured_resource)
            self.setDisabled(True)

    def capture_resource_callback(self, uri, msg):
        try:
            self.emit(SIGNAL("capture"), msg)
        except: 
            pass

    def release_resource_callback(self, uri, msg):
        try:
            self.emit(SIGNAL("release"), msg)
        except:
            pass

    def error_resource_callback(self, rtn):
        try:
            self.emit(SIGNAL("error"), rtn)
        except:
            pass
    
    def _show_capture_resource_message(self, rtn):
        if rtn:
            QMessageBox.warning(self, 'SUCCESS', "CAPTURE!!!!", QMessageBox.Ok | QMessageBox.Ok)
        else:
            QMessageBox.warning(self, 'FAIL', "FAIURE CAPTURE!!!!", QMessageBox.Ok | QMessageBox.Ok)
        self._lock.acquire()
        if rtn:
            for k in self.resource_item_list.keys():
                if self.resource_item_list[k] == self.current_resource:
                    k.setBackground(0, QBrush(Qt.SolidPattern))
                    k.setBackgroundColor(0, QColor(0, 255, 0, 255))
                    resource_name = k.text(0)
                    k.setText(0, str(resource_name) + " (captured)")
                else:
                    k.setBackground(0, QBrush(Qt.NoBrush))
                    k.setBackgroundColor(0, QColor(0, 0, 0, 0))
                    resource_name = k.text(0)
            self.capture_resource_btn.setEnabled(False)
            self.release_resource_btn.setEnabled(True)
            self.current_captured_resource = self.current_resource
        self.setDisabled(False)
        self._lock.release()

    def _show_release_resource_message(self, rtn):
        if rtn:
            QMessageBox.warning(self, 'SUCCESS', "RELEASE!!!!", QMessageBox.Ok | QMessageBox.Ok)
        else:
            QMessageBox.warning(self, 'FAIL', "FAIURE RELEASE!!!!", QMessageBox.Ok | QMessageBox.Ok)

        self._lock.acquire()
        if rtn:
            for k in self.resource_item_list.keys():
                if self.resource_item_list[k] == self.current_captured_resource:
                    k.setBackground(0, QBrush(Qt.NoBrush))
                    k.setBackgroundColor(0, QColor(0, 0, 0, 0))
                    resource_name= k.text(0)
                    k.setText(0, resource_name[:resource_name.find(" (captured)")])
        self.setDisabled(False)
        self.capture_resource_btn.setEnabled(True)
        self.release_resource_btn.setEnabled(False)
        self.current_captured_resource= None
        self._lock.release()

    def _show_error_resource_message(self, error_message):
        QMessageBox.warning(self, 'ERROR', error_message, QMessageBox.Ok | QMessageBox.Ok)
        self.setDisabled(False)
        self.capture_resource_btn.setEnabled(True)
        self.release_resource_btn.setEnabled(True)

    def refresh_resource_list_callback(self, resource_list):
        try:
            self.emit(SIGNAL("refresh_resource_list"), resource_list)
        except: 
            pass


    def _refresh_resource_list(self, resource_list):
        self._lock.acquire()
        self.resource_list_tree_widget.clear()
        self.resource_item_list = {}

        for r in resource_list:
            uri = rocon_uri.parse(r)
            resource_item = QTreeWidgetItem(self.resource_list_tree_widget)
            resource_item.setText(0, uri.name.string)
            self.resource_item_list[resource_item] = r
        self._lock.release()

    def shutdown(self):
        self._release_resource()
