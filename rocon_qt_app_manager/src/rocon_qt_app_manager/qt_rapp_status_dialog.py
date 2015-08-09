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

import rospkg
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QEvent, SIGNAL
from python_qt_binding.QtGui import QDialog, QCursor, QSpacerItem

import rocon_std_msgs.msg as rocon_std_msgs
from rocon_qt_library.utils import show_message
from . import utils
from PyQt4.Qt import QCheckBox

##############################################################################
# Dialog
##############################################################################


class QtRappDialog(QDialog):

    def __init__(self, parent, rapp, start_rapp_hook, stop_rapp_hook, is_running=False):
        super(QtRappDialog, self).__init__(parent)

        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_qt_app_manager'), 'ui', 'app_dialog.ui')
        loadUi(path, self)
        self._rapp = rapp
        self._start_rapp_hook = start_rapp_hook
        self._stop_rapp_hook = stop_rapp_hook
        self._is_running = is_running

        self._parameters_items = []
        self._remappings_items = []

        self._init_rapp_infos()

        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)

    def _init_rapp_infos(self):
        self._init_overview()
        self._init_start_stop_buttons()
        self._init_implementations()
        self._init_public_parameters()
        self._init_public_interface()

    def _init_overview(self):
        self.setWindowTitle(self._rapp['display_name'])

        pixmap = utils.get_qpixmap(self._rapp['icon'])
        self.rapp_icon.setPixmap(pixmap)

        self.rapp_name.setText("%s (%s)" % (self._rapp['display_name'], self._rapp['name']))
        self.rapp_description.setText(self._rapp['description'])

    def _init_start_stop_buttons(self):
        if self._is_running:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        else:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)

        self.start_button.pressed.connect(self._press_start_button)
        self.stop_button.pressed.connect(self._press_stop_button)

    def _init_implementations(self):
        for impl in self._rapp['implementations']:
            self.rapp_impls.addItem(impl)

        idx = self.rapp_impls.findText(self._rapp['preferred'])

        if idx >= 0:
            self.rapp_impls.setCurrentIndex(idx)

    def _init_public_parameters(self):
        if len(self._rapp['public_parameters']) > 0:
            self.parameters.setColumnStretch(1, 0)
            self.parameters.setRowStretch(2, 0)
            for p in self._rapp['public_parameters']:
                if isinstance(p.value, bool):
                    label, box = utils.create_label_checkbox_pair(p.key, p.value)
                else:
                    label, box = utils.create_label_textedit_pair(p.key, p.value)
                self.parameters.addWidget(label)
                self.parameters.addWidget(box)
                self._parameters_items.append((p.key, box))
        else:
            label = utils.create_label("No public parameters")
            self.parameters.addWidget(label)

    def _init_public_interface(self):
        flag = False
        if len(self._rapp['public_interface']):
            self.remappings.setColumnStretch(1, 0)
            self.remappings.setRowStretch(2, 0)

            for i in self._rapp['public_interface']:
                type_name = i.key
                remap_list = eval(i.value)

                if len(remap_list) > 0:
                    qname = utils.create_label(type_name, is_bold=True)
                    self.remappings.addWidget(qname)
                    self.remappings.addItem(QSpacerItem(20, 40))
                    for remap in remap_list:
                        key = remap['name']
                        remap_type = remap['type']
                        n = "%s  [%s]" % (key, remap_type)
                        name, textedit = utils.create_label_textedit_pair(n, key)
                        self.remappings.addWidget(name)
                        self.remappings.addWidget(textedit)
                        self._remappings_items.append((key, textedit))
                        flag = True

        if not flag:
            label = utils.create_label("No public interface")
            self.remappings.addWidget(label)

    def _press_start_button(self):
        name, remappings, parameters = self._prepare_start_rapp()
        result = self._start_rapp_hook(name, remappings, parameters)

        if result.started:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        else:
            show_message(self, str(result.started), result.message)

    def _press_stop_button(self):
        result = self._stop_rapp_hook()
        if result.stopped:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
        else:
            show_message(self, str(result.stopped), result.message)

    def _prepare_start_rapp(self):
        parameters = []
        for name, box in self._parameters_items:
            if isinstance(box, QCheckBox):
                value = "true" if box.isChecked() else "false"
            else:
                value = box.toPlainText().strip()
            parameters.append(rocon_std_msgs.KeyValue(name, value))
        remappings = [(k, v.toPlainText().strip()) for k, v in self._remappings_items if k != v.toPlainText().strip()]
        impl = self.rapp_impls.currentText()

        return impl, remappings, parameters

    def showEvent(self, event):
        geom = self.frameGeometry()
        geom.moveTopLeft(QCursor.pos())
        self.setGeometry(geom)
        super(QDialog, self).showEvent(event)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.WindowDeactivate:
            self.close()
            return True
        return False
