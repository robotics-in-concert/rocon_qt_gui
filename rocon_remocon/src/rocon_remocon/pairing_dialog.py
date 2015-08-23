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
import rocon_interaction_msgs.msg as interaction_msgs
import rocon_std_msgs.msg as rocon_std_msgs
from rocon_qt_library.utils import show_message
import rospkg

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QEvent, SIGNAL
from python_qt_binding.QtGui import QDialog, QCursor, QSpacerItem, QCheckBox


from . import icon
from . import utils

##############################################################################
# Dialog
##############################################################################


class PairingDialog(QDialog):

    def __init__(self, parent, pairing, start_pairing_hook, stop_pairing_hook, is_running=False):
        super(PairingDialog, self).__init__(parent)

        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'pairing_dialog.ui')
        loadUi(path, self)
        self.pairing = pairing
        self._start_pairing_hook = start_pairing_hook
        self._stop_pairing_hook = stop_pairing_hook
        self._is_running = is_running
        self._parameters_items = []
        self._remappings_items = []
        self._init_pairing_infos()
        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)
        self.adjustSize()  # resize to contents

    def _init_pairing_infos(self):
        self._init_overview()
        self._init_start_stop_buttons()
        self._init_parameters()
        self._init_remappings()

    def _init_overview(self):
        self.setWindowTitle(self.pairing.name)
        pixmap = icon.rocon_icon_to_qpixmap(self.pairing.icon)
        self.pairing_icon.setPixmap(pixmap)
        self.pairing_name.setText("<b>%s</b><br/><small>%s</small>" % (self.pairing.name, self.pairing.rapp))
        self.pairing_description.setText("<br/>" + self.pairing.description + "<br/><br/>")

    def _init_start_stop_buttons(self):
        if self._is_running:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        else:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
        self.start_button.pressed.connect(self._press_start_button)
        self.stop_button.pressed.connect(self._press_stop_button)

    def _init_remappings(self):
        # if self.pairing.remappings:
        if False:
            row = 0
            for remapping in self.pairing.remappings:
                l = utils.create_label(remapping.remap_from, is_bold=False)
                l.setAlignment(Qt.AlignRight)
                self.remappings.addWidget(l, row, 0)
                # self.remappings.addItem(QSpacerItem(20, 40))
                l = utils.create_label(" -> ", is_bold=False)
                l.setAlignment(Qt.AlignCenter)
                self.remappings.addWidget(l, row, 1)
                l = utils.create_label(remapping.remap_to, is_bold=False)
                l.setAlignment(Qt.AlignLeft)
                self.remappings.addWidget(l, row, 2)
                row += 1
        else:
            self.frame_remappings.hide()
            self.pairing_remappings_title.hide()

    def _init_parameters(self):
        # if len(self.pairing.parameters) > 0:
        if False:
            self.parameters.setColumnStretch(1, 0)
            self.parameters.setRowStretch(2, 0)
            for p in self.pairing.parameters:
                if isinstance(p.value, bool):
                    label, box = utils.create_label_checkbox_pair(p.key, p.value)
                else:
                    label, box = utils.create_label_textedit_pair(p.key, p.value)
                self.parameters.addWidget(label)
                self.parameters.addWidget(box)
                self._parameters_items.append((p.key, box))
        else:
            self.frame_parameters.hide()
            self.pairing_parameters_title.hide()

    def _press_start_button(self):
        response = self._start_pairing_hook(self.pairing.name)
        if response.result == interaction_msgs.ErrorCodes.SUCCESS:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        else:
            show_message(self, str(response.result), response.message)

    def _press_stop_button(self):
        response = self._stop_pairing_hook(self.pairing.name)
        if response.result == interaction_msgs.ErrorCodes.SUCCESS:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
        else:
            show_message(self, str(response.result), response.message)

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
