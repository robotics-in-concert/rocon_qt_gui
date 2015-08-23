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
from python_qt_binding.QtGui import QDialog, QCursor, QSpacerItem, QCheckBox

import rocon_std_msgs.msg as rocon_std_msgs
from rocon_qt_library.utils import show_message

from . import icon
from . import utils

##############################################################################
# Dialog
##############################################################################


class InteractionDialog(QDialog):

    def __init__(self, parent, interaction, start_interaction_hook, stop_interaction_hook, is_running=False):
        super(InteractionDialog, self).__init__(parent)

        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'interaction_dialog.ui')
        loadUi(path, self)
        self.interaction = interaction
        self._start_interaction_hook = start_interaction_hook
        self._stop_interaction_hook = stop_interaction_hook
        self._is_running = is_running
        self._remappings_items = []
        self._init_interaction_infos()
        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)
        self.adjustSize()  # resize to contents

    def _init_interaction_infos(self):
        self._init_overview()
        self._init_start_stop_buttons()
        self._init_remappings()
        self._init_parameters()

    def _init_overview(self):
        self.setWindowTitle(self.interaction.name)
        pixmap = icon.rocon_icon_to_qpixmap(self.interaction.icon)
        self.icon.setPixmap(pixmap)
        self.name.setText("<b>%s</b><br/><small>%s</small>" % (self.interaction.name, self.interaction.command))
        self.description.setText("<br/>" + self.interaction.description + "<br/><br/>")

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
        # if self.interaction.remappings:
        if False:
            row = 0
            for remapping in self.interaction.remappings:
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
            self.remappings_title.hide()

    def _init_parameters(self):
        # if self.interaction.parameters:
        if False:
            self.parameters.setText("<br/>" + self.interaction.parameters + "<br/><br/>")
        else:
            self.parameters.hide()
            self.parameters_title.hide()

    def _press_start_button(self):
        name, remappings, parameters = self._prepare_start_interaction()
        result = self._start_interaction_hook(name, remappings, parameters)

        if result.started:
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(True)
        else:
            show_message(self, str(result.started), result.message)

    def _press_stop_button(self):
        result = self._stop_interaction_hook()
        if result.stopped:
            self.start_button.setEnabled(True)
            self.stop_button.setEnabled(False)
        else:
            show_message(self, str(result.stopped), result.message)

    def _prepare_start_interaction(self):
        parameters = []
        for name, box in self._parameters_items:
            if isinstance(box, QCheckBox):
                value = "true" if box.isChecked() else "false"
            else:
                value = box.toPlainText().strip()
            parameters.append(rocon_std_msgs.KeyValue(name, value))
        remappings = [(k, v.toPlainText().strip()) for k, v in self._remappings_items if k != v.toPlainText().strip()]
        impl = self.interaction_impls.currentText()

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
