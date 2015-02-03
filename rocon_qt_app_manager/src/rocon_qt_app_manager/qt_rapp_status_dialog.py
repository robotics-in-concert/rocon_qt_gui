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
from python_qt_binding.QtCore import Qt, QEvent
from python_qt_binding.QtGui import QDialog, QCursor, QSpacerItem

from .utils import get_qpixmap, create_label_textedit_pair, create_label

class QtRappDialog(QDialog):

    def __init__(self, parent, rapp):
        super(QtRappDialog, self).__init__(parent)

        rospack = rospkg.RosPack()
        path = os.path.join(rospack.get_path('rocon_qt_app_manager'),'ui','app_dialog.ui')
        loadUi(path, self)
        self._rapp = rapp

        self._init_rapp_infos()

        self.setFocusPolicy(Qt.StrongFocus)
        self.installEventFilter(self)


    def _init_rapp_infos(self):
        self.setWindowTitle(self._rapp['display_name'])
    
        pixmap= get_qpixmap(self._rapp['icon']) 
        self.rapp_icon.setPixmap(pixmap)

        self.rapp_name.setText("%s(%s)"%(self._rapp['display_name'], self._rapp['name']))
        self.rapp_description.setText(self._rapp['description'])

        for impl in self._rapp['implementations']:
            self.rapp_impls.addItem(impl)

        if len(self._rapp['public_parameters']) > 0:
            self.parameters.setColumnStretch(1,0)
            self.parameters.setRowStretch(2,0)
            for p in self._rapp['public_parameters']:
                name, textedit= create_label_textedit_pair(p.key, p.value)
                self.parameters.addWidget(name)
                self.parameters.addWidget(textedit)
        else:
            label = create_label("No public parameters")
            self.parameters.addWidget(label)


        flag = False
        if len(self._rapp['public_interface']):
            self.remappings.setColumnStretch(1,0)
            self.remappings.setRowStretch(2,0)

            for i in self._rapp['public_interface']:
                type_name = i.key 
                remap_list = eval(i.value)

                if len(remap_list) > 0:
                    qname = create_label(type_name, is_bold=True)                
                    self.remappings.addWidget(qname)
                    self.remappings.addItem(QSpacerItem(20,40))
                    for remap in remap_list:
                        key = remap['name']
                        remap_type = remap['type']
                        n = "%s  [%s]"%(key, remap_type)
                        name, textedit = create_label_textedit_pair(n, key)
                        self.remappings.addWidget(name)
                        self.remappings.addWidget(textedit)
                        flag= True

        if not flag:
            label = create_label("No public interface")
            self.remappings.addWidget(label)

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

