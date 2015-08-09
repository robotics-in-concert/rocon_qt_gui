#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################

#system
from __future__ import division

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QIcon, QPixmap, QStandardItem, QHBoxLayout, QLabel, QTextEdit, QSizePolicy, QFont, QCheckBox

############################################
# Rapp item
############################################
class QRappItem(QStandardItem):
    def __init__(self, rapp, running):
        QStandardItem.__init__(self, rapp['display_name'])
        self.setSizeHint(QSize(100,100))
        icon = get_qicon(rapp['icon'])
        self.setIcon(icon)
        f = QFont()
        f.setPointSize(10)
        self.setFont(f)
        self.setToolTip(rapp['description'])
        self.setEditable(False)
        self.setRapp(rapp)
        self.setEnabled(running)

    def compare_name(self, display_name, name):
        if self._rapp['display_name'] == display_name and self._rapp['name'] == name:
            return True
        else:
            return False
        
    def setRapp(self, rapp):
        self._rapp = rapp
    def getRapp(self):
        return self._rapp

##############################################################################
# Utils
##############################################################################
def get_qicon(icon):
    pixmap = QPixmap()
    pixmap.loadFromData(icon.data, format=icon.format)
    return QIcon(pixmap)

def get_qpixmap(icon):
    pixmap = QPixmap()
    pixmap.loadFromData(icon.data, format=icon.format)
    return pixmap

def create_label_textedit_pair(key, value):
    '''
        Probabaly there should be better way to lay out param and remappings
    '''
    #param_layout = QHBoxLayout()
    name = QLabel(key)
    name.setToolTip(key)
    name.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
    name.setMinimumWidth(400)
    name.setMaximumHeight(30)
    name.setWordWrap(True)

    textedit = QTextEdit()
    textedit.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
    textedit.setMinimumWidth(320)
    textedit.setMaximumHeight(30)
    textedit.append(str(value))
    return name, textedit


def create_label_checkbox_pair(key, value):
    label = QLabel(key)
    label.setToolTip(key)
    label.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Preferred)
    label.setMinimumWidth(400)
    label.setMaximumHeight(30)
    label.setWordWrap(True)

    checkbox = QCheckBox()
    checkbox.setChecked(value)
    return label, checkbox


def create_label(name, is_bold=False):
    qname = QLabel(name)
    f = QFont()
    f.setBold(is_bold)
    qname.setFont(f)
    return qname 
