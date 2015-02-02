#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################

#system
from __future__ import division

from python_qt_binding.QtCore import QSize
from python_qt_binding.QtGui import QIcon, QPixmap, QStandardItem, QHBoxLayout, QLabel, QTextEdit, QSizePolicy

############################################
# Rapp item
############################################
class QRappItem(QStandardItem):
    def __init__(self, rapp):
        QStandardItem.__init__(self, rapp['display_name'])
        self.setSizeHint(QSize(100,100))
        icon = get_qicon(rapp['icon'])
        self.setIcon(icon)
        self.setToolTip(rapp['description'])
        self.setEditable(False)
        self.setRapp(rapp)
        
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
    param_layout = QHBoxLayout()
    name = QLabel(key)
    name.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
    name.setMinimumSize(40,30)

    textedit = QTextEdit() 
    textedit.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Ignored)
    textedit.setMinimumSize(0,30)
    textedit.append(str(value))
    param_layout.addWidget(name)
    param_layout.addWidget(textedit)
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
