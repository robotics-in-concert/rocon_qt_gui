#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division
import os

from python_qt_binding import loadUi

# QtCore
#    SIGNAL, SLOT,  pyqtSlot, pyqtSignal
#    QString, QStringList,
#    QPoint, QEvent, QFile, QIODevice,
#    QAbstractListModel
#from python_qt_binding.QtCore import Qt, QAbstractListModel
# QtGui
#    QMainWindow,
#    QCheckBox, QComboBox,
#    QTextEdit, QCompleter,
#    QFileDialog,
#    QPushButton
#    QGraphicsScene, QImage, QPainter, QBrush, QColor, QPen, QPixmap
from python_qt_binding.QtGui import QWidget, QPixmap
# QtSvg
#    QSvgGenerator

from qt_gui.plugin import Plugin

import rospkg
# import concert_service_info
import concert_service_utilities

##############################################################################
# Plugin
##############################################################################
class ServiceInfo(Plugin):
    def __init__(self, context):
        super(ServiceInfo, self).__init__(context)
        self.initialised = False
        self.setObjectName('ServiceInfo')
        self._current_dotcode = None
        counter = 0
        service_info = concert_service_utilities.service_information.get_services_info()
        dict = service_info[1] #dict = {}
        nameList = service_info[0] #nameList = []

        # Create Widget 
        self._widget = QWidget()
        self._widget.setObjectName('ConcertServiceInfoUi')
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_qt_service_info'), 'ui', 'concert_service_info.ui')
        loadUi(ui_file, self._widget)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        for value in nameList:
            info_text = "<html>"
            info_text += "<p><b>Resource Name: </b>" + dict[value].resource_name + "</p>"
            info_text += "<p><b>Name: </b>" + dict[value].name + "</p>"
            info_text += "<p><b>Description: </b>" + dict[value].description + "</p>"
            info_text += "<p><b>Author: </b>" + dict[value].author + "</p>"
            info_text += "<p><b>Priority: </b>" + str(dict[value].priority) + "</p>"
            info_text += "<p><b>Launcher: </b>" + dict[value].launcher_type + "</p>"
            info_text += "<p><b>Status: </b>" + str(dict[value].status) + "</p>"
            info_text += "<p><b>Enabled: </b>" + str(dict[value].enabled) + "</p>"
            info_text += "<p><b>Icon: </b>" + str(dict[value].icon.resource_name) + "</p>"
            info_text += "</html>"

            if counter == 0:
                self._widget.app_info.appendHtml(info_text)
                pixmap2 = QPixmap()
                pixmap2.loadFromData(dict[value].icon.data, format=dict[value].icon.format)
                if QPixmap.isNull(pixmap2):
                    icon_file2 = os.path.join(rospack.get_path('concert_qt_service_info'), 'resources', 'images', 'unknown.png')
                    pixmap2 = QPixmap(icon_file2, format="png")
                    self._widget.app_icon2.setPixmap(pixmap2)
                    self._widget.app_icon2.resize(pixmap2.width(), pixmap2.height())
                if not QPixmap.isNull(pixmap2):
                    self._widget.app_icon2.setPixmap(pixmap2)
                    self._widget.app_icon2.resize(pixmap2.width(), pixmap2.height())
            if counter == 1:
                self._widget.app_info2.appendHtml(info_text)
                pixmap = QPixmap()
                pixmap.loadFromData(dict[value].icon.data, format=dict[value].icon.format)
                if QPixmap.isNull(pixmap):
                    icon_file = os.path.join(rospack.get_path('concert_qt_service_info'), 'resources', 'images', 'unknown.png')
                    pixmap = QPixmap(icon_file, format="png")
                    self._widget.app_icon.setPixmap(pixmap)
                    self._widget.app_icon.resize(pixmap.width(), pixmap.height())   
                if not QPixmap.isNull(pixmap):
                    self._widget.app_icon.setPixmap(pixmap)
                    self._widget.app_icon.resize(pixmap.width(), pixmap.height())
            counter = counter + 1 

        context.add_widget(self._widget)

    def shutdown_plugin(self):
        pass
