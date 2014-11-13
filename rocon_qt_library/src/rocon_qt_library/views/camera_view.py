#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division

from python_qt_binding.QtCore import Qt, SIGNAL, pyqtSlot, QRectF  # QPointF, QRectF, 
from python_qt_binding.QtGui import QGraphicsView, QGraphicsScene, QPixmap

import os
import rospkg
import sensor_msgs.msg as sensor_msgs

##############################################################################
# Class
##############################################################################


class QCameraView(QGraphicsView):
    """
    Accepts an image of a teleop compressed image type and draws that in the
    scene/view.
    """

    def __init__(self, parent=None):
        super(QCameraView, self).__init__(parent)
        self.scene = QGraphicsScene(self)
        self.setScene(self.scene)
        self._load_default_image()

        self.connect(self, SIGNAL("load_default_image"), self._load_default_image)

    def _load_default_image(self):
        joystick_icon = os.path.join(rospkg.RosPack().get_path('rocon_bubble_icons'), 'icons', 'joystick.png')
        pixmap = QPixmap(joystick_icon, format="png")
        if self.scene:
            self.scene.addPixmap(pixmap)
            self.scene.update()
            self.fitInView(QRectF(0, 0, self.scene.width(), self.scene.height()), Qt.KeepAspectRatio)

    def load_default_image(self): 
        self.emit(SIGNAL("load_default_image"))
        
    @pyqtSlot(sensor_msgs.CompressedImage, name='image_received')
    def on_compressed_image_received(self, image):
        '''
        :param sensor_msgs.CompressedImage image: convert and display this in the QGraphicsView.
        '''
        if len(self.scene.items()) > 1:
            self.scene.removeItem(self.scene.items()[0])
        pixmap = QPixmap()
        pixmap.loadFromData(image.data, format=image.format)
        self.scene.addPixmap(pixmap)
        self.scene.update()
        # setting fitInvView seems sensitive to here or prior to scene update
        self.fitInView(QRectF(0, 0, self.scene.width(), self.scene.height()), Qt.KeepAspectRatio)

    def resizeEvent(self, evt=None):
        self.fitInView(QRectF(0, 0, self.scene.width(), self.scene.height()), Qt.KeepAspectRatio)
