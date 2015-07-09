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
import math
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtCore import Signal

import rospkg
import rospy
import sensor_msgs.msg as sensor_msgs
from rocon_qt_library.widgets import QResourceChooser
from rocon_qt_library.views import QCameraView
from rocon_qt_library.interfaces import ResourceChooserInterface

from qt_gui.plugin import Plugin

##############################################################################
# Image Stream App
##############################################################################

class ImageStream(Plugin):

    image_received = Signal(sensor_msgs.CompressedImage, name="image_received")

    def __init__(self, context):
        self._lock = threading.Lock()
        self._context = context
        super(ImageStream, self).__init__(context)
        # I'd like these to be also configurable via the gui
        self.initialised = False
        self.is_setting_dlg_live = False

        self._widget = QWidget()
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_qt_image_stream'), 'ui', 'concert_image_stream.ui')
        loadUi(ui_file, self._widget, {'QResourceChooser' : QResourceChooser, 'QCameraView' : QCameraView})
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        ##### Setting Resource Chooser Interface ####
        self._set_resource_chooser_interface()

        context.add_widget(self._widget)
        self.setObjectName('ImageStream')

        self._default_compressed_image_topic = '/image_stream/compressed_image'
        self.image_received.connect(self._widget.camera_view.on_compressed_image_received)

    def shutdown(self):
        with self._lock: 
            if self._image_stream_interface:
                self._image_stream_interface.shutdown()
                self._image_stream_interface = None

    def _set_resource_chooser_interface(self):
        capture_timeout = rospy.get_param('~capture_timeout', 15.0)
        service_name = rospy.get_param('~service_name')
        available_resource_topic = '/services/%s/available_image_stream'%service_name
        capture_resource_pair_topic = '/services/%s/capture_image_stream'%service_name
        capture_resource_callbacks = [self._widget.resource_chooser_widget.capture_resource_callback, self._init_image_stream_interface]
        release_resource_callbacks = [self._widget.resource_chooser_widget.release_resource_callback, self._uninit_image_stream_interface]
        error_resource_callbacks   = [self._widget.resource_chooser_widget.error_resource_callback]
        refresh_resource_list_callbacks = [self._widget.resource_chooser_widget.refresh_resource_list_callback]

        self._resource_chooser_interface = ResourceChooserInterface(capture_timeout, available_resource_topic, capture_resource_pair_topic, capture_resource_callbacks, release_resource_callbacks, error_resource_callbacks, refresh_resource_list_callbacks)

        capture_event_callbacks = [self._resource_chooser_interface.capture_resource]
        release_event_callbacks = [self._resource_chooser_interface.release_resource]
        self._widget.resource_chooser_widget.set_callbacks(capture_event_callbacks, release_event_callbacks)

    def _init_image_stream_interface(self, uri, msg):
        if msg.result:
            compressed_image_topic = self._get_remapped_topic(self._default_compressed_image_topic, msg.remappings)

            with self._lock:
                self._sub_image = rospy.Subscriber(compressed_image_topic, sensor_msgs.CompressedImage, self._process_image)

    def _get_remapped_topic(self, remap_from, remappings):
        for r in remappings:
            if r.remap_from == remap_from:
                return r.remap_to
        return remap_from

    def  _uninit_image_stream_interface(self, uri, msg):
        with self._lock:    
            self._sub_image.unregister()
            self._sub_image = None
            self._widget.camera_view.load_default_image()

    def _process_image(self, msg):
        self.image_received.emit(msg)

    def shutdown_plugin(self):
        self._widget.resource_chooser_widget.shutdown()
