#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#

#
# Imports
#

from __future__ import division
import os
import math
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget

import rospkg
import rospy
from rocon_qt_library.widgets import QMapAnnotation
from rocon_qt_library.interfaces import MapAnnotationInterface

from qt_gui.plugin import Plugin

#
# Map Annotation App
#


class MapAnnotation(Plugin):

    def __init__(self, context):
        self._lock = threading.Lock()
        self._context = context
        super(MapAnnotation, self).__init__(context)
        # I'd like these to be also configurable via the gui
        self.initialised = False
        self.is_setting_dlg_live = False

        self._widget = QWidget()
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_qt_map_annotation'), 'ui', 'concert_map_annotation.ui')

        loadUi(ui_file, self._widget, {'QMapAnnotation': QMapAnnotation})
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._default_map_topic = 'map'
        self._default_viz_markers_topic = 'viz_markers'
        self._default_save_annotation_srv = 'save_annotation'

        context.add_widget(self._widget)
        self.setObjectName('Map Annotation')

        # Setting Interface ####
        self._set_map_annotation_interface()

    def _set_map_annotation_interface(self):
        # remappinags
        map_topic = self._default_map_topic
        viz_markers_topic = "/annotation/" + self._default_viz_markers_topic

        save_annotation_srv = self._default_save_annotation_srv

        viz_markers_slot = self._widget.map_annotation.draw_viz_markers
        scene_slot = self._widget.map_annotation.draw_scene

        self._widget.map_annotation.init_map_annotation_interface(
            map_topic=map_topic,
            viz_markers_received_slot=viz_markers_slot,
            viz_markers_topic=viz_markers_topic,
            scene_update_slot=scene_slot)

    def _get_remaps(self, remap_from, remappings):
        for r in remappings:
            if r.remap_from == remap_from:
                return r.remap_to
        return remap_from
