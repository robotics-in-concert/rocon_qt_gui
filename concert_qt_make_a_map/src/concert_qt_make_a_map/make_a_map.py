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
from rocon_qt_library.widgets import QResourceChooser, QVideoTeleop, QSlamWidget
from rocon_qt_library.interfaces import ResourceChooserInterface, SlamWidgetInterface

from qt_gui.plugin import Plugin

#
# Make a Map App
#


class MakeAMap(Plugin):

    def __init__(self, context):
        self._lock = threading.Lock()
        self._context = context
        super(MakeAMap, self).__init__(context)
        # I'd like these to be also configurable via the gui
        self.initialised = False
        self.is_setting_dlg_live = False

        self._widget = QWidget()
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('concert_qt_make_a_map'), 'ui', 'concert_make_a_map.ui')
        loadUi(ui_file, self._widget, {'QResourceChooser': QResourceChooser, 'QVideoTeleop': QVideoTeleop, 'QSlamWidget': QSlamWidget})
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Setting Resource Chooser Interface ####
        self._set_resource_chooser_interface()

        context.add_widget(self._widget)
        self.setObjectName('Make a Map')

        self._default_cmd_vel_topic = '/teleop/cmd_vel'
        self._default_compressed_image_topic = '/teleop/compressed_image'
        self._default_map_topic = 'map'
        self._default_scan_topic = '/make_a_map/scan'
        self._default_robot_pose = 'robot_pose'
        self._default_wc_namespace_param = 'wc_namespace'
        self._default_wc_namespace = 'world_canvas'

    def _set_resource_chooser_interface(self):
        capture_timeout = rospy.get_param('~capture_timeout', 15.0)
        service_name = rospy.get_param('~service_name')

        # TODO : Make it configurable
        available_resource_topic = '/services/%s/available_make_a_map'%service_name
        capture_resource_pair_topic = '/services/%s/capture_make_a_map'%service_name
        capture_resource_callbacks = [self._widget.resource_chooser_widget.capture_resource_callback, self._init_teleop_interface, self._set_slam_view_interface]
        release_resource_callbacks = [self._widget.resource_chooser_widget.release_resource_callback, self._uninit_teleop_interface]
        error_resource_callbacks = [self._widget.resource_chooser_widget.error_resource_callback]
        refresh_resource_list_callbacks = [self._widget.resource_chooser_widget.refresh_resource_list_callback]

        self._resource_chooser_interface = ResourceChooserInterface(capture_timeout, available_resource_topic, capture_resource_pair_topic, capture_resource_callbacks, release_resource_callbacks, error_resource_callbacks, refresh_resource_list_callbacks)

        capture_event_callbacks = [self._resource_chooser_interface.capture_resource]
        release_event_callbacks = [self._resource_chooser_interface.release_resource, self._unset_slam_interface]
        self._widget.resource_chooser_widget.set_callbacks(capture_event_callbacks, release_event_callbacks)

    def _init_teleop_interface(self, uri, msg):
        if msg.result:
            cmd_vel_topic = self._get_remaps(self._default_cmd_vel_topic, msg.remappings)
            compressed_image_topic = self._get_remaps(self._default_compressed_image_topic, msg.remappings)

            with self._lock:
                self._widget.video_teleop_widget.init_teleop_interface(cmd_vel_topic_name=cmd_vel_topic, compressed_image_topic_name=compressed_image_topic)

    def _get_remaps(self, remap_from, remappings):
        for r in remappings:
            if r.remap_from == remap_from:
                return r.remap_to
        return remap_from

    def _uninit_teleop_interface(self, uri, msg):
        if msg.result:
            with self._lock:
                self._widget.video_teleop_widget.reset()

    def _unset_slam_interface(self, uri):
        self._widget.slam_widget.unset_slam_interface()

    def _set_slam_view_interface(self, uri, msg):
        if msg.result:
            scan_slot = self._widget.slam_widget.draw_scan
            robot_pose_slot = self._widget.slam_widget.draw_robot_pose

            wc_namespace_param = rospy.get_param('~wc_namespace_param')
            wc_namespace = rospy.get_param(wc_namespace_param, self._default_wc_namespace)
            map_topic = self._get_remaps(self._default_map_topic , msg.remappings)
            scan_topic = self._get_remaps(self._default_scan_topic, msg.remappings)
            robot_pose_topic = self._get_remaps(self._default_robot_pose, msg.remappings)
            map_saved_callbacks = [self._widget.slam_widget.map_saved_callback]
            self._widget.slam_widget.init_slam_widget_interface(map_topic=map_topic,
                                                                scan_received_slot=scan_slot,
                                                                scan_topic=scan_topic,
                                                                robot_pose_received_slot=robot_pose_slot,
                                                                robot_pose_topic=robot_pose_topic,
                                                                wc_namespace=wc_namespace,
                                                                map_saved_callbacks=map_saved_callbacks)

    def shutdown_plugin(self):
        self._widget.slam_widget.unset_slam_interface()
        self._widget.video_teleop_widget.reset()
        self._widget.resource_chooser_widget.shutdown()
