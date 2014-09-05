# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import numpy
import random
from math import sqrt, atan, pi, degrees

import rospy
import tf
from tf.transformations import quaternion_from_euler

import nav_msgs.msg as nav_msgs

from python_qt_binding.QtCore import Signal, QObject, pyqtSlot

class SlamInterface(QObject):

    map_received = Signal(nav_msgs.OccupancyGrid, name='map_received')

    def __init__(self, map_received_slot=None, map_topic='map', polygons=['move_base/local_costmap/obstacle_layer_footprint/footprint_stamped'], _tf=None):
        super(SlamInterface, self).__init__()

        if map_received_slot is not None:
            self.map_received.connect(map_received_slot)
        self.destroyed.connect(self.close)

        if _tf is None:
            self._tf = tf.TransformListener()
        else:
            self._tf = _tf
        self.sub_map = rospy.Subscriber(map_topic, nav_msgs.OccupancyGrid, self.map_cb)


    def map_cb(self, msg):
        """
        Update the map 
        
        :param nav_msgs.OccupancyGrid msg: a map from system
        """
        self.map_received.emit(msg)

    def close(self):
        if self.map_sub:
            self.map_sub.unregister()
        for p in self._paths.itervalues():
            if p.sub:
                p.sub.unregister()

        for p in self._polygons.itervalues():
            if p.sub:
                p.sub.unregister()

        super(SlamInterface, self).close()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be saved
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO add any settings to be restored
        pass
