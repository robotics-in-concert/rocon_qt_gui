#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from python_qt_binding.QtCore import QObject, Signal, pyqtSlot

import geometry_msgs.msg as geometry_msgs
import rospy
import sensor_msgs.msg as sensor_msgs
import threading

##############################################################################
# RobotInterface
##############################################################################

class VideoTeleopInterface(QObject):

    cmd_vel_publishing_interval = 0.1  # seconds

    __slots__ = [
                '_cmd_vel',                  # geometry_msgs.Twist
                '_cmd_vel_publisher_timer',
                '_cmd_vel_publisher',
                '_compressed_image_subscriber',
                '_lock'
                ]
    image_received = Signal(sensor_msgs.CompressedImage, name="image_received")

    def __init__(self,
                 image_received_slot,
                 cmd_vel_topic_name='cmd_vel',
                 compressed_image_topic_name='compressed_image'):
        """
        Initialise the robot interface with slot connections back to whatever
        qt views are attached and the ros topic names to be used.

        :param slot image_received_slot: slot to connect to the ``image_received`` signal.
        :param str cmd_vel_topic_name: topic name for the command velocity publisher.
        :param str compressed_image_topic_name: topic name for the compressed image subscriber.
        """
        self._lock = threading.Lock()
        self._cmd_vel = geometry_msgs.Twist()
        self._cmd_vel.linear.x = self._cmd_vel.angular.z = 0.0
        super(VideoTeleopInterface, self).__init__()
        self._cmd_vel_publisher = None
        self._compressed_image_subscriber = None
        if image_received_slot is not None:
            self.image_received.connect(image_received_slot)
        self._cmd_vel_publisher = rospy.Publisher(cmd_vel_topic_name, geometry_msgs.Twist, latch=True, queue_size=10)
        self._compressed_image_subscriber = rospy.Subscriber(compressed_image_topic_name, sensor_msgs.CompressedImage, self._ros_subscriber_image_callback)
        self._cmd_vel_publisher_timer = \
            rospy.Timer(rospy.Duration(VideoTeleopInterface.cmd_vel_publishing_interval),
                        self._publish_cmd_vel
                        )

    def _ros_subscriber_image_callback(self, msg):
        """
        Update the teleop image

        :param sensor_msgs.CompressedImage msg: an image stream feed from the robot
        """
        # todo convert the image here
        self.image_received.emit(msg)

    def _publish_cmd_vel(self, unused_event):
        """
        Convert the command into an appropriate one for the
        cmd_vel topic and publish.
        """
        if self._cmd_vel_publisher is not None:
            with self._lock:
                self._cmd_vel_publisher.publish(self._cmd_vel)

    def shutdown(self):
        """
        There is a problem in rospy here which spams you with a warning
        when the cmd_vel is unregistered. This is just noise I believe (it shouldn't
        be printing anything). The publisher gets unregistered properly. Should
        be safe to ignore (lots of work to actually follow up).
        """
        self._cmd_vel_publisher_timer.shutdown()
        with self._lock:
            if self._cmd_vel_publisher:
                self._cmd_vel_publisher.unregister()
            if self._compressed_image_subscriber:
                self._compressed_image_subscriber.unregister()
            self._cmd_vel_publisher = self._compressed_image_subscriber = None

    @property
    def cmd_vel(self):
        """
        Returns this cmd_vel in a 2-tuple appropriate for 2d teleop'ing robots,
        i.e. (linear, angular).

        :returns: the last configured cmd_vel as a 2-tuple.
        :rtype: (float, float)
        """
        return (self._cmd_vel.linear.x, self._cmd_vel.angular.z)

    @cmd_vel.setter
    def cmd_vel(self, value):
        """
        Convenient setter from a 2-tuple. The 2-tuple takes the form (linear, angular)
        with units in (m/s, rad/s).

        Usage:

        .. code-block:: python

           teleop_interaface.cmd_vel = (1.0, 0.35)  # linear x, angular z values)
        """
        with self._lock:
            (self._cmd_vel.linear.x, self._cmd_vel.angular.z) = value
