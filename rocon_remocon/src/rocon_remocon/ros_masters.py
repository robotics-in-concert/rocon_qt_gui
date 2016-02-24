#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import rocon_console.console as console
import rosgraph
import socket
import threading
import time
import uuid


##############################################################################
# Methods
##############################################################################


def check_for_master(ros_master_uri=None):
    '''
    Simple check for a ros master's availability without having to initialise
    this process as a ros node.

    :param str ros_master_uri: accepts a url for the ros master, else resorts to looking up the env variable.
    :return pid of the master or None
    '''
    if ros_master_uri is None:
        try:
            ros_master_uri = os.environ["ROS_MASTER_URI"]
        except KeyError:
            print(console.red + "[ERROR] ROS_MASTER_URI not set....do try to give us something to work on!" + console.reset)
            return False
    caller_id = "/find_ros_master" + "_" + uuid.uuid4().hex
    rosgraph_master = rosgraph.Master(caller_id)
    try:
        return rosgraph_master.getPid()
    except socket.error:
        return None

##############################################################################
# Classes
##############################################################################


class RosMasterMonitor(object):
    '''
      Spins up a loop which continuously checks for the availability of
      the ros master.
    '''
    def __init__(self, period=5, callback_function=None, ros_master_uri=None):
        '''
        The callback should have the signature:

        void callback_function(bool new_master_found)

        :param int period: loop time for checking in seconds.
        :param func callback_function: execute user code if the state changed
        :param str ros_msater_uri: the url of the ros master to lookup (if None, checks env).
        '''
        if ros_master_uri is None:
            try:
                self._ros_master_uri = os.environ["ROS_MASTER_URI"]
            except KeyError:
                print(console.red + "[ERROR] ROS_MASTER_URI not set....do try to give us something to work on!" + console.reset)
                return
        else:
            self._ros_master_uri = ros_master_uri
        self._sleep_period_seconds = period
        self._callback_function = callback_function if callback_function is not None else self._default_callback_function
        # initialise it
        self.pid = check_for_master(self._ros_master_uri)
        self._callback_function(self.is_available())
        self._check_for_master_timer = threading.Timer(period, self._wait_for_master)
        self._check_for_master_timer.start()

    def is_available(self):
        return True if self.pid is not None else False

    def shutdown(self):
        self._check_for_master_timer.cancel()

    def _default_callback_function(self, new_master_found):
        pass

    def _wait_for_master(self):
        new_pid = check_for_master(self._ros_master_uri)
        if new_pid is None:
            if self.pid is not None:
                # TODO : check for a timeout here and use callback function with None
                self.pid = new_pid
                self._callback_function(self.is_available())
        else:
            if self.pid is None or self.pid != new_pid:
                self.pid = new_pid
                self._callback_function(self.is_available())
        # trigger a new check
        self._check_for_master_timer = threading.Timer(self._sleep_period_seconds, self._wait_for_master)
        self._check_for_master_timer.start()


##############################################################################
# Testies
##############################################################################

def _user_callback_test(available):
    if available:
        print(console.green + "Ros Master available at " + console.yellow + os.environ["ROS_MASTER_URI"] + console.green + "." + console.reset)
    else:
        print(console.red + "ROS Master unavailable" + console.reset)

if __name__ == '__main__':
    ros_master = RosMasterMonitor(period=1, callback_function=_user_callback_test)
    while(True):
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            ros_master.shutdown()
            break
