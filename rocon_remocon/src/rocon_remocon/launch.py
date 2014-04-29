#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

##############################################################################
# Classes
##############################################################################


class LaunchInfo(object):
    __slots__ = [
        'name',      # unique name for this launch
        'running',   # running or not
        'process',   # the actual process handle (e.g. subprocess return)
        'shutdown',  # shutdown
    ]

    def __init__(self, name, running, process, shutdown):
        """
        :param name str:
        :param running bool:
        :param process rocon_python_utils.system.Popen:
        :param shutdown function:
        """
        self.name = name
        self.running = running
        self.process = process
        self.shutdown = shutdown
