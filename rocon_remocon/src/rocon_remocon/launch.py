#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import signal
import rocon_console.console as console

##############################################################################
# Classes
##############################################################################


class LaunchInfo(object):
    __slots__ = [
        'name',      # unique name for this launch
        'running',   # running or not
        'process',   # the actual process handle (e.g. subprocess return)
    ]

    def __init__(self, name, running, process):
        """
        :param str name:
        :param bool running:
        :param rocon_python_utils.system.Popen process:
        """
        self.name = name
        self.running = running
        self.process = process

    def shutdown(self):
        self.process.send_signal(signal.SIGINT)


class RosLaunchInfo(LaunchInfo):
    __slots__ = [
        'temporary_files',          # list of temporary files that need to be unlinked
        '_terminal_shutdown_hook',  # handle to a rocon_launch.terminals.Terminal class' shutdown method
    ]

    def __init__(self, name, running, process, terminal_shutdown_hook, temporary_files=[]):
        """
        :param str name:
        :param bool running:
        :param rocon_python_utils.system.Popen process:
        :param func terminal_shutdown_hook: function to call on shutdown to kill the roslaunched terminal window
        :param tempfile.NamedTemporaryFile[] temporary_files: files that need to be unlinked later.
        """
        super(RosLaunchInfo, self).__init__(name, running, process)
        self.temporary_files = temporary_files
        self._terminal_shutdown_hook = terminal_shutdown_hook
        #self._shutdown = partial(roslaunch_terminal.shutdown_roslaunch_windows, [self.process], False)  # hold = False

    def shutdown(self):
        self._terminal_shutdown_hook(processes=[self.process], hold=False)
        for temporary_file in self.temporary_files:
            console.logdebug("  unlinking %s" % temporary_file.name)
            try:
                os.unlink(temporary_file.name)
            except OSError:
                pass
