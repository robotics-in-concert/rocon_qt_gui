#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from functools import partial
import os
import rocon_launch
import rocon_python_utils
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
#         """
#         This code is some complicated kill code I also use in the turtle herders to
#         kill of Popened rocon_launch instances. Just sending it SIGINT signals doesn't
#         work - that will kill the roslaunch's inside, but doesn't kill the konsoles
#         (why?).
# 
#         If the code settles down, might be good to centralise this code in rocon_launch.
#         """
#         #self.process.send_signal(signal.SIGINT)
#         console.loginfo("Shutting down interaction with pid: %s" % self.process.pid)
#         roslaunch_pids = rocon_launch.get_roslaunch_pids(self.process.pid)
#         console.logdebug("  roslaunch pids: %s" % roslaunch_pids)
#         # The os.kills aren't required if a concert is doing a full shutdown since
#         # it disperses signals everywhere anyway. This is important if we implement the
#         # disable from inside the remocon though
#         for pid in roslaunch_pids:
#             try:
#                 os.kill(pid, signal.SIGINT)  # sighup or sigint? I can't remember - this is same as rocon_launch code
#             except OSError:
#                 continue
#         for pid in roslaunch_pids:
#             console.logdebug("  waiting on roslaunch pid %s" % pid)
#             result = rocon_python_utils.system.wait_pid(pid)
#             console.logdebug("  pid %s exited with result %s" % (pid, result))
# #         time.sleep(1)  # Do we need this?
#         console.logdebug("  now killing the terminal %s" % self.process.pid)
#         try:
#             os.killpg(self.process.pid, signal.SIGTERM)
#             #self.process.terminate()
#         except OSError:
#             console.logdebug("  process already died naturally")
#             pass
#         for temporary_file in self.temporary_files:
#             console.logdebug("  unlinking %s" % temporary_file.name)
#             try:
#                 os.unlink(temporary_file.name)
#             except OSError:
#                 pass
