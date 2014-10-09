#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

from .converts import *
from .world_canvas_utils import *
from python_qt_binding.QtGui import QMessageBox

def show_message(parent, title, message):
    QMessageBox.warning(parent, str(title), str(message), QMessageBox.Ok | QMessageBox.Ok)
