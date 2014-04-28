#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import string
import uuid

import rocon_console.console as console

from . import utils

##############################################################################
# Methods
##############################################################################


def rocon_masters_cache_path():
    return os.path.join(utils.get_settings_cache_home(), "rocon_master.cache")

##############################################################################
# Classes
##############################################################################


class RoconMaster(object):
    __slots__ = [
        'index',
        'name',
        'uri',
        'host_name',
        'description',
        'icon',
        'flag',
        'current_row',
    ]

    def __str__(self):
        s = self.name
        return s


class RoconMasters(object):

    __slots__ = [
        'rocon_masters',  # { uuid : RoconMaster }
    ]

    def __init__(self):
        self.rocon_masters = {}
        self.load()

    ######################################
    # Emulating a dict
    ######################################

    def __contains__(self, index):
        return index in self.rocon_masters.keys()

    def __getitem__(self, index):
        return self.rocon_masters[index]

    def __len__(self):
        return len(self.rocon_masters)

    def keys(self):
        return self.rocon_masters.keys()

    def values(self):
        return self.rocon_masters.values()

    def clear(self):
        self.rocon_masters.clear()

    ######################################
    # Introspection
    ######################################

    def __str__(self):
        s = console.cyan + 'Rocon Masters:' + console.reset
        for rocon_master in self.rocon_masters.values():
            s += '\n  ' + console.yellow + str(rocon_master) + console.reset
        return s

    ######################################
    # Utility Functions
    ######################################

    def delete(self, index):
        del self.rocon_masters[index]

    def add(self, uri, host_name):
        rocon_master = RoconMaster()
        rocon_master.index = str(uuid.uuid4())
        rocon_master.name = "Unknown"
        rocon_master.uri = uri
        rocon_master.host_name = host_name
        rocon_master.icon = "unknown.png"
        rocon_master.description = ""
        rocon_master.flag = "0"
        self.rocon_masters[rocon_master.index] = rocon_master
        return rocon_master

    ######################################
    # Caching
    ######################################

    def load(self):
        """
        Gets a list of rocon masters and their info from cached data stored upon
        last exit of this application.
        """
        #read cache and display the rocon master list
        self.rocon_masters = {}
        try:
            cache_rocon_master_info_list = open(rocon_masters_cache_path(), 'r')
        except:
            console.logdebug("Remocon : no cached settings found, moving on.")
            return
        lines = cache_rocon_master_info_list.readlines()
        for line in lines:
            if line.count("[index="):
                rocon_master = RoconMaster()
                rocon_master.index = line[string.find(line, "[index=") + len("[index="):string.find(line, ",name=")]
                rocon_master.name = line[string.find(line, "name=") + len("name="):string.find(line, ",master_uri=")]
                rocon_master.uri = line[string.find(line, ",master_uri=") + len(",master_uri="):string.find(line, ",host_name=")]
                rocon_master.host_name = line[string.find(line, ",host_name=") + len(",host_name="):string.find(line, ",description=")]
                rocon_master.description = line[string.find(line, ",description=") + len(",description="):string.find(line, ",icon=")]
                rocon_master.icon = line[string.find(line, ",icon=") + len(",icon="):string.find(line, ",flag=")]
                rocon_master.flag = line[string.find(line, ",flag=") + len(",flag="):string.find(line, "]")]
                self.rocon_masters[rocon_master.index] = rocon_master
        cache_rocon_master_info_list.close()

    def dump(self):
        """
        Dump the rocon masters to a cache file.
        """
        try:
            cache_rocon_master_info_list = open(rocon_masters_cache_path(), 'w')
        except:
            console.logerror("Remocon : no directory or file: %s" % rocon_masters_cache_path())
            return
        for rocon_master in self.rocon_masters.values():
            rocon_master_elem = '['
            rocon_master_elem += 'index=' + str(rocon_master.index) + ','
            rocon_master_elem += 'name=' + str(rocon_master.name) + ','
            rocon_master_elem += 'master_uri=' + str(rocon_master.uri) + ','
            rocon_master_elem += 'host_name=' + str(rocon_master.host_name) + ','
            rocon_master_elem += 'description=' + str(rocon_master.description) + ','
            rocon_master_elem += 'icon=' + rocon_master.icon + ','
            rocon_master_elem += 'flag=' + rocon_master.flag
            rocon_master_elem += ']\n'

            cache_rocon_master_info_list.write(rocon_master_elem)
        cache_rocon_master_info_list.close()
