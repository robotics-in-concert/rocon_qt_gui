#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import functools
import os
import rocon_console.console as console
import rocon_interactions
import rocon_interaction_msgs.msg as interaction_msgs
import rocon_interaction_msgs.srv as interaction_srvs
import rocon_launch
import rocon_python_comms
import rocon_python_utils
import rocon_std_msgs.msg as rocon_std_msgs
import rocon_uri
import rosgraph
import rospkg
import rospy
import sys
import urllib
import uuid
import yaml

from python_qt_binding.QtCore import QObject, Signal

from . import launch
from . import utils
from .launched_interactions import LaunchedInteractions

##############################################################################
# Methods
##############################################################################


def get_namespaces():
    """
    Get the name space where an interactions manager is running.
    @return name space
    @rtype str
    """
    try:
        service_names = rocon_python_comms.find_service('rocon_interaction_msgs/GetInteractions',
                                                        timeout=rospy.rostime.Duration(5.0),
                                                        unique=False
                                                        )
    except rocon_python_comms.NotFoundException:
        rospy.logerr("Remocon : failed to find an interactions manager, aborting.")
        sys.exit(1)
    return [rosgraph.names.namespace(service_name) for service_name in service_names]


def get_pairings(interactions_namespace):
    if interactions_namespace is None:
        return rocon_interactions.PairingsTable()
    get_pairings = rospy.ServiceProxy(interactions_namespace + "get_pairings", interaction_srvs.GetPairings)
    request = interaction_srvs.GetPairingsRequest()
    response = get_pairings(request)
    pairings_table = rocon_interactions.PairingsTable()
    pairings_table.load(response.pairings)
    return pairings_table


def get_interactions(interactions_namespace, platform_rocon_uri):
    if interactions_namespace is None:
        return rocon_interactions.InteractionsTable()
    get_interactions = rospy.ServiceProxy(interactions_namespace + "get_interactions", interaction_srvs.GetInteractions)
    request = interaction_srvs.GetInteractionsRequest(groups=[], uri=platform_rocon_uri)
    response = get_interactions(request)
    interactions_table = rocon_interactions.InteractionsTable()
    interactions_table.load(response.interactions)
    return interactions_table


##############################################################################
# Interactions Manager
##############################################################################


class InteractionsRemocon(QObject):

    # PySide signals are always defined as class attributes (GPL Pyqt4 Signals use pyqtSignal)
    signal_updated = Signal()

    def __init__(self):
        super(InteractionsRemocon, self).__init__()
        self.key = uuid.uuid4()
        self.name = "rqt_remocon_" + self.key.hex
        self.namespaces = get_namespaces()
        self.active_namespace = None if not self.namespaces else self.namespaces[0]
        self.rocon_uri = rocon_uri.parse(
            rocon_uri.generate_platform_rocon_uri('pc', self.name) + "|" + utils.get_web_browser_codename()
        )
        self.active_pairing = None
        self.active_paired_interaction_hashes = []
        self.launched_interactions = LaunchedInteractions()

        # be also great to have a configurable icon...with a default
        self.platform_info = rocon_std_msgs.MasterInfo(version=rocon_std_msgs.Strings.ROCON_VERSION,
                                                       rocon_uri=str(self.rocon_uri),
                                                       icon=rocon_std_msgs.Icon(),
                                                       description=""
                                                       )

        # Load Data
        self.pairings_table = get_pairings(self.active_namespace)
        self.interactions_table = get_interactions(self.active_namespace, "rocon:/")

        self.service_proxies = rocon_python_comms.utils.ServiceProxies(
            [
                (self.active_namespace + "request_interaction", interaction_srvs.RequestInteraction),
                (self.active_namespace + "start_pairing", interaction_srvs.StartPairing),
                (self.active_namespace + "stop_pairing", interaction_srvs.StopPairing)
            ]
        )
        self.remocon_status_publisher = rospy.Publisher("/remocons/" + self.name, interaction_msgs.RemoconStatus, latch=True, queue_size=5)
        self.subscribers = rocon_python_comms.utils.Subscribers(
            [
                (self.active_namespace + "pairing_status", interaction_msgs.PairingStatus, self._subscribe_pairing_status_callback)
            ]
        )
        self._publish_remocon_status()

    def connect(self, refresh_slot):
        self.signal_updated.connect(refresh_slot)

    def start_pairing(self, name):
        request = interaction_srvs.StartPairingRequest(name)
        response = self.service_proxies.start_pairing(request)
        return response

    def stop_pairing(self, name):
        request = interaction_srvs.StopPairingRequest(name)
        response = self.service_proxies.stop_pairing(request)
        return response

    def start_interaction(self, interaction_hash):
        interaction = self.interactions_table.find(interaction_hash)
        if interaction is None:
            console.logerror("Couldn't find interaction with hash '%s'" % interaction_hash)
            return (False, "interaction key %s not found in interactions table" % interaction_hash)
        console.logdebug("Requesting interaction '%s'" % interaction.name)
        response = self.service_proxies.request_interaction(remocon=self.name, hash=interaction_hash)
        if response.result == interaction_msgs.ErrorCodes.SUCCESS:
            console.logdebug("  request granted")
            try:
                (app_executable, start_app_handler) = self._determine_interaction_type(interaction)
            except rocon_interactions.InvalidInteraction as e:
                return (False, ("invalid interaction specified [%s]" % str(e)))
            result = start_app_handler(interaction, app_executable)
            if result:
                if interaction.is_paired_type():
                    self.active_paired_interaction_hashes.append(interaction_hash)
                self.signal_updated.emit()
                self._publish_remocon_status()
                return (result, "success")
            else:
                return (result, "unknown")
        else:
            console.logwarn("  request rejected [%s]" % response.message)
            return False, ("interaction request rejected [%s]" % response.message)

    def stop_interaction(self, interaction_hash):
        """
        This stops all launches for an interaction of a particular type.
        """
        interaction = self.interactions_table.find(interaction_hash)
        if interaction is None:
            console.logerror("Couldn't find interaction with hash '%s'" % interaction_hash)
            return (False, "interaction key %s not found in interactions table" % interaction_hash)
        console.loginfo("Stopping all instances of '%s'" % interaction.name)
        try:
            for unused_launch_name, launch_info in self.launched_interactions.get_launch_details(interaction.hash).iteritems():
                if launch_info.running:
                    launch_info.shutdown()
                    console.loginfo("  instance stopped [%s]" % (launch_info.name))
                elif launch_info.process is None:
                    launch_info.running = False
                    console.loginfo("  no attached interaction process to stop [%s]" % (launch_info.name))
                else:
                    console.loginfo("  instance is already stopped [%s]" % (launch_info.name))
            self.launched_interactions.clear_launch_details(interaction.hash)
        except Exception as e:
            console.logerror("Error trying to stop all instances of an interaction [%s][%s]" % (type(e), str(e)))
            # this is bad...should not create bottomless exception buckets.
            return (False, "unknown failure - (%s)(%s)" % (type(e), str(e)))
        if interaction.is_paired_type():
            self.active_paired_interaction_hashes = [h for h in self.active_paired_interaction_hashes if h != interaction_hash]
        self.signal_updated.emit()
        self._publish_remocon_status()
        return (True, "success")

    def stop_all_interactions(self):
        """
        This is the big showstopper - stop them all!
        """
        console.loginfo("Stopping all interactions")
        for interaction_hash in self.launched_interactions.active():
            self.stop_interaction(interaction_hash)

    ##############################################################################
    # Execution
    ##############################################################################

    def _determine_interaction_type(self, interaction):
        '''
          Classifies the interaction based on the name string and some intelligent
          (well, reasonably) parsing of that string.
           - paired dummy (by empty name)
           - ros launcher (by .launch extension)
           - ros runnable (by roslib find_resource success)
           - web app      (by web_interactions.parse)
           - web url      (by web_interactions.parse)
           - global executable (fallback option)
        '''
        # pairing trigger (i.e. dummy interaction)
        if not interaction.command:
            console.logdebug("Interactive Client : start a dummy interaction for triggering a pair")
            return ('', self._start_dummy_interaction)
        # roslaunch
        try:
            launcher_filename = rocon_python_utils.ros.find_resource_from_string(interaction.command, extension='launch')
            if interaction.remappings:
                raise rocon_interactions.InvalidInteraction("remappings are not yet enabled for roslaunchable interactions (workaround: try remapping via interaction parameters and roslaunch args)[%s]" % interaction.command)
            console.logdebug("Starting a roslaunchable interaction [%s]" % interaction.command)
            return (launcher_filename, self._start_roslaunch_interaction)
        except (rospkg.ResourceNotFound, ValueError):
            unused_filename, extension = os.path.splitext(interaction.command)
            if extension == '.launch':
                raise rocon_interactions.InvalidInteraction("could not find %s on the filesystem" % interaction.command)
            else:
                pass
        # rosrun
        try:
            rosrunnable_filename = rocon_python_utils.ros.find_resource_from_string(interaction.command)
            console.logdebug("Starting a rosrunnable interaction [%s]" % interaction.command)
            return (rosrunnable_filename, self._start_rosrunnable_interaction)
        except rospkg.ResourceNotFound:
            pass
        except Exception:
            pass
        # web url/app
        web_interaction = rocon_interactions.web_interactions.parse(interaction.command)
        if web_interaction is not None:
            if web_interaction.is_web_url():
                console.logdebug("Starting a weburl interaction [%s]" % web_interaction.url)
                return (web_interaction.url, self._start_weburl_interaction)
            elif web_interaction.is_web_app():
                console.logdebug("Starting a webapp interaction [%s]" % web_interaction.url)
                return (web_interaction.url, self._start_webapp_interaction)
        # executable
        if rocon_python_utils.system.which(interaction.command) is not None:
            console.logdebug("Starting a global executable interaction [%s]")
            return (interaction.command, self._start_global_executable_interaction)
        else:
            raise rocon_interactions.InvalidInteraction("could not find a valid rosrunnable or global executable for '%s' (mispelt, not installed?)" % interaction.command)

    def _start_dummy_interaction(self, interaction, unused_filename):
        console.logdebug("Starting a dummy interaction [%s]" % interaction.command)
        anonymous_name = interaction.name.lower().replace(' ', '_') + "_" + uuid.uuid4().hex
        # process_listener = partial(self._process_listeners, anonymous_name, 1)
        # process = rocon_python_utils.system.Popen([rosrunnable_filename], postexec_fn=process_listener)
        self.launched_interactions.add(interaction.hash,
                                       anonymous_name,
                                       launch.LaunchInfo(anonymous_name, True, None)
                                       )  # empty shutdown function
        return True

    def _start_roslaunch_interaction(self, interaction, roslaunch_filename):
        '''
          Start a ros launchable application, applying parameters and remappings if specified.
        '''
        anonymous_name = interaction.name.lower().replace(" ", "_") + "_" + uuid.uuid4().hex
        launch_configuration = rocon_launch.RosLaunchConfiguration(
            name=roslaunch_filename,
            package=None,
            port=self._ros_master_port,
            title=interaction.name,
            namespace=interaction.namespace,
            args=self._prepare_roslaunch_args(interaction.parameters),
            options="--screen"
        )
        process_listener = functools.partial(self._process_listeners, anonymous_name, 1)
        (process, meta_roslauncher) = self._roslaunch_terminal.spawn_roslaunch_window(launch_configuration, postexec_fn=process_listener)
        self.launched_interactions.add(
            interaction.hash,
            anonymous_name,
            launch.RosLaunchInfo(anonymous_name, True,
                                 process,
                                 self._roslaunch_terminal.shutdown_roslaunch_windows,
                                 [meta_roslauncher]
                                 )
        )
        return True

    def _start_rosrunnable_interaction(self, interaction, rosrunnable_filename):
        '''
          Launch a rosrunnable application. This does not apply any parameters
          or remappings (yet).
        '''
        # the following is guaranteed since we came back from find_resource calls earlier
        # note we're overriding the rosrunnable filename here - rosrun doesn't actually take the full path.
        package_name, rosrunnable_filename = interaction.command.split('/')
        name = os.path.basename(rosrunnable_filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = functools.partial(self._process_listeners, anonymous_name, 1)
        cmd = ['rosrun', package_name, rosrunnable_filename, '__name:=%s' % anonymous_name]
        remapping_args = []
        for remap in interaction.remappings:
            remapping_args.append(remap.remap_from + ":=" + remap.remap_to)
        cmd.extend(remapping_args)
        cmd.extend(self._prepare_command_line_parameters(interaction.parameters))
        console.logdebug("  rosrunnable command %s" % cmd)
        process = rocon_python_utils.system.Popen(cmd, postexec_fn=process_listener)
        self.launched_interactions.add(
            interaction.hash,
            anonymous_name,
            launch.LaunchInfo(anonymous_name, True, process)
        )
        return True

    def _start_global_executable_interaction(self, interaction, filename):
        console.logwarn("Interactive Client : starting global executable [%s]" % interaction.command)
        name = os.path.basename(filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = functools.partial(self._process_listeners, anonymous_name, 1)
        cmd = [filename]
        remapping_args = []
        for remap in interaction.remappings:
            remapping_args.append(remap.remap_from + ":=" + remap.remap_to)
        cmd.extend(remapping_args)
        cmd.extend(self._prepare_command_line_parameters(interaction.parameters))
        console.logdebug("Interactive Client : global executable command %s" % cmd)
        process = rocon_python_utils.system.Popen(cmd, postexec_fn=process_listener)
        self.launched_interactions.add(
            interaction.hash,
            anonymous_name,
            launch.LaunchInfo(anonymous_name, True, process)
        )
        return True

    def _start_weburl_interaction(self, interaction, url):
        """
        We only need the url here and then do a system check for a web browser.
        """
        web_browser = utils.get_web_browser()
        if web_browser is not None:
            name = os.path.basename(web_browser).replace('.', '_')
            anonymous_name = name + "_" + uuid.uuid4().hex
            process_listener = functools.partial(self._process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([web_browser, "--new-window", url], postexec_fn=process_listener)
            self.launched_interactions.add(
                interaction.hash,
                anonymous_name,
                launch.LaunchInfo(anonymous_name, True, process)
            )
            return True
        else:
            return False

    def _start_webapp_interaction(self, interaction, base_url):
        """
        Need to work out the extended url (with args, parameters and remappings) here and then feed that to a
        detected browser.

        :param base_url str: the web app url without all of the attached variables.
        """
        web_browser = utils.get_web_browser()
        if web_browser is not None:
            url = self._prepare_webapp_url(interaction, base_url)
            name = os.path.basename(web_browser).replace('.', '_')
            anonymous_name = name + "_" + uuid.uuid4().hex
            process_listener = functools.partial(self._process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([web_browser, "--new-window", url], postexec_fn=process_listener)
            self.launched_interactions.add(
                interaction.hash,
                anonymous_name,
                launch.LaunchInfo(anonymous_name, True, process)
            )
            return True
        else:
            return False

    def _prepare_webapp_url(self, interaction, base_url):
        """
           url synthesiser for sending remappings and parameters information.
           We convert the interaction parameter (yaml string) and remapping (rocon_std_msgs.Remapping[])
           variables into generic python list/dictionary objects and convert these into
           json strings as it makes it easier for web apps to handle them.
        """
        interaction_data = {}
        interaction_data['name'] = interaction.name
        # parameters
        interaction_data['parameters'] = yaml.load(interaction.parameters)
        # remappings
        interaction_data['remappings'] = {}  # need to create a dictionary for easy parsing (note: interaction.remappings is a list of rocon_std_msgs.Remapping)
        for r in interaction.remappings:
            interaction_data['remappings'][r.remap_from] = r.remap_to
        # package all the data in json format and dump it to one query string variable
        console.logdebug("Remocon Info : web app query string %s" % interaction_data)
        query_string_mappings = {}
        query_string_mappings['interaction_data'] = json.dumps(interaction_data)
        # constructing the url
        return base_url + "?" + urllib.urlencode(query_string_mappings)

    def _prepare_command_line_parameters(self, interaction_parameters):
        """
        Convert the interaction specified yaml string into command line parameters that can
        be passed to rosrunnable or global nodes.

        :param str interaction_parameters: parameters specified as a yaml string

        :returns: the parameters as command line args
        :rtype: str[]
        """
        parameters = []
        parameter_dictionary = yaml.load(interaction_parameters)  # convert from yaml string into python dictionary
        if parameter_dictionary is not None:  # None when there is no yaml configuration
            for name, value in parameter_dictionary.items():
                if type(value) is types.DictType or type(value) is types.ListType:
                    parameters.append('_' + name + ':=' + yaml.dump(value))
                else:  # it's a dict or list, so dump it
                    parameters.append('_' + name + ':=' + str(value))
        return parameters

    def _prepare_roslaunch_args(self, interaction_parameters):
        """
        Convert the interaction specified yaml string into roslaunch args
        to be passed to the roslaunchable. Note that we only use a constrained
        subset of yaml to be compatible with roslaunch args here.
        The root type has to be a dict and values themselves
        may not be dicts or lists.

        :param str interaction_parameters: parameters specified as a yaml string

        :returns: the parameters as roslaunch args key-value pairs
        :rtype: list of (name, value) pairs
        """
        args = []
        parameters = yaml.load(interaction_parameters)  # convert from yaml string into python dictionary
        if parameters is not None:
            if type(parameters) is types.DictType:
                for name, value in parameters.items():
                    if type(value) is types.DictType or type(value) is types.ListType:
                        console.logwarn("Ignoring invalid parameter for roslaunch arg (simple key-value pairs only) [%s][%s]" % (name, value))
                    else:
                        args.append((name, value))
            else:
                console.logwarn("Ignoring invalid parameters for roslaunch args (must be a simple key-value dict) [%s]" % parameters)
        return args

    def _process_listeners(self, name, exit_code):
        '''
          This is usually run as a post-executing function to the interaction and so will do the
          cleanup when the user's side of the interaction has terminated.

          Note that there are other means of stopping & cleanup for interactions:

          - via the remocon stop buttons (self.stop_interaction, self.stop_all_interactions)
          - via a rapp manager status callback when it is a pairing interaction

          There is some common code (namely del launched_interactions element, check pairing, publish remocon) so
          if changing that flow, be sure to check the code in self.stop_interaction()

          @param str name : name of the launched process stored in the interactions launch_list dict.
          @param int exit_code : could be utilised from roslaunched processes but not currently used.
        '''
        terminated = False
        for interaction in self.interactions_table.interactions:
            if self.launched_interactions.remove(interaction.hash, name):
                # toggle the pairing indicator if it was a pairing interaction
                if interaction.is_paired_type() and interaction.hash in self.active_paired_interaction_hashes:
                    self.active_paired_interaction_hashes = [interaction_hash for interaction_hash in self.active_paired_interaction_hashes if interaction_hash != interaction.hash]
                if not self.launched_interactions.get_launch_details(interaction.hash):
                    # inform the gui to update
                    self.signal_updated.emit()
                # update the rocon interactions handler
                self._publish_remocon_status()
                terminated = True
                break
        if not terminated:
            console.logwarn("A process listener detected a terminating interaction, but nothing to do.")
            console.logwarn("Probably mopped up by gui or independently terminating pairing requirement. [%s]" % name)
        else:
            console.logdebug("A process listener detected a terminating interaction & mopped up appropriately [%s]" % name)

    ##############################################################################
    # ROS API
    ##############################################################################

    def _subscribe_pairing_status_callback(self, msg):
        if msg.active_pairing:
            self.active_pairing = self.pairings_table.find(msg.active_pairing)
            print("Pairing status updated: %s" % self.active_pairing.name)
        else:
            print("Pairing status updated: none")
            for active_interaction_hash in self.active_paired_interaction_hashes:
                self.stop_interaction(active_interaction_hash)
            self.active_pairing = None
        self.signal_updated.emit()

    def _publish_remocon_status(self):
        remocon_status = interaction_msgs.RemoconStatus()
        remocon_status.platform_info = self.platform_info
        remocon_status.uuid = str(self.key.hex)
        remocon_status.version = rocon_std_msgs.Strings.ROCON_VERSION
        running_interactions = []
        for interaction_hash in self.launched_interactions.active():
            running_interactions.append(interaction_hash)
        remocon_status.running_interactions = running_interactions
        console.logdebug("Remocon : publishing remocon status")
        self.remocon_status_publisher.publish(remocon_status)