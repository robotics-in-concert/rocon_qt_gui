#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import os
import uuid
from functools import partial
import signal
import tempfile
import urllib
import yaml
import json

import rospy
import rocon_python_utils
import roslaunch.parent
import rospkg
from rospkg.os_detect import OsDetect
import rocon_uri
from rocon_console import console
import rocon_std_msgs.msg as rocon_std_msgs
from rocon_app_manager_msgs.msg import ErrorCodes
import rocon_interaction_msgs.msg as rocon_interaction_msgs
import rocon_interaction_msgs.srv as rocon_interaction_srvs
import rocon_interactions.web_interactions as web_interactions
import rocon_python_comms
import rocon_interactions

from . import utils
from .launch import LaunchInfo

##############################################################################
# InteractiveClient
##############################################################################


class InteractiveClient():

    def __init__(self, stop_interaction_postexec_fn):
        '''
          @param stop_app_postexec_fn : callback to fire when a listener detects an app getting stopped.
          @type method with no args
        '''
        self._stop_interaction_postexec_fn = stop_interaction_postexec_fn
        self.interactions = {}
        self.is_connect = False
        self.key = uuid.uuid4()

        # this might be naive and only work well on ubuntu...
        os_codename = OsDetect().get_codename()
        # this would be good as a persistant variable so the user can set something like 'Bob'
        self.name = "rqt_remocon_" + self.key.hex
        self.rocon_uri = rocon_uri.parse(
                            "rocon:/pc/" + self.name + "/" + rocon_std_msgs.Strings.URI_WILDCARD + "/" + os_codename
                            )
        # be also great to have a configurable icon...with a default
        self.platform_info = rocon_std_msgs.PlatformInfo(version=rocon_std_msgs.Strings.ROCON_VERSION,
                                                       uri=str(self.rocon_uri),
                                                       icon=rocon_std_msgs.Icon()
                                                       )
        console.logdebug("InteractiveClient : initialised")
        self.pairing = None  # if an interaction is currently pairing, this will store its hash

    def _connect(self, rocon_master_name="", ros_master_uri="http://localhost:11311", host_name='localhost'):

        # uri is obtained from the user, stored in ros_master_uri
        os.environ["ROS_MASTER_URI"] = ros_master_uri
        os.environ["ROS_HOSTNAME"] = host_name

        console.logdebug("InteractiveClient : Connection Details")
        console.logdebug("InteractiveClient :   Node Name: " + self.name)
        console.logdebug("InteractiveClient :   ROS_MASTER_URI: " + ros_master_uri)
        console.logdebug("InteractiveClient :   ROS_HOSTNAME: " + host_name)

        # Need to make sure we give init a unique node name and we need a unique uuid
        # for the remocon-role manager interaction anyway:
        rospy.init_node(self.name, disable_signals=True)
        try:
            get_interactions_service_name = rocon_python_comms.find_service('rocon_interaction_msgs/GetInteractions', timeout=rospy.rostime.Duration(5.0), unique=True)
            get_roles_service_name = rocon_python_comms.find_service('rocon_interaction_msgs/GetRoles', timeout=rospy.rostime.Duration(5.0), unique=True)
            request_interaction_service_name = rocon_python_comms.find_service('rocon_interaction_msgs/RequestInteraction', timeout=rospy.rostime.Duration(5.0), unique=True)
        except rocon_python_comms.NotFoundException as e:
            message = "failed to find all of the interactions' publications and services [%s]" % str(e)
            console.logerror("InteractiveClient : %s" % message)
            return (False, message)

        self.get_interactions_service_proxy = rospy.ServiceProxy(get_interactions_service_name, rocon_interaction_srvs.GetInteractions)
        self.get_roles_service_proxy = rospy.ServiceProxy(get_roles_service_name, rocon_interaction_srvs.GetRoles)
        self.request_interaction_service_proxy = rospy.ServiceProxy(request_interaction_service_name, rocon_interaction_srvs.RequestInteraction)
        self.remocon_status_pub = rospy.Publisher("remocons/" + self.name, rocon_interaction_msgs.RemoconStatus, latch=True)

        try:
            # if its available, should be quick to find this one since we found the others...
            pairing_topic_name = rocon_python_comms.find_topic('rocon_interaction_msgs/Pair', timeout=rospy.rostime.Duration(0.5), unique=True)
            self.pairing_status_subscriber = rospy.Subscriber(pairing_topic_name, rocon_interaction_msgs.Pair, self._subscribe_pairing_status_callback)
        except rocon_python_comms.NotFoundException as e:
            console.logdebug("InteractiveClient : support for paired interactions disabled [not found]")

        self._publish_remocon_status()
        self.is_connect = True
        return (True, "success")

    def _disconnect(self):
        if(self.is_connect != True):
            self.shutdown()

    def _is_shutdown(self):
        print "[remocon_info] shut down is complete"

    def shutdown(self):
        if(self.is_connect != True):
            return
        else:
            console.logdebug("InteractiveClient : shutting down all rapps")
            for app_hash in self.interactions.keys():
                self.stop_interaction(app_hash)

            self.interactions = {}
            self.is_connect = False

            rospy.signal_shutdown("shut down remocon_info")
            while not rospy.is_shutdown():
                rospy.rostime.wallsleep(0.1)

            self.is_connect = False
            self.remocon_status_pub.unregister()

            self.remocon_status_pub = None
            console.logdebug("InteractiveClient : has shutdown.")

    def get_role_list(self):
        if not self.is_connect:
            rospy.logwarn("InteractiveClient : aborting a request to 'get_roles' as we are not connected to a rocon interactions manager.")
            return []
        try:
            response = self.get_roles_service_proxy(self.platform_info.uri)
        except (rospy.ROSInterruptException, rospy.ServiceException):
            return []
        return response.roles

    def _select_role(self, role_name):
        roles = []
        roles.append(role_name)

        call_result = self.get_interactions_service_proxy(roles, self.platform_info.uri)
        console.logdebug("InteractiveClient : call result")
        self.interactions = {}
        for interaction in call_result.interactions:
            if(interaction.role == role_name):
                # TODO should create a class out of this mash, dicts of dicts are prone to mistakes and hard to introspect.
                #if self.interactions.has_key(interaction.hash):
                #    pass
                #else:
                #    self.interactions[interaction.hash]={}
                #    self.interactions[interaction.hash]['launch_list'] ={}
                self.interactions[interaction.hash] = {}
                self.interactions[interaction.hash]['launch_list'] = {}

                self.interactions[interaction.hash]['name'] = interaction.name
                self.interactions[interaction.hash]['compatibility'] = interaction.compatibility
                icon_name = interaction.icon.resource_name.split('/').pop()
                if interaction.icon.data:
                    icon = open(os.path.join(utils.get_icon_cache_home(), icon_name), 'w')
                    icon.write(interaction.icon.data)
                    icon.close()
                self.interactions[interaction.hash]['icon'] = icon_name
                self.interactions[interaction.hash]['display_name'] = interaction.display_name
                self.interactions[interaction.hash]['description'] = interaction.description
                self.interactions[interaction.hash]['namespace'] = interaction.namespace
                self.interactions[interaction.hash]['max'] = interaction.max
                self.interactions[interaction.hash]['remappings'] = interaction.remappings
                self.interactions[interaction.hash]['parameters'] = interaction.parameters
                self.interactions[interaction.hash]['hash'] = interaction.hash
                self.interactions[interaction.hash]['pairing'] = interaction.pairing.rapp if interaction.pairing.rapp else None

    def start_interaction(self, interaction_hash):
        """
        :returns: result of the effort to start an interaction, with a message if there was an error.
        :rtype: (bool, message)
        """
        if not interaction_hash in self.interactions.keys():
            return (False, "interaction key %s not found in interactions table" % interaction_hash)
        interaction = self.interactions[interaction_hash]
        if self.pairing and interaction['pairing']:
            paired_interaction_display_name = self.interactions[self.pairing]['display_name']
            return (False, "remocon already pairing (%s,%s) and additional pairing is not permitted " % (interaction['pairing'], paired_interaction_display_name))

        #get the permission
        call_result = self.request_interaction_service_proxy(remocon=self.name, hash=interaction['hash'])

        if call_result.error_code == ErrorCodes.SUCCESS:
            console.logdebug("InteractiveClient : interaction request granted")
            try:
                (app_executable, start_app_handler) = self._determine_interaction_type(interaction['name'])
            except rocon_interactions.InvalidInteraction as e:
                return False, ("invalid interaction specified [%s]" % str(e))
            result = start_app_handler(interaction, app_executable)
            if result:
                self._publish_remocon_status()
                if interaction['pairing']:
                    self.pairing = interaction['hash']
                return (result, "success")
            else:
                return (result, "unknown")
        else:
            return False, ("interaction request rejected [%s]" % call_result.message)
        return (True, "success")

    def _determine_interaction_type(self, interaction_name):
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
        if not interaction_name:
            console.logdebug("InteractiveClient : start a dummy interaction for triggering a pair [%s]")
            return ('', self._start_dummy_interaction)
        # roslaunch
        try:
            launcher_filename = rocon_python_utils.ros.find_resource_from_string(interaction_name, extension='launch')
            console.logdebug("InteractiveClient : regular start app [%s]")
            return (launcher_filename, self._start_roslaunch_interaction)
        except (rospkg.ResourceNotFound, ValueError):
            unused_filename, extension = os.path.splitext(interaction_name)
            if extension == '.launch':
                raise rocon_interactions.InvalidInteraction("could not find %s on the filesystem" % interaction_name)
            else:
                pass
        # rosrun
        try:
            rosrunnable_filename = rocon_python_utils.ros.find_resource_from_string(interaction_name)
            console.logdebug("InteractiveClient : start_app_rosrunnable [%s]")
            return (rosrunnable_filename, self._start_rosrunnable_interaction)
        except rospkg.ResourceNotFound:
            pass
        except Exception:
            pass
        # web url/app
        web_interaction = web_interactions.parse(interaction_name)
        if web_interaction is not None:
            if web_interaction.is_web_url():
                console.logdebug("InteractiveClient : _start_weburl_interaction [%s]" % web_interaction.url)
                return (web_interaction.url, self._start_weburl_interaction)
            elif web_interaction.is_web_app():
                console.logdebug("InteractiveClient : _start_webapp_interaction [%s]" % web_interaction.url)
                return (web_interaction.url, self._start_webapp_interaction)
        # executable
        if rocon_python_utils.system.which(interaction_name) is not None:
            console.logdebug("InteractiveClient : _start_global_executable_interaction [%s]")
            return (interaction_name, self._start_global_executable_interaction)
        else:
            raise rocon_interactions.InvalidInteraction("could not find a valid rosrunnable or global executable for '%s' (mispelt, not installed?)" % interaction_name)

    def _start_dummy_interaction(self, interaction, unused_filename):
        console.loginfo("InteractiveClient : starting paired dummy interaction")
        anonymous_name = interaction['name'] + "_" + uuid.uuid4().hex
        #process_listener = partial(self._process_listeners, anonymous_name, 1)
        #process = rocon_python_utils.system.Popen([rosrunnable_filename], postexec_fn=process_listener)
        interaction['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, None, lambda: None)  # empty shutdown function
        return True

    def _start_roslaunch_interaction(self, interaction, roslaunch_filename):
        '''
          Start a ros launchable application, applying parameters and remappings if specified.
        '''
        name_space = '/service/' + interaction['namespace']
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        #add parameters
        launch_text  = '<launch>\n'  #@IgnorePep8 noqa
        launch_text += '    <group ns="%s">\n' % (name_space)
        launch_text += '        <rosparam>%s</rosparam>\n' % (interaction['parameters'])
        launch_text += '        <include file="%s"/>\n' % (roslaunch_filename)
        launch_text += '    </group>\n'
        launch_text += '</launch>\n'
        temp.write(launch_text)
        temp.close()  # unlink it later
        self.listener = roslaunch.pmon.ProcessListener()
        self.listener.process_died = self._process_listeners
        try:
            _launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),
                                                            [temp.name],
                                                            is_core=False,
                                                            process_listeners=[self.listener])
            _launch._load_config()
            for N in _launch.config.nodes:
                for k in interaction['remappings']:
                    N.remap_args.append([(name_space + '/' + k.remap_from).replace('//', '/'), k.remap_to])
            _launch.start()

            process_name = str(_launch.pm.get_process_names_with_spawn_count()[0][0][0])
            interaction['launch_list'][process_name] = LaunchInfo(process_name, True, _launch, _launch.shutdown)
            return True
        except Exception, inst:
            print inst
            interaction['running'] = str(False)
            print "Fail to launch: %s" % (interaction['name'])
            return False

    def _start_rosrunnable_interaction(self, interaction, rosrunnable_filename):
        '''
          Launch a rosrunnable application. This does not apply any parameters
          or remappings (yet).
        '''
        # the following is guaranteed since we came back from find_resource calls earlier
        # note we're overriding the rosrunnable filename here - rosrun doesn't actually take the full path.
        package_name, rosrunnable_filename = interaction['name'].split('/')
        name = os.path.basename(rosrunnable_filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = partial(self._process_listeners, anonymous_name, 1)
        command_args = ['rosrun', package_name, rosrunnable_filename, '__name:=%s' % anonymous_name]
        remapping_args = []
        for remap in interaction['remappings']:
            remapping_args.append(remap.remap_from + ":=" + remap.remap_to)
        command_args.extend(remapping_args)
        process = rocon_python_utils.system.Popen(command_args, postexec_fn=process_listener)
        interaction['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, process, partial(process.send_signal, signal.SIGINT))
        return True

    def _start_global_executable_interaction(self, interaction, filename):
        name = os.path.basename(filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = partial(self._process_listeners, anonymous_name, 1)
        process = rocon_python_utils.system.Popen([filename], postexec_fn=process_listener)
        interaction['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, process, partial(process.send_signal, signal.SIGINT))
        return True

    def _start_weburl_interaction(self, interaction, url):
        """
        We only need the url here and then do a system check for a web browser.
        """
        web_browser = utils.get_web_browser()
        if web_browser is not None:
            name = os.path.basename(web_browser).replace('.', '_')
            anonymous_name = name + "_" + uuid.uuid4().hex
            process_listener = partial(self._process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([web_browser, "--new-window", url], postexec_fn=process_listener)
            interaction['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, process, partial(process.send_signal, signal.SIGINT))
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
            process_listener = partial(self._process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([web_browser, "--new-window", url], postexec_fn=process_listener)
            interaction['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, process, partial(process.send_signal, signal.SIGINT))
            return True
        else:
            return False

    def stop_interaction(self, interaction_hash):
        """
        This stops all launches for an interaction of a particular type.
        """
        if not interaction_hash in self.interactions:
            return (False, "interaction key %s not found in interactions table" % interaction_hash)
        interaction = self.interactions[interaction_hash]
        #print("InteractiveClient : Launched App List- %s" % str(interaction["launch_list"]))
        try:
            for launch_info in interaction["launch_list"].values():
                if launch_info.running:
                    launch_info.shutdown()
                    del self.interactions[interaction_hash]["launch_list"][launch_info.name]
                    print "InteractiveClient : %s APP STOP" % (launch_info.name)
                elif launch_info.process == None:
                    launch_info.running = False
                    del self.interactions[interaction_hash]["launch_list"][launch_info.name]
                    print "InteractiveClient : %s APP LAUNCH IS NONE" % (launch_info.name)
                else:
                    del self.interactions[interaction_hash]["launch_list"][launch_info.name]
                    print "InteractiveClient : %s APP IS ALREADY STOP" % (launch_info.name)
        except Exception as e:
            # this is bad...should not create bottomless exception buckets.
            return (False, "unknown failure - (%s)(%s)" % (type(e), str(e)))
        print "InteractiveClient : updated app list- %s" % str(self.interactions[interaction_hash]["launch_list"])
        if interaction['pairing']:
            self.pairing = None
        self._publish_remocon_status()
        return (True, "success")

    def _process_listeners(self, name, exit_code):
        '''
          Callback function used to catch terminating applications and cleanup appropriately.

          @param name : name of the launched process stored in the interactions index.
          @type str

          @param exit_code : could be utilised from roslaunched processes but not currently used.
          @type int
        '''
        for interaction in self.interactions.values():
            if name in interaction['launch_list']:
                del interaction['launch_list'][name]
                # toggle the pairing indicator if it was a pairing interaction
                if interaction['pairing']:
                    self.pairing = None
                if not interaction['launch_list']:
                    console.logdebug("InteractiveClient : process_listener caught terminating interaction [%s]" % name)
                    # inform the gui to update if necessary
                    self._stop_interaction_postexec_fn()
                    # update the rocon interactions handler
                    self._publish_remocon_status()

    ######################################
    # Ros Comms
    ######################################

    def _publish_remocon_status(self):
        remocon_status = rocon_interaction_msgs.RemoconStatus()
        remocon_status.platform_info = self.platform_info
        remocon_status.uuid = str(self.key.hex)
        remocon_status.version = rocon_std_msgs.Strings.ROCON_VERSION
        running_interactions = []
        for interaction_hash in self.interactions.keys():
            for unused_process_name in self.interactions[interaction_hash]["launch_list"].keys():
                running_interactions.append(interaction_hash)
        remocon_status.running_interactions = running_interactions
        print "[remocon_info] publish remocon status"
        self.remocon_status_pub.publish(remocon_status)

    def _subscribe_pairing_status_callback(self, msg):
        console.logdebug("InteractiveClient : pairing status callback [%s][%s]" % (msg.rapp, msg.remocon))
        if self.pairing:
            if not msg.rapp and msg.remocon == self.name:
                console.logdebug("InteractiveClient : the rapp in this paired interaction terminated")
                # there will only ever be one allowed instance of a paired interaction, so ok to call a stop to all instances of this interaction type
                self.stop_interaction(self.pairing)
                # updat the gui
                self._stop_interaction_postexec_fn()

    ######################################
    # Utilities
    ######################################

    def _prepare_webapp_url(self, interaction, base_url):
        """
           url synthesiser for sending remappings and parameters information.
           We convert the interaction parameter (yaml string) and remapping (rocon_std_msgs.Remapping[])
           variables into generic python list/dictionary objects and convert these into
           json strings as it makes it easier for web apps to handle them.
        """
        interaction_data = {}
        interaction_data['display_name'] = interaction['display_name']
        # parameters
        interaction_data['parameters'] = yaml.load(interaction['parameters'])
        # remappings
        interaction_data['remappings'] = {}  # need to create a dictionary for easy parsing (note: interaction['remappings'] is a list of rocon_std_msgs.Remapping)
        for r in interaction['remappings']:
            interaction_data['remappings'][r.remap_from] = r.remap_to
        # package all the data in json format and dump it to one query string variable
        console.logdebug("Remocon Info : web app query string %s" % interaction_data)
        query_string_mappings = {}
        query_string_mappings['interaction_data'] = json.dumps(interaction_data)
        # constructing the url
        return base_url + "?" + urllib.urlencode(query_string_mappings)
