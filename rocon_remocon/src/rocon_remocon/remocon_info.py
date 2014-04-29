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

from . import utils
from .launch import LaunchInfo

##############################################################################
# RemoconInfo
##############################################################################


class RemoconInfo():
    def __init__(self, stop_app_postexec_fn):
        '''
          @param stop_app_postexec_fn : callback to fire when a listener detects an app getting stopped.
          @type method with no args
        '''
        self._stop_app_postexec_fn = stop_app_postexec_fn
        self.interactions = {}
        self.rocon_master_info = {}
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
        print("[remocon_info] info component initialised")

    def __del__(self):
        print("[remocon_info] : info component destroyed")

    def _connect(self, rocon_master_name="", ros_master_uri="http://localhost:11311", host_name='localhost'):

        # uri is obtained from the user, stored in ros_master_uri
        os.environ["ROS_MASTER_URI"] = ros_master_uri
        os.environ["ROS_HOSTNAME"] = host_name

        print "[remocon_info] connect RemoconInfo "
        print "[remocon_info] ROS_MASTER_URI: " + str(os.environ["ROS_MASTER_URI"])
        print "[remocon_info] Node Name: " + self.name
        # Need to make sure we give it a unique node name and we need a unique uuid
        # for the remocon-role manager interaction anyway:

        rospy.init_node(self.name, disable_signals=True)

        try:
            rocon_master_info_topic_name = rocon_python_comms.find_topic('rocon_std_msgs/MasterInfo', timeout=rospy.rostime.Duration(5.0), unique=True)
            get_interactions_service_name = rocon_python_comms.find_service('rocon_interaction_msgs/GetInteractions', timeout=rospy.rostime.Duration(5.0), unique=True)
            get_roles_service_name = rocon_python_comms.find_service('rocon_interaction_msgs/GetRoles', timeout=rospy.rostime.Duration(5.0), unique=True)
            request_interaction_service_name = rocon_python_comms.find_service('rocon_interaction_msgs/RequestInteraction', timeout=rospy.rostime.Duration(5.0), unique=True)
        except rocon_python_comms.NotFoundException as e:
            console.logerror("RemoconInfo : failed to find either rocon master info or interactions topics and services' [%s]" % str(e))
            return False

        self.info_sub = rospy.Subscriber(rocon_master_info_topic_name, rocon_std_msgs.MasterInfo, self._info_callback)
        self.get_interactions_service_proxy = rospy.ServiceProxy(get_interactions_service_name, rocon_interaction_srvs.GetInteractions)
        self.get_roles_service_proxy = rospy.ServiceProxy(get_roles_service_name, rocon_interaction_srvs.GetRoles)
        self.request_interaction_service_proxy = rospy.ServiceProxy(request_interaction_service_name, rocon_interaction_srvs.RequestInteraction)
        self.remocon_status_pub = rospy.Publisher("remocons/" + self.name, rocon_interaction_msgs.RemoconStatus, latch=True)

        self._pub_remocon_status()
        self.is_connect = True
        self.is_valid_info = False
        return True

    def _disconnect(self):
        self._shutdown()

    def _is_shutdown(self):
        print "[remocon_info] shut down is complete"

    def _shutdown(self):
        if(self.is_connect != True):
            console.logwarn("RemoconInfo : tried to shutdown already disconnected remocon")
        else:
            console.logdebug("RemoconInfo : shutting down all apps")
            for app_hash in self.interactions.keys():
                self._stop_app(app_hash)

            self.interactions = {}
            self.is_connect = False

            rospy.signal_shutdown("shut down remocon_info")
            while not rospy.is_shutdown():
                rospy.rostime.wallsleep(0.1)

            self.is_connect = False
            self.info_sub.unregister()
            self.remocon_status_pub.unregister()

            self.info_sub = None
            self.remocon_status_pub = None
            console.logdebug("RemoconInfo : has shutdown.")

    def _pub_remocon_status(self):
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

    def get_role_list(self):
        try:
            response = self.get_roles_service_proxy(self.platform_info.uri)
        except (rospy.ROSInterruptException, rospy.ServiceException):
            return []
        return response.roles

    def _select_role(self, role_name):
        roles = []
        roles.append(role_name)

        call_result = self.get_interactions_service_proxy(roles, self.platform_info.uri)
        print "[remocon_info]: call result"
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

    def get_rocon_master_info(self):
        console.logdebug("RemoconInfo : retrieving rocon master information")
        time_out_cnt = 0
        while not rospy.is_shutdown():
            if len(self.rocon_master_info) != 0:
                break
            else:
                rospy.sleep(rospy.Duration(0.2))
            time_out_cnt += 1
            if time_out_cnt > 5:
                console.logwarn("RemoconInfo : timed out waiting for rocon master information")
                break
        return self.rocon_master_info

    def _info_callback(self, data):
        print "[remocon_info] sample info call back calling!!!!"
        self.is_valid_info = False
        rocon_master_name = data.name

        icon_name = data.icon.resource_name.split('/').pop()
        # delete rocon master info in cache
        if data.icon.data:
            icon = open(os.path.join(utils.get_icon_cache_home(), icon_name), 'w')
            icon.write(data.icon.data)
            icon.close()

        self.rocon_master_info = {}
        self.rocon_master_info['name'] = rocon_master_name
        self.rocon_master_info['description'] = data.description
        self.rocon_master_info['icon'] = icon_name

        self.is_valid_info = True

    def _start_app(self, app_hash):
        if not app_hash in self.interactions.keys():
            print "[remocon_info] HAS NO KEY"
            return False

        interaction = self.interactions[app_hash]
        #get the permission
        call_result = self.request_interaction_service_proxy(remocon=self.name, hash=interaction['hash'])

        if call_result.error_code == ErrorCodes.SUCCESS:
            print "[remocon_info] permisson ok"
            (app_executable, start_app_handler) = self._determine_app_type(interaction['name'])
            result = start_app_handler(interaction, app_executable)
            if result:
                self._pub_remocon_status()
            return result
        else:
            print "[remocon_info] permission failure"
            return False
        return True

    def _determine_app_type(self, app_name):
        '''
          Classifies the app based on the app name string and some intelligent
          (well reasonably) parsing of that string.
           - paired dummy (by empty name) 
           - ros launcher (by .launch extension)
           - ros runnable (by roslib find_resource success)
           - web app      (by web_interactions.parse)
           - web url      (by web_interactions.parse)
           - global executable (fallback option)
        '''
        # pairing trigger (i.e. dummy interaction)
        if not app_name:
            console.logdebug("RemoconInfo : start a dummy interaction for triggering a pair [%s]")
            return ('', self._start_dummy_interaction)
        # roslaunch
        try:
            app_filename = rocon_python_utils.ros.find_resource_from_string(app_name, extension='launch')
            console.logdebug("RemoconInfo : regular start app [%s]")
            return (app_filename, self._start_app_launch)
        except (rospkg.ResourceNotFound, ValueError):
            pass
        # rosrun
        try:
            app_filename = rocon_python_utils.ros.find_resource_from_string(app_name)
            console.logdebug("RemoconInfo : start_app_rosrunnable [%s]")
            return (app_filename, self._start_app_rosrunnable)
        except rospkg.ResourceNotFound:
            pass
        except Exception:
            pass
        # web url/app
        web_interaction = web_interactions.parse(app_name)
        if web_interaction is not None:
            if web_interaction.is_web_url():
                console.logdebug("RemoconInfo : start_app_weburl [%s]" % web_interaction.url)
                return (web_interaction.url, self._start_app_weburl)
            elif web_interaction.is_web_app():
                console.logdebug("RemoconInfo : start_app_webapp [%s]" % web_interaction.url)
                return (web_interaction.url, self._start_app_webapp)
        # executable
        console.logdebug("RemoconInfo : start_app_global_executable [%s]")
        return (app_name, self._start_app_global_executable)

    def _start_dummy_interaction(self, interaction, unused_filename):
        console.loginfo("RemoconInfo : starting paired dummy interaction")

    def _start_app_launch(self, app, roslaunch_filename):
        '''
          Start a ros launchable application, applying parameters and remappings if specified.
        '''
        name_space = ""
        name_space += '/service/' + app['namespace']
        temp = tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        #add parameters
        launch_text = ""
        launch_text += '<launch>\n'
        launch_text += '    <group ns="%s">\n' % (name_space)
        launch_text += '        <rosparam>%s</rosparam>\n' % (app['parameters'])
        launch_text += '        <include file="%s"/>\n' % (roslaunch_filename)
        launch_text += '    </group>\n'
        launch_text += '</launch>\n'
        temp.write(launch_text)
        temp.close()  # unlink it later
        self.listener = roslaunch.pmon.ProcessListener()
        self.listener.process_died = self.process_listeners
        try:
            _launch = roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),
                                                            [temp.name],
                                                            is_core=False,
                                                            process_listeners=[self.listener])
            _launch._load_config()
            for N in _launch.config.nodes:
                for k in app['remappings']:
                    N.remap_args.append([(name_space + '/' + k.remap_from).replace('//', '/'), k.remap_to])
            _launch.start()

            process_name = str(_launch.pm.get_process_names_with_spawn_count()[0][0][0])
            app['launch_list'][process_name] = LaunchInfo(process_name, True, _launch, _launch.shutdown)
            return True
        except Exception, inst:
            print inst
            app['running'] = str(False)
            print "Fail to launch: %s" % (app['name'])
            return False

    def _start_app_rosrunnable(self, app, rosrunnable_filename):
        '''
          Launch a rosrunnable application. This does not apply any parameters
          or remappings (yet).
        '''
        # the following is guaranteed since we came back from find_resource calls earlier
        # note we're overriding the rosrunnable filename here - rosrun doesn't actually take the full path.
        package_name, rosrunnable_filename = app['name'].split('/')
        name = os.path.basename(rosrunnable_filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = partial(self.process_listeners, anonymous_name, 1)
        command_args = ['rosrun', package_name, rosrunnable_filename, '__name:=%s' % anonymous_name]
        remapping_args = []
        for remap in app['remappings']:
            remapping_args.append(remap.remap_from + ":=" + remap.remap_to)
        command_args.extend(remapping_args)
        process = rocon_python_utils.system.Popen(command_args, postexec_fn=process_listener)
        app['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, process, partial(process.send_signal, signal.SIGINT))
        return True

    def _start_app_global_executable(self, app, rosrunnable_filename):
        name = os.path.basename(rosrunnable_filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = partial(self.process_listeners, anonymous_name, 1)
        process = rocon_python_utils.system.Popen([rosrunnable_filename], postexec_fn=process_listener)
        app['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, process, partial(process.send_signal, signal.SIGINT))
        return True

    def _start_app_weburl(self, app, url):
        """
        We only need the url here and then do a system check for a web browser.
        """
        web_browser = self._check_webbrowser()
        if web_browser is not None:
            name = os.path.basename(web_browser).replace('.', '_')
            anonymous_name = name + "_" + uuid.uuid4().hex
            process_listener = partial(self.process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([web_browser, "--new-window", url], postexec_fn=process_listener)
            app['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, process, partial(process.send_signal, signal.SIGINT))
            return True
        else:
            return False

    def _start_app_webapp(self, app, base_url):
        """
        Need to work out the extended url (with args, parameters and remappings) here and then feed that to a
        detected browser.

        :param base_url str: the web app url without all of the attached variables.
        """
        web_browser = self._check_webbrowser()
        if web_browser is not None:
            url = self._get_webapp_url(app, base_url)
            name = os.path.basename(web_browser).replace('.', '_')
            anonymous_name = name + "_" + uuid.uuid4().hex
            process_listener = partial(self.process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([web_browser, "--new-window", url], postexec_fn=process_listener)
            app['launch_list'][anonymous_name] = LaunchInfo(anonymous_name, True, process, partial(process.send_signal, signal.SIGINT))
            return True
        else:
            return False

    def _get_webapp_url(self, app, base_url):
        """
           url syntheiser for sending remappings and parameters information.
           We convert the interaction parameter (yaml string) and remapping (rocon_std_msgs.Remapping[])
           variables into generic python list/dictionary objects and convert these into
           json strings as it makes it easier for web apps to handle them.
        """
        interaction_data = {}
        interaction_data['display_name'] = app['display_name']
        # parameters
        interaction_data['parameters'] = yaml.load(app['parameters'])
        # remappings
        interaction_data['remappings'] = {}  # need to create a dictionary for easy parsing (note: app['remappings'] is a list of rocon_std_msgs.Remapping)
        for r in app['remappings']:
            interaction_data['remappings'][r.remap_from] = r.remap_to
        # package all the data in json format and dump it to one query string variable
        console.logdebug("Remocon Info : web app query string %s" % interaction_data)
        query_string_mappings = {}
        query_string_mappings['interaction_data'] = json.dumps(interaction_data)
        # constructing the url
        return base_url + "?" + urllib.urlencode(query_string_mappings)

    def _check_webbrowser(self):
        if rocon_python_utils.system.which("google-chrome"):
            return 'google-chrome'
        elif rocon_python_utils.system.which("google-chrome-unstable"):
            return 'google-chrome-unstable'
        elif rocon_python_utils.system.which("chromium-browser"):
            return 'chromium-browser'
        return None

    def _stop_app(self, app_hash):
        if not app_hash in self.interactions:
            print "[remocon_info] HAS NO KEY"
            return False
        print("[remocon_info] Launched App List- %s" % str(self.interactions[app_hash]["launch_list"]))
        try:
            for launch_info in self.interactions[app_hash]["launch_list"].values():
                process_name = launch_info.name
                is_app_running = launch_info.running
                if is_app_running:
                    launch_info.shutdown()
                    del self.interactions[app_hash]["launch_list"][launch_info.name]
                    print "[remocon_info] %s APP STOP" % (process_name)
                elif launch_info.process == None:
                    launch_info.running = False
                    del self.interactions[app_hash]["launch_list"][launch_info.name]
                    print "[remocon_info] %s APP LAUNCH IS NONE" % (process_name)
                else:
                    del self.interactions[app_hash]["launch_list"][launch_info.name]
                    print "[remocon_info] %s APP IS ALREADY STOP" % (process_name)
        except Exception as e:
            print "[remocon_info] APP STOP PROCESS IS FAILURE %s %s" % (str(e), type(e))
            return False
        print "[remocon_info] updated app list- %s" % str(self.interactions[app_hash]["launch_list"])
        self._pub_remocon_status()
        return True

    def process_listeners(self, name, exit_code):
        '''
          Callback function used to catch terminating applications and cleanup appropriately.

          @param name : name of the launched process stored in the interactions index.
          @type str

          @param exit_code : could be utilised from roslaunched processes but not currently used.
          @type int
        '''
        for unused_interaction_hash, v in self.interactions.iteritems():
            if name in v['launch_list']:
                del v['launch_list'][name]
                if not v['launch_list']:
                    console.logdebug("RemoconInfo : process_listener caught terminating app [%s]" % name)
                    # inform the gui to update if necessary
                    self._stop_app_postexec_fn()
                    # update the rocon interactions handler
                    self._pub_remocon_status()
