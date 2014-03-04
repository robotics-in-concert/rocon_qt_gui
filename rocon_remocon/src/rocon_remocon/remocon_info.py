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
from urlparse import urlparse
import urllib

#ros
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
import rocon_python_comms

from . import utils

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
        self.app_list = {}
        self.rocon_master_info = {}
        self.is_connect = False
        self.is_app_running = False
        self.key = uuid.uuid4()

        self.app_pid = 0

        # this might be naive and only work well on ubuntu...
        os_codename = OsDetect().get_codename()
        # this would be great as a configurable parameter
        name = "rqt_remocon_" + self.key.hex
        self.rocon_uri = rocon_uri.parse(
                            "rocon:/pc/" + name + "/" + rocon_std_msgs.Strings.URI_WILDCARD + "/" + os_codename
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
        # remocon name would be good as a persistant configuration variable by the user
        # so they can set something like 'Bob'.
        remocon_name = 'rqt_remocon'
        unique_name = remocon_name + "_" + self.key.hex

        # uri is obtained from the user, stored in ros_master_uri
        os.environ["ROS_MASTER_URI"] = ros_master_uri
        os.environ["ROS_HOSTNAME"] = host_name

        print "[remocon_info] connect RemoconInfo "
        print "[remocon_info] ROS_MASTER_URI: " + str(os.environ["ROS_MASTER_URI"])
        print "[remocon_info] Node Name: " + str(unique_name)
        # Need to make sure we give it a unique node name and we need a unique uuid
        # for the remocon-role manager interaction anyway:

        rospy.init_node(unique_name, disable_signals=True)

        try:
            rocon_master_info_topic_name = rocon_python_comms.find_topic('rocon_std_msgs/MasterInfo', timeout=rospy.rostime.Duration(5.0), unique=True)
            roles_topic_name = rocon_python_comms.find_topic('rocon_interaction_msgs/Roles', timeout=rospy.rostime.Duration(5.0), unique=True)
            get_interactions_service_name = rocon_python_comms.find_service('rocon_interaction_msgs/GetInteractions', timeout=rospy.rostime.Duration(15.0), unique=True)
            request_interaction_service_name = rocon_python_comms.find_service('rocon_interaction_msgs/RequestInteraction', timeout=rospy.rostime.Duration(15.0), unique=True)
        except rocon_python_comms.NotFoundException as e:
            console.logerror("RemoconInfo : failed to find either rocon master info or interactions topics and services' [%s]" % str(e))
            return False

        self.info_sub = rospy.Subscriber(rocon_master_info_topic_name, rocon_std_msgs.MasterInfo, self._info_callback)
        self.get_interactions_service_proxy = rospy.ServiceProxy(get_interactions_service_name, rocon_interaction_srvs.GetInteractions)
        self.request_interaction_service_proxy = rospy.ServiceProxy(request_interaction_service_name, rocon_interaction_srvs.RequestInteraction)
        self.remocon_status_pub = rospy.Publisher("remocons/" + unique_name, rocon_interaction_msgs.RemoconStatus, latch=True)

        self._pub_remocon_status(0, False)
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
            for app_hash in self.app_list.keys():
                self._stop_app(app_hash)

            self.app_list = {}
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

    def _pub_remocon_status(self, app_hash, running_app):
        remocon_status = rocon_interaction_msgs.RemoconStatus()
        remocon_status.platform_info = self.platform_info
        remocon_status.uuid = str(self.key.hex)
        remocon_status.running_app = running_app
        remocon_status.hash = app_hash
        print "[remocon_info] publish remocon status"
        self.remocon_status_pub.publish(remocon_status)

    def get_role_list(self):
        '''
          Currently we just call the get_interactions service which will
          filter the interactions table against our compatibilty rocon_uri.
          We can then extract the valid roles for us from the interaction
          list that returns.
        '''
        # Might want to put a filtering get_roles service on the interactions
        # manager. See:
        #    https://github.com/robotics-in-concert/rocon_qt_gui/issues/52
        try:
            response = self.get_interactions_service_proxy([], self.platform_info.uri)
        except (rospy.ROSInterruptException, rospy.ServiceException):
            return []
        return list(set([i.role for i in response.interactions]))

    def _select_role(self, role_name):
        roles = []
        roles.append(role_name)

        call_result = self.get_interactions_service_proxy(roles, self.platform_info.uri)
        print "[remocon_info]: call result"
        self.app_list = {}
        for interaction in call_result.interactions:
            if(interaction.role == role_name):
                app_hash = interaction.hash
                #if self.app_list.has_key(app_hash):
                #    pass
                #else:
                #    self.app_list[app_hash]={}
                #    self.app_list[app_hash]['launch_list'] ={}
                self.app_list[app_hash] = {}
                self.app_list[app_hash]['launch_list'] = {}

                self.app_list[app_hash]['name'] = interaction.name
                self.app_list[app_hash]['compatibility'] = interaction.compatibility
                icon_name = interaction.icon.resource_name.split('/').pop()
                if interaction.icon.data:
                    icon = open(os.path.join(utils.get_icon_cache_home(), icon_name), 'w')
                    icon.write(interaction.icon.data)
                    icon.close()
                self.app_list[app_hash]['icon'] = icon_name
                self.app_list[app_hash]['display_name'] = interaction.display_name
                self.app_list[app_hash]['description'] = interaction.description
                self.app_list[app_hash]['namespace'] = interaction.namespace
                self.app_list[app_hash]['max'] = interaction.max
                self.app_list[app_hash]['remappings'] = interaction.remappings
                self.app_list[app_hash]['parameters'] = interaction.parameters
                self.app_list[app_hash]['hash'] = interaction.hash

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

    def _get_app_list(self):
        return self.app_list
        pass

    def _start_app(self, app_hash):
        if not app_hash in self.app_list.keys():
            print "[remocon_info] HAS NO KEY"
            return False

        if self.is_app_running == True:
            print "[remocon_info] APP ALREADY RUNNING NOW "
            return  False

        app = self.app_list[app_hash]
        #get the permission
        call_result = self.request_interaction_service_proxy(app['hash'])

        if call_result.error_code == ErrorCodes.SUCCESS:
            print "[remocon_info] permisson ok"
            (app_executable, start_app_handler) = self._determine_app_type(app['name'])
            result = start_app_handler(app, app_executable)
            self._pub_remocon_status(app_hash, result)
            return result
        else:
            print "[remocon_info] permission failure"
            return False
        return True

    def _determine_app_type(self, app_name):
        '''
          Classifies the app based on the app name string and some intelligent
          (well reasonably) parsing of that string.
           - ros launcher (by .launch extension)
           - ros runnable (by roslib find_resource success)
           - web app      (by urlparse scheme check against http)
           - global executable (fallback option)
        '''
        try:
            app_filename = rocon_python_utils.ros.find_resource_from_string(app_name, extension='launch')
            return (app_filename, self._start_app_launch)
        except (rospkg.ResourceNotFound, ValueError):
            pass
        try:
            app_filename = rocon_python_utils.ros.find_resource_from_string(app_name)
            return (app_filename, self._start_app_rosrunnable)
        except rospkg.ResourceNotFound:
            pass
        except Exception:
            pass
        o = urlparse(app_name)
        if o.scheme == 'http':
            return (app_name, self._start_app_webapp)
        else:
            return (app_name, self._start_app_global_executable)

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
            app['launch_list'][process_name] = {}
            app['launch_list'][process_name]['name'] = process_name
            app['launch_list'][process_name]['running'] = str(True)
            app['launch_list'][process_name]['process'] = _launch
            app['launch_list'][process_name]['shutdown'] = _launch.shutdown
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
        process = rocon_python_utils.system.Popen(['rosrun', package_name, rosrunnable_filename, '__name:=%s' % anonymous_name], postexec_fn=process_listener)
        app['launch_list'][anonymous_name] = {}
        app['launch_list'][anonymous_name]['name'] = anonymous_name
        app['launch_list'][anonymous_name]['running'] = str(True)
        app['launch_list'][anonymous_name]['process'] = process
        app['launch_list'][anonymous_name]['shutdown'] = partial(process.send_signal, signal.SIGINT)
        return True

    def _start_app_global_executable(self, app, rosrunnable_filename):
        name = os.path.basename(rosrunnable_filename).replace('.', '_')
        anonymous_name = name + "_" + uuid.uuid4().hex
        process_listener = partial(self.process_listeners, anonymous_name, 1)
        process = rocon_python_utils.system.Popen([rosrunnable_filename], postexec_fn=process_listener)
        app['launch_list'][anonymous_name] = {}
        app['launch_list'][anonymous_name]['name'] = anonymous_name
        app['launch_list'][anonymous_name]['running'] = str(True)
        app['launch_list'][anonymous_name]['process'] = process
        app['launch_list'][anonymous_name]['shutdown'] = partial(process.send_signal, signal.SIGINT)
        return True

    def _start_app_webapp(self, app, rosrunnable_filename):
        if self._check_webbrowser():
            rosrunnable_filename = "google-chrome"
            url = self._get_webapp_url(app)
            name = os.path.basename(rosrunnable_filename).replace('.', '_')
            anonymous_name = name + "_" + uuid.uuid4().hex
            process_listener = partial(self.process_listeners, anonymous_name, 1)
            process = rocon_python_utils.system.Popen([rosrunnable_filename, "--new-window", url], postexec_fn=process_listener)
            app['launch_list'][anonymous_name] = {}
            app['launch_list'][anonymous_name]['name'] = anonymous_name
            app['launch_list'][anonymous_name]['running'] = str(True)
            app['launch_list'][anonymous_name]['process'] = process
            app['launch_list'][anonymous_name]['shutdown'] = partial(process.send_signal, signal.SIGINT)
            return True
        else:
            return False

    def _get_webapp_url(self, app):
        """
           url syntheiser for sending remappings and parameters information
        """
        url = app['name']
        #url += "?" + "MasterURI=" + str(os.environ["ROS_MASTER_URI"])
        if len(app['parameters']) != 0:
            url += "&" + "params=" + urllib.quote_plus(app['parameters'])
        if len(app['remappings']) != 0:
            remaps = "{"
            for remapping in app['remappings']:
                remaps += "\'" + remapping.remap_from + "\':\'" + remapping.remap_to + "\',"
            remaps = remaps[0:len(remaps) - 1] + "}"
            print remaps
            url += "&" + "remaps=" + urllib.quote_plus(remaps)
        return url

    def _check_webbrowser(self):
        check_result = rocon_python_utils.system.which("google-chrome")
        if check_result:
            return True
        elif check_result == None:
            return False
        else:
            return False

    def _stop_app(self, app_hash):
        if not app_hash in self.app_list:
            print "[remocon_info] HAS NO KEY"
            return False
        print "[remocon_info]Launched App List- %s" % str(self.app_list[app_hash]["launch_list"])
        try:
            for k in self.app_list[app_hash]["launch_list"].values():
                process_name = k["name"]
                is_app_running = k["running"]
                if is_app_running == "True":
                    k['shutdown']()
                    del self.app_list[app_hash]["launch_list"][k["name"]]
                    print "[remocon_info] %s APP STOP" % (process_name)
                elif k['process'] == None:
                    k["running"] = "False"
                    del self.app_list[app_hash]["launch_list"][k["name"]]
                    print "[remocon_info] %s APP LAUNCH IS NONE" % (process_name)
                else:
                    del self.app_list[app_hash]["launch_list"][k["name"]]
                    print "[remocon_info] %s APP IS ALREADY STOP" % (process_name)
        except Exception as e:
            print "[remocon_info] APP STOP PROCESS IS FAILURE %s %s" % (str(e), type(e))
            return False
        print "[remocon_info] updated app list- %s" % str(self.app_list[app_hash]["launch_list"])
        self._pub_remocon_status(app_hash, False)
        return True

    def process_listeners(self, name, exit_code):
        '''
          Callback function used to catch terminating applications and cleanup appropriately.

          @param name : name of the launched process stored in the app_list index.
          @type str

          @param exit_code : could be utilised from roslaunched processes but not currently used.
          @type int
        '''
        for app_hash, v in self.app_list.iteritems():
            if name in v['launch_list']:
                del v['launch_list'][name]
                if not v['launch_list']:
                    console.logdebug("RemoconInfo : process_listener caught terminating app [%s]" % name)
                    # inform the gui to update if necessary
                    self._stop_app_postexec_fn()
                    # update the rocon interactions handler
                    self._pub_remocon_status(app_hash, False)
