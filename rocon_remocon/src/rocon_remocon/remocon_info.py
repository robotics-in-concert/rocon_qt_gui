#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################
#system
import os
import uuid
import string
import subprocess
import signal
import tempfile
#ros
import rospy
import rosservice
import rocon_utilities
import roslaunch.parent
#ros message and service
from uuid_msgs.msg import UniqueID
#rocon message and service
from rocon_std_msgs.msg import PlatformInfo
from rocon_std_msgs.msg import Icon
from rocon_app_manager_msgs.msg import ErrorCodes
#concert message and service
from concert_msgs.msg import ConcertInfo
from concert_msgs.msg import Roles
from concert_msgs.msg import RemoconStatus
from concert_msgs.srv import GetRolesAndApps
from concert_msgs.srv import RequestInteraction

##############################################################################
# Remocon Info
##############################################################################
                        
class RemoconInfo():
    def __init__(self):
        self.role_list={}
        self.app_list={}    
        self.concert_info={}
        self.is_connect=False
        self.is_app_running=False    
        self.key=uuid.uuid4()
        self.temp_icon_path= "%s/.ros/rocon/remocon/image/"%(os.getenv("HOME"))
        
        self.app_pid=0
        
        self.platform_info=PlatformInfo() 
        self.platform_info.os=PlatformInfo().OS_LINUX
        self.platform_info.version=PlatformInfo().VERSION_UBUNTU_PRECISE
        self.platform_info.platform=PlatformInfo().PLATFORM_PC
        self.platform_info.system=PlatformInfo().SYSTEM_RQT
        self.platform_info.name="rqt_remocon"
        
        print "[remocon_info] init complete"
    
    def _connect(self,concert_name="", concert_ip="127.0.0.1",host_name='127.0.0.1',concert_port="11311"):
        # remocon name would be good as a persistant configuration variable by the user
        # so they can set something like 'Bob'.
        remocon_name='rqt_remocon'
        unique_name=remocon_name + "_" + self.key.hex

        # uri is obtained from the user, stored in ros_master_uri
        os.environ["ROS_MASTER_URI"]='http://'+str(concert_ip)+':'+str(concert_port)
        ## get host name
        #Todo
        os.environ["ROS_HOSTNAME"]=host_name
        
        print "[remocon_info] connect RemoconInfo "
        print "[remocon_info] ROS_MASTER_URI: "+str(os.environ["ROS_MASTER_URI"])
        print "[remocon_info] Node Name: "+ str(unique_name)
        # Need to make sure we give it a unique node name and we need a unique uuid
        # for the remocon-role manager interaction anyway:
        
        rospy.init_node(unique_name,disable_signals=True)
        
        if not self._check_valid_concert():
            return False

        self.role_sub=rospy.Subscriber("/concert/interactions/roles",Roles, self._roles_callback)
        self.info_sub=rospy.Subscriber("/concert/info", ConcertInfo , self._info_callback)
        
        self.remocon_status_pub=rospy.Publisher("remocons/"+unique_name,RemoconStatus,latch=True)
        
        self._pub_remocon_status("",False)
        self.is_connect=True
        self.is_valid_role=False
        self.is_valid_info=False
     
        return True
    def _disconnect(self):
        self._shutdown()
        pass
            
    def _check_valid_concert(self):
        ret_value=False
        topic_cnt=0
        topic_list=rospy.get_published_topics()
        for k in topic_list:
            if k[0]=='/concert/interactions/roles':
                topic_cnt +=1
            if k[0]=='/concert/info':
                topic_cnt +=1
            if topic_cnt >=2:
                ret_value=True
                break
                        
        return ret_value
    
    def _is_shutdown(self):
        print "[remocon_info] shut down is complete"        
        pass

    def _shutdown(self):
    
        if(self.is_connect !=True):
            print "already disconnection"
        else:
            self.role_list={}
            self.app_list={}
            self.is_connect=False

            rospy.signal_shutdown("shut down remocon_info")
            while not rospy.is_shutdown():
                #sleep some
                pass
            self.is_connect=False
            
            self.role_sub.unregister()
            self.info_sub.unregister()
            self.remocon_status_pub.unregister()
            
            self.role_sub=None
            self.info_sub=None
            self.remocon_status_pub=None
          
            print "[remocon_info] shutdown RemoconInfo"

    def _pub_remocon_status(self,app_name,running_app):   
        
        remocon_status=RemoconStatus()
        
        remocon_status.platform_info.os=PlatformInfo().OS_LINUX
        remocon_status.platform_info.version=PlatformInfo().VERSION_UBUNTU_PRECISE
        remocon_status.platform_info.platform=PlatformInfo().PLATFORM_PC
        remocon_status.platform_info.system=PlatformInfo().SYSTEM_RQT
        remocon_status.platform_info.name="rqt_remocon"
        
        remocon_status.uuid=str(self.key.hex)
        remocon_status.running_app=running_app
        remocon_status.app_name=app_name  
        
        print "[remocon_info] publish remocon status"
        self.remocon_status_pub.publish(remocon_status)
        
        pass
  
    def _get_role_list(self):
        print "[remocon_info] get role list"
        time_out_cnt=0;
        while not rospy.is_shutdown():
            if len(self.role_list) !=0:
                break
            else:
                rospy.sleep(rospy.Duration(0.2))
            time_out_cnt+=1
            if time_out_cnt>5:
                print "[remocon_info] get role list: time out 1s"
                break
        return self.role_list
 
    def _select_role(self,role_name):
        
        roles=[]
        roles.append(role_name)
        
        platform_info=PlatformInfo()
        platform_info.os='*'
        platform_info.version='*'
        platform_info.platform='pc'
        platform_info.system='ros'
        platform_info.name='*'
        
        #icon 
        # Todo     
        
        service_handle=rospy.ServiceProxy("/concert/interactions/get_roles_and_apps", GetRolesAndApps)
        call_result=service_handle(roles,platform_info)
        print "[remocon_info]: call result"
        self.app_list={}
        for k in call_result.data:
            if(k.role==role_name):
                for l in k.remocon_apps:
                    app_name=l.name
                    self.app_list[app_name]={}
                    self.app_list[app_name]['name']=l.name
                    self.app_list[app_name]['platform_info']=l.platform_info
                    #todo icon
                    icon_name = l.icon.resource_name.split('/').pop()
                    icon = open(self.temp_icon_path+icon_name,'w')
                    icon.write(l.icon.data)
                    icon.close()  
                    self.app_list[app_name]['icon']=icon_name
                    self.app_list[app_name]['display_name']=l.display_name
                    self.app_list[app_name]['description']=l.description
                    self.app_list[app_name]['service_name']=l.service_name
                    self.app_list[app_name]['max']=l.max
                    self.app_list[app_name]['remappings']=l.remappings
                    self.app_list[app_name]['parameters']=l.parameters
                    self.app_list[app_name]['launch_list'] ={}
                    #self.app_list[app_name]['launch']=None
                    #self.app_list[app_name]['running']=str(False)
                    
        pass
       
    def _roles_callback(self,data):
        print "[remocon_info] sample roles call back calling!!!!"
        #self.is_valid_role=False
        for k in data.list:
            role_name=k
            self.role_list[k]={}
            self.role_list[k]['name']=k
        self.is_valid_role=True
        pass 
    
    def _get_concert_info(self):
        print "[remocon_info] get concert info"
        time_out_cnt=0
        while not rospy.is_shutdown():
            if len(self.concert_info) !=0:
                break
            else:
                rospy.sleep(rospy.Duration(0.2))
            time_out_cnt+=1
            if time_out_cnt>5:
                print "[remocon_info] get role list: time out 1s"
                break

        return self.concert_info
        pass    
    
    def _info_callback(self,data):
        print "[remocon_info] sample info call back calling!!!!"
        self.is_valid_info=False
        concert_name=data.name 
        
        icon_name=data.icon.resource_name.split('/').pop()
        # delete concert info in cache
        icon=open(self.temp_icon_path+icon_name,'w')
        icon.write(data.icon.data)
        icon.close()  

        self.concert_info={}
        self.concert_info['name']=concert_name
        self.concert_info['description']=data.description
        self.concert_info['icon']= icon_name 
        
        self.is_valid_info=True
        pass 

    def _get_app_list(self):
        return self.app_list        
        pass
        
    def _start_app(self,role_name,app_name):
        if not self.app_list.has_key(app_name):
            print "[remocon_info] HAS NO KEY"
            return
        
        if self.is_app_running==True:
            print "[remocon_info] APP ALREADY RUNNING NOW "
            return  
            
        #get the permission
        service_handle=rospy.ServiceProxy("/concert/interactions/request_interaction", RequestInteraction)
        
        service_name=self.app_list[app_name]['service_name']
        platform_info=self.platform_info
        remappings=self.app_list[app_name]['remappings']
        parameters=self.app_list[app_name]['parameters']
        
        call_result=service_handle(platform_info,role_name,service_name,app_name)

        if call_result.error_code==ErrorCodes.SUCCESS:
            print "[remocon_info] permisson ok"
            if self._start_app_launch(app_name,service_name,remappings,parameters):
                #start launcher
                self._pub_remocon_status(app_name,True)
            else:
                self._pub_remocon_status(app_name,False)
            pass
        
    def _stop_app(self,app_name):
        print self.app_list[app_name]["launch_list"]    
        
        for k in self.app_list[app_name]["launch_list"].values():
            process_name = k["name"]
            is_app_running = k["running"]
            
            if is_app_running == "True":
                k["launch"].shutdown()    
                del self.app_list[app_name]["launch_list"][k["name"]]
                print "[remocon_info] %s APP STOP"%(process_name)        
            elif k["launch"] == None:
                k["running"] = "False"
                del self.app_list[app_name]["launch_list"][k["name"]]
                print "[remocon_info] %s APP LAUNCH IS NONE"%(process_name)
            else:
                del self.app_list[app_name]["launch_list"][k["name"]]
                print "[remocon_info] %s APP IS ALREADY STOP"%(process_name)
        pass 
        
        print self.app_list[app_name]["launch_list"]    
    
    def _start_app_launch(self,app_name,service_name,remappings,parameters):
      
        #start launch file
        pkg_name=app_name.split('/')[0]
        launch_name=app_name.split('/')[1]+'.launch'
        remappings_role=""
        #check the launch file validation
        
        try:
            full_path=rocon_utilities.find_resource(pkg_name,launch_name)
        except:
            print "[remocon_info] FAULT [%s] PACKAGE OR [%s] LAUNCH FILE"%(pkg_name,launch_name)
            return False

        name_space=""
        name_space+='/service/'+service_name
        full_path=rocon_utilities.find_resource(pkg_name,launch_name)
        temp=tempfile.NamedTemporaryFile(mode='w+t', delete=False)
        
        #add parameters
        launch_text=""
        launch_text+='<launch>\n'
        launch_text+='    <group ns="%s">\n'%(name_space)
        launch_text+='        <rosparam>%s</rosparam>\n'%(parameters)
        launch_text+='        <include file="%s"/>\n'%(full_path)
        launch_text+='    </group>\n'
        launch_text+='</launch>\n'    
        temp.write(launch_text)
        temp.close()  # unlink it later
        self.listener = roslaunch.pmon.ProcessListener()
        self.listener.process_died = self.process_listeners
        try:
            _launch=roslaunch.parent.ROSLaunchParent(rospy.get_param("/run_id"),
                                                            [temp.name],
                                                            is_core=False,
                                                            process_listeners=[self.listener])    
            _launch._load_config()
            remap_size=len(remappings)
            for N in _launch.config.nodes:
                for k in remappings:
                    N.remap_args.append([(name_space+'/'+k.remap_from).replace('//','/'),k.remap_to])
            _launch.start()

            process_name = str(_launch.pm.get_process_names_with_spawn_count()[0][0][0])
            self.app_list[app_name]['launch_list'][process_name] = {}
            self.app_list[app_name]['launch_list'][process_name]['name'] = process_name
            self.app_list[app_name]['launch_list'][process_name]['running'] = str(True)
            self.app_list[app_name]['launch_list'][process_name]['launch'] = _launch
            return True
            
        except Exception, inst:
            print inst
            self.app_list[app_name]['running'] = str(False)
            print "Fail to launch: %s"%(app_name)
            return False
        
    def process_listeners(self,name,exit_code):
        print "call process_listeners"
        for k in self.app_list.values():
            if k['launch_list'].has_key(name):                
                del k['launch_list'][name]       
        pass
        
        
