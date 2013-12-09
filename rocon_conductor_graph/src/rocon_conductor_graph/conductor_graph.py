#!/usr/bin/env python

##############################################################################
# Imports
##############################################################################

from __future__ import division
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, QAbstractListModel, pyqtSignal, pyqtSlot,SIGNAL,SLOT,QSize
from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget,QLabel, QComboBox
from python_qt_binding.QtGui import QSizePolicy,QTextEdit ,QCompleter, QBrush, QColor, QPen, QPushButton
from python_qt_binding.QtGui import QVBoxLayout, QHBoxLayout, QMessageBox,QTabWidget, QPlainTextEdit
from python_qt_binding.QtGui import QGridLayout,QTextCursor,QToolTip, QDialog,QGraphicsItem
from python_qt_binding.QtSvg import QSvgGenerator

import rosgraph.impl.graph
import rosservice
import rostopic
import rospkg

######################

import rosnode
import roslib
import rospy

from concert_msgs.msg import ConcertClients

from rocon_std_msgs.msg import Remapping
from rocon_std_msgs.srv import GetPlatformInfo

from rocon_app_manager_msgs.srv import Status, Invite, StartApp, StopApp
###########################

from .dotcode import RosGraphDotcodeGenerator
from .interactive_graphics_view import InteractiveGraphicsView
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_gui.plugin import Plugin

# pydot requires some hacks
from qt_dotgraph.pydotfactory import PydotFactory
# TODO: use pygraphviz instead, but non-deterministic layout will first be resolved in graphviz 2.30
# from qtgui_plugin.pygraphvizfactory import PygraphvizFactory

from conductor_graph_info import ConductorGraphInfo

##############################################################################
# Utility Classes
##############################################################################

class RepeatedWordCompleter(QCompleter):
    """A completer that completes multiple times from a list"""

    def init(self, parent=None):
        QCompleter.init(self, parent)

    def pathFromIndex(self, index):
        path=QCompleter.pathFromIndex(self, index)
        lst=str(self.widget().text()).split(',')
        if len(lst) > 1:
            path='%s, %s' % (','.join(lst[:-1]), path)
        return path

    def splitPath(self, path):
        path=str(path.split(',')[-1]).lstrip(' ')
        return [path]


class GraphEventHandler():
    def __init__(self,tabWidget,item,callback_func):
        self._tabWidget=tabWidget
        self._callback_func=callback_func
        self._item=item
    def NodeEvent(self,event):
        self._callback_func(event)
        for k in range(self._tabWidget.count()):
             if self._tabWidget.tabText(k)==self._item._label.text():
                self._tabWidget.setCurrentIndex (k)
    def EdgeEvent(self,event):
        self._callback_func(event)
        self._item.set_color(QColor(0,0,255))
        
class NamespaceCompletionModel(QAbstractListModel):
    """Ros package and stacknames"""
    def __init__(self, linewidget, topics_only):
        super(QAbstractListModel, self).__init__(linewidget)
        self.names=[]

    def refresh(self, names):
        namesset=set()
        for n in names:
            namesset.add(str(n).strip())
            namesset.add("-%s" % (str(n).strip()))
        self.names=sorted(namesset)

    def rowCount(self, parent):
        return len(self.names)

    def data(self, index, role):
        if index.isValid() and (role==Qt.DisplayRole or role==Qt.EditRole):
            return self.names[index.row()]
        return None

##############################################################################
# Dynamic Argument Layer Classes
##############################################################################
class DynamicArgumentLayer():
    def __init__(self,dialog_layout,name='',add=False, params=[]):
        self.dlg_layout=dialog_layout
        self.name=name
        self.add=add
        self.params=params
        self.params_list=[]
        
        params_item=[]
        for k in self.params:
            param_name=k[0]
            param_type=k[1]
            param_widget=None
            params_item.append([param_name,param_widget,param_type])
        self.params_list.append(params_item)

        print "DAL: %s"%(self.params_list)
        
        self.arg_ver_sub_widget=QWidget()
        self.arg_ver_layout=QVBoxLayout(self.arg_ver_sub_widget)   
        self.arg_ver_layout.setContentsMargins (0,0,0,0)
        self._create_layout()
        
        pass
        
    def _create_layout(self):
        name_hor_sub_widget=QWidget()
        name_hor_layout=QHBoxLayout(name_hor_sub_widget)   
        
        name_widget=QLabel(self.name+": ")
        name_hor_layout.addWidget(name_widget)
        if self.add==True:
            btn_add=QPushButton("+",name_hor_sub_widget)

            btn_add.clicked.connect(self._push_param)
            btn_add.clicked.connect(self._update_item)
            name_hor_layout.addWidget(btn_add)                    
          
            btn_subtract=QPushButton("-",name_hor_sub_widget)
            btn_subtract.clicked.connect(self._pop_param)
            btn_subtract.clicked.connect(self._update_item)
            name_hor_layout.addWidget(btn_subtract)
            pass
            
        
        self.arg_ver_layout.addWidget(name_hor_sub_widget)
        self.dlg_layout.addWidget(self.arg_ver_sub_widget)
        self._update_item()
        pass
    
    def _update_item(self):
        widget_layout=self.arg_ver_layout
        item_list=self.params_list
        
        widget_list=widget_layout.parentWidget().children()
        while len(widget_list) > 2:
            added_arg_widget=widget_list.pop()
            widget_layout.removeWidget(added_arg_widget)
            added_arg_widget.setParent(None)   
            added_arg_widget.deleteLater()

        #resize
        dialog_widget=widget_layout.parentWidget().parentWidget()             
        dialog_widget.resize(dialog_widget.minimumSize())        
      
        for l in item_list:
            params_hor_sub_widget=QWidget()
            params_hor_layout=QHBoxLayout(params_hor_sub_widget)   
            
            for k in l:        
                param_name=k[0]
                param_type=k[2]
                name_widget=QLabel(param_name+": ")
                if param_type == 'string' or param_type == 'int':
                    k[1]=QTextEdit()
                    k[1].setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
                    k[1].setMinimumSize(0,30)
                    k[1].append("")
                elif param_type == 'bool':
                    k[1]=QTextEdit()
                    k[1] = QComboBox() 
                    k[1].setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
                    k[1].setMinimumSize(0,30)
                    
                    k[1].addItem("True",True)
                    k[1].addItem("False",False)

                params_hor_layout.addWidget(name_widget)
                params_hor_layout.addWidget(k[1])
            widget_layout.addWidget(params_hor_sub_widget)
        
    def _push_param(self):
        params_item=[]
        for k in self.params:
            param_name=k
            param_widget=None
            params_item.append([param_name,param_widget])
        self.params_list.append(params_item)

    def _pop_param(self):
        if len(self.params_list) > 1:
            self.params_list.pop()
        else:
            pass
            
    def _get_param_list(self):
        return self.params_list
        pass

##############################################################################
# ConductorGraph Classes
##############################################################################
        
class ConductorGraph(Plugin):

    _deferred_fit_in_view=Signal()
    _client_list_update_signal=Signal()
    
    def __init__(self, context):
        self._context=context
        super(ConductorGraph, self).__init__(context)
        self.initialised=False
        self.setObjectName('Conductor Graph')
        self._current_dotcode=None
        self._node_items=None
        self._edge_items=None
        self._node_item_events={}
        self._edge_item_events={}
        self._client_info_list={}
        self._widget=QWidget()
        self.cur_selected_client_name = ""
        self.pre_selected_client_name = ""
            
        # factory builds generic dotcode items
        self.dotcode_factory=PydotFactory()
        # self.dotcode_factory=PygraphvizFactory()
        self.dotcode_generator=RosGraphDotcodeGenerator()
        self.dot_to_qt=DotToQtGenerator()
        
        self._graph=ConductorGraphInfo()
        self._graph._reg_event_callback(self._update_client_list)
        self._graph._reg_period_callback(self._set_network_statisics)
        
        rospack=rospkg.RosPack()
        ui_file=os.path.join(rospack.get_path('rocon_conductor_graph'), 'ui', 'conductor_graph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('ConductorGraphUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene=QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        #self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('window-new'))
        self._widget.refresh_graph_push_button.pressed.connect(self._update_conductor_graph)

        self._widget.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        self._widget.fit_in_view_push_button.pressed.connect(self._fit_in_view)

        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()
        
        self._widget.tabWidget.currentChanged.connect(self._change_client_tab)
        self._client_list_update_signal.connect(self._update_conductor_graph)
        
        #rospy.Subscriber(concert_msgs.Strings.CONCERT_CLIENT_CHANGES, ConcertClients, self._update_client_list)
        
        context.add_widget(self._widget)
    
    def restore_settings(self, plugin_settings, instance_settings):
        self.initialised=True
        self._refresh_rosgraph()
    def shutdown_plugin(self):
        pass
    
    def _update_conductor_graph(self):
        # re-enable controls customizing fetched ROS graph

        self._refresh_rosgraph()
        self._update_client_tab()

    def _refresh_rosgraph(self):
        if not self.initialised:
            return
        self._update_graph_view(self._generate_dotcode())
        
    def _generate_dotcode(self):
        return self.dotcode_generator.generate_dotcode(rosgraphinst=self._graph,
                                                       dotcode_factory=self.dotcode_factory,
                                                       orientation='LR'
                                                       )
    def _update_graph_view(self, dotcode): 
        if dotcode==self._current_dotcode:
            return
        self._current_dotcode=dotcode
        self._redraw_graph_view()
   
    def _update_client_list(self):
        print "[conductor graph]: _update_client_list"       
        self._client_list_update_signal.emit()
        pass
    
    def _start_service(self,node_name,service_name):
        
        service=self._graph._client_info_list[node_name]['gateway_name']+"/"+service_name  
        info_text='' 
        
        if service_name=='status':
            service_handle=rospy.ServiceProxy(service, Status)
            call_result=service_handle()
            
            info_text="<html>"
            info_text +="<p>-------------------------------------------</p>"
            info_text +="<p><b>application_namespace: </b>" +call_result.application_namespace+"</p>"
            info_text +="<p><b>remote_controller: </b>" +call_result.remote_controller+"</p>"
            info_text +="<p><b>application_status: </b>" +call_result.application_status+"</p>"
            info_text +="</html>"
            self._client_list_update_signal.emit()
            
        elif service_name=='platform_info':
            service_handle=rospy.ServiceProxy(service, GetPlatformInfo)
            call_result=service_handle()
            
            info_text="<html>"
            info_text +="<p>-------------------------------------------</p>"
            info_text +="<p><b>os: </b>" +call_result.platform_info.os+"</p>"
            info_text +="<p><b>version: </b>" +call_result.platform_info.version+"</p>"
            info_text +="<p><b>system: </b>" +call_result.platform_info.system+"</p>"
            info_text +="<p><b>name: </b>" +call_result.platform_info.name+"</p>"
            info_text +="</html>"
            self._client_list_update_signal.emit()
            
        elif service_name=='invite':
            #sesrvice
            service_handle=rospy.ServiceProxy(service, Invite) 
            #dialog
            dlg=QDialog(self._widget) 
            dlg.setMinimumSize(400,0)
            dlg.setMaximumSize(400,0)
            dlg.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Expanding)
            #dialog layout
            ver_layout=QVBoxLayout(dlg)           
            ver_layout.setContentsMargins (0,0,0,0)

            dynamic_arg=[]
            dynamic_arg.append(DynamicArgumentLayer(ver_layout,'Remote Target Name',False,[('remote_target_name','string')]))
            dynamic_arg.append(DynamicArgumentLayer(ver_layout,'Application Namespace',False,[('application_namespace','string')]))
            dynamic_arg.append(DynamicArgumentLayer(ver_layout,'Cancel',False,[('cancel','bool')]))
            #button
            button_hor_sub_widget=QWidget()
            button_hor_layout=QHBoxLayout(button_hor_sub_widget)   
                   
            btn_call=QPushButton("Call")
            btn_cancel=QPushButton("cancel")
            
            btn_call.clicked.connect(lambda: dlg.done(0))
            btn_call.clicked.connect(lambda : self._call_invite_service(service,service_handle,dynamic_arg))

            btn_cancel.clicked.connect(lambda: dlg.done(0))
            #add button
            button_hor_layout.addWidget(btn_call)            
            button_hor_layout.addWidget(btn_cancel)
            #add button layout            
            ver_layout.addWidget(button_hor_sub_widget)

            dlg.setVisible(True)        

        elif service_name=='start_app':
            #sesrvice
            service_handle=rospy.ServiceProxy(service, StartApp) 
            #dialog
            dlg=QDialog(self._widget) 
            dlg.setMinimumSize(400,0)
            dlg.setMaximumSize(400,0)
            dlg.setSizePolicy(QSizePolicy.Fixed,QSizePolicy.Expanding)
            #dialog layout
            ver_layout=QVBoxLayout(dlg)           
            ver_layout.setContentsMargins (0,0,0,0)

            dynamic_arg=[]
            dynamic_arg.append(DynamicArgumentLayer(ver_layout,'Name',False,[('name','string')]))
            dynamic_arg.append(DynamicArgumentLayer(ver_layout,'Remappings',True,[('remap to','string'),('remap from','string')]))
            #button
            button_hor_sub_widget=QWidget()
            button_hor_layout=QHBoxLayout(button_hor_sub_widget)   
                   
            btn_call=QPushButton("Call")
            btn_cancel=QPushButton("cancel")
            
            btn_call.clicked.connect(lambda: dlg.done(0))
            btn_call.clicked.connect(lambda : self._call_start_app_service(service,service_handle,dynamic_arg))
            
            btn_cancel.clicked.connect(lambda: dlg.done(0))
            #add button
            button_hor_layout.addWidget(btn_call)            
            button_hor_layout.addWidget(btn_cancel)
            #add button layout            
            ver_layout.addWidget(button_hor_sub_widget)

            dlg.setVisible(True)        

        elif service_name=='stop_app':
            service_handle=rospy.ServiceProxy(service, StopApp)
            call_result=service_handle()

            info_text="<html>"
            info_text +="<p>-------------------------------------------</p>"
            info_text +="<p><b>stopped: </b>" +str(call_result.stopped)+"</p>"
            info_text +="<p><b>error_code: </b>" +str(call_result.error_code)+"</p>"
            info_text +="<p><b>message: </b>" +call_result.message+"</p>"
            info_text +="</html>"

            print 'stop app'
            self._update_client_tab()
        else:
            print 'has no service'
            return
        
        # display the result of calling service  
        # get tab widget handle
        service_text_widget=None
        cur_tab_widget=self._widget.tabWidget.currentWidget()        
        
        if cur_tab_widget==None:
            return
            
        object_name='services_text_widget'
        for k in cur_tab_widget.children():
            if k.objectName().count(object_name) >=1 :
                service_text_widget=k
                break
        if service_text_widget==None:
            return
            
        service_text_widget.clear()
        service_text_widget.appendHtml(info_text)

    def _call_invite_service(self,service,service_handle,dynamic_arg):        
        print "Invite Service: %s"%service
        remote_target_name=""
        application_namespace=""
        cancel=False

        for k in dynamic_arg:
            if k.name=='Remote Target Name':
                item_widget=k._get_param_list()[0][0][1]
                remote_target_name=item_widget.toPlainText()
            
            elif k.name=='Application Namespace':
                item_widgetwidget=k._get_param_list()[0][0][1]
                application_namespace=item_widget.toPlainText()
            
            elif k.name=='Cancel':
                item_widget=k._get_param_list()[0][0][1]    
                cancel=item_widget.itemData(item_widget.currentIndex())
        #calling service
        call_result=service_handle(remote_target_name,application_namespace,cancel)
        #status update
        self._client_list_update_signal.emit()
        # display the result of calling service  

        print "-------call_result-------"
        info_text="<html>"
        info_text +="<p>-------------------------------------------</p>"
        info_text +="<p><b>result: </b>" +str(call_result.result)+"</p>"
        info_text +="<p><b>error_code: </b>" +str(call_result.error_code)+"</p>"
        info_text +="<p><b>message: </b>" +call_result.message+"</p>"
        info_text +="</html>"        
        # get tab widget handle
        service_text_widget=None
        cur_tab_widget=self._widget.tabWidget.currentWidget()        
        if cur_tab_widget==None:
            return

        object_name='services_text_widget'
        for k in cur_tab_widget.children():
            if k.objectName().count(object_name) >=1 :
                service_text_widget=k
                break
        if service_text_widget==None:
            return
            
        service_text_widget.clear()
        service_text_widget.appendHtml(info_text)

        print call_result
        pass
    
    def _call_start_app_service(self,service,service_handle,dynamic_arg):
        print "Start App Service: %s"%service
        name=""
        remappings=[]
        for k in dynamic_arg:
            if k.name=='Name':
                name=k._get_param_list()[0][0][1].toPlainText()
            elif k.name=='Remappings':    
                for l in k._get_param_list():
                    remap_to=l[0][1].toPlainText()
                    remap_from=l[1][1].toPlainText()
                    remappings.append(Remapping(remap_to,remap_from))
        #calling service
        call_result=service_handle(name,remappings)
        #status update
        self._client_list_update_signal.emit()

        # display the result of calling service          
        print "-------call_result-------"
        info_text = ''
        info_text="<html>"
        info_text +="<p>-------------------------------------------</p>"
        info_text +="<p><b>started: </b>" +str(call_result.started)+"</p>"
        info_text +="<p><b>error_code: </b>" +str(call_result.error_code)+"</p>"
        info_text +="<p><b>message: </b>" +call_result.message+"</p>"
        info_text +="<p><b>app_namespace: </b>" +call_result.app_namespace+"</p>"
        info_text +="</html>"
        # get tab widget handle
        service_text_widget=None
        cur_tab_widget=self._widget.tabWidget.currentWidget()        
        if cur_tab_widget==None:
            return
        object_name='services_text_widget'
        for k in cur_tab_widget.children():
            if k.objectName().count(object_name) >=1 :
                service_text_widget=k
                break
        if service_text_widget==None:
            return
            
        service_text_widget.clear()
        service_text_widget.appendHtml(info_text)

        pass

    def _update_client_tab(self):
        print '[_update_client_tab]'
        self.pre_selected_client_name = self.cur_selected_client_name
        self._widget.tabWidget.clear()   
        
        for k in self._graph._client_info_list.values(): 
            main_widget=QWidget()
           
            ver_layout=QVBoxLayout(main_widget)
           
            ver_layout.setContentsMargins (9,9,9,9)
            ver_layout.setSizeConstraint (ver_layout.SetDefaultConstraint)
            
            sub_widget=QWidget()
            sub_widget.setAccessibleName('sub_widget')
            btn_grid_layout=QGridLayout(sub_widget)

            btn_grid_layout.setContentsMargins (9,9,9,9)

            btn_grid_layout.setColumnStretch (1, 0)
            btn_grid_layout.setRowStretch (2, 0)

            invite_btn=QPushButton("invite")
            platform_info_btn=QPushButton("platform_info")
            status_btn=QPushButton("status")
            start_app_btn=QPushButton("start_app")
            stop_app_btn=QPushButton("stop_app")
            get_app_list_app_btn=QPushButton("get_app_list")                        

            invite_btn.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"invite"))
            platform_info_btn.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"platform_info"))  
            status_btn.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"status"))  
            start_app_btn.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"start_app"))  
            stop_app_btn.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"stop_app"))
            get_app_list_app_btn.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"get_app_list_app"))    
                    
            btn_grid_layout.addWidget(invite_btn)
            btn_grid_layout.addWidget(platform_info_btn)
            btn_grid_layout.addWidget(status_btn)
            btn_grid_layout.addWidget(start_app_btn)
            btn_grid_layout.addWidget(stop_app_btn)
             
            ver_layout.addWidget(sub_widget)            
            
            app_context_widget=QPlainTextEdit()
            app_context_widget.setObjectName(k["name"]+'_'+'app_context_widget')
            app_context_widget.setAccessibleName('app_context_widget')
            app_context_widget.appendHtml(k["app_context"])
            app_context_widget.setReadOnly(True) 
            
            cursor = app_context_widget.textCursor()
            cursor.movePosition(QTextCursor.Start,QTextCursor.MoveAnchor,0)
            app_context_widget.setTextCursor(cursor)
            ver_layout.addWidget(app_context_widget)
            
            services_text_widget=QPlainTextEdit()
            services_text_widget.setObjectName(k["name"]+'_'+'services_text_widget')
            services_text_widget.setReadOnly(True) 
            cursor = services_text_widget.textCursor()
            cursor.movePosition(QTextCursor.Start,QTextCursor.MoveAnchor,0)
            services_text_widget.setTextCursor(cursor)            
            ver_layout.addWidget(services_text_widget)
            
            # new icon
            path=""
            if k["is_new"]==True:
                path=os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../resources/images/new.gif")            

            #add tab
            self._widget.tabWidget.addTab(main_widget,QIcon(path), k["name"]);

        #set previous selected tab
        print "update item [%s]"%self.pre_selected_client_name
        for k in range(self._widget.tabWidget.count()):
            tab_text=self._widget.tabWidget.tabText(k)
            if tab_text == self.pre_selected_client_name:
                self._widget.tabWidget.setCurrentIndex(k)

    def _change_client_tab(self,index):
        self.cur_selected_client_name = self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex())
        if self._widget.tabWidget.widget(index) !=None:
            for k in  self._widget.tabWidget.widget(index).children():
                if k.objectName().count("services_text_widget"):
                    k.clear()
        pass    
        
    def _set_network_statisics(self):
        if self._edge_items == None:
            return
        else:
            for edge_items in self._edge_items.itervalues():
                for edge_item in edge_items:
                     edge_dst_name=edge_item.to_node._label.text()
                     edge_item.setToolTip(str(self._graph._client_info_list[edge_dst_name]['conn_stats']))
                     
    def _redraw_graph_view(self):
        self._scene.clear()
        self._node_item_events={}
        self._edge_item_events={}
        self._node_items=None
        self._edge_items=None

        if self._widget.highlight_connections_check_box.isChecked():
            highlight_level=3
        else:
            highlight_level=1
            
        highlight_level=3 if self._widget.highlight_connections_check_box.isChecked() else 1

        # layout graph and create qt items
        (nodes, edges)=self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level=highlight_level,
                                                            same_label_siblings=True)
        self._node_items=nodes
        self._edge_items=edges

        # if we wish to make special nodes, do that here (maybe subclass GraphItem, just like NodeItem does)
        #node
        for node_item in nodes.itervalues():
            # set the color of conductor to orange           
            if node_item._label.text()==self._graph._concert_conductor_name:
                royal_blue=QColor(65, 105, 255)
                node_item._default_color=royal_blue
                node_item.set_color(royal_blue)

            # redefine mouse event
            self._node_item_events[node_item._label.text()]=GraphEventHandler(self._widget.tabWidget,node_item,node_item.mouseDoubleClickEvent);
            node_item.mouseDoubleClickEvent=self._node_item_events[node_item._label.text()].NodeEvent;
            
            self._scene.addItem(node_item)
            
        #edge
        for edge_items in edges.itervalues():
            for edge_item in edge_items:
                #redefine the edge hover event
                
                self._edge_item_events[edge_item._label.text()]=GraphEventHandler(self._widget.tabWidget,edge_item,edge_item._label.hoverEnterEvent);
                edge_item._label.hoverEnterEvent =self._edge_item_events[edge_item._label.text()].EdgeEvent;
                
                #self._edge_item_events[edge_item._label.text()]=GraphEventHandler(self._widget.tabWidget,edge_item,edge_item.mouseDoubleClickEvent);
                #edge_item.mouseDoubleClickEvent=self._edge_item_events[edge_item._label.text()].EdgeEvent;

                edge_item.add_to_scene(self._scene)

                #set the color of node as connection strength one of red, yellow, green
                edge_dst_name=edge_item.to_node._label.text()
                if self._graph._client_info_list.has_key(edge_dst_name):   
                  connection_strength=self._graph._client_info_list[edge_dst_name]['connection_strength']
                  if connection_strength=='very_strong':
                      green=QColor(0, 255, 0)
                      edge_item._default_color=green
                      edge_item.set_color(green)

                  elif connection_strength=='strong':
                      green_yellow=QColor(125, 255,0)
                      edge_item._default_color=green_yellow
                      edge_item.set_color(green_yellow)
                        
                  elif connection_strength=='normal':
                      yellow=QColor(238, 238,0)
                      edge_item._default_color=yellow
                      edge_item.set_color(yellow)

                  elif connection_strength=='weak':
                      yellow_red=QColor(255, 125,0)
                      edge_item._default_color=yellow_red
                      edge_item.set_color(yellow_red)
                      
                  elif connection_strength=='very_weak':
                      red=QColor(255, 0,0)
                      edge_item._default_color=red
                      edge_item.set_color(red)
                #set the tooltip about network information
                edge_item.setToolTip(str(self._graph._client_info_list[edge_dst_name]['conn_stats']))    

        self._scene.setSceneRect(self._scene.itemsBoundingRect())
  
        if self._widget.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

