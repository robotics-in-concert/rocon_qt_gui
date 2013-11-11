#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_gateway_graph/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, QAbstractListModel, pyqtSignal, pyqtSlot,SIGNAL,SLOT

from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget,QLabel, QComboBox
from python_qt_binding.QtGui import QSizePolicy,QTextEdit ,QCompleter, QBrush,QDialog, QColor, QPen, QPushButton
from python_qt_binding.QtGui import QTabWidget, QPlainTextEdit,QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox
from python_qt_binding.QtGui import QTreeWidgetItem

from python_qt_binding.QtSvg import QSvgGenerator


#import rosgraph.impl.graph
#import rosservice
#import rostopic
import rospkg

######################
#dwlee
import rosnode
import roslib
import rospy
from concert_msgs.msg import ConcertClients
from rocon_app_manager_msgs.srv import GetPlatformInfo, Status, Invite, StartApp, StopApp
from rocon_app_manager_msgs.msg import PlatformInfo
###########################

#from .dotcode import RosGraphDotcodeGenerator
#from .interactive_graphics_view import InteractiveGraphicsView
#from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_gui.plugin import Plugin

# pydot requires some hacks
#from qt_dotgraph.pydotfactory import PydotFactory

# TODO: use pygraphviz instead, but non-deterministic layout will first be resolved in graphviz 2.30
# from qtgui_plugin.pygraphvizfactory import PygraphvizFactory


#from rocon_gateway import Graph
#from conductor_graph_info import ConductorGraphInfo


##############################################################################
# Gateway Classes
##############################################################################


class AdminApp(Plugin):    
    def __init__(self, context):
        self._context = context
        super(AdminApp, self).__init__(context)
        self.initialised = False
        self.setObjectName('Admin App')
        self._current_dotcode = None

        self.service_list = {}
        self.current_service = ""
        self.is_setting_dlg_live = False;
        
        self._widget = QWidget()
        
        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_admin_app'), 'ui', 'admin_app.ui')
        self._widget.setObjectName('AdminApphUi')
        loadUi(ui_file, self._widget, {})
      
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._widget.enable_btn.pressed.connect(self._enable_service)
        self._widget.disable_btn.pressed.connect(self._disable_service)
        self._widget.setting_btn.pressed.connect(self._setting_service)
           
        self._widget.service_tree_widget.itemClicked.connect(self._select_sesrvice) #concert item click event
        
        
                     
        context.add_widget(self._widget)
        
        self._update_service_list()
        
        
   
    def _update_service_list(self):
        
        self.service_list = {}
        
        #get service list
        service_name = "Delivery Service"
        self.service_list[service_name ] = {}      
        self.service_list[service_name ]["name"] = "Additional Service"
        self.service_list[service_name ]["client_list"] = self._get_client_list(service_name)
        
        
        #Top service
        service_item = QTreeWidgetItem()
        service_item.setText (0, service_name)
        
        #set Top Level Font
        font = service_item.font(0)        
        font.setPointSize(20)
        font.setBold(True)
        service_item.setFont(0,font)
        #set client item
        
        for k in self.service_list[service_name ]["client_list"]:
            client_item = QTreeWidgetItem()
            client_item.setText (0, k)
            font = client_item.font(0)        
            font.setPointSize(15)
            client_item.setFont(0,font)
            service_item.addChild (client_item)
        self._widget.service_tree_widget.addTopLevelItem(service_item)

        
        
        for k in range(self._widget.service_tree_widget.topLevelItemCount()):
           service_name = self._widget.service_tree_widget.topLevelItem(k).text(0)
           self.service_list[service_name ] = {}      
           self.service_list[service_name ]["name"]=service_name
           self.service_list[service_name ]["client_list"]=self._get_client_list(service_name)

           if(k ==0):
              self._update_client_list(service_name)        
              
        pass    
        

    def _update_client_list(self,service_name):
        client_list = self.service_list[service_name ]["client_list"]
        self._widget.client_list_widget.clear()
        for k in client_list: 
            
            main_widget=QWidget()
           
            ver_layout = QVBoxLayout(main_widget)
           
            ver_layout.setContentsMargins (9,9,9,9)
            ver_layout.setSizeConstraint (ver_layout.SetDefaultConstraint)
            
            sub_widget = QWidget()
            sub_widget.setAccessibleName('sub_widget')
             
            ver_layout.addWidget(sub_widget)            
            
            client_context_widget = QPlainTextEdit()
            client_context_widget.setObjectName(k+'_'+'app_context_widget')
            client_context_widget.setAccessibleName('app_context_widget')
            client_context_widget.appendPlainText("client name is "+k)
            #app_context_widget.appendHtml(k["app_context"])

            ver_layout.addWidget(client_context_widget)
            
            #add tab
            self._widget.client_list_widget.addTab(main_widget, k);

        pass
    
    
    
    def _get_service_info(self,service_name):
        self._widget.service_info_text.clear()
        self._widget.service_info_text.appendPlainText(service_name)
        pass
        
       
    def _select_sesrvice(self,Item):
   
        if(Item.text(0) in self.service_list):
            print '_select_service: '+ Item.text(0)
            self._get_service_info(Item.text(0))
            self.current_service = Item.text(0)
            
        else:
            print '_select_app: '+ Item.text(0)

        pass
    
    def _get_client_list(self,service_name):
        ##function call
        client_list = []
        ##function call
        client_list.append("client1");
        client_list.append("client2");
        client_list.append("client3");
        print client_list
        return client_list
        pass

    def _get_client_info(self):
        
        pass     
    
    def _setting_service(self):
    
        if self.is_setting_dlg_live:
            print "Dialog is live!!"
            self._setting_dlg.done(0)
            
          #dialog
        self._setting_dlg = QDialog(self._widget)             
        self._setting_dlg.setWindowTitle("Seting Configuration")
        self._setting_dlg.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        self._setting_dlg.setMinimumSize(500,0)
        dlg_rect = self._setting_dlg.geometry()

        #dialog layout
        ver_layout = QVBoxLayout(self._setting_dlg)
        ver_layout.setContentsMargins (9,9,9,9)
        
        #param layout
        text_grid_sub_widget = QWidget()
        text_grid_layout = QGridLayout(text_grid_sub_widget)            
        text_grid_layout.setColumnStretch (1, 0)
        text_grid_layout.setRowStretch (2, 0)

        #param 1
        name =u""
        title_widget1 = QLabel("Param1: ")
        context_widget1 = QTextEdit()
        context_widget1.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        context_widget1.setMinimumSize(0,30)
        context_widget1.append("")
        
        #param 2
        cancel=False
        title_widget2 = QLabel("Param2: ")           
        context_widget2 = QTextEdit()
        context_widget2.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        context_widget2.setMinimumSize(0,30)
        context_widget2.append("")
        
        #param 3
        cancel=False
        title_widget3 = QLabel("Param2: ")           
        context_widget3 = QTextEdit()
        context_widget3.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        context_widget3.setMinimumSize(0,30)
        context_widget3.append("")
        
        #add param
        text_grid_layout.addWidget(title_widget1)
        text_grid_layout.addWidget(context_widget1)
        
        text_grid_layout.addWidget(title_widget2)
        text_grid_layout.addWidget(context_widget2)
        
        text_grid_layout.addWidget(title_widget3)
        text_grid_layout.addWidget(context_widget3)
     
        #add param layout
        ver_layout.addWidget(text_grid_sub_widget) 
        
        #button layout
        button_hor_sub_widget = QWidget()
        button_hor_layout = QHBoxLayout(button_hor_sub_widget)

        params = {}
        params['param1'] = context_widget1
        params['param2'] = context_widget2
        params['param3'] = context_widget3
       
        #button
        btn_call = QPushButton("Set")
        btn_cancel = QPushButton("Cancel")
      
        btn_call.clicked.connect(lambda: self._setting_dlg.done(0))
        btn_call.clicked.connect(lambda: self._set_configuration(params))

        btn_cancel.clicked.connect(lambda: self._setting_dlg.done(0))
        
        #add button
        button_hor_layout.addWidget(btn_call)            
        button_hor_layout.addWidget(btn_cancel)

        #add button layout            
        ver_layout.addWidget(button_hor_sub_widget)
        self._setting_dlg.setVisible(True)
        self._setting_dlg.finished.connect(self._destroy_setting_dlg)
        self.is_setting_dlg_live = True
        
        
        pass
    
    def _enable_service(self):
        print "enable service: "+ self.current_service
        pass
        
        
    def _disable_service(self):
        print "disable service: "+ self.current_service
        pass
        
    def _destroy_setting_dlg(self):
        print "Distory!!!"
        self.is_setting_dlg_live = False
        pass
   
    def _set_configuration(self, params):        
        print self.current_service+" set param: "
        print "param1: "+ params['param1'].toPlainText()
        print "param2: "+ params['param2'].toPlainText()
        print "param3: "+ params['param3'].toPlainText()   
        pass        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
  
