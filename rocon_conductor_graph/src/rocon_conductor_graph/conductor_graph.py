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
from python_qt_binding.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget,QLabel, QComboBox, QSizePolicy,QTextEdit ,QCompleter, QBrush,QDialog, QColor, QPen, QPushButton, QTabWidget, QPlainTextEdit,QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox
from python_qt_binding.QtSvg import QSvgGenerator

import rosgraph.impl.graph
import rosservice
import rostopic
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

from .dotcode import RosGraphDotcodeGenerator
from .interactive_graphics_view import InteractiveGraphicsView
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_gui.plugin import Plugin

# pydot requires some hacks
from qt_dotgraph.pydotfactory import PydotFactory
# TODO: use pygraphviz instead, but non-deterministic layout will first be resolved in graphviz 2.30
# from qtgui_plugin.pygraphvizfactory import PygraphvizFactory

from rocon_gateway import Graph
from conductor_graph_info import ConductorGraphInfo



##############################################################################
# Utility Classes
##############################################################################


class RepeatedWordCompleter(QCompleter):
    """A completer that completes multiple times from a list"""

    def init(self, parent=None):
        QCompleter.init(self, parent)

    def pathFromIndex(self, index):
        path = QCompleter.pathFromIndex(self, index)
        lst = str(self.widget().text()).split(',')
        if len(lst) > 1:
            path = '%s, %s' % (','.join(lst[:-1]), path)
        return path

    def splitPath(self, path):
        path = str(path.split(',')[-1]).lstrip(' ')
        return [path]


class NodeEventHandler():
    def __init__(self,tabWidget,node_item,callback_func):
        self._tabWidget = tabWidget
        self._callback_func = callback_func
        self._node_item = node_item


    def NodeEvent(self,event):
        for k in range(self._tabWidget.count()):
             if self._tabWidget.tabText(k) == self._node_item._label.text():
                self._tabWidget.setCurrentIndex (k)
 
        
class NamespaceCompletionModel(QAbstractListModel):
    """Ros package and stacknames"""
    def __init__(self, linewidget, topics_only):
        super(QAbstractListModel, self).__init__(linewidget)
        self.names = []

    def refresh(self, names):
        namesset = set()
        for n in names:
            namesset.add(str(n).strip())
            namesset.add("-%s" % (str(n).strip()))
        self.names = sorted(namesset)

    def rowCount(self, parent):
        return len(self.names)

    def data(self, index, role):
        if index.isValid() and (role == Qt.DisplayRole or role == Qt.EditRole):
            return self.names[index.row()]
        return None

##############################################################################
# Gateway Classes
##############################################################################


class ConductorGraph(Plugin):

    _deferred_fit_in_view = Signal()
    _client_list_update_signal = Signal()
    
    def __init__(self, context):
        self._context = context
        super(ConductorGraph, self).__init__(context)
        self.initialised = False
        self.setObjectName('Conductor Graph')
        self._current_dotcode = None
        
        self._node_item_events = {}
        self._client_info_list = {}
        self._widget = QWidget()
        

        # factory builds generic dotcode items
        self.dotcode_factory = PydotFactory()
        # self.dotcode_factory = PygraphvizFactory()
        self.dotcode_generator = RosGraphDotcodeGenerator()
        self.dot_to_qt = DotToQtGenerator()
        self._graph = ConductorGraphInfo()

        rospack = rospkg.RosPack()
        ui_file = os.path.join(rospack.get_path('rocon_conductor_graph'), 'ui', 'conductor_graph.ui')
        #ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'ui', 'conductor_graph.ui')
        loadUi(ui_file, self._widget, {'InteractiveGraphicsView': InteractiveGraphicsView})
        self._widget.setObjectName('ConductorGraphUi')

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        self._scene = QGraphicsScene()
        self._scene.setBackgroundBrush(Qt.white)
        self._widget.graphics_view.setScene(self._scene)

        #self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('window-new'))
        self._widget.refresh_graph_push_button.pressed.connect(self._update_conductor_graph)

        self._widget.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)
        self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        self._widget.fit_in_view_push_button.pressed.connect(self._fit_in_view)

        self._update_conductor_graph()
        self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        self._deferred_fit_in_view.emit()
        
        #add by dwlee
        self._widget.tabWidget.currentChanged.connect(lambda index: self._test_function(index));
        self._client_list_update_signal.connect(self._update_conductor_graph)
        rospy.Subscriber("/concert/list_concert_clients", ConcertClients, self._update_client_list)
        
        context.add_widget(self._widget)
  
    def _test_function(self, index):

        # get tab widget handle
        service_text_widget = None        
        cur_tab_widget = self._widget.tabWidget.currentWidget()
        
        if cur_tab_widget == None:
            return
        
        object_name = 'services_text_widget'
        for k in cur_tab_widget.children():
            if k.objectName().count(object_name) >= 1 :
                service_text_widget = k
                break
       
        if service_text_widget == None:
            return
            
        service_text_widget.clear()

        
    def restore_settings(self, plugin_settings, instance_settings):
        self.initialised = True
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
        if dotcode == self._current_dotcode:
            return
        self._current_dotcode = dotcode
        self._redraw_graph_view()
   
    def _update_client_list(self,data):
        self._client_list_update_signal.emit()
        pass
    
    def _start_service(self,node_name,service_name):

        # get tab widget handle
        service_text_widget = None

        cur_tab_widget = self._widget.tabWidget.currentWidget()        
        if cur_tab_widget == None:
            return
            
        object_name = 'services_text_widget'
        for k in cur_tab_widget.children():
            if k.objectName().count(object_name) >= 1 :
                service_text_widget = k
                break
       
        if service_text_widget == None:
            return
            
        service_text_widget.clear()
        
        service = self._graph._client_info_list[node_name]['gateway_name']+"/"+service_name  
        info_text = '' 
        
        if service_name == 'status':
            service_handle = rospy.ServiceProxy(service, Status)
            call_result = service_handle()
            
            info_text = "<html>"
            info_text += "<p>-------------------------------------------</p>"
            info_text += "<p><b>application_namespace: </b>" +call_result.application_namespace+"</p>"
            info_text += "<p><b>remote_controller: </b>" +call_result.remote_controller+"</p>"
            info_text += "<p><b>application_status: </b>" +call_result.application_status+"</p>"
            info_text +="</html>"
            
        elif service_name == 'platform_info':
        
            service_handle = rospy.ServiceProxy(service, GetPlatformInfo)
            call_result = service_handle()
            
            info_text = "<html>"
            info_text += "<p>-------------------------------------------</p>"
            info_text += "<p><b>platform: </b>" +call_result.platform_info.platform+"</p>"
            info_text += "<p><b>system: </b>" +call_result.platform_info.system+"</p>"
            info_text += "<p><b>robot: </b>" +call_result.platform_info.robot+"</p>"
            info_text += "<p><b>name: </b>" +call_result.platform_info.name+"</p>"
            info_text +="</html>"

        elif service_name == 'invite':
            #sesrvice
            service_handle = rospy.ServiceProxy(service, Invite)            
            #dialog
            dlg = QDialog(self._widget) 
            
            dlg.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
            dlg.setMinimumSize(500,0)
            dlg_rect = dlg.geometry()

            #dialog layout
            ver_layout = QVBoxLayout(dlg)           
            ver_layout.setContentsMargins (9,9,9,9)
            
            #param layout
            text_grid_sub_widget = QWidget()
            text_grid_layout = QGridLayout(text_grid_sub_widget)            
            text_grid_layout.setColumnStretch (1, 0)
            text_grid_layout.setRowStretch (2, 0)

            #param 1
            remote_target_name =u""
            title_widget1 = QLabel("remote_target_name: ")
            context_widget1 = QTextEdit()
            context_widget1.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
            context_widget1.setMinimumSize(0,30)
            context_widget1.append("")
            
            #param 2
            application_namespace=u""
            title_widget2 = QLabel("application_namespace: ")
            context_widget2 = QTextEdit()
            context_widget2.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
            context_widget2.setMinimumSize(0,30)
            context_widget2.append("")
            
            #param 3
            cancel=False
            title_widget3 = QLabel("cancel: ")           
            context_widget3 = QComboBox() 
            context_widget3.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
            context_widget3.setMinimumSize(0,30)
            
            context_widget3.addItem("True",True)
            context_widget3.addItem("False",False)
            
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

            #button
            btn_call = QPushButton("Call")
            btn_cancel = QPushButton("cancel")
    
            btn_call.clicked.connect(lambda: dlg.done(0))
            btn_call.clicked.connect(lambda : service_handle(context_widget1.toPlainText(),
                                                        context_widget2.toPlainText(),
                                                        context_widget3.itemData(context_widget3.currentIndex())
                                                        ))
            
            btn_cancel.clicked.connect(lambda: dlg.done(0))
            
            #add button
            button_hor_layout.addWidget(btn_call)            
            button_hor_layout.addWidget(btn_cancel)

            #add button layout            
            ver_layout.addWidget(button_hor_sub_widget)

            dlg.setVisible(True)

        elif service_name == 'start_app':
            #service
            service_handle = rospy.ServiceProxy(service, StartApp)            
            #dialog
            dlg = QDialog(self._widget) 
            
            dlg.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
            dlg.setMinimumSize(500,0)
            dlg_rect = dlg.geometry()

            #dialog layout
            ver_layout = QVBoxLayout(dlg)           
            ver_layout.setContentsMargins (9,9,9,9)
            
            #param layout
            text_grid_sub_widget = QWidget()
            text_grid_layout = QGridLayout(text_grid_sub_widget)            
            text_grid_layout.setColumnStretch (1, 0)
            text_grid_layout.setRowStretch (2, 0)

            #param 1
            name =u""
            title_widget1 = QLabel("name: ")
            context_widget1 = QTextEdit()
            context_widget1.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
            context_widget1.setMinimumSize(0,30)
            context_widget1.append("")
            
            #param 2
            cancel=False
            title_widget2 = QLabel("remappings: ")           
            context_widget2 = QTextEdit()
            context_widget2.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
            context_widget2.setMinimumSize(0,30)
            context_widget2.append("")
            
            #add param
            text_grid_layout.addWidget(title_widget1)
            text_grid_layout.addWidget(context_widget1)
            
            text_grid_layout.addWidget(title_widget2)
            text_grid_layout.addWidget(context_widget2)
         
            #add param layout
            ver_layout.addWidget(text_grid_sub_widget) 
            
            #button layout
            button_hor_sub_widget = QWidget()
            button_hor_layout = QHBoxLayout(button_hor_sub_widget)

            #button
            btn_call = QPushButton("Call")
            btn_cancel = QPushButton("cancel")
    
            btn_call.clicked.connect(lambda: dlg.done(0))
            btn_call.clicked.connect(lambda : service_handle(context_widget1.toPlainText(),
                                                        []))
            
            btn_cancel.clicked.connect(lambda: dlg.done(0))
            
            #add button
            button_hor_layout.addWidget(btn_call)            
            button_hor_layout.addWidget(btn_cancel)

            #add button layout            
            ver_layout.addWidget(button_hor_sub_widget)

            dlg.setVisible(True)
            
            print 'start app'
        
        elif service_name == 'stop_app':
            service_handle = rospy.ServiceProxy(service, StopApp)
            call_result = service_handle()
            print 'stop app'
        else:
            print 'has no service'

        service_text_widget.appendHtml(info_text)
        
        
    def _update_client_tab(self):
        self._widget.tabWidget.clear()    
        for k in self._graph._client_info_list.values(): 
            main_widget=QWidget()
           
            ver_layout = QVBoxLayout(main_widget)
           
            ver_layout.setContentsMargins (9,9,9,9)
            ver_layout.setSizeConstraint (ver_layout.SetDefaultConstraint)
            
            sub_widget = QWidget()
            sub_widget.setAccessibleName('sub_widget')
            btn_grid_layout = QGridLayout(sub_widget)

            btn_grid_layout.setContentsMargins (9,9,9,9)

            btn_grid_layout.setColumnStretch (1, 0)
            btn_grid_layout.setRowStretch (2, 0)

            btn_invite = QPushButton("invite")
            btn_platform_info = QPushButton("platform_info")
            btn_status = QPushButton("status")
            btn_start_app = QPushButton("start_app")
            btn_stop_app = QPushButton("stop_app")            

            btn_invite.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"invite"))
            btn_platform_info.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"platform_info"))  
            btn_status.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"status"))  
            btn_start_app.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"start_app"))  
            btn_stop_app.clicked.connect(lambda: self._start_service(self._widget.tabWidget.tabText(self._widget.tabWidget.currentIndex()),"stop_app"))  
                    
            btn_grid_layout.addWidget(btn_invite)
            btn_grid_layout.addWidget(btn_platform_info)
            btn_grid_layout.addWidget(btn_status)
            btn_grid_layout.addWidget(btn_start_app)
            btn_grid_layout.addWidget(btn_stop_app)
             
            ver_layout.addWidget(sub_widget)            
            
            app_context_widget = QPlainTextEdit()
            app_context_widget.setObjectName(k["app_name"]+'_'+'app_context_widget')
            app_context_widget.setAccessibleName('app_context_widget')
            app_context_widget.appendHtml(k["app_context"])
            
            ver_layout.addWidget(app_context_widget)
            
            services_text_widget = QPlainTextEdit()
            services_text_widget.setObjectName(k["app_name"]+'_'+'services_text_widget')
            ver_layout.addWidget(services_text_widget)
            
            # new icon
            path = ""
            if k["isNew"] == True:
                path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../resources/images/new.gif")            
            
            #add tab
            self._widget.tabWidget.addTab(main_widget,QIcon(path), k["app_name"]);
        
    def _redraw_graph_view(self):
        self._scene.clear()
        self._node_item_events = {}

        if self._widget.highlight_connections_check_box.isChecked():
            highlight_level = 3
        else:
            highlight_level = 1

        # layout graph and create qt items
        (nodes, edges) = self.dot_to_qt.dotcode_to_qt_items(self._current_dotcode,
                                                            highlight_level=highlight_level,
                                                            same_label_siblings=True)
        # if we wish to make special nodes, do that here (maybe subclass GraphItem, just like NodeItem does)
        #node
        for node_item in nodes.itervalues():
            # set the color of conductor to orange           
            if node_item._label.text() == self._graph._concert_conductor_name:
                royal_blue = QColor(65, 105, 255)
                node_item._default_color = royal_blue
                node_item.set_color(royal_blue)
            
            #set the uuid
            
            #print self._graph._client_info_list
            if self._graph._client_info_list.has_key(str(node_item._label.text())):         
                node_item.setToolTip(self._graph._client_info_list[node_item._label.text()]['uuid'])
          
            # redefine mouse event
            self._node_item_events[node_item._label.text()] = NodeEventHandler(self._widget.tabWidget,node_item,node_item.mouseDoubleClickEvent);
            node_item.mouseDoubleClickEvent = self._node_item_events[node_item._label.text()].NodeEvent;
            
            self._scene.addItem(node_item)
            
        #edge
        for edge_items in edges.itervalues():
            for edge_item in edge_items:
                edge_item.add_to_scene(self._scene)
                 #set the color of node as connection strength one of red, yellow, green
                edge_dst_name = edge_item.to_node._label.text()
                if self._graph._client_info_list.has_key(edge_dst_name):   
                  
                  connection_strength = self._graph._client_info_list[edge_dst_name]['connection_strength']
                  if connection_strength == 'very_strong':
                      green = QColor(0, 255, 0)
                      edge_item._default_color = green
                      edge_item.set_color(green)

                  elif connection_strength == 'strong':
                      green_yellow = QColor(125, 255,0)
                      edge_item._default_color = green_yellow
                      edge_item.set_color(green_yellow)
                        
                  elif connection_strength == 'normal':
                      yellow = QColor(238, 238,0)
                      edge_item._default_color = yellow
                      edge_item.set_color(yellow)

                  elif connection_strength == 'weak':
                      yellow_red = QColor(255, 125,0)
                      edge_item._default_color = yellow_red
                      edge_item.set_color(yellow_red)
                      
                  elif connection_strength == 'very_weak':
                      red = QColor(255, 0,0)
                      edge_item._default_color = red
                      edge_item.set_color(red)
    
        self._scene.setSceneRect(self._scene.itemsBoundingRect())
  
        if self._widget.auto_fit_graph_check_box.isChecked():
            self._fit_in_view()

    def _fit_in_view(self):
        self._widget.graphics_view.fitInView(self._scene.itemsBoundingRect(), Qt.KeepAspectRatio)

