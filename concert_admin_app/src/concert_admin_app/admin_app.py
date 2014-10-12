#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_qt_gui/license/LICENSE
#
##############################################################################
# Imports
##############################################################################
#system
from __future__ import division
import os
#pyqt
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QFile, QIODevice, Qt, Signal, QAbstractListModel, pyqtSignal, pyqtSlot,SIGNAL,SLOT
from python_qt_binding.QtGui import QFileDialog, QPainter, QWidget
from python_qt_binding.QtGui import QSizePolicy,QTextEdit ,QCompleter, QDialog, QColor, QPen, QPushButton
from python_qt_binding.QtGui import QTabWidget, QPlainTextEdit, QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox
from python_qt_binding.QtGui import QLabel
from python_qt_binding.QtGui import QTreeWidgetItem
from python_qt_binding.QtSvg import QSvgGenerator
#ros
import rospkg
#rqt
from qt_gui.plugin import Plugin

from .admin_app_interface import AdminAppInterface


##############################################################################
# Admin App
##############################################################################


class AdminApp(Plugin):

    _refresh_service_list_signal=Signal()

    def __init__(self, context):
        self._context=context
        super(AdminApp, self).__init__(context)
        self.initialised=False
        self.setObjectName('Admin App')

        self.is_setting_dlg_live=False;
        self._widget=QWidget()
        
        rospack=rospkg.RosPack()
        ui_file=os.path.join(rospack.get_path('concert_admin_app'), 'ui', 'admin_app.ui')
        self._widget.setObjectName('AdminApphUi')
        loadUi(ui_file, self._widget, {})
      
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))

        
        self.admin_app_interface = AdminAppInterface()
        self.admin_app_interface._reg_event_callback(self._refresh_service)
        self.current_service= None

        self.params_layout = None
        self.params_layout_items = []

        self._init_event()
        self._init_widget()
        
        context.add_widget(self._widget)
    
    def _init_event(self):
        self._widget.enable_disable_btn.pressed.connect(self._toggle_service)
        self._widget.refresh_btn.pressed.connect(self._refresh_service)
        
        self._widget.service_tree_widget.itemClicked.connect(self._select_service_tree_item) #concert item click event
        self._refresh_service_list_signal.connect(self._update_service_list)
        
        pass
    
    def _init_widget(self):
        self.params_layout = self._widget.findChildren(QGridLayout, "params_layout")[0]
        self.params_layout_items = []

        self._update_service_list()

        if not self.current_service:
            self._widget.enable_disable_btn.setDisabled(True)
        pass

    def _toggle_service(self):
        if self.current_service['enabled']:
            print "Enable Service: %s"%self.current_service['name']
            self.admin_app_interface.disable_service(self.current_service['name'])
        else:
            print "Disable Service: %s"%self.current_service['name']
            self.admin_app_interface.eable_service(self.current_service['name'])

    def _refresh_service(self):
        self._refresh_service_list_signal.emit()
        pass
    
    def _update_service_list(self):
        self._widget.service_tree_widget.clear()
        self._widget.service_info_text.clear()
        self._widgetitem_service_pair = {}
        service_list = self.admin_app_interface.service_list
        
        for k in service_list.values():
            #Top service
            service_item=QTreeWidgetItem(self._widget.service_tree_widget)
            #service_item=QTreeWidgetItem()
            service_item.setText (0, k['name'])
            
            #set Top Level Font
            font=service_item.font(0)        
            font.setPointSize(20)
            font.setBold(True)
            service_item.setFont(0,font)
            
            #set client item
            for l in k["client_list"]:
                client_item=QTreeWidgetItem()
                client_item.setText (0, l)
                font=client_item.font(0)        
                font.setPointSize(15)
                client_item.setFont(0,font)
                service_item.addChild (client_item)

            self._widgetitem_service_pair[service_item] = k
            
            #self._widget.service_tree_widget.addTopLevelItem(service_item)
        pass    
    
    def _set_service_info(self,service_name):
        service_list = self.admin_app_interface.service_list
        self._widget.service_info_text.clear()
        self._widget.service_info_text.appendHtml(service_list[service_name]['context'])
        pass
        
    def _select_service_tree_item(self,item):
        if item.parent() == None:
            selected_service = self._widgetitem_service_pair[item]
            
            print '_select_service: '+ selected_service['name']
            self._set_service_info(selected_service['name'])
            self.current_service = selected_service
            self._set_parameter(self.current_service['parameters'])
            
            if self.current_service['enabled']:
                self._widget.enable_disable_btn.setText("Disable")
                self._widget.enable_disable_btn.setDisabled(False)
            else :
                self._widget.enable_disable_btn.setText("Enable")
                self._widget.enable_disable_btn.setDisabled(False)


            
    def _set_parameter(self, parameters_path):
        print parameters_path
        params = self.admin_app_interface.get_srv_parameters(parameters_path)
        if self.params_layout_items:
            for item in self.params_layout_items:
                self.params_layout.removeWidget(item[0])
                self.params_layout.removeWidget(item[1])
        if params:
            self.params_layout.setColumnStretch (1, 0)
            self.params_layout.setRowStretch (2, 0)
            
            for param in params.keys():
                label =  QLabel(param)
                self.params_layout.addWidget(label)
                value = QTextEdit(params[param])
                value.setMaximumHeight(30) 
                self.params_layout.addWidget(value)
                self.params_layout_items.append((label, value))

            
    def _get_client_list(self,service_name):
        ##function call
        client_list=[]
        ##function call
        if service_name == "Delivery Service":
            client_list.append("client1");
            client_list.append("client2");
            client_list.append("client3");
        elif service_name == "Clean Service":
            client_list.append("client4");
            client_list.append("client5");
            client_list.append("client6");
        return client_list
        pass

    def _get_client_info(self):
        
        pass     
    
    def _setting_service(self):
    
        if self.is_setting_dlg_live:
            print "Dialog is live!!"
            self._setting_dlg.done(0)
            
        #dialog
        self._setting_dlg=QDialog(self._widget)             
        self._setting_dlg.setWindowTitle("Seting Configuration")
        self._setting_dlg.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        self._setting_dlg.setMinimumSize(500,0)
        dlg_rect=self._setting_dlg.geometry()

        #dialog layout
        ver_layout=QVBoxLayout(self._setting_dlg)
        ver_layout.setContentsMargins (9,9,9,9)
        
        #param layout
        text_grid_sub_widget=QWidget()
        text_grid_layout=QGridLayout(text_grid_sub_widget)            
        text_grid_layout.setColumnStretch (1, 0)
        text_grid_layout.setRowStretch (2, 0)

        #param 1
        name=u""
        title_widget1=QLabel("Param1: ")
        context_widget1=QTextEdit()
        context_widget1.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        context_widget1.setMinimumSize(0,30)
        context_widget1.append("")
        
        #param 2
        cancel=False
        title_widget2=QLabel("Param2: ")           
        context_widget2=QTextEdit()
        context_widget2.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        context_widget2.setMinimumSize(0,30)
        context_widget2.append("")
        
        #param 3
        cancel=False
        title_widget3=QLabel("Param2: ")           
        context_widget3=QTextEdit()
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
        button_hor_sub_widget=QWidget()
        button_hor_layout=QHBoxLayout(button_hor_sub_widget)

        params={}
        params['param1']=context_widget1
        params['param2']=context_widget2
        params['param3']=context_widget3
       
        #button
        btn_call=QPushButton("Set")
        btn_cancel=QPushButton("Cancel")
      
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
        self.is_setting_dlg_live=True
        
        pass

    def _set_configuration(self, params):        
        print self.current_service['name']+" set param: "
        print "param1: "+ params['param1'].toPlainText()
        print "param2: "+ params['param2'].toPlainText()
        print "param3: "+ params['param3'].toPlainText()   
        pass        
