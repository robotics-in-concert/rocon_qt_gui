#!/usr/bin/env python
#
# License: BSD
#   https://raw.github.com/robotics-in-concert/rocon_multimaster/master/rocon_remocon/LICENSE
#
##############################################################################
# Imports
##############################################################################

from __future__ import division
import os

from PyQt4 import uic
from PyQt4.QtCore import QFile, QIODevice, Qt, QAbstractListModel, pyqtSignal, pyqtSlot,SIGNAL,SLOT, QPoint
from PyQt4.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget,QLabel, QComboBox
from PyQt4.QtGui import QSizePolicy,QTextEdit ,QCompleter, QBrush,QDialog, QColor, QPen, QPushButton
from PyQt4.QtGui import QTabWidget, QPlainTextEdit,QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox
from PyQt4.QtGui import QMainWindow
from PyQt4.QtSvg import QSvgGenerator

class Remocon(QMainWindow):
    def __init__(self, parent, title):
        self._context = parent
        super(Remocon, self).__init__(parent)
        self.initialised = False
        self.setObjectName('Remocon')
        #self._current_dotcode = None
        
        #self._node_item_events = {}
        #self._client_info_list = {}
        self._widget_main = QWidget()
        self._widget_app_list = QWidget()                
        self._concert_cnt = 0;

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../ui/remocon.ui")
        uic.loadUi(path, self._widget_main)

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../ui/applist.ui")
        uic.loadUi(path, self._widget_app_list)
       
        #main widget
        self._widget_main.list_widget.itemDoubleClicked.connect(self._select_concert) #concert item double click event
        
        #app list widget
        self._widget_app_list.exit_button.pressed.connect(self._uninit_app_list)
        self._widget_app_list.list_widget.itemClicked.connect(self._select_app) #concert item click event
        self._widget_app_list.start_app_button.pressed.connect(self._start_app)
        self._widget_app_list.stop_app_button.pressed.connect(self._stop_app)
      
        #self._widget_app_list.test_1_button.pressed.connect(self._add_app_list_item)
            
        #test button
        self._widget_main.add_concert_list_button.pressed.connect(self._add_concert_list_item)
        self._widget_main.test_button_1.pressed.connect(self._test_1);
        #init       
        self._init()

########################################################################        
## test button
########################################################################        
    def _temp(self,arg):
        pass
            
    def _test_1(self):
        self._widget_app_list.show()
        pass
########################################################################        

    def _init(self):
        print '_init'
        self._concert_cnt = self._widget_main.list_widget.count()
        self._widget_main.show()
        self._connect_dlg_isValid = False
        
        self._current_selected_concert = ""
        self._current_selected_app = ""
        
        pass

    def _update_concert_list(self):
        print '_update_concert_list'
        pass
        
    def _refresh_concert_list(self):
        print '_refresh_concert_list'
        pass
    
    def _add_concert_list_item(self):
        print '_add_concert_list_item'
        
        concert_name = str(self._widget_main.list_widget.count())+'_concert' 
        self._widget_main.list_widget.insertItem(0,concert_name)
        
        #setting the list font
        font = self._widget_main.list_widget.item(0).font()        
        font.setPointSize(25)
        self._widget_main.list_widget.item(0).setFont(font)
        #setToolTip
        concert_info =""
        concert_info +="concert_name: "+concert_name+"\n"
        concert_info +="description:  "+"hello world"
        self._widget_main.list_widget.item(0).setToolTip(concert_info)
        pass

    def _connect_concert(self, params):        
        print "param1: "+ params['param1'].toPlainText()
        print "param2: "+ params['param2'].toPlainText()
        print "param3: "+ params['param3'].toPlainText()   
        self._widget_app_list.show()
        self._widget_app_list.move(self._widget_main.pos())
        self._widget_main.hide()
        pass
    
    def _select_app(self,Item):
    
        selected_app_name = Item.text()
        
        self._widget_app_list.app_info.clear()
        self._widget_app_list.app_info.appendPlainText(selected_app_name)
        self._current_selected_app = selected_app_name

    def _stop_app(self):
        print "Stop app: "+ self._current_selected_app
        pass
    
    def _start_app(self):
        print "Start app: "+ self._current_selected_app
        pass    
        
    def _select_concert(self, Item):

        list_widget = Item.listWidget()        
        if(list_widget.count() == list_widget.currentRow()+1):
            print "select last widget"
            return
        print '_connect_concert: '+ Item.text()
        
        if self._connect_dlg_isValid:
            print "Dialog is live!!"
            self._connect_dlg.done(0)
                
        #dialog
        self._connect_dlg = QDialog(self._widget_main)             
        self._connect_dlg.setWindowTitle("Seting Configuration")
        self._connect_dlg.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        self._connect_dlg.setMinimumSize(500,0)
        dlg_rect = self._connect_dlg.geometry()

        #dialog layout
        ver_layout = QVBoxLayout(self._connect_dlg)
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
        btn_call = QPushButton("Connect")
        btn_cancel = QPushButton("Cancel")
      
        btn_call.clicked.connect(lambda: self._connect_dlg.done(0))
        btn_call.clicked.connect(lambda: self._connect_concert(params))

        btn_cancel.clicked.connect(lambda: self._connect_dlg.done(0))
        
        #add button
        button_hor_layout.addWidget(btn_call)            
        button_hor_layout.addWidget(btn_cancel)

        #add button layout            
        ver_layout.addWidget(button_hor_sub_widget)
        self._connect_dlg.setVisible(True)
        self._connect_dlg.finished.connect(self._destroy_connect_dlg)
        self._connect_dlg_isValid = True
        pass
        
    def _destroy_connect_dlg(self):
        print "Distory!!!"
        self._connect_dlg_isValid = False
        pass

###########################################################################################################################
##app list widget
###########################################################################################################################
    def _init_app_list(self):
        pass
        
    def _uninit_app_list(self):
       
        self._widget_main.show()
        self._widget_main.move(self._widget_app_list.pos())
        self._widget_app_list.hide()
        pass    
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
