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
         
        
        ## factory builds generic dotcode items
        #self.dotcode_factory = PydotFactory()
        ## self.dotcode_factory = PygraphvizFactory()
        #self.dotcode_generator = RosGraphDotcodeGenerator()
        #self.dot_to_qt = DotToQtGenerator()
        #self._graph = ConductorGraphInfo()

        #rospack = rospkg.RosPack()
        #ui_file = os.path.join(rospack.get_path('rocon_remocon'), 'ui', 'remocon.ui')
       
        #self._widget.setObjectName('Remocon')

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../ui/remocon.ui")
        uic.loadUi(path, self._widget_main)

        path = os.path.join(os.path.dirname(os.path.abspath(__file__)),"../ui/applist.ui")
        uic.loadUi(path, self._widget_app_list)
     
        #for Graphical UI
        #self._scene = QGraphicsScene()
        #self._scene.setBackgroundBrush(Qt.white)
        #self._widget.graphics_view.setScene(self._scene)
      
        #For button Setting        
        ##self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('view-refresh'))
        #self._widget.refresh_graph_push_button.setIcon(QIcon.fromTheme('window-new'))
        #self._widget.refresh_graph_push_button.pressed.connect(self._update_conductor_graph)

        #self._widget.highlight_connections_check_box.toggled.connect(self._redraw_graph_view)
        #self._widget.auto_fit_graph_check_box.toggled.connect(self._redraw_graph_view)
        #self._widget.fit_in_view_push_button.setIcon(QIcon.fromTheme('zoom-original'))
        #self._widget.fit_in_view_push_button.pressed.connect(self._fit_in_view)

        #self._update_conductor_graph()
        #self._deferred_fit_in_view.connect(self._fit_in_view, Qt.QueuedConnection)
        #self._deferred_fit_in_view.emit()
        
        #add by dwlee
        #self._widget.tabWidget.currentChanged.connect(lambda index: self._test_function(index));
        #self._client_list_update_signal.connect(self._update_conductor_graph)
        #rospy.Subscriber("/concert/list_concert_clients", ConcertClients, self._update_client_list)
        
        #main widget
        self._widget_main.listWidget.itemDoubleClicked.connect(self._select_concert) #concert item double click event
        
        #app list widget
        self._widget_app_list.exit_button.pressed.connect(self._uninit_widget_app_list)  
        
        #test button
        self._widget_main.add_concert_list_button.pressed.connect(self._add_concert_list_item);
        self._widget_main.test_button_1.pressed.connect(self._test_1);
        
        
        self._init()
    
 
        
    def _temp(self,arg):
        pass
            
    def _test_1(self):
        self._widget_app_list.show()
        pass
    
    def _uninit_widget_app_list(self):
       
        self._widget_main.show()
        self._widget_main.move(self._widget_app_list.pos())
        self._widget_app_list.hide()
        pass    
    
    def _init(self):
        print '_init'
        self._concert_cnt = self._widget_main.listWidget.count()
        self._widget_main.show()
        pass

    def _update_concert_list(self):
        print '_update_concert_list'
        pass
        
    def _refresh_concert_list(self):
        print '_refresh_concert_list'
        pass
    
    def _add_concert_list_item(self):
        print '_add_concert_list_item'
        
        concert_name = str(self._widget_main.listWidget.count())+'_concert' 
        self._widget_main.listWidget.insertItem(0,concert_name)
        
        #setting the list font
        font = self._widget_main.listWidget.item(0).font()        
        font.setPointSize(25)
        self._widget_main.listWidget.item(0).setFont(font)
        #setToolTip
        concert_info =""
        concert_info +="concert_name: "+concert_name+"\n"
        concert_info +="description:  "+"hello world"
        self._widget_main.listWidget.item(0).setToolTip(concert_info)
        pass

    def _connect_concert(self, params):        
        print "param1: "+ params['param1'].toPlainText()
        print "param2: "+ params['param2'].toPlainText()
        print "param3: "+ params['param3'].toPlainText()   
        
        self._widget_app_list.show()
        self._widget_app_list.move(self._widget_main.pos())
        self._widget_main.hide()
        pass
    
    def _select_concert(self, Item):
        
        list_widget = Item.listWidget()        
        if(list_widget.count() == list_widget.currentRow()+1):
            print "select last widget"
            return
        print '_connect_concert: '+ Item.text()
        
        #if(self.is_login_dlg_valid):
        #    return 
    
        #dialog
        dlg = QDialog(self._widget_main) 
        
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
        btn_call = QPushButton("Call")
        btn_cancel = QPushButton("cancel")
      
        btn_call.clicked.connect(lambda: dlg.done(0))
        btn_call.clicked.connect(lambda: self._connect_concert(params))

        btn_cancel.clicked.connect(lambda: dlg.done(0))
        
        #add button
        button_hor_layout.addWidget(btn_call)            
        button_hor_layout.addWidget(btn_cancel)

        #add button layout            
        ver_layout.addWidget(button_hor_sub_widget)
        dlg.setVisible(True)

        pass
        
