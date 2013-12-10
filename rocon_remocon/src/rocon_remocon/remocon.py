#!/usr/bin/env python
##############################################################################
# Imports
##############################################################################
#system
import sys
import os
import subprocess
import atexit
import signal
import string
import uuid
import time

#pyqt
from PyQt4 import uic
from PyQt4.QtCore import pyqtSlot,SIGNAL,SLOT, QPoint, QString,QEvent
from PyQt4.QtCore import QFile, QIODevice, Qt, QAbstractListModel, pyqtSignal, QSize
from PyQt4.QtGui import QFileDialog, QGraphicsScene, QIcon, QImage, QPainter, QWidget,QLabel, QComboBox
from PyQt4.QtGui import QSizePolicy,QTextEdit ,QCompleter, QBrush,QDialog, QColor, QPen, QPushButton
from PyQt4.QtGui import QTabWidget, QPlainTextEdit,QGridLayout, QVBoxLayout, QHBoxLayout, QMessageBox
from PyQt4.QtGui import QMainWindow
from PyQt4.QtSvg import QSvgGenerator

##ros
from remocon_info import RemoconInfo

##############################################################################
# Remocon Role
##############################################################################

class RemoconRole(QMainWindow):
    def __init__(self, parent, title,application,concert_index="", concert_name="", concert_ip='127.0.0.1', host_name='127.0.0.1'):
        self.concert_index= concert_index
        self.concert_ip= concert_ip
        self.concert_name= concert_name    
        self.host_name= host_name    
        self._context= parent
        self.application = application

        super(RemoconRole, self).__init__(parent)
        self.initialised= False
        
        self._widget_app_list= QWidget()                
        self._widget_role_list= QWidget()
        
        self.concert_list= {}
        
        self.role_list= {}
        self.cur_selected_role= 0
        
        self.app_list= {}
        self.cur_selected_app= 0
        
        self.remocon_info= RemoconInfo()
    
        path= os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../ui/applist.ui")
        uic.loadUi(path, self._widget_app_list)
        
        path= os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../ui/rolelist.ui")
        uic.loadUi(path, self._widget_role_list)

        self.temp_icon_path= "%s/.ros/rocon/remocon/image/"%(os.getenv("HOME"))   
        self.temp_cache_path="%s/.ros/rocon/remocon/cache/"%(os.getenv("HOME"))
        if not os.path.isdir(self.temp_cache_path):
            os.makedirs(self.temp_cache_path)
        self.temp_cache_path+="concert_info_list.cache"
        self.scripts_path= os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../scripts/")

        #role list widget
        self._widget_role_list.role_list_widget.setIconSize(QSize(50,50))
        self._widget_role_list.role_list_widget.itemDoubleClicked.connect(self._select_role_list) 
        self._widget_role_list.back_btn.pressed.connect(self._back_role_list) 
        self._widget_role_list.refresh_btn.pressed.connect(self._refresh_role_list)  
        #app list widget
        self._widget_app_list.app_list_widget.setIconSize(QSize(50,50))
        self._widget_app_list.app_list_widget.itemDoubleClicked.connect(self._start_app)                
        self._widget_app_list.back_btn.pressed.connect(self._uninit_app_list)
        self._widget_app_list.app_list_widget.itemClicked.connect(self._select_app_list) #concert item click event
        self._widget_app_list.start_app_btn.pressed.connect(self._start_app)
        self._widget_app_list.stop_app_btn.pressed.connect(self._stop_app)
        self._widget_app_list.refresh_btn.pressed.connect(self._refresh_app_list)
        #init
        self._init()     

    def _init(self):

        self._read_cache()
        self._init_role_list()   
        self._widget_role_list.show()
        self.initialised= True        
        
        pass
################################################################################################################
##role list widget
################################################################################################################
    def _init_role_list(self):        

        if not self.remocon_info._connect(self.concert_name, self.concert_ip,self.host_name):
            return False
        self._refresh_role_list()    
        return True
        
    def _uninit_role_list(self):
        print "[RemoconRole]_uninit_role_list"
        self.remocon_info._shutdown()
        self.cur_selected_role= 0    
     
    def _select_role_list(self,Item):
        print '_select_role_list: '+ Item.text()
        self.cur_selected_role= str(Item.text())

        self.remocon_info._select_role(self.cur_selected_role)
        
        self._widget_app_list.show()
        self._widget_app_list.move(self._widget_role_list.pos())
        self._widget_role_list.hide()
        self._init_app_list()
        pass   
        
    def _back_role_list(self):
        self._uninit_role_list()
        execute_path= self.scripts_path+'rocon_remocon' ##command
        execute_path += " "+"'"+self.host_name+"'" ##arg1
        
        os.execv(self.scripts_path+'rocon_remocon',['',self.host_name])
        
        print "Spawning: %s"%(execute_path)
        pass

    def _refresh_role_list(self):
        self.role_list= {}
        self._widget_role_list.role_list_widget.clear()
        
        #get role list
        self.role_list= self.remocon_info._get_role_list()
       
        #set list widget item
        for k in self.role_list.values():
            self._widget_role_list.role_list_widget.insertItem(0, k['name'])
            #setting the list font
            font= self._widget_role_list.role_list_widget.item(0).font()        
            font.setPointSize(13)
            self._widget_role_list.role_list_widget.item(0).setFont(font)
        
        ##get concert info
        concert_info= self.remocon_info._get_concert_info()
        
        if concert_info.has_key('name'):
            self.concert_list[self.concert_index]['name']= concert_info['name']
        if concert_info.has_key('description'):
            self.concert_list[self.concert_index]['description']= concert_info['description']
        if concert_info.has_key('icon'):
            self.concert_list[self.concert_index]['icon']= concert_info['icon']

        self.concert_list[self.concert_index]['flag']= '1'        
        self._write_cache()
 
################################################################################################################
##app list widget
################################################################################################################
    
    def _init_app_list(self):
        self._refresh_app_list()
        pass
        
    
    def _uninit_app_list(self):
        self._widget_role_list.show()
        self._widget_role_list.move(self._widget_app_list.pos())
        self._widget_app_list.hide()
        pass    
       
    def _refresh_app_list(self):
        self.app_list= {}
        self.app_list= self.remocon_info._get_app_list()
        self._widget_app_list.app_list_widget.clear()
        
        index= 0
        
        for k in self.app_list.values():
            k['index']= index
            index= index+1
            
            self._widget_app_list.app_list_widget.insertItem(0, k['display_name'])
            #setting the list font
            font= self._widget_app_list.app_list_widget.item(0).font()        
            font.setPointSize(13)
            self._widget_app_list.app_list_widget.item(0).setFont(font)
            #setting the icon
            
            app_icon = k['icon']
            if len(app_icon) and app_icon == "Unknown.png":
                icon= QIcon(self.icon_path+app_icon)
                self._widget_app_list.app_list_widget.item(0).setIcon(icon)
            elif len(app_icon):
                icon= QIcon(self.temp_icon_path+app_icon)
                self._widget_app_list.app_list_widget.item(0).setIcon(icon)
            else:
                print concert_name+': No icon'
            pass
        
        pass
     
    def _select_app_list(self,Item):
        
        list_widget= Item.listWidget()
        cur_index= list_widget.count()-list_widget.currentRow()-1
        
        for k in self.app_list.values():
            if(k['index']== cur_index):
                self.cur_selected_app= k['name']
                break
    
        self._widget_app_list.app_info.clear()

        info_text= ""
        info_text= "<html>"
        info_text += "<p>-------------------------------------------</p>"
        info_text += "<p><b>name: </b>" + self.app_list[self.cur_selected_app]['name']+"</p>"

        info_text += "<p><b>  ---------------------</b>"+"</p>"
        info_text += "<p><b>  platform info </b>"+"</p>"
        
        
        info_text += "<p><b>  os: </b>"+self.app_list[self.cur_selected_app]['platform_info'].os+"</p>"
        info_text += "<p><b>  version: </b>"+self.app_list[self.cur_selected_app]['platform_info'].version+"</p>"
        info_text += "<p><b>  platform: </b>"+self.app_list[self.cur_selected_app]['platform_info'].platform+"</p>"
        info_text += "<p><b>  system: </b>"+self.app_list[self.cur_selected_app]['platform_info'].system+"</p>"
        info_text += "<p><b>  name: </b>"+self.app_list[self.cur_selected_app]['platform_info'].name+"</p>"
        info_text += "<p><b>  ---------------------</b>"+"</p>"
        
        info_text += "<p><b>display name: </b>"+self.app_list[self.cur_selected_app]['display_name']+"</p>"
        info_text += "<p><b>description: </b>"+self.app_list[self.cur_selected_app]['description']+"</p>"
        info_text += "<p><b>service name: </b>"+self.app_list[self.cur_selected_app]['service_name']+"</p>"
        info_text += "<p><b>max: </b>"+str(self.app_list[self.cur_selected_app]['max'])+"</p>"

        info_text += "<p><b>  ---------------------</b>"+"</p>"
        info_text += "<p><b>remappings: </b>" +str(self.app_list[self.cur_selected_app]['remappings'])+"</p>"
        info_text += "<p><b>parameters: </b>" +str(self.app_list[self.cur_selected_app]['parameters'])+"</p>"
        
        info_text +="</html>"
        
        self._widget_app_list.app_info.appendHtml(info_text)
        
    def _stop_app(self):
        print "Stop app: "+ str(self.cur_selected_app)
        self.remocon_info._stop_app(self.cur_selected_app)
        pass
          
    def _start_app(self):
        print "Start app: "+ str(self.cur_selected_app)
        self.remocon_info._start_app(self.cur_selected_role,self.cur_selected_app)
        pass
    
    def _write_cache(self):
        
        try:
            cache_concert_info_list= open(self.temp_cache_path,'w')
        except:
            print "No directory or file: %s"%(self.temp_cache_path)
            return

        for k in self.concert_list.values():
            concert_index= k['index']
            concert_name= k['name']
            concert_ip= k['ip']
            concert_icon= k['icon']
            concert_description= k['description']
            concert_flag= k['flag']

            concert_elem= '['
            concert_elem +='index='+str(concert_index)+','
            concert_elem +='name='+str(concert_name) + ','
            concert_elem +='ip='+str(concert_ip) + ','
            concert_elem +='description='+str(concert_description)+ ','
            concert_elem +='icon='+concert_icon+ ','
            concert_elem +='flag='+concert_flag
            concert_elem +=']\n'
            
            cache_concert_info_list.write(concert_elem)
            
        cache_concert_info_list.close()
        pass
        
    def _read_cache(self):
        #read cache and display the concert list
        try:
            cache_concert_info_list= open(self.temp_cache_path,'r')
        except:
            print "No directory or file: %s"%(self.temp_cache_path)
            return
        lines= cache_concert_info_list.readlines()
        
        for line in lines:
            concert_index= line[string.find(line, "[index=")+len("[index="):string.find(line, ",name=")]
            concert_name= line[string.find(line, "name=")+len("name="):string.find(line, ",ip=")]
            concert_ip= line[string.find(line, ",ip=")+len(",ip="):string.find(line, ",description=")]
            concert_description= line[string.find(line, ",description=")+len(",description="):string.find(line, ",icon=")]
            concert_icon= line[string.find(line, ",icon=")+len(",icon="):string.find(line, ",flag=")]
            concert_flag= line[string.find(line, ",flag=")+len(",flag="):string.find(line, "]")]

            self.concert_list[concert_index]= {}
            self.concert_list[concert_index]['index']= concert_index
            self.concert_list[concert_index]['name']= concert_name
            self.concert_list[concert_index]['ip']=  concert_ip
            self.concert_list[concert_index]['icon']=  concert_icon
            self.concert_list[concert_index]['description']=  concert_description
            self.concert_list[concert_index]['flag']=  concert_flag
       
        cache_concert_info_list.close()
        pass

#################################################################        
##Remocon Main
#################################################################
class RemoconMain(QMainWindow):
    def __init__(self, parent, title, application, host_name):
        self._context= parent
       
        super(RemoconMain, self).__init__()
        self.initialised= False
        self.setObjectName('Remocon')
        self.host_name= host_name
        self.application = application
        
        self._widget_main= QWidget()
      
        self.concert_list= {};
        self.cur_selected_concert= 0

        path= os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../ui/remocon.ui")
        uic.loadUi(path, self._widget_main)

        self.temp_cache_path="%s/.ros/rocon/remocon/cache/"%(os.getenv("HOME"))
        if not os.path.isdir(self.temp_cache_path):
            os.makedirs(self.temp_cache_psath)
        self.temp_cache_path+="concert_info_list.cache"
        
        self.icon_path= os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../resources/images/")
        self.temp_icon_path= "%s/.ros/rocon/remocon/image/"%(os.getenv("HOME"))
        self.scripts_path= os.path.join(os.path.dirname(os.path.abspath(__file__)),"../../scripts/")
        
        #main widget
        self._widget_main.list_widget.setIconSize(QSize(50,50))
        self._widget_main.list_widget.itemDoubleClicked.connect(self._connect_concert) #concert item double click event
        self._widget_main.list_widget.itemClicked.connect(self._select_concert) #concert item double click event

        self._widget_main.add_concert_btn.pressed.connect(self._set_add_concert) #add button event
        self._widget_main.delete_btn.pressed.connect(self._delete_concert) #delete button event
        self._widget_main.delete_all_btn.pressed.connect(self._delete_all_concert) #delete all button event
        self._widget_main.refresh_btn.pressed.connect(self._refresh_concert_list) #refresh all button event
             
        #init
          
        self._init()
        self._widget_main.show()     
    def __del__(self):
        print '[RemoconMain]: Destory'
########################################################################        
    def _init(self):
        
        self._connect_dlg_isValid= False
        self._current_selected_concert= ""
        self._refresh_concert_list()        
        pass
    
    def _check_up(self):
        host_name=self.host_name
        print host_name
        for k in self.concert_list.values():
            concert_ip= k['ip']
            output= subprocess.Popen([self.scripts_path+"rocon_remocon_check_up", concert_ip,host_name],stdout=subprocess.PIPE) 
            time_out_cnt= 0
            while True:
                print "checking: "+concert_ip
                result= output.poll() 
                if time_out_cnt > 10:
                    print "timeout: "+concert_ip
                    try:
                        output.terminate()
                    except:
                        print "Error: output.terminate()"
                    
                    k['name']= "Unknown"
                    k['description']="Unknown."
                    k['icon']= "Unknown.png"
                    break

                elif result== 0:
                    args=output.communicate()[0]
                    k['name']= args.split('\n')[0]
                    k['description']= args.split('\n')[1]
                    k['icon']= args.split('\n')[2]
                    
                    if k['name']=="Unknown":
                        k['flag']='0'
                    else:
                        k['flag']='1'    
                    break
                    
                time.sleep(0.1)        
                time_out_cnt+=1
            pass
                
    def _read_cache(self):
        #read cache and display the concert list
        try:
            cache_concert_info_list= open(self.temp_cache_path,'r')
        except:
            print "No directory or file: %s"%(self.temp_cache_path)
            return 
            
        lines= cache_concert_info_list.readlines()
        
        for line in lines:
            concert_index= line[string.find(line, "[index=")+len("[index="):string.find(line, ",name=")]
            concert_name= line[string.find(line, "name=")+len("name="):string.find(line, ",ip=")]
            concert_ip= line[string.find(line, ",ip=")+len(",ip="):string.find(line, ",description=")]
            concert_description= line[string.find(line, ",description=")+len(",description="):string.find(line, ",icon=")]
            concert_icon= line[string.find(line, ",icon=")+len(",icon="):string.find(line, ",flag=")]
            concert_flag= line[string.find(line, ",flag=")+len(",flag="):string.find(line, "]")]
          
            self.concert_list[concert_index]= {}
            self.concert_list[concert_index]['index']= concert_index
            self.concert_list[concert_index]['name']= concert_name
            self.concert_list[concert_index]['ip']=  concert_ip
            self.concert_list[concert_index]['icon']=  concert_icon
            self.concert_list[concert_index]['description']=  concert_description
            self.concert_list[concert_index]['flag']=  concert_flag
       
        cache_concert_info_list.close()
        pass
    def _delete_all_concert(self):
        for k in self.concert_list.values():
            del self.concert_list[k["index"]]
            
        self._update_concert_list()
        pass
    def _delete_concert(self):
        if self.concert_list.has_key(self.cur_selected_concert):      
            del self.concert_list[self.cur_selected_concert]
        
        self._update_concert_list()
        pass
    def _add_concert(self, params):
        concert_ip= str(params['param1'].toPlainText())
        
        concert_index= str(uuid.uuid4())
        print "IP: "+ str(concert_ip)
        self.concert_list[concert_index]= {}
        self.concert_list[concert_index]['index']= concert_index
        self.concert_list[concert_index]['name']= "Unknown"
        self.concert_list[concert_index]['ip']=  concert_ip
        self.concert_list[concert_index]['icon']= "Unknown.png" 
        self.concert_list[concert_index]['description']= "" 
        self.concert_list[concert_index]['flag']= "0"
     
        self._update_concert_list()

        pass
        
    def _set_add_concert(self):
        print '_add_concert'
        if self._connect_dlg_isValid:
            print "Dialog is live!!"
            self._connect_dlg.done(0)
                
        #dialog
        self._connect_dlg= QDialog(self._widget_main)             
        self._connect_dlg.setWindowTitle("Add Concert")
        self._connect_dlg.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        self._connect_dlg.setMinimumSize(500,0)
        dlg_rect= self._connect_dlg.geometry()

        #dialog layout
        ver_layout= QVBoxLayout(self._connect_dlg)
        ver_layout.setContentsMargins (9,9,9,9)
        
        #param layout
        text_grid_sub_widget= QWidget()
        text_grid_layout= QGridLayout(text_grid_sub_widget)            
        text_grid_layout.setColumnStretch (1, 0)
        text_grid_layout.setRowStretch (2, 0)

        #param 1
        name=u""
        title_widget1= QLabel("IP: ")
        context_widget1= QTextEdit()
        context_widget1.setSizePolicy(QSizePolicy.MinimumExpanding,QSizePolicy.Ignored)
        context_widget1.setMinimumSize(0,30)
        context_widget1.append("")
        
        #add param
        text_grid_layout.addWidget(title_widget1)
        text_grid_layout.addWidget(context_widget1)
     
        #add param layout
        ver_layout.addWidget(text_grid_sub_widget) 
        
        #button layout
        button_hor_sub_widget= QWidget()
        button_hor_layout= QHBoxLayout(button_hor_sub_widget)

        params= {}
        params['param1']= context_widget1
       
        #button
        btn_call= QPushButton("Add")
        btn_cancel= QPushButton("Cancel")
      
        btn_call.clicked.connect(lambda: self._connect_dlg.done(0))
        btn_call.clicked.connect(lambda: self._add_concert(params))

        btn_cancel.clicked.connect(lambda: self._connect_dlg.done(0))
        
        #add button
        button_hor_layout.addWidget(btn_call)            
        button_hor_layout.addWidget(btn_cancel)

        #add button layout            
        ver_layout.addWidget(button_hor_sub_widget)
        self._connect_dlg.setVisible(True)
        self._connect_dlg.finished.connect(self._destroy_connect_dlg)
        self._connect_dlg_isValid= True

        pass

    def _refresh_concert_list(self):
        print '_refresh_concert_list'
        self._widget_main.list_info_widget.clear()
        self._read_cache()
        self._check_up()
        self._update_concert_list()
        pass
        
    def _update_concert_list(self):        

        print '_update_concert_list'
        self._widget_main.list_widget.clear()
        try:
            cache_concert_info_list= open(self.temp_cache_path,'w')
        except:
            print "No directory or file: %s"%(self.temp_cache_path)
            return 
            
        for k in self.concert_list.values():
            self._add_concert_list_item(k)
            concert_index= k['index']
            concert_name= k['name']
            concert_ip= k['ip']
            concert_icon= k['icon']
            concert_description= k['description']
            concert_flag= k['flag']

            concert_elem= '['
            concert_elem +='index='+str(concert_index)+','
            concert_elem +='name='+str(concert_name) + ','
            concert_elem +='ip='+str(concert_ip) + ','
            concert_elem +='description='+str(concert_description)+ ','
            concert_elem +='icon='+concert_icon+ ','
            concert_elem +='flag='+concert_flag
            concert_elem +=']\n'
            
            cache_concert_info_list.write(concert_elem)
        cache_concert_info_list.close()

    def _add_concert_list_item(self,concert):
        print '_add_concert_list_item'
        concert_index= concert['index']
        concert_name= concert['name']
        concert_ip= concert['ip']
        concert_icon= concert['icon']
        concert_description= concert['description']
        concert['cur_row']= str(self._widget_main.list_widget.count())
        
        display_name=str(concert_name)+"["+str(concert_ip)+"]"   
        self._widget_main.list_widget.insertItem(self._widget_main.list_widget.count(),display_name )

        #setting the list font
        
        
        font= self._widget_main.list_widget.item(self._widget_main.list_widget.count()-1).font()        
        font.setPointSize(13)
        self._widget_main.list_widget.item(self._widget_main.list_widget.count()-1).setFont(font)

        #setToolTip
        concert_info=""
        concert_info +="concert_index: "+str(concert_index)+"\n"
        concert_info +="concert_name: "+str(concert_name)+"\n"
        concert_info +="ip:  "+str(concert_ip)+"\n"
        concert_info +="description:  "+str(concert_description)
        self._widget_main.list_widget.item(self._widget_main.list_widget.count()-1).setToolTip(concert_info)
        
        #set icon
        if len(concert_icon) and concert_icon == "Unknown.png":
            icon= QIcon(self.icon_path+concert_icon)
            self._widget_main.list_widget.item(self._widget_main.list_widget.count()-1).setIcon(icon)
        elif len(concert_icon):
            icon= QIcon(self.temp_icon_path+concert_icon)
            self._widget_main.list_widget.item(self._widget_main.list_widget.count()-1).setIcon(icon)
        else:
            print concert_name+': No icon'
        pass

    def _select_concert(self , Item):
        list_widget=  Item.listWidget()
        for k in self.concert_list.values():
            if k["cur_row"]== str(list_widget.currentRow()):
                #delete icon
                #os.remove(self.icon_path+self.concert_list[str(self.cur_selected_concert)]['icon'])           
                self.cur_selected_concert= k['index']    
                break
        pass
   
        self._widget_main.list_info_widget.clear()
        info_text= ""
        info_text= "<html>"
        info_text += "<p>-------------------------------------------</p>"
        info_text += "<p><b>name: </b>" +str(self.concert_list[self.cur_selected_concert]['name'])+"</p>"
        info_text += "<p><b>ip: </b>" +str(self.concert_list[self.cur_selected_concert]['ip'])+"</p>"
        info_text += "<p><b>description: </b>" +str(self.concert_list[self.cur_selected_concert]['description'])+"</p>"
        info_text += "<p>-------------------------------------------</p>"
        info_text +="</html>"
        self._widget_main.list_info_widget.appendHtml(info_text)

    def _destroy_connect_dlg(self):
        print "[Dialog] Distory!!!"
        self._connect_dlg_isValid= False
        pass
  
    def _connect_concert(self):

        concert_name= str(self.concert_list[self.cur_selected_concert]['name'])
        concert_ip= str(self.concert_list[self.cur_selected_concert]['ip'])
        
        concert_index= str(self.cur_selected_concert)
        concert_host_name= str(self.host_name)

        if self.concert_list[concert_index]['flag']== '0':    
            reply = QMessageBox.warning(self, 'ERROR',
            "YOU SELECT NO CONCERT", QMessageBox.Ok|QMessageBox.Ok)
            return

        execute_path= self.scripts_path+'rocon_remocon_sub' ##command
        execute_path += " "+"'"+concert_index+"'" ##arg1
        execute_path += " "+"'"+concert_name+"'" ##arg2
        execute_path += " "+"'"+concert_ip+"'" ##arg3
        execute_path += " "+"'"+concert_host_name+"'" ##arg4

        self._widget_main.hide()
        os.execv(self.scripts_path+'rocon_remocon_sub',["",concert_index,concert_name,concert_ip,concert_host_name])
        print "Spawning: %s"%(execute_path)

        pass

