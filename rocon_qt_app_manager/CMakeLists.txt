cmake_minimum_required(VERSION 2.8.3)
project(rocon_qt_app_manager)

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

install( 
    FILES qt_app_manager_plugin.xml 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

