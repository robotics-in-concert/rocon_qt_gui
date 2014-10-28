##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rocon_qt_teleop)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()
catkin_python_setup()

##############################################################################
# Installs
##############################################################################

install( 
    FILES rocon_teleop_plugin.xml 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
