##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_qt_service_info)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

##############################################################################
# Installs
##############################################################################

#install(
#    FILES gateway_plugin.xml 
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )
#
#install(DIRECTORY resources
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
#
#install(DIRECTORY ui
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#)
install(PROGRAMS scripts/concert_qt_service_info
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
