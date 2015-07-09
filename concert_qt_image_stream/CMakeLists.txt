##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(concert_qt_image_stream)

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
    FILES concert_image_stream_plugin.xml 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


