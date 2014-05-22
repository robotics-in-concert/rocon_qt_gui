cmake_minimum_required(VERSION 2.8.3)
project(concert_conductor_graph)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)
catkin_package()

catkin_python_setup()

install(
    FILES conductor_plugin.xml 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

install(DIRECTORY resources
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY ui
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
