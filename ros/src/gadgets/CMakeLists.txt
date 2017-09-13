cmake_minimum_required(VERSION 2.8.3)
project(gadgets)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    rospy
    styx_msgs
)

## Install Python scripts / modules
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

## The catkin_package macro generates cmake config files for your package
catkin_package(
    CATKIN_DEPENDS rospy styx_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${catkin_INCLUDE_DIRS}
)

## Mark script files for installation
catkin_install_python(
    PROGRAMS bin/plotter
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
