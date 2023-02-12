cmake_minimum_required(VERSION 3.0.2)
project(jackal_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs nav_msgs tf2 tf2_ros)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES turtlesim_py
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
catkin_install_python(PROGRAMS goal.py waypoint.py waypoint_test.py reset_bot.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
