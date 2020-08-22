cmake_minimum_required(VERSION 2.8.3)
project(aruco_analyzer)

# Find catkin macros and libraries
# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    rospy 
    std_msgs 
    message_generation 
    geometry_msgs 
    tf2_ros
    cv_bridge
)

# Uncomment this if the package has a setup.py. This macro ensures
# modules and scripts declared therein get installed
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

# Generate messages in the 'msg' folder
add_message_files(
   FILES
   sys_resources.msg
 )

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

###################################################
## Declare things to be passed to other projects ##
###################################################

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES rover_behaviour_manager
   CATKIN_DEPENDS rospy std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

#############
## Install ##
#############

# Mark executable scripts (Python etc.) for installation
# in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  nodes/aruco_detection_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(DIRECTORY
  config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
