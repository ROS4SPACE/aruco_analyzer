cmake_minimum_required(VERSION 2.8.3)
project(aruco_analyzer)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    rospy 
    std_msgs 
    message_generation 
    geometry_msgs 
    tf2_ros
    cv_bridge
    tf_conversions
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and scripts declared therein get installed
catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   sys_resources.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
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

###########
## Build ##
###########

## Specify additional locations of header files
# add_definitions(-std=c++11)

# include_directories(${catkin_INCLUDE_DIRS})

# link_directories(lib/i386 lib/x86_64)

## Declare a cpp library
#add_library(rover_behaviour_manager src/${PROJECT_NAME}/rover_behaviour_manager.cpp)

## Declare a cpp executable
#add_executable(aruco_analyzer src/main.cpp)

## Add dependencies to the executable
#add_dependencies(aruco_analyzer ${${PROJECT_NAME}_EXPORTED_TARGETS}
#${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

#target_link_libraries(aruco_analyzer ${catkin_LIBRARIES} ${OpenCV_LIBS})

#############
## Install ##
#############

# catkin_install_python(PROGRAMS 
#     src/aruco_analyzer/aruco_detection
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# install(PROGRAMS 
#     scripts/aruco_detection_ros_wrapper
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS rover_behaviour_manager rover_behaviour_manager_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rover_behaviour_manager.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
