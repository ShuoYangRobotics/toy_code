cmake_minimum_required(VERSION 2.8.3)
project(serial_to_uav)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS nav_msgs tf roscpp rospy std_msgs geometry_msgs message_generation)

## code to find serial communication library
FIND_PATH(SERIAL_INCLUDE_DIR SerialStream.h
   /usr/include
   /usr/local/include
)

FIND_LIBRARY(SERIAL_LIBRARY NAMES  serial libserial
   PATHS
   /usr/lib
   /usr/local/lib
)

if (SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)
   set(SERIAL_FOUND TRUE)
   set(SERIAL_LIBRARIES ${SERIAL_LIBRARY})
else (SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)
   set(SERIAL_FOUND FALSE)
endif (SERIAL_INCLUDE_DIR AND SERIAL_LIBRARY)

if (SERIAL_FOUND)
      message(STATUS "Found LibSerial: ${SERIAL_LIBRARIES}")
else (SERIAL_FOUND)
	   message(STATUS "don't find LibSerial")
endif (SERIAL_FOUND)

MARK_AS_ADVANCED(SERIAL_INCLUDE_DIR SERIAL_LIBRARIES SERIAL_LIBRARY)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
#   Message1.msg
  UAV.msg
  uav_ctrl.msg
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
  #std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_to_uav
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
#add_library(serial_to_uav
#  src/serial_talker.cpp
#)

## Declare a cpp executable
## find Yu Yun math library
file(GLOB UAV_MATH_LIB_SOURCES /mnt/iarc/UAV_ROS_pkgs/src/irobot_tracker/src/math_lib/*.c)
include_directories("/mnt/iarc/UAV_ROS_pkgs/src/irobot_tracker/src/math_lib")
SET_SOURCE_FILES_PROPERTIES(${UAV_MATH_LIB_SOURCES} PROPERTIES LANGUAGE CXX)

#add_executable(serial_to_uav_node src/serial_talker.cpp ${UAV_MATH_LIB_SOURCES})
add_executable(serial_to_uav_node src/serial_FSM.cpp ${UAV_MATH_LIB_SOURCES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(serial_to_uav_node serial_to_uav_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(serial_to_uav_node
  ${catkin_LIBRARIES}
  ${SERIAL_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS serial_to_uav serial_to_uav_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serial_to_uav.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
