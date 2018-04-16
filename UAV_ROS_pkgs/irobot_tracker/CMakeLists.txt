cmake_minimum_required(VERSION 2.8.3)
project(irobot_tracker)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/lib/cmake/")
find_package(aruco REQUIRED )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS message_filters geometry_msgs roscpp std_msgs cv_bridge image_transport tf message_generation)

## System dependencies are found with CMake's conventions
set(Boost_USE_STATIC_LIBS ON)
find_package( Boost COMPONENTS iostreams system filesystem REQUIRED )
include_directories(${Boost_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )

set(EIGEN_NO_DEBUG OFF)
find_package( Eigen3 REQUIRED )
include_directories(${EIGEN3_INCLUDE_DIR})

## find Yu Yun math library
file(GLOB UAV_MATH_LIB_SOURCES src/math_lib/*.c)
include_directories("${PROJECT_SOURCE_DIR}/src/math_lib")
SET_SOURCE_FILES_PROPERTIES(${UAV_MATH_LIB_SOURCES} PROPERTIES LANGUAGE CXX)


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
  trackInfo.msg
  trackerDebug.msg
#   Message2.msg
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
  geometry_msgs
    #std_msgs
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
  INCLUDE_DIRS include
  LIBRARIES pid SRUKF
#  CATKIN_DEPENDS geometry_msgs roscpp std_msgs tf
#  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})


## Declare a cpp library
add_library(pid
   src/pid.cpp
)

add_library(SRUKF
   src/SRUKF.cpp
   ${UAV_MATH_LIB_SOURCES}
)

## Declare a cpp executable
add_executable(SRUKF_controller src/SRUKF_controller.cpp ${UAV_MATH_LIB_SOURCES})
add_executable(aruco_tracker src/aruco_tracker.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(irobot_tracker_node irobot_tracker_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(irobot_tracker_node
#   ${catkin_LIBRARIES}
# )
target_link_libraries( SRUKF_controller pid SRUKF ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries( aruco_tracker uav_math ${OpenCV_LIBS} ${aruco_LIBS} ${catkin_LIBRARIES})

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
# install(TARGETS irobot_tracker irobot_tracker_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_irobot_tracker.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
