cmake_minimum_required(VERSION 2.8.3)
project(read_web_cam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp std_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
message(STATUS "Found OpenCV: ${OpenCV_LIBS}")

#This command helped me to solve the problem:
#pkg-config libusb-1.0 --libs --cflags --variable=libdir --print-variables
#pkg-config opencv --cflags --libs

## code to find libusb-1.0 library
FIND_PATH(LIBUSB_1_INCLUDE_DIR libusb.h
   /usr/include/libusb-1.0
)
message(STATUS "Found libusb.h: ${LIBUSB_1_INCLUDE_DIR}")

FIND_LIBRARY(LIBUSB_1_LIBRARY NAMES usb-1.0 
   /usr/lib/arm-linux-gnueabihf
)
message(STATUS "Found libusb 1.0 library: ${LIBUSB_1_LIBRARY}")

if (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
   set(LIBUSB_1_FOUND TRUE)
   set(LIBUSB_1_LIBRARIES ${LIBUSB_1_LIBRARY})
else (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)
   set(LIBUSB_1_FOUND FALSE)
endif (LIBUSB_1_INCLUDE_DIR AND LIBUSB_1_LIBRARY)

if (LIBUSB_1_FOUND)
      message(STATUS "Found LibLIBUSB_1: ${LIBUSB_1_LIBRARIES}")
else (LIBUSB_1_FOUND)
	   message(STATUS "don't find LibLIBUSB_1")
endif (LIBUSB_1_FOUND)

MARK_AS_ADVANCED(LIBUSB_1_INCLUDE_DIR LIBUSB_1_LIBRARIES LIBUSB_1_LIBRARY)

## code to find ASI library
FIND_PATH(ASI_INCLUDE_DIR ASICamera.h
   /usr/include
   /usr/local/include
)
message(STATUS "Found ASICamera: ${ASI_INCLUDE_DIR}")

FIND_LIBRARY(ASI_LIBRARY NAMES ASICamera libASICamera
   /usr/lib
)
message(STATUS "Found ASICamera library: ${ASI_LIBRARY}")

if (ASI_INCLUDE_DIR AND ASI_LIBRARY)
   set(ASI_FOUND TRUE)
   set(ASI_LIBRARIES ${ASI_LIBRARY})
else (ASI_INCLUDE_DIR AND ASI_LIBRARY)
   set(ASI_FOUND FALSE)
endif (ASI_INCLUDE_DIR AND ASI_LIBRARY)

if (ASI_FOUND)
      message(STATUS "Found LibASI: ${ASI_LIBRARIES}")
else (ASI_FOUND)
	   message(STATUS "don't find LibASI")
endif (ASI_FOUND)

MARK_AS_ADVANCED(ASI_INCLUDE_DIR ASI_LIBRARIES ASI_LIBRARY)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

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
#  LIBRARIES read_web_cam
#  CATKIN_DEPENDS cv_bridge image_transport opencv2 roscpp std_msgs
#  DEPENDS system_lib
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
# add_library(read_web_cam
#   src/${PROJECT_NAME}/read_web_cam.cpp
# )

## Declare a cpp executable
add_executable(read_web_cam src/publish.cpp)
add_executable(ASIRead src/ASIRead.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(read_web_cam_node read_web_cam_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(read_web_cam
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

target_link_libraries( ASIRead
    ${catkin_LIBRARIES}
    ${ASI_LIBRARIES}
    ${LIBUSB_1_LIBRARIES}
    ${OpenCV_LIBS}
    rt
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
# install(TARGETS read_web_cam read_web_cam_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_read_web_cam.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
