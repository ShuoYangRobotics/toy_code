cmake_minimum_required(VERSION 2.8.3)
project(test CXX)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
#SET(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11 -march=native -O3 -Wall")
#in order to use GDB to debug the program
SET(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
include_directories(
	include 
	${EIGEN3_INCLUDE_DIR}
)

add_executable(test
	src/main.cpp
)

target_link_libraries(test)