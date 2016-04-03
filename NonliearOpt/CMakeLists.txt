cmake_minimum_required(VERSION 2.8.3)
project(test CXX)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(Eigen3 REQUIRED)
include_directories(
	include 
	${EIGEN3_INCLUDE_DIR}
)
add_library(types_lib
	include/types/Vector.h
	src/Vector.cpp

	include/types/Manifold.h
	src/Manifold.cpp
)


add_executable(test
	src/main.cpp
)

target_link_libraries(test types_lib)