cmake_minimum_required(VERSION 2.8.3)
project(line_intersection_test)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(Eigen3 REQUIRED)

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.54.0 COMPONENTS system filesystem iostreams) 

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
	include_directories(${EIGEN3_INCLUDE_DIR})
	add_executable(test main.cpp)
    target_link_libraries(test ${Boost_LIBRARIES} util)
endif()


