cmake_minimum_required(VERSION 2.8.3)
project(test CXX)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
SET(CMAKE_CXX_FLAGS "-DNDEBUG -std=c++11 -march=native -O3 -Wall")
#in order to use GDB to debug the program
SET(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)

set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.54.0 COMPONENTS system filesystem iostreams) 

include_directories(
	include 
	${EIGEN3_INCLUDE_DIR}
)

add_executable(test
	src/main.cpp
	src/Estimator.cpp
	src/CholeskyCovariance.cpp
	src/SO2.cpp
)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
	target_link_libraries(test ${Boost_LIBRARIES})
endif()