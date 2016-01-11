cmake_minimum_required(VERSION 2.8.3)
project(test)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(test main.cpp)

target_link_libraries(test
)

