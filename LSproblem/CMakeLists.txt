cmake_minimum_required(VERSION 2.8.3)
project(LSProblem)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(testLM main.cpp)

target_link_libraries(testLM
)

