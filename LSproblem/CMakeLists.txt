cmake_minimum_required(VERSION 2.8.3)
project(LSProblem)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
include_directories(${EIGEN3_INCLUDE_DIR})
add_executable(testLM main.cpp)
add_executable(testDiff testNumericalDiff.cpp)
add_executable(testSparse testSparse.cpp)

target_link_libraries(testLM
)


target_link_libraries(testSparse
	${QT_LIBRARIES}
)
