cmake_minimum_required(VERSION 2.8.3)
project(test)
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

find_package(Eigen3 REQUIRED)
find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)

INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR})

include_directories(
		${PROJECT_SOURCE_DIR}
		${EIGEN3_INCLUDE_DIR}
		)
add_executable(test ba_demo.cpp)

target_link_libraries(test
${PROJECT_SOURCE_DIR}/lib/libg2o_core.so
${PROJECT_SOURCE_DIR}/lib/libg2o_stuff.so
${PROJECT_SOURCE_DIR}/lib/libg2o_stuff.so
${PROJECT_SOURCE_DIR}/lib/libg2o_cli.so                   
${PROJECT_SOURCE_DIR}/lib/libg2o_interactive.so     
${PROJECT_SOURCE_DIR}/lib/libg2o_solver_csparse.so         
${PROJECT_SOURCE_DIR}/lib/libg2o_stuff.so          
${PROJECT_SOURCE_DIR}/lib/libg2o_types_slam2d.so
${PROJECT_SOURCE_DIR}/lib/libg2o_core.so                  
${PROJECT_SOURCE_DIR}/lib/libg2o_interface.so       
${PROJECT_SOURCE_DIR}/lib/libg2o_solver_dense.so           
${PROJECT_SOURCE_DIR}/lib/libg2o_types_data.so     
${PROJECT_SOURCE_DIR}/lib/libg2o_types_slam3d.so
${PROJECT_SOURCE_DIR}/lib/libg2o_csparse_extension.so     
${PROJECT_SOURCE_DIR}/lib/libg2o_opengl_helper.so   
${PROJECT_SOURCE_DIR}/lib/libg2o_solver_eigen.so           
${PROJECT_SOURCE_DIR}/lib/libg2o_types_icp.so
${PROJECT_SOURCE_DIR}/lib/libg2o_ext_csparse.so           
${PROJECT_SOURCE_DIR}/lib/libg2o_parser.so          
${PROJECT_SOURCE_DIR}/lib/libg2o_solver_pcg.so             
${PROJECT_SOURCE_DIR}/lib/libg2o_types_sba.so
${PROJECT_SOURCE_DIR}/lib/libg2o_ext_freeglut_minimal.so  
${PROJECT_SOURCE_DIR}/lib/libg2o_simulator.so       
${PROJECT_SOURCE_DIR}/lib/libg2o_solver_slam2d_linear.so   
${PROJECT_SOURCE_DIR}/lib/libg2o_types_sclam2d.so
${PROJECT_SOURCE_DIR}/lib/libg2o_incremental.so           
${PROJECT_SOURCE_DIR}/lib/libg2o_solver_cholmod.so  
${PROJECT_SOURCE_DIR}/lib/libg2o_solver_structure_only.so  
${PROJECT_SOURCE_DIR}/lib/libg2o_types_sim3.so

)
