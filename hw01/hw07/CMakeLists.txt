CMAKE_MINIMUM_REQUIRED(VERSION 3.0)
PROJECT(DirectBA)
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_STANDARD 14)

SET(Sophus_LIBRARIES libSophus.so)
include_directories("/usr/include/eigen3")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

Find_Package(G2O REQUIRED)
Find_Package(Eigen3 REQUIRED)
Find_Package(Ceres REQUIRED)
Find_Package(Sophus REQUIRED)
Find_Package(CSparse REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

#SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

include_directories(${PROJECT_SOURCE_DIR} ${EIGEN3_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
include_directories( ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} )
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(directBA directBA.cpp)

target_link_libraries(directBA ${G2O_LIBS}  ${Sophus_LIBRARIES} ${OpenCV_LIBS}  ${Pangolin_LIBRARIES} )