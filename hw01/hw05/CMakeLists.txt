cmake_minimum_required(VERSION 3.0)
project(ComputeORB)

#IF(NOT DEFINED ${CMAKE_BUILD_TYPE})
    SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

MESSAGE(STATUS "CMAKE_BUILD_TYPE IS ${CMAKE_BUILD_TYPE}")

SET(CMAKE_CXX_STANDARD 11)
SET(Sophus_LIBRARIES libSophus.so)
#寻找opencv库
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)

#添加头文件
include_directories(${OpenCV_INCLUDE_DIRS} "/usr/include/eigen3" "/usr/local/include/sophus")
include_directories(${Sophus_INCLUDE_DIRS})

# add_executable(computeORB computeORB.cpp)

#链接OpenCV库 和tbb库
# target_link_libraries(computeORB ${OpenCV_LIBS} tbb)

add_executable(E2Rt E2Rt.cpp)

target_link_libraries(E2Rt ${Sophus_LIBRARIES})


add_executable(gn_ba GN-BA.cpp)

target_link_libraries(gn_ba ${Sophus_LIBRARIES})