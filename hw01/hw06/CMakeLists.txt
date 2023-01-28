CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

PROJECT(OpticalFlow)

#IF(NOT DEFINED ${CMAKE_BUILD_TYPE})
SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

MESSAGE(STATUS "CMAKE_BUILD_TYPE IS ${CMAKE_BUILD_TYPE}")

SET(CMAKE_CXX_STANDARD 17)
SET(Sophus_LIBRARIES libSophus.so)

include_directories("/usr/include/eigen3")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(optical_flow optical_flow.cpp)

target_link_libraries(optical_flow ${OpenCV_LIBS})

add_executable(direct_method direct_method.cpp)

target_link_libraries(direct_method ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES})