cmake_minimum_required(VERSION 2.8)

project(DrawTrajectory)

set(EXECUTABLE_NAME runhw03)
SET(Sophus_LIBRARIES libSophus.so)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(${EXECUTABLE_NAME} draw_trajectory.cpp)
target_link_libraries(${EXECUTABLE_NAME} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})

