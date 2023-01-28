cmake_minimum_required(VERSION 3.10)

project(hw01)

set(LIBRARY_NAME hw01)

add_library(${LIBRARY_NAME} hello.cpp)

find_package (glog 0.6.0 REQUIRED)

target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

set(EXECUTABLE_NAME runhw01)
add_executable(${EXECUTABLE_NAME} useHello.cpp)

target_link_libraries(${EXECUTABLE_NAME} ${LIBRARY_NAME} glog::glog)
