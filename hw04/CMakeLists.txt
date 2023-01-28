cmake_minimum_required( VERSION 2.8 )
project( hw04 )

# 添加c++ 11标准支持
set( CMAKE_CXX_FLAGS "-std=c++11" )

# 寻找OpenCV库
find_package( OpenCV 3 REQUIRED )
find_package(Pangolin REQUIRED)
# 添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
include_directories( "/usr/include/eigen3" )

add_executable( undistort undistort_image.cpp )
# 链接OpenCV库
target_link_libraries( undistort ${OpenCV_LIBS} )

add_executable( fisheye fisheye.cpp )
# 链接OpenCV库
target_link_libraries( fisheye ${OpenCV_LIBS} )

add_executable( disparity disparity.cpp )
# 链接OpenCV库
target_link_libraries( disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})

add_executable( gaussnewton gaussnewton.cpp )
# 链接OpenCV库
target_link_libraries( gaussnewton ${OpenCV_LIBS})
