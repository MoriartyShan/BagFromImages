cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(BagFromImages)

rosbuild_init()

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(glog REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rosbuild_add_executable(${PROJECT_NAME}
main.cc
Thirdparty/DLib/FileFunctions.cpp)
rosbuild_add_boost_directories()
rosbuild_link_boost(${PROJECT_NAME} thread system)
target_link_libraries (${PROJECT_NAME}
glog::glog
/usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
${OpenCV_LIBS})
