cmake_minimum_required(VERSION 3.0.2)
project(pcap2ros)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++17 )

set( CMAKE_CXX_FLAGS "-std=c++17 -O3" )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rosbag
  sensor_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(TINS REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcap2ros
 CATKIN_DEPENDS roscpp std_msgs rosbag sensor_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcap2ros src/pcap2ros.cpp)

target_link_libraries(pcap2ros 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES} 
  tins
)

