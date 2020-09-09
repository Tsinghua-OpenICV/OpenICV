cmake_minimum_required(VERSION 2.8.3)
project(cluster)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)

find_package(PCL 1.8 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

add_executable(plane_ground_filter_node src/plane_ground_filter_node.cpp src/plane_ground_filter_core.cpp)


target_link_libraries(plane_ground_filter_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
