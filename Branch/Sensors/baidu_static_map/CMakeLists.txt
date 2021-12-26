icv_set_module_vars(baidu_static_map)

include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_CXX_STANDARD 17 )
icv_set_module_vars(baidu_static_map)

add_subdirectory(apollo_hdmap)

find_package(Protobuf REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)



set(HDMAP_LIABRARY apollo_hdmap/build/libhdmap.so)

include_directories(
  include
  apollo_hdmap
  ${absl_INCLUDE_DIRS}
  # ${catkin_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${hdmap_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenICV_ini_path}/Branch/Kernel/Include
)

add_library(${MODULE_TARGET_NAME}  SHARED
  src/baidu_static_map.cc
)
target_link_libraries(
  ${MODULE_TARGET_NAME} PUBLIC icvCore  ${Boost_LIBRARIES} ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
  hdmap
  proj
  tinyxml2
  glog
  gflags
  yaml-cpp
)


icv_set_solution_folder(icvbaidu_static_map Sensors)

icv_install_module(baidu_static_map)

