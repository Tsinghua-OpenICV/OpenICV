cmake_minimum_required(VERSION 2.8.3)
project(oxford_gps_eth)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  gps_common
  tf
)

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
  message(STATUS "Enabling coverage testing")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
endif()

if (NOT "${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif ("${CMAKE_CXX_STANDARD_COMPUTED_DEFAULT}" STREQUAL "98")
  message(STATUS "Changing CXX_STANDARD from C++98 to C++11")
  set(CMAKE_CXX_STANDARD 11)
endif()

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_gps_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_gps_node ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_gps_node PROPERTIES OUTPUT_NAME gps_node PREFIX "")

install(TARGETS ${PROJECT_NAME}_gps_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  add_subdirectory(tests)
endif()
