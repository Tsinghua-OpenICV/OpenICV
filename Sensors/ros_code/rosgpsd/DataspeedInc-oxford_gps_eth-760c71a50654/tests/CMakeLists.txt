### Unit tests
#
#   Only configured when CATKIN_ENABLE_TESTING is true.

# These dependencies are only needed for unit testing
find_package(roslaunch REQUIRED)
find_package(rostest REQUIRED)

# Check all the launch/*.launch files
roslaunch_add_file_check(../launch)

# C++ gtests
catkin_add_gtest(test_packet test_packet.cpp)
target_link_libraries(test_packet ${catkin_LIBRARIES})

# ROS rostests
add_rostest_gtest(test_system system.test system.cpp)
add_dependencies(test_system ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_system ${catkin_LIBRARIES})

