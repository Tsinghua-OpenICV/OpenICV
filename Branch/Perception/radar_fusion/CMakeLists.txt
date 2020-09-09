cmake_minimum_required(VERSION 2.8.3)
project(ros_fusion)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  #eigen_conversions
)


find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
message(STATUS "OpenCV Include: " ${OpenCV_INCLUDE_DIRS})
#link_directories(${OpenCV_LIBRARY_DIRS})
message(STATUS "OpenCV Library: " ${OpenCV_LIBS})	#${OpenCV_LIBS}	${OpenCV_LIBRARY_DIRS}
add_definitions(${OpenCV_DEFINITIONS})


## System dependencies are found with CMake's conventions

find_package(Boost REQUIRED)

FIND_PACKAGE( REQUIRED COMPONENTS common io visualization filters)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2 -DNDEBUG -fopenmp")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    DriveObs.msg
    DriveObsArray.msg
    DriveRadarObs.msg
    DriveRadarObsArray.msg

    ivsensorlrrobj.msg
    radarlrrobject.msg

    ivsensorsrrobj.msg
    radarsrrobject.msg

    ObjectArray.msg
    Object.msg
    
    track.msg
    TrackArray.msg

    BoundingBox.msg
    BoundingBoxes.msg

    ld_LaneParam.msg
    ld_Frame.msg
    ld_Point.msg
    ld_Coeff.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES idriverSensorGps
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${EIGEN_INCLUDE_DIR})

## Declare a C++ library
# add_library(idriverSensorGps
#   src/${PROJECT_NAME}/idriverSensorGps.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(idriverSensorGps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable -- ros_fusion for fusing all sensor data
 add_executable(ros_fusion 
    src/main_fusion.cpp
    src/fusion.h
    src/objects/obsf_define.h

    src/objects/obsf_header.h
    
    src/objects/obsf_obs_vector.h
    src/objects/obsf_obs2d.h
    src/objects/obsf_obs3d.h
    src/objects/obsf_obs_frame.h
    src/objects/obsf_fusion_track.h
    src/objects/obsf_fusion_track.cpp
    src/objects/obsf_track.h
    src/objects/obsf_track.cpp

    src/filters/base_filter_wrap.h
    #src/filters/cov_table.h
    #src/filters/kalman_filter_wrap.h
    #src/filters/kalman_filter_wrap.cpp

    src/filters/fusion_kalman_filter.h
    src/filters/fusion_kalman_filter.cpp

    src/filters/obsf_uncertainty_fusion_engine.h
    src/filters/obsf_uncertainty_fusion_engine.cpp 


   ## src/visualization/obsf_pcl_visual.cpp
  ##  src/visualization/obsf_pcl_visual.h
  ##  src/visualization/pcl_visual_util.cpp
  ##  src/visualization/pcl_visual_util.h
  ##  src/visualization/ros_util.h
    )

## Declare a C++ executable -- ros_fusion for fusing all sensor data
 ##add_executable(ros_fusion_camera_radar 
   ## src/main_fusion_camera_radar.cpp
   ## src/fusion_camera_radar.h


##    src/objects/obsf_define.h
 ##   src/objects/obsf_header.h
    
 ##   src/objects/obsf_obs_vector.h
  ##  src/objects/obsf_obs3d.h
   ## src/objects/obsf_obs_frame.h
  ##  src/objects/obsf_fusion_track.h
   ## src/objects/obsf_fusion_track.cpp
 ##   src/objects/obsf_track.h
 ##   src/objects/obsf_track.cpp

  ##  src/filters/base_filter_wrap.h
    #src/filters/cov_table.h
    #src/filters/kalman_filter_wrap.h
    #src/filters/kalman_filter_wrap.cpp

  ##  src/filters/fusion_kalman_filter.h
  ##  src/filters/fusion_kalman_filter.cpp

  ##  src/filters/obsf_uncertainty_fusion_engine.h
  ##  src/filters/obsf_uncertainty_fusion_engine.cpp 


  ##  src/visualization/obsf_pcl_visual.cpp
  ##  src/visualization/obsf_pcl_visual.h
   ## src/visualization/pcl_visual_util.cpp
  ##  src/visualization/pcl_visual_util.h
  ##  src/visualization/ros_util.h
  ##  )

## Add cmake target dependencies of the executable          ros_fusion
## same as for the library above
 add_dependencies(ros_fusion ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
 target_link_libraries(ros_fusion
   ${catkin_LIBRARIES}
   ${Eigen_LIBRARIES}
   
 )

## Add cmake target dependencies of the executable          ros_fusion_camera_radar
## same as for the library above
## add_dependencies(ros_fusion_camera_radar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against
 ##target_link_libraries(ros_fusion_camera_radar
 ##  ${catkin_LIBRARIES}
##   ${Eigen_LIBRARIES}
##   ${PCL_LIBRARIES}
## )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS idriverSensorGps idriverSensorGps_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_idriverSensorGps.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
