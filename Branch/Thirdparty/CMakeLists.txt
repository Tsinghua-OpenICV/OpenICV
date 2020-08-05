cmake_minimum_required(VERSION 3.4)
set(THIRDPARTY_PATH ${CMAKE_CURRENT_SOURCE_DIR})
#message("list =${THIRDPARTY_PATH}")
if(OPENICV_R_BOOST)
add_subdirectory(src/boost)
endif()

if(OPENICV_R_OPENCV)
add_subdirectory(src/opencv)
endif()

if(OPENICV_R_EIGEN)
add_subdirectory(src/eigen)
endif()

if(OPENICV_R_ZMQ)
add_subdirectory(src/zmq)
endif()

if(OPENICV_R_PCL)
add_subdirectory(src/pcl)
endif()

if(OPENICV_R_MSGPACK)
add_subdirectory(src/msgpack)
endif()
