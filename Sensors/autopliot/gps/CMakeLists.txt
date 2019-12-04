cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(GpsDriver)
ADD_DEFINITIONS(-std=c++11)

include_directories (/usr/local/include)
include_directories (/usr/include/jsoncpp)
link_directories(/usr/local/lib)
link_libraries("/usr/local/lib/librtcm.so")

find_package(Boost COMPONENTS thread system REQUIRED)
set( BOOST_THREAD_LIBRARY "-lboost_thread")
set( Boost_SYSTEM_LIBRARY "-lboost_system")
set( LPTHREAD "-lpthread")
set( ZMQ_LIBRARIES "-lzmq")
set( JSON_LIBRARIES "-ljsoncpp")

set(CMAKE_CXX_FLAGS "-std=c++11")
set(SRC_LIST gpsDriver.cpp) 
add_executable(gpsDriver ${SRC_LIST})
target_link_libraries (gpsDriver ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${ZMQ_LIBRARIES} ${JSON_LIBRARIES} ${LPTHREAD})


