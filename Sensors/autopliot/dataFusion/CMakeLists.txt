cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(dataFusion)
ADD_DEFINITIONS(-std=c++11)

include_directories (/usr/local/include)
include_directories (/usr/include/jsoncpp)
link_directories (/usr/local/lib)

find_package(Boost COMPONENTS thread system REQUIRED)
set( BOOST_THREAD_LIBRARY "-lboost_thread")
set( Boost_SYSTEM_LIBRARY "-lboost_system")
set( LPTHREAD "-lpthread")
set( ZMQ_LIBRARIES "-lzmq" )
set( JSON_LIBRARIES "-ljsoncpp")

add_executable(dataFusion dataFusion.cpp)
target_link_libraries(dataFusion ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${ZMQ_LIBRARIES} ${JSON_LIBRARIES} ${LPTHREAD}) 
