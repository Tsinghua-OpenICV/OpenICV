cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(UartTranslater)
ADD_DEFINITIONS(-std=c++11)

include_directories (/usr/local/include)
include_directories (/usr/include/jsoncpp)
link_directories(/usr/local/lib)

find_package(Boost COMPONENTS thread system REQUIRED)
set( BOOST_THREAD_LIBRARY "-lboost_thread")
set( Boost_SYSTEM_LIBRARY "-lboost_system")
set( LPTHREAD "-lpthread")
set( ZMQ_LIBRARIES "-lzmq")
set( JSON_LIBRARIES "-ljsoncpp")

set(CMAKE_CXX_FLAGS "-std=c++11")
set(SRC_LIST uartTranslater.cpp main.cpp)
add_executable(uartTranslater ${SRC_LIST})
target_link_libraries (uartTranslater ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${ZMQ_LIBRARIES} ${JSON_LIBRARIES} ${LPTHREAD})


