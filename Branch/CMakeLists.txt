cmake_minimum_required(VERSION 3.4)
project(OpenICV)


set(OpenICV_path ${CMAKE_CURRENT_SOURCE_DIR})
string(REGEX REPLACE "(.+)\\/Branch.*" "\\1" OpenICV_ini_path ${OpenICV_path})
set(CMAKE_INSTALL_PREFIX ${OpenICV_ini_path}/ICVOS CACHE PATH ${OpenICV_ini_path}/ICVOS FORCE)

# Set default compiling options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

# Set binary output folder to make binary easily discovered
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
message("binary==${OpenICV_ini_path}")


set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/bin")
message("installd==${CMAKE_INSTALL_RPATH}")

set(BOOST_ROOT "/usr/local/")
set(BOOST_NO_SYSTEM_PATH true)
#set(OpenCV_DIR "/usr/local/share/OpenCV")
# Options
add_definitions(-DOPENICV_PREFER_BOOST)
option(OPENICV_BUILD_EXAMPLES "Build examples" OFF)

option(OPENICV_BUILD_TOOLS "Build built-in tools" ON)
option(OPENICV_WITH_EIGEN "Build with Eigen extensions" ON)
option(OPENICV_WITH_OPENCV "Build with OpenCV extensions" ON)
option(OPENICV_WITH_PCL "Build with PCL extensions" ON)


option(OPENICV_R_BOOST "rebuild boost" ON)
option(OPENICV_R_PCL "rebuild PCL" OFF)
option(OPENICV_R_OPENCV "rebuild OPENCV" OFF)
option(OPENICV_R_EIGEN "rebuild EIGEN" OFF)
option(OPENICV_R_ZMQ "rebuild ZMQ" OFF)
option(OPENICV_R_MSGPACK "rebuild MSGPACK" OFF)
if(OPENICV_ENABLE_REMOTE)
add_definitions(-DOPENICV_ENABLE_REMOTE)
endif()

set( BOOST_VERSION_TB 66)

# Add cmake macros and functions
set(CMAKE_MODULE_PATH "${OpenICV_SOURCE_DIR}/CMake/Modules/" ${CMAKE_MODULE_PATH})
include(${OpenICV_SOURCE_DIR}/CMake/Common.cmake)

include_directories(${OpenICV_SOURCE_DIR}/Include
)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

# Process dependencies
add_subdirectory(Thirdparty)

# Modules

add_subdirectory(Source)
add_subdirectory(Tools)
add_subdirectory(Sensors)
add_subdirectory(Perception)
add_subdirectory(Planning)
# install export target
install(EXPORT icvExport
    FILE OpenICVConfig.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/OpenICV
)
