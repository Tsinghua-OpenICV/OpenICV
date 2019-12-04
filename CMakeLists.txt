cmake_minimum_required(VERSION 3.4)
project(OpenICV)

# Set default compiling options
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O2 -fpic")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -fpic")
# Set binary output folder to make binary easily discovered
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
message("binary==${CMAKE_BINARY_DIR}")
set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/Documents/openicv/ICVOS")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/bin")
message("installd==${CMAKE_INSTALL_PREFIX}")
set(OpenCV_DIR "/usr/local/share/OpenCV")
#set(BOOST_ROOT "/home/skou/install/boost_1_68_0")
#set(BOOST_NO_SYSTEM_PATH true)
# Options
option(OPENICV_ENABLE_REMOTE "Enable multi process and remote processing support" ON) # if disabled, serialization and net module are not needed

option(OPENICV_PREFER_BOOST "Use boost when possible" ON)

option(OPENICV_BUILD_EXAMPLES "Build examples" OFF)
option(OPENICV_BUILD_TESTS "Build test cases" OFF)
option(OPENICV_BUILD_TOOLS "Build built-in tools" ON)
option(OPENICV_WITH_EIGEN "Build with Eigen extensions" OFF)
option(OPENICV_WITH_OPENCV "Build with OpenCV extensions" OFF)
option(OPENICV_WITH_PCL "Build with PCL extensions" OFF)

if(OPENICV_BUILD_TESTS)
    enable_testing()
endif()
if(OPENICV_PREFER_BOOST)
    add_definitions(-DOPENICV_PREFER_BOOST)
endif()

# Add cmake macros and functions
set(CMAKE_MODULE_PATH "${OpenICV_SOURCE_DIR}/CMake/Modules/" ${CMAKE_MODULE_PATH})
include(${OpenICV_SOURCE_DIR}/CMake/Common.cmake)

include_directories(${OpenICV_SOURCE_DIR}/Include)
#link_directories(${CMAKE_INSTALL_PREFIX}/lib)

# Process dependencies
add_subdirectory(ThirdParty)

# Modules
add_subdirectory(Examples)
add_subdirectory(Source)
add_subdirectory(Tools)
add_subdirectory(Tests)
add_subdirectory(Wrap)
add_subdirectory(Sensors)
# install export target
install(EXPORT icvExport
    FILE OpenICVConfig.cmake
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/OpenICV
)
