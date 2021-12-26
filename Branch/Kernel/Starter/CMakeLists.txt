find_package(Boost COMPONENTS  program_options  system filesystem log REQUIRED)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_LOG_DYN_LINK)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

find_package(PkgConfig)
pkg_check_modules(PC_ZeroMQ QUIET zmq)
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )


add_executable(icvStarter Entry.cxx)
target_link_libraries(icvStarter PUBLIC icvCore icvEngine icvBasis icvZmqNet pthread ${Boost_LIBRARIES} msgpackc ${ZeroMQ_LIBRARY})
#include_directories(${OpenICV_SOURCE_DIR}/Include

target_include_directories(icvStarter SYSTEM PUBLIC ${ZeroMQ_INCLUDE_DIR}
$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>  
    $<INSTALL_INTERFACE:>)
icv_install_module(Starter)
