icv_set_module_vars(Core)

include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
find_package(Boost REQUIRED system chrono log)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
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


add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC ${Boost_LIBRARIES} ${ZeroMQ_LIBRARY} msgpackc)
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${ZeroMQ_INCLUDE_DIR}
$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>  
    $<INSTALL_INTERFACE:>)

icv_install_module(Core)
