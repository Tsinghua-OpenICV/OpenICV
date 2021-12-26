icv_set_module_vars(ZmqNet)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
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

# header only libraries: asio
#add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

add_library(${MODULE_TARGET_NAME}  SHARED  ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
link_directories(${MODULE_TARGET_NAME} ${Boost_LIBRARY_DIR} )
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore ${Boost_LIBRARIES})

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

icv_install_module(ZmqNet)
