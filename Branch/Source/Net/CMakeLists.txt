icv_set_module_vars(Net)
set(${MODULE_HDR_NAME}
    ../../Include/OpenICV/Net/icvUdpReceiverSource.h
)

find_package(Boost REQUIRED thread log)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
# header only libraries: asio
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore ${Boost_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC 
$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>  
    $<INSTALL_INTERFACE:>)


icv_install_module(Net)
