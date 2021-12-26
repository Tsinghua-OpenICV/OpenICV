icv_set_module_vars(UsefulFunction)

include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

find_package(Boost REQUIRED thread log)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

# header only libraries: type_traits variant interprocess


find_package(Protobuf REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}} ${OpenICV_ini_path}/Branch/Kernel/Include/OpenICV/Basis/icvProtoBufData.pb/icvPBData.pb.cc ${OpenICV_ini_path}/Branch/Kernel/Include/OpenICV/Basis/icvProtoBufData.pb/icvTestData.pb.cc)
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvBasis icvCore ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES} )

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC 
 $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>  
    $<INSTALL_INTERFACE:>)


icv_install_module(UsefulFunction)
