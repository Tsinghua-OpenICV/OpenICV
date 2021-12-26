icv_set_module_vars(CANCANFD)
#find_package(Boost REQUIRED log)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore  ${Boost_LIBRARIES} )
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} )

icv_set_solution_folder(icvCANCANFD Sensors)

icv_install_module(CANCANFD)
