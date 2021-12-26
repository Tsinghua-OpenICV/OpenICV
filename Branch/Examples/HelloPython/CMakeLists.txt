icv_set_module_vars(HelloPython)

include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

find_package(OpenCV REQUIRED)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvBasis ${OpenCV_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${OpenCV_INCLUDE_DIRS})

icv_install_module(HelloPython)
