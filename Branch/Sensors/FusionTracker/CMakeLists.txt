icv_set_module_vars(FusionTracker)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(OpenCV REQUIRED ) 
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore  ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

icv_set_solution_folder(icvFusionTracker Sensors)

icv_install_module(FusionTracker)

