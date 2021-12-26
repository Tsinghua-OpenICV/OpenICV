icv_set_module_vars(RecordReplay_old)
#find_package(Boost REQUIRED log)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
add_definitions(-DBOOST_LOG_DYN_LINK)
#set(OpenCV_DIR "/usr/local/share/OpenCV")
find_package(OpenCV REQUIRED ) 
find_package(PCL REQUIRED ) 
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore icvBasis  ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

#message(" booost===   ${Boost_LIBRARIES}")
#message(" opencv===   ${OpenCV_LIBS}")
icv_set_solution_folder(icvRecordReplay_old Functions)

icv_install_module(RecordReplay_old)
