icv_set_module_vars(RecordReplay)
#find_package(Boost REQUIRED log)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

add_definitions(-DBOOST_LOG_DYN_LINK)
find_package(OpenCV REQUIRED ) 
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

find_package(PCL REQUIRED ) 
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(  ${PCL_LIBRARIES})

target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore icvBasis ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

#message(" booost===   ${Boost_LIBRARIES}")
#message(" opencv===   ${OpenCV_LIBS}")
icv_set_solution_folder(icvRecordReplay RecordPlayer)

icv_install_module(RecordReplay)
