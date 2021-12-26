icv_set_module_vars(Comm)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)
add_library(icvComm SHARED VideoImageSource.cxx VideoImageSink.cxx)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
find_package(OpenCV REQUIRED ) 
add_definitions(-DBOOST_LOG_DYN_LINK)

target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore  ${Boost_LIBRARIES} ${OpenCV_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
icv_set_solution_folder(icvComm  Functions)

icv_install_module(Comm)
