icv_set_module_vars(Display)
#find_package(Boost REQUIRED log)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

add_definitions(-DBOOST_LOG_DYN_LINK)
#find_package(OpenCV REQUIRED ) 
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore icvExtensionOpenCV ${Boost_LIBRARIES} )
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

#message(" booost===   ${Boost_LIBRARIES}")
#message(" opencv===   ${OpenCV_LIBS}")
icv_set_solution_folder(icvDisplay  Sensors)

icv_install_module(Display)
