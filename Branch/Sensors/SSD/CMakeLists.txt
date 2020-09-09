icv_set_module_vars(SSD)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

add_definitions(-DBOOST_LOG_DYN_LINK)
add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
include_directories( ./include)


target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore icvExtensionOpenCV ${Boost_LIBRARIES}  pthread hineon n2cube dputils)
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

icv_set_solution_folder(icvSSD Sensors)

icv_install_module(SSD)







