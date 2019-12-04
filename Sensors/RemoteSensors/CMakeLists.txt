
icv_set_module_vars(RemoteSensors)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )


add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

TARGET_LINK_LIBRARIES( ${MODULE_TARGET_NAME}  icvCore icvNet ${Boost_LIBRARIES})
icv_set_solution_folder(${MODULE_TARGET_NAME}  Sensors)
icv_install_module(RemoteSensors)
