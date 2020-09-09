icv_set_module_vars(Comm)

add_library(icvComm SHARED VideoImageSource.cxx VideoImageSink.cxx)

icv_set_solution_folder(icvComm Sensors)

add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

add_definitions(-DBOOST_LOG_DYN_LINK)

target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore icvExtensionOpenCV ${Boost_LIBRARIES} )
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

icv_install_module(Comm)
