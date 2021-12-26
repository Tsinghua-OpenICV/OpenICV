icv_set_module_vars(ZmqTest)

add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
find_package(cppzmq)

add_library(${MODULE_TARGET_NAME}  SHARED  ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
link_directories(${MODULE_TARGET_NAME} ${Boost_LIBRARY_DIR} )
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore icvBasis icvNet ${Boost_LIBRARIES} cppzmq)

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

icv_set_solution_folder(icvZmqTest Sensors)
icv_install_module(ZmqTest)
