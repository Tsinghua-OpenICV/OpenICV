icv_set_module_vars(Basis)
file(GLOB ${MODULE_HDR_NAME} ../../Include/OpenICV/Basis/*.h)

find_package(Boost REQUIRED thread log)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

# header only libraries: type_traits variant interprocess

find_package(msgpack 3.2.0 REQUIRED)
message("msgpack   =    ${msgpack_LIBRARIES} ${msgpack_INCLUDE_DIRS}")
add_library(${MODULE_TARGET_NAME} STATIC ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore ${Boost_LIBRARIES} ${msgpack_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS} ${msgpack_INCLUDE_DIRS})

icv_install_module(Basis)
