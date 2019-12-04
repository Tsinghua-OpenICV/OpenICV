icv_set_module_vars(Core)
file(GLOB ${MODULE_HDR_NAME} ../../Include/OpenICV/Core/*.h)

find_package(Boost REQUIRED system chrono log)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
# header only libraries: type_traits dll variant

add_library(${MODULE_TARGET_NAME} STATIC ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC ${Boost_LIBRARIES})
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

icv_install_module(Core)
