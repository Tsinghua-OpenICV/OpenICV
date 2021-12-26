
icv_set_module_vars(helaRadar)
find_package(Boost COMPONENTS   system filesystem log  REQUIRED)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

add_definitions(-DBOOST_LOG_DYN_LINK)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} )

add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES( ${MODULE_TARGET_NAME}  icvCore icvNet ${Boost_LIBRARIES})
icv_set_solution_folder(${MODULE_TARGET_NAME}  Sensors)

icv_install_module(helaRadar)

