icv_set_module_vars(Engine)
include_directories(${OpenICV_ini_path}/Branch/Kernel/Include)

option(OPENICV_ENABLE_CONFIG_INI  "Enable INI configuration file support" ON)
option(OPENICV_ENABLE_CONFIG_YAML "Enable YAML configuration file support" OFF)
option(OPENICV_ENABLE_CONFIG_JSON "Enable JSON configuration file support" OFF)
option(OPENICV_ENABLE_CONFIG_XML  "Enable XML configuration file support" OFF)

find_package(Boost REQUIRED filesystem log)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)
# header only libraries: type_traits dll algorithm property_tree

add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
if (UNIX)
    target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore ${Boost_LIBRARIES} dl)
else ()
    target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore ${Boost_LIBRARIES})
endif ()
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC 
$<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>  
    $<INSTALL_INTERFACE:>)

icv_install_module(Engine)
