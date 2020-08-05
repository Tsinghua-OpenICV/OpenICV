icv_set_module_vars(Basis)
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
set(CMAKE_INSTALL_PREFIX ${OpenICV_ini_path}/ICVOS)
endif()
file(GLOB ${MODULE_HDR_NAME} ../../Include/OpenICV/Basis/*.h)

find_package(Boost REQUIRED thread log)
add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)

# header only libraries: type_traits variant interprocess



add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvCore ${Boost_LIBRARIES} msgpack)

target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC 
 $<BUILD_INTERFACE:${Boost_INCLUDE_DIRS}>  
    $<INSTALL_INTERFACE:>)



icv_install_module(Basis)
