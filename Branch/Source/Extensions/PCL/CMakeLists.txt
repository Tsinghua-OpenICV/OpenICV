icv_set_module_vars(ExtensionPCL)

file(GLOB ${MODULE_HDR_NAME}
    ../../../Include/OpenICV/Extensions/PCL/*.h
    ../../../Include/OpenICV/Extensions/PCL/*.hxx)
list(APPEND ${MODULE_HDR_NAME} ../../../Include/OpenICV/icvPCL.h)

find_package(PCL REQUIRED)

add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvBasis ${PCL_LIBRARIES})
#message("-------------------------------------\n PCL_LIBS = ${PCL_LIBRARIES}")
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${PCL_INCLUDE_DIRS})

# FIXME: Remove this
set_target_properties(${MODULE_TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX)

add_definitions(-DBOOST_LOG_DYN_LINK)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_CONFIG_SUPPRESS_OUTDATED_MESSAGE)



foreach(_HEADER ${${MODULE_HDR_NAME}})
    get_filename_component(_HEADER ${_HEADER} ABSOLUTE)
    get_filename_component(_HEADER_DIR ${_HEADER} DIRECTORY)
    string(FIND ${_HEADER_DIR} "OpenICV" _HEADER_LOC REVERSE)
    string(SUBSTRING ${_HEADER_DIR} ${_HEADER_LOC} -1 _HEADER_REL)
    install(FILES ${_HEADER} DESTINATION include/${_HEADER_REL})
endforeach()


icv_install_module(ExtensionPCL)