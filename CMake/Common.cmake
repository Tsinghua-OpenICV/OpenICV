########## Module essentials ##########

# Set up essential names for a module
macro(icv_set_module_names MNAME)
    set(MODULE_NAME ${MNAME})                       # Module name
    string(TOUPPER ${MNAME} MODULE_NAME_UPPER)      # Module name (upper case)
    set(MODULE_TARGET_NAME icv${MNAME})             # Module library name
    set(MODULE_SRC_NAME ${MODULE_NAME_UPPER}_SRCS)  # List of module sources
    set(MODULE_HDR_NAME ${MODULE_NAME_UPPER}_HDRS)  # List of module headers
endmacro()

macro(icv_set_module_vars MNAME)
    icv_set_module_names(${MNAME})
    file(GLOB_RECURSE ${MODULE_SRC_NAME} *.cxx)

endmacro()

macro(icv_install_module MNAME)
    install(TARGETS icv${MNAME}
        COMPONENT ${MNAME}
        EXPORT icvExport
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION bin
        ARCHIVE DESTINATION lib
    )

    foreach(_HEADER ${${MODULE_HDR_NAME}})
        get_filename_component(_HEADER ${_HEADER} ABSOLUTE)
        get_filename_component(_HEADER_DIR ${_HEADER} DIRECTORY)
        string(FIND ${_HEADER_DIR} "OpenICV" _HEADER_LOC REVERSE)
        string(SUBSTRING ${_HEADER_DIR} ${_HEADER_LOC} -1 _HEADER_REL)
        install(FILES ${_HEADER} DESTINATION include/${_HEADER_REL})
    endforeach()
endmacro()

########## IDE settings ##########

set(USE_PROJECT_FOLDERS "IDE Solution folders" (MSVC_IDE OR CMAKE_GENERATOR MATCHES Xcode))
if(USE_PROJECT_FOLDERS)
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)
    set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif()

function(icv_set_solution_folder target folder)
    if(USE_PROJECT_FOLDERS)
        set_target_properties(${target} PROPERTIES FOLDER "${folder}")
    endif()
endfunction()

########## Build configurations ##########
set(CMAKE_DEBUG_POSTFIX _d)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)
if(BUILD_SHARED_LIBS)
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON) # generate .lib symbols
endif()

# if (WIN32 AND CMAKE_SYSTEM_VERSION)
#     set(ver ${CMAKE_SYSTEM_VERSION})
#     string(REPLACE "." "" ver ${ver})
#     string(REGEX REPLACE "([0-9])" "0\\1" ver ${ver})

#     set(${version} "0x${ver}")
#     add_definitions(-D_WIN32_WINNT=${version})
#     message(STATUS "Windows version: " ${version})
# endif()

########## TODO ##########
# TODO: Add macro to define library
# TODO: Add macro to process dependency
