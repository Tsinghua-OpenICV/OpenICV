icv_set_module_vars(ExtensionEigen)

file(GLOB ${MODULE_HDR_NAME}
    ../../../Include/OpenICV/Extensions/Eigen/*.h
    ../../../Include/OpenICV/Extensions/Eigen/*.hxx)
list(APPEND ${MODULE_HDR_NAME} ../../../Include/OpenICV/icvEigen.h)

find_package(Eigen3 REQUIRED)

add_library(${MODULE_TARGET_NAME} SHARED ${${MODULE_SRC_NAME}} ${${MODULE_HDR_NAME}})
target_link_libraries(${MODULE_TARGET_NAME} PUBLIC icvBasis)
target_include_directories(${MODULE_TARGET_NAME} SYSTEM PUBLIC ${EIGEN_INCLUDE_DIRS})

# FIXME: Remove this
set_target_properties(${MODULE_TARGET_NAME} PROPERTIES LINKER_LANGUAGE CXX)

# icv_install_module(ExtensionEigen)
foreach(_HEADER ${${MODULE_HDR_NAME}})
    get_filename_component(_HEADER ${_HEADER} ABSOLUTE)
    get_filename_component(_HEADER_DIR ${_HEADER} DIRECTORY)
    string(FIND ${_HEADER_DIR} "OpenICV" _HEADER_LOC REVERSE)
    string(SUBSTRING ${_HEADER_DIR} ${_HEADER_LOC} -1 _HEADER_REL)
    install(FILES ${_HEADER} DESTINATION include/${_HEADER_REL})
endforeach()
