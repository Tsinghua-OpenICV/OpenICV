# Add sources
set(MODULE_NAME util)

set(PROJECT_NAME "msf_lib_${MODULE_NAME}")
get_filename_component(PROJ_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../ ABSOLUTE)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${PROJ_ROOT})

add_library(${PROJECT_NAME} SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/src/util.cc)
target_link_libraries(${PROJECT_NAME} libGeographiccc)

set_target_properties(${PROJECT_NAME} PROPERTIES
                      LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/lib)