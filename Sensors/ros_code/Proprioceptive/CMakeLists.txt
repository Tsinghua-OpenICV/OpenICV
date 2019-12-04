project(XbowComponent)

cmake_minimum_required(VERSION 2.6)
cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0043 NEW)
################################################################################
add_definitions( -DXBOWCOMPONENT_EXPORTS )
add_definitions (${QT_DEFINITIONS})
################################################################################
# FILES
set(PROJECT_SOURCE_FILES 
  # add your sources file to compile here
  crossbowComponent.hpp
  crossbowComponent.cpp
  VG700decoder.hpp
  VG700decoder.cpp
  ../driver/Win32SerialPort.cpp
  ../driver/Win32SerialPort.h
  )
  
set(FILES_TO_MOC 
  # add here the header files that you want to MOC
  ../driver/Win32SerialPort.h
  crossbowComponent.hpp
  )
  
set(LIB_TO_LINK
  # add your specific libraries to link here
  )

#common directives for creating the plugin
pacpus_plugin(PLUGIN_CPP PLUGIN_H ${PROJECT_NAME})

include_directories(${CMAKE_SOURCE_DIR})
include_directories( 
    ${PROJECT_BINARY_DIR}
    ${QT_INCLUDE_DIR}
    ${PACPUS_INCLUDE_DIR}
    ${PACPUS_INCLUDE_DIR}/Pacpus/
)

# ========================================
# Link directories
# ========================================
link_directories( 
	${PACPUS_LIB_DIR}
)

message("Creating source files for the plugin(s)")
message(STATUS PLUGIN_H=${PLUGIN_H})
message(STATUS PLUGIN_CPP=${PLUGIN_CPP})

qt_wrap_cpp(PLUGIN_MOC_SOURCES 
    ${PLUGIN_H}
    ${FILES_TO_MOC}
)
################################################################################
# BUILD AND LINK
pacpus_add_library(${PROJECT_NAME} SHARED
    ${PROJECT_SOURCE_FILES}
    ${PLUGIN_CPP}
    ${PLUGIN_MOC_SOURCES}
)
# LIBS
list(APPEND LIBS
    optimized FileLib debug FileLib_d
    optimized PacpusLib debug PacpusLib_d
    optimized PacpusTools debug PacpusTools_d
    ${QT_LIBRARIES}
)
if(WIN32)
    list(APPEND LIBS
        optimized ROAD_TIME debug ROAD_TIME_d
    )
endif()
# LINK

target_link_libraries(${PROJECT_NAME} 
		      ${PACPUS_LIBRARIES} 
	              ${PACPUS_DEPENDENCIES_LIB} 
                      ${LIBS}
                      ${QT_LIBRARIES}  
                     )
qt5_use_modules(${PROJECT_NAME} Network)
################################################################################
# FOLDERS
pacpus_folder(${PROJECT_NAME} "components")
################################################################################
# INSTALL
pacpus_install(${PROJECT_NAME}) 
